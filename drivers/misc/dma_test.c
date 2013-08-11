/*
 * Generic on-chip SRAM allocation driver
 *
 * Copyright (C) 2012 Philipp Zabel, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/freezer.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

struct dmatest_info {
	u32 buf_size;
	bool running;

	struct device *dev;
	struct mutex lock;
	struct dentry *root;
};

static int dmatest_run(struct dmatest_info *dmatest)
{
	size_t buf_size = dmatest->buf_size;
	dma_addr_t src_buf_dma, dst_buf_dma;
	u8 *src_buf, *dst_buf;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	dma_cookie_t cookie;
	unsigned int i;
	int ret = 0;

	dev_info(dmatest->dev, "Running test with buf_size = %u\n", buf_size);

	src_buf = dma_alloc_coherent(dmatest->dev, buf_size,
					&src_buf_dma, GFP_KERNEL);
	if (!src_buf) {
		dev_err(dmatest->dev, "failed to allocate src buffer\n");
		return -ENOMEM;
	}

	dst_buf = dma_alloc_coherent(dmatest->dev, buf_size,
					&dst_buf_dma, GFP_KERNEL);
	if (!dst_buf) {
		dev_err(dmatest->dev, "failed to allocate dst buffer\n");
		ret = -ENOMEM;
		goto src_free;
	}

	memset(src_buf, 0x55, buf_size);
	memset(dst_buf, 0xaa, buf_size);

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	chan = dma_request_channel(mask, NULL, NULL);
	if (!chan) {
		dev_err(dmatest->dev, "failed to request DMA channel\n");
		ret = -ENODEV;
		goto dst_free;
	}

	cookie = dma_async_memcpy_buf_to_buf(chan, dst_buf, src_buf, buf_size);
	dma_sync_wait(chan, cookie);

	dma_release_channel(chan);

	for (i = 0; i < buf_size; ++i)
		if (dst_buf[i] != 0x55)
			break;

	if (i != buf_size) {
		dev_err(dmatest->dev, "DMA copy error on byte %u\n", i);
		ret = -EFAULT;
		goto dst_free;
	}

	dev_info(dmatest->dev, "Test finished successfully\n");

dst_free:
	dma_free_coherent(dmatest->dev, buf_size, dst_buf, dst_buf_dma);
src_free:
	dma_free_coherent(dmatest->dev, buf_size, src_buf, src_buf_dma);

	return ret;
}

static ssize_t dtf_read_run(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct dmatest_info *info = file->private_data;
	char buf[3];

	mutex_lock(&info->lock);

	if (info->running)
		buf[0] = '1';
	else
		buf[0] = '0';

	mutex_unlock(&info->lock);
	buf[1] = '\n';
	buf[2] = 0x00;
	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t dtf_write_run(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct dmatest_info *info = file->private_data;

	mutex_lock(&info->lock);
	info->running = true;
	mutex_unlock(&info->lock);

	dmatest_run(info);

	mutex_lock(&info->lock);
	info->running = false;
	mutex_unlock(&info->lock);

	return count;
}

static const struct file_operations dtf_run_fops = {
	.read	= dtf_read_run,
	.write	= dtf_write_run,
	.open	= simple_open,
	.llseek	= default_llseek,
};

static int dmatest_register_dbgfs(struct dmatest_info *info)
{
	struct dentry *d;
	int ret = -ENOMEM;

	d = debugfs_create_dir("dmatest", NULL);
	if (IS_ERR(d))
		return PTR_ERR(d);
	if (!d)
		goto err_root;

	info->root = d;

	/* Test parameters */

	d = debugfs_create_u32("test_buf_size", S_IWUSR | S_IRUGO,
					info->root, &info->buf_size);
	if (IS_ERR_OR_NULL(d))
		goto err_node;

	/* Run or stop threaded test */
	d = debugfs_create_file("run", S_IWUSR | S_IRUGO,
					info->root, info, &dtf_run_fops);
	if (IS_ERR_OR_NULL(d))
		goto err_node;

	return 0;

err_node:
	debugfs_remove_recursive(info->root);
err_root:
	pr_err("dmatest: Failed to initialize debugfs\n");
	return ret;
}

static int dmatest_probe(struct platform_device *pdev)
{
	struct dmatest_info *dmatest;
	int ret;

	dmatest = devm_kzalloc(&pdev->dev, sizeof(*dmatest), GFP_KERNEL);
	if (!dmatest)
		return -ENOMEM;

	dmatest->buf_size = SZ_4K;
	dmatest->dev = &pdev->dev;
	mutex_init(&dmatest->lock);
	dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));

	ret = dmatest_register_dbgfs(dmatest);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dmatest);

	return 0;
}

static int dmatest_remove(struct platform_device *pdev)
{
	struct dmatest_info *dmatest = platform_get_drvdata(pdev);

	debugfs_remove_recursive(dmatest->root);

	return 0;
}

static struct platform_driver dmatest_driver = {
	.driver = {
		.name = "dma-test",
	},
	.probe = dmatest_probe,
	.remove = dmatest_remove,
};
module_platform_driver(dmatest_driver);

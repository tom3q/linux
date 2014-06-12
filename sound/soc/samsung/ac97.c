/* sound/soc/samsung/ac97.c
 *
 * ALSA SoC Audio Layer - Samsung AC97 Controller driver
 *
 * Copyright (c) 2014 Tomasz Figa <tomasz.figa@gmai.com>
 *
 * Loosely based on old s3c24xx-ac97 driver.
 *
 * Copyright (c) 2010 Samsung Electronics Co. Ltd
 *	Author: Jaswinder Singh <jassisinghbrar@gmail.com>
 *	Credits: Graeme Gregory, Sean Choi
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/amba/pl08x.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>

#include <linux/platform_data/asoc-s3c.h>

#define GLBCTRL_REG			0x00

#define GLBCTRL_CODECREADYIE		(1 << 22)
#define GLBCTRL_PCMOUTURIE		(1 << 21)
#define GLBCTRL_PCMINORIE		(1 << 20)
#define GLBCTRL_MICINORIE		(1 << 19)
#define GLBCTRL_PCMOUTTIE		(1 << 18)
#define GLBCTRL_PCMINTIE		(1 << 17)
#define GLBCTRL_MICINTIE		(1 << 16)
#define GLBCTRL_PCMOUTTM_OFF		(0 << 12)
#define GLBCTRL_PCMOUTTM_PIO		(1 << 12)
#define GLBCTRL_PCMOUTTM_DMA		(2 << 12)
#define GLBCTRL_PCMOUTTM_MASK		(3 << 12)
#define GLBCTRL_PCMINTM_OFF		(0 << 10)
#define GLBCTRL_PCMINTM_PIO		(1 << 10)
#define GLBCTRL_PCMINTM_DMA		(2 << 10)
#define GLBCTRL_PCMINTM_MASK		(3 << 10)
#define GLBCTRL_MICINTM_OFF		(0 << 8)
#define GLBCTRL_MICINTM_PIO		(1 << 8)
#define GLBCTRL_MICINTM_DMA		(2 << 8)
#define GLBCTRL_MICINTM_MASK		(3 << 8)
#define GLBCTRL_TRANSFERDATAENABLE	(1 << 3)
#define GLBCTRL_ACLINKON		(1 << 2)
#define GLBCTRL_WARMRESET		(1 << 1)
#define GLBCTRL_COLDRESET		(1 << 0)

#define GLBCTRL_INT_SHIFT		24

#define GLBSTAT_REG			0x04

#define GLBSTAT_CODECREADY		(1 << 22)
#define GLBSTAT_PCMOUTUR		(1 << 21)
#define GLBSTAT_PCMINORI		(1 << 20)
#define GLBSTAT_MICINORI		(1 << 19)
#define GLBSTAT_PCMOUTTI		(1 << 18)
#define GLBSTAT_PCMINTI			(1 << 17)
#define GLBSTAT_MICINTI			(1 << 16)
#define GLBSTAT_MAINSTATE_IDLE		(0 << 0)
#define GLBSTAT_MAINSTATE_INIT		(1 << 0)
#define GLBSTAT_MAINSTATE_READY		(2 << 0)
#define GLBSTAT_MAINSTATE_ACTIVE	(3 << 0)
#define GLBSTAT_MAINSTATE_LP		(4 << 0)
#define GLBSTAT_MAINSTATE_WARM		(5 << 0)
#define GLBSTAT_MAINSTATE_MASK		(7 << 0)

#define GLBSTAT_INT_MASK		(GLBSTAT_CODECREADY | GLBSTAT_PCMOUTUR \
					| GLBSTAT_PCMINORI | GLBSTAT_MICINORI \
					| GLBSTAT_PCMOUTTI | GLBSTAT_PCMINTI \
					| GLBSTAT_MICINTI)
#define GBLSTAT_INT_SHIFT		16

#define CODEC_CMD_REG			0x08

#define CODEC_CMD_READ			(1 << 23)

#define STAT_REG			0x0c
#define PCM_ADDR_REG			0x10
#define PCM_DATA_REG			0x18
#define MIC_DATA_REG			0x1c

#define AC_CMD_ADDR(x)			(x << 16)
#define AC_CMD_DATA(x)			(x & 0xffff)

#ifdef CONFIG_ARCH_S3C64XX
#define filter_fn pl08x_filter_id
#else
#define filter_fn NULL
#endif

enum {
	dai_pcm,
	dai_mic
};

struct samsung_ac97 {
	struct device *dev;

	struct resource *iores;
	void __iomem *regs;
	struct clk *clk;
	int irq;

	struct snd_dmaengine_dai_dma_data playback_data;
	struct snd_dmaengine_dai_dma_data capture_data;
	struct snd_dmaengine_dai_dma_data mic_data;
	struct snd_ac97_bus_ops bus_ops;

	spinlock_t slock;
	struct mutex lock;
	struct completion done;
};

static inline struct samsung_ac97 *dai_to_ac97(struct snd_soc_dai *dai)
{
	return snd_soc_dai_get_drvdata(dai);
}

static inline struct samsung_ac97 *snd_ac97_to_ac97(struct snd_ac97 *ac97)
{
	return container_of(ac97->bus->ops, struct samsung_ac97, bus_ops);
}

/*
 * AC97 control
 */

static void samsung_ac97_activate(struct samsung_ac97 *ac97)
{
	unsigned long timeout;
	unsigned long flags;
	u32 ctrl, stat;

	stat = readl(ac97->regs + GLBSTAT_REG) & GLBSTAT_MAINSTATE_MASK;
	if (stat == GLBSTAT_MAINSTATE_ACTIVE)
		return; /* Return if already active */

	reinit_completion(&ac97->done);

	spin_lock_irqsave(&ac97->slock, flags);

	ctrl = readl(ac97->regs + GLBCTRL_REG);
	ctrl = GLBCTRL_ACLINKON;
	writel(ctrl, ac97->regs + GLBCTRL_REG);

	spin_unlock_irqrestore(&ac97->slock, flags);

	msleep(1);

	spin_lock_irqsave(&ac97->slock, flags);

	ctrl = readl(ac97->regs + GLBCTRL_REG);
	ctrl |= GLBCTRL_TRANSFERDATAENABLE;
	writel(ctrl, ac97->regs + GLBCTRL_REG);

	spin_unlock_irqrestore(&ac97->slock, flags);

	msleep(1);

	spin_lock_irqsave(&ac97->slock, flags);

	ctrl = readl(ac97->regs + GLBCTRL_REG);
	ctrl |= GLBCTRL_CODECREADYIE;
	writel(ctrl, ac97->regs + GLBCTRL_REG);

	spin_unlock_irqrestore(&ac97->slock, flags);

	timeout = wait_for_completion_timeout(&ac97->done,
						msecs_to_jiffies(1000));
	if (!timeout)
		dev_err(ac97->dev, "unable to activate");
}

static unsigned short samsung_ac97_read(struct snd_ac97 *snd_ac97,
					unsigned short reg)
{
	struct samsung_ac97 *ac97 = snd_ac97_to_ac97(snd_ac97);
	u32 ctrl, ac_codec_cmd;
	unsigned long flags;
	u32 stat;
	u16 data;
	u8 addr;

	mutex_lock(&ac97->lock);

	samsung_ac97_activate(ac97);

	reinit_completion(&ac97->done);

	ac_codec_cmd = readl(ac97->regs + CODEC_CMD_REG);
	ac_codec_cmd = CODEC_CMD_READ | AC_CMD_ADDR(reg);
	writel(ac_codec_cmd, ac97->regs + CODEC_CMD_REG);

	udelay(50);

	spin_lock_irqsave(&ac97->slock, flags);

	ctrl = readl(ac97->regs + GLBCTRL_REG);
	ctrl |= GLBCTRL_CODECREADYIE;
	writel(ctrl, ac97->regs + GLBCTRL_REG);

	spin_unlock_irqrestore(&ac97->slock, flags);

	if (!wait_for_completion_timeout(&ac97->done, HZ))
		dev_err(ac97->dev, "unable to read");

	stat = readl(ac97->regs + STAT_REG);
	addr = (stat >> 16) & 0x7f;
	data = stat & 0xffff;

	if (addr != reg)
		dev_err(ac97->dev, "req addr = %02x, rep addr = %02x\n",
			reg, addr);

	mutex_unlock(&ac97->lock);

	return data;
}

static void samsung_ac97_write(struct snd_ac97 *snd_ac97, unsigned short reg,
			       unsigned short val)
{
	struct samsung_ac97 *ac97 = snd_ac97_to_ac97(snd_ac97);
	u32 ctrl, ac_codec_cmd;
	unsigned long flags;

	mutex_lock(&ac97->lock);

	samsung_ac97_activate(ac97);

	reinit_completion(&ac97->done);

	ac_codec_cmd = readl(ac97->regs + CODEC_CMD_REG);
	ac_codec_cmd = AC_CMD_ADDR(reg) | AC_CMD_DATA(val);
	writel(ac_codec_cmd, ac97->regs + CODEC_CMD_REG);

	udelay(50);

	spin_lock_irqsave(&ac97->slock, flags);

	ctrl = readl(ac97->regs + GLBCTRL_REG);
	ctrl |= GLBCTRL_CODECREADYIE;
	writel(ctrl, ac97->regs + GLBCTRL_REG);

	spin_unlock_irqrestore(&ac97->slock, flags);

	if (!wait_for_completion_timeout(&ac97->done, HZ))
		dev_err(ac97->dev, "unable to write");

	ac_codec_cmd = readl(ac97->regs + CODEC_CMD_REG);
	ac_codec_cmd |= CODEC_CMD_READ;
	writel(ac_codec_cmd, ac97->regs + CODEC_CMD_REG);

	mutex_unlock(&ac97->lock);
}

static void samsung_ac97_cold_reset(struct snd_ac97 *snd_ac97)
{
	struct samsung_ac97 *ac97 = snd_ac97_to_ac97(snd_ac97);

	dev_dbg(ac97->dev, "cold reset\n");

	writel(GLBCTRL_COLDRESET, ac97->regs + GLBCTRL_REG);
	msleep(1);

	writel(0, ac97->regs + GLBCTRL_REG);
	msleep(1);
}

static void samsung_ac97_warm_reset(struct snd_ac97 *snd_ac97)
{
	struct samsung_ac97 *ac97 = snd_ac97_to_ac97(snd_ac97);

	dev_dbg(ac97->dev, "warm reset\n");

	writel(GLBCTRL_WARMRESET, ac97->regs + GLBCTRL_REG);
	msleep(1);

	writel(0, ac97->regs + GLBCTRL_REG);
	msleep(1);

	samsung_ac97_activate(ac97);
}

static struct snd_ac97_bus_ops samsung_ac97_ops = {
	.read       = samsung_ac97_read,
	.write      = samsung_ac97_write,
	.warm_reset = samsung_ac97_warm_reset,
	.reset      = samsung_ac97_cold_reset,
};

/*
 * IRQ handler
 */

static irqreturn_t samsung_ac97_irq(int irq, void *dev_id)
{
	struct samsung_ac97 *ac97 = dev_id;
	unsigned long flags;
	u32 stat, ctrl;

	stat = readl(ac97->regs + GLBSTAT_REG) & GLBSTAT_INT_MASK;

	if (stat & GLBSTAT_CODECREADY) {
		spin_lock_irqsave(&ac97->slock, flags);

		ctrl = readl(ac97->regs + GLBCTRL_REG);
		ctrl &= ~GLBCTRL_CODECREADYIE;
		writel(ctrl, ac97->regs + GLBCTRL_REG);

		spin_unlock_irqrestore(&ac97->slock, flags);

		complete(&ac97->done);
	}

	spin_lock_irqsave(&ac97->slock, flags);

	ctrl = readl(ac97->regs + GLBCTRL_REG);
	ctrl |= (stat >> GBLSTAT_INT_SHIFT) << GLBCTRL_INT_SHIFT;
	writel(ctrl, ac97->regs + GLBCTRL_REG);

	spin_unlock_irqrestore(&ac97->slock, flags);

	return IRQ_HANDLED;
}

/*
 * Main DAI
 */

static int samsung_ac97_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct samsung_ac97 *ac97 = dai_to_ac97(rtd->cpu_dai);
	unsigned long flags;
	u32 mask;
	u32 ctrl;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		mask = GLBCTRL_PCMINTM_DMA;
	else
		mask = GLBCTRL_PCMOUTTM_DMA;

	spin_lock_irqsave(&ac97->slock, flags);

	ctrl = readl(ac97->regs + GLBCTRL_REG);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ctrl |= mask;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ctrl &= ~mask;
		break;

	default:
		WARN_ON(1);
	}

	writel(ctrl, ac97->regs + GLBCTRL_REG);

	spin_unlock_irqrestore(&ac97->slock, flags);

	return 0;
}

static const struct snd_soc_dai_ops samsung_ac97_dai_ops = {
	.trigger	= samsung_ac97_trigger,
};

static int samsung_ac97_dai_probe(struct snd_soc_dai *dai)
{
	struct samsung_ac97 *ac97 = dai_to_ac97(dai);

	ac97->playback_data.chan_name = "tx";
	ac97->playback_data.addr = ac97->iores->start + PCM_DATA_REG;
	ac97->playback_data.addr_width = 4;

	ac97->capture_data.chan_name = "rx";
	ac97->capture_data.addr = ac97->iores->start + PCM_DATA_REG;
	ac97->capture_data.addr_width = 4;

	snd_soc_dai_init_dma_data(dai, &ac97->playback_data,
					&ac97->capture_data);

	return 0;
}

/*
 * Mic DAI
 */

static int samsung_ac97_mic_trigger(struct snd_pcm_substream *substream,
				    int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct samsung_ac97 *ac97 = dai_to_ac97(rtd->cpu_dai);
	unsigned long flags;
	u32 ctrl;

	spin_lock_irqsave(&ac97->slock, flags);

	ctrl = readl(ac97->regs + GLBCTRL_REG);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ctrl |= GLBCTRL_MICINTM_DMA;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ctrl &= ~GLBCTRL_MICINTM_MASK;
		break;

	default:
		WARN_ON(1);
	}

	writel(ctrl, ac97->regs + GLBCTRL_REG);

	spin_unlock_irqrestore(&ac97->slock, flags);

	return 0;
}

static const struct snd_soc_dai_ops samsung_ac97_mic_dai_ops = {
	.trigger	= samsung_ac97_mic_trigger,
};

static int samsung_ac97_mic_dai_probe(struct snd_soc_dai *dai)
{
	struct samsung_ac97 *ac97 = dai_to_ac97(dai);

	ac97->mic_data.chan_name = "mic";
	ac97->mic_data.addr = ac97->iores->start + MIC_DATA_REG;
	ac97->mic_data.addr_width = 2;

	snd_soc_dai_init_dma_data(dai, NULL, &ac97->mic_data);

	return 0;
}

/*
 * Platform driver
 */

static struct snd_soc_dai_driver samsung_ac97_dais[] = {
	[dai_pcm] = {
		.name =	"samsung-ac97",
		.ac97_control = 1,
		.playback = {
			.stream_name = "AC97 Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "AC97 Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.probe = samsung_ac97_dai_probe,
		.ops = &samsung_ac97_dai_ops,
	},
	[dai_mic] = {
		.name = "samsung-ac97-mic",
		.ac97_control = 1,
		.capture = {
			.stream_name = "AC97 Mic Capture",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.probe = samsung_ac97_mic_dai_probe,
		.ops = &samsung_ac97_mic_dai_ops,
	},
};

static const struct snd_soc_component_driver samsung_ac97_component = {
	.name = "samsung-ac97",
};

static const struct snd_dmaengine_pcm_config samsung_dmaengine_pcm_config = {
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.compat_filter_fn = filter_fn,
};

#ifdef CONFIG_PM_RUNTIME
static int samsung_ac97_runtime_suspend(struct device *dev)
{
	struct samsung_ac97 *ac97 = dev_get_drvdata(dev);

	clk_disable_unprepare(ac97->clk);

	return 0;
}

static int samsung_ac97_runtime_resume(struct device *dev)
{
	struct samsung_ac97 *ac97 = dev_get_drvdata(dev);

	clk_prepare_enable(ac97->clk);

	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static int samsung_ac97_probe(struct platform_device *pdev)
{
	struct s3c_audio_pdata *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	struct samsung_ac97 *ac97;
	int ret;

	ac97 = devm_kzalloc(&pdev->dev, sizeof(*ac97), GFP_KERNEL);
	if (!ac97)
		return -ENOMEM;
	ac97->dev = &pdev->dev;
	platform_set_drvdata(pdev, ac97);

	ac97->iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ac97->regs = devm_ioremap_resource(&pdev->dev, ac97->iores);
	if (IS_ERR(ac97->regs))
		return PTR_ERR(ac97->regs);

	ac97->irq = platform_get_irq(pdev, 0);
	if (ac97->irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ resource\n");
		return ac97->irq;
	}

	ac97->clk = devm_clk_get(&pdev->dev, "ac97");
	if (IS_ERR(ac97->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(ac97->clk);
	}

	init_completion(&ac97->done);
	mutex_init(&ac97->lock);
	spin_lock_init(&ac97->slock);
	memcpy(&ac97->bus_ops, &samsung_ac97_ops, sizeof(ac97->bus_ops));

	if (pdata && pdata->cfg_gpio) {
		ret = pdata->cfg_gpio(pdev);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to configure GPIO pins\n");
			return ret;
		}
	}

	ret = devm_request_irq(&pdev->dev, ac97->irq, samsung_ac97_irq, 0,
				dev_name(&pdev->dev), ac97);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request interrupt\n");
		return ret;
	}

	ret = devm_snd_soc_register_component(&pdev->dev,
						&samsung_ac97_component,
						samsung_ac97_dais,
						ARRAY_SIZE(samsung_ac97_dais));
	if (ret) {
		dev_err(&pdev->dev, "failed to register component\n");
		return ret;
	}

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev,
				&samsung_dmaengine_pcm_config,
				SND_DMAENGINE_PCM_FLAG_CUSTOM_CHANNEL_NAME |
				SND_DMAENGINE_PCM_FLAG_COMPAT);
	if (ret) {
		dev_err(&pdev->dev, "failed to register DMA engine PCM\n");
		return ret;
	}

	ret = snd_soc_set_ac97_ops(&ac97->bus_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to set AC97 ops\n");
		return ret;
	}

	pm_runtime_enable(&pdev->dev);

	if (np)
		of_platform_populate(np, NULL, NULL, &pdev->dev);

	return 0;
}

static int samsung_ac97_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct dev_pm_ops samsung_ac97_pm_ops = {
	SET_RUNTIME_PM_OPS(samsung_ac97_runtime_suspend,
				samsung_ac97_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id samsung_ac97_of_matches[] = {
	{ .compatible = "samsung,s3c2410-ac97", },
	{ /* sentinel */ }
};
#endif

static struct platform_driver samsung_ac97_driver = {
	.probe  = samsung_ac97_probe,
	.remove = samsung_ac97_remove,
	.driver = {
		.name = "samsung-ac97",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(samsung_ac97_of_matches),
		.pm = &samsung_ac97_pm_ops,
	},
};
module_platform_driver(samsung_ac97_driver);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("AC97 driver for the Samsung SoCs");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:samsung-ac97");

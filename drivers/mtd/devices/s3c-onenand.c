/*
 * Samsung S3C64XX/S5PC100 OneNAND driver
 *
 *  Copyright 2013 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * Based on OneNAND subsystem. Credits for parts borrowed from there
 * shall go to original authors.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/onenand.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>

/*
 * OneNAND Controller registers
 */
#define MEM_CFG_REG			0x000
#define BURST_LEN_REG			0x010
#define MEM_RESET_REG			0x020
#define INT_ERR_STAT_REG		0x030
#define INT_ERR_MASK_REG		0x040
#define INT_ERR_ACK_REG			0x050
#define ECC_ERR_STAT_REG		0x060
#define MANUFACT_ID_REG			0x070
#define DEVICE_ID_REG			0x080
#define DATA_BUF_SIZE_REG		0x090
#define BOOT_BUF_SIZE_REG		0x0a0
#define BUF_AMOUNT_REG			0x0b0
#define TECH_REG			0x0c0
#define FBA_WIDTH_REG			0x0d0
#define FPA_WIDTH_REG			0x0e0
#define FSA_WIDTH_REG			0x0f0
#define REVISION_REG			0x100
#define DATARAM0_REG			0x110	/* s3c64xx only */
#define DATARAM1_REG			0x120	/* s3c64xx only */
#define SYNC_MODE_REG			0x130
#define TRANS_SPARE_REG			0x140
#define DBS_DFS_WIDTH_REG		0x160	/* s3c64xx only */
#define PAGE_CNT_REG			0x170
#define ERR_PAGE_ADDR_REG		0x180
#define BURST_RD_LAT_REG		0x190
#define INT_PIN_ENABLE_REG		0x1a0
#define INT_MON_CYC_REG			0x1b0
#define ACC_CLOCK_REG			0x1c0
#define SLOW_RD_PATH_REG		0x1d0	/* s3c64xx only */
#define ERR_BLK_ADDR_REG		0x1e0
#define FLASH_VER_ID_REG		0x1f0
#define BANK_EN_REG			0x220	/* s5pc100 only */
#define WTCHDG_RST_L_REG		0x260	/* s5pc100 only */
#define WTCHDG_RST_H_REG		0x270	/* s5pc100 only */
#define SYNC_WRITE_REG			0x280	/* s5pc100 only */
#define CACHE_READ_REG			0x290	/* s5pc100 only */
#define COLD_RST_DLY_REG		0x2a0	/* s5pc100 only */
#define DDP_DEVICE_REG			0x2b0	/* s5pc100 only */
#define MULTI_PLANE_REG			0x2c0	/* s5pc100 only */
#define MEM_CNT_REG			0x2d0	/* s5pc100 only */
#define TRANS_MODE_REG			0x2e0	/* s5pc100 only */
#define DEV_STAT_REG			0x2f0	/* s5pc100 only */
#define FLASH_AUX_CNTRL_REG		0x300	/* s3c6410 only */
#define ECC_ERR_STAT_2_REG		0x300	/* s5pc100 only */
#define FLASH_AFIFO_CNT_REG		0x310	/* s3c6410 only */
#define ECC_ERR_STAT_3_REG		0x310	/* s5pc100 only */
#define ECC_ERR_STAT_4_REG		0x320	/* s5pc100 only */
#define EFCT_BUF_CNT_REG		0x330	/* s5pc100 only */
#define DEV_PAGE_SIZE_REG		0x340	/* s5pc100 only */
#define SUPERLOAD_EN_REG		0x350	/* s5pc100 only */
#define CACHE_PRG_EN_REG		0x360	/* s5pc100 only */
#define SINGLE_PAGE_BUF_REG		0x370	/* s5pc100 only */
#define OFFSET_ADDR_REG			0x380	/* s5pc100 only */
#define INT_MON_STATUS			0x390	/* s5pc100 only */

#define ONENAND_MEM_RESET_HOT		0x3
#define ONENAND_MEM_RESET_COLD		0x2
#define ONENAND_MEM_RESET_WARM		0x1

#define CACHE_OP_ERR			(1 << 13)
#define RST_CMP				(1 << 12)
#define RDY_ACT				(1 << 11)
#define INT_ACT				(1 << 10)
#define UNSUP_CMD			(1 << 9)
#define LOCKED_BLK			(1 << 8)
#define BLK_RW_CMP			(1 << 7)
#define ERS_CMP				(1 << 6)
#define PGM_CMP				(1 << 5)
#define LOAD_CMP			(1 << 4)
#define ERS_FAIL			(1 << 3)
#define PGM_FAIL			(1 << 2)
#define INT_TO				(1 << 1)
#define LD_FAIL_ECC_ERR			(1 << 0)

#define TSRF				(1 << 0)

#define ERASE_STATUS_CMD		0x00
#define MULTI_ERASE_SET_CMD		0x01
#define ERASE_START_CMD			0x03
#define UNLOCK_START_CMD		0x08
#define UNLOCK_END_CMD			0x09
#define LOCK_START_CMD			0x0a
#define LOCK_END_CMD			0x0b
#define LOCK_TIGHT_START_CMD		0x0c
#define LOCK_TIGHT_END_CMD		0x0d
#define UNLOCK_ALL_CMD			0x0e
#define RMW_LOAD_PAGE_CMD		0x10
#define RMW_WRITE_PAGE_CMD		0x11
#define OTP_ACCESS_CMD			0x12
#define SPARE_ACCESS_ONLY_CMD		0x13	/* s5pc100 only? */
#define MAIN_ACCESS_ONLY_CMD		0x14
#define ERASE_VERIFY_CMD		0x15
#define MAIN_SPARE_ACCESS_CMD		0x16	/* s5pc100 only? */
#define COPY_SRC_ADDR_SET_CMD		0x1000
#define COPY_DST_ADDR_SET_CMD		0x2000
#define PIPELINE_READ_CMD		0x4000
#define PIPELINE_2X_WRITE_CMD		0x4100
#define PIPELINE_2X_READ_CMD		0x4200	/* s5pc100 only? */
#define PIPELINE_2WAY_READ_CMD		0x4300	/* s5pc100 only? */
#define PIPELINE_4WAY_READ_CMD		0x4400	/* s5pc100 only? */
#define PIPELINE_2WAY_WRITE_CMD		0x4600	/* s5pc100 only? */
#define PIPELINE_4WAY_WRITE_CMD		0x4700	/* s5pc100 only? */

#define MAP_00				(0x0)
#define MAP_01				(0x1)
#define MAP_10				(0x2)
#define MAP_11				(0x3)

struct s3c_onenand_variant {
	u8 cmd_map_shift;
	u8 fba_shift;
	u8 fpa_shift;
	u8 fsa_shift;
};

struct s3c_onenand {
	struct mtd_info mtd;

	struct device *dev;
	const struct s3c_onenand_variant *variant;
	void __iomem *base;
	void __iomem *ahb_addr;

	unsigned int manuf_id;
	unsigned int device_id;
	unsigned int version_id;
	unsigned int technology;
	unsigned int options;
};

/*
 * onenand_oob_128 - oob info for OneNAND with 4KB page
 *
 * Based on specification:
 * 4Gb M-die OneNAND Flash (KFM4G16Q4M, KFN8G16Q4M). Rev. 1.3, Apr. 2010
 *
 * For eccpos we expose only 64 bytes out of 72 (see struct nand_ecclayout)
 *
 * oobfree uses the spare area fields marked as
 * "Managed by internal ECC logic for Logical Sector Number area"
 */
static struct nand_ecclayout onenand_oob_128 = {
	.eccbytes	= 64,
	.eccpos		= {
		7, 8, 9, 10, 11, 12, 13, 14, 15,
		23, 24, 25, 26, 27, 28, 29, 30, 31,
		39, 40, 41, 42, 43, 44, 45, 46, 47,
		55, 56, 57, 58, 59, 60, 61, 62, 63,
		71, 72, 73, 74, 75, 76, 77, 78, 79,
		87, 88, 89, 90, 91, 92, 93, 94, 95,
		103, 104, 105, 106, 107, 108, 109, 110, 111,
		119
	},
	.oobfree	= {
		{2, 3}, {18, 3}, {34, 3}, {50, 3},
		{66, 3}, {82, 3}, {98, 3}, {114, 3}
	}
};

/**
 * onenand_oob_64 - oob info for large (2KB) page
 */
static struct nand_ecclayout onenand_oob_64 = {
	.eccbytes	= 20,
	.eccpos		= {
		8, 9, 10, 11, 12,
		24, 25, 26, 27, 28,
		40, 41, 42, 43, 44,
		56, 57, 58, 59, 60,
		},
	.oobfree	= {
		{2, 3}, {14, 2}, {18, 3}, {30, 2},
		{34, 3}, {46, 2}, {50, 3}, {62, 2}
	}
};

/**
 * onenand_oob_32 - oob info for middle (1KB) page
 */
static struct nand_ecclayout onenand_oob_32 = {
	.eccbytes	= 10,
	.eccpos		= {
		8, 9, 10, 11, 12,
		24, 25, 26, 27, 28,
		},
	.oobfree	= { {2, 3}, {14, 2}, {18, 3}, {30, 2} }
};

/**
 * onenand_get_density - [DEFAULT] Get OneNAND density
 * @param dev_id	OneNAND device ID
 *
 * Get OneNAND density from device ID
 */
static inline int onenand_get_density(int dev_id)
{
	int density = dev_id >> ONENAND_DEVICE_DENSITY_SHIFT;
	return (density & ONENAND_DEVICE_DENSITY_MASK);
}

static const struct onenand_manufacturers onenand_manuf_ids[] = {
        {ONENAND_MFR_SAMSUNG, "Samsung"},
	{ONENAND_MFR_NUMONYX, "Numonyx"},
};

/**
 * onenand_print_device_info - Print device & version ID
 * @param device        device ID
 * @param version	version ID
 *
 * Print device & version ID
 */
static void onenand_print_device_info(struct s3c_onenand *this)
{
	int vcc, demuxed, ddp, density, flexonenand;
	int size = ARRAY_SIZE(onenand_manuf_ids);
	char *name;
        int i;

	for (i = 0; i < size; i++)
                if (this->manuf_id == onenand_manuf_ids[i].id)
                        break;

	if (i < size)
		name = onenand_manuf_ids[i].name;
	else
		name = "Unknown";

        vcc = this->device_id & ONENAND_DEVICE_VCC_MASK;
        demuxed = this->device_id & ONENAND_DEVICE_IS_DEMUX;
        ddp = this->device_id & ONENAND_DEVICE_IS_DDP;
        density = onenand_get_density(this->device_id);
	flexonenand = this->device_id & DEVICE_IS_FLEXONENAND;
	dev_info(this->dev, "%s%sOneNAND%s %dMB %sV 16-bit (0x%02x)\n",
		demuxed ? "" : "Muxed ",
		flexonenand ? "Flex-" : "",
                ddp ? "(DDP)" : "",
                (16 << density),
                vcc ? "2.65/3.3" : "1.8",
                this->device_id);
	dev_info(this->dev, "OneNAND version = 0x%04x\n", this->version_id);
	dev_info(this->dev, "OneNAND Manufacturer: %s (0x%0x)\n",
							name, this->manuf_id);
}

/**
 * onenand_check_features - Check and set OneNAND features
 * @param mtd		MTD data structure
 *
 * Check and set OneNAND features
 * - lock scheme
 * - two plane
 */
static void onenand_check_features(struct s3c_onenand *this)
{
	unsigned int density, process, numbufs;

	/* Lock scheme depends on density and process */
	density = onenand_get_density(this->device_id);
	process = this->version_id >> ONENAND_VERSION_PROCESS_SHIFT;
	numbufs = readl(this->base + BUF_AMOUNT_REG) >> 8;

	/* Lock scheme */
	switch (density) {
	case ONENAND_DEVICE_DENSITY_4Gb:
		if (ONENAND_IS_DDP(this))
			this->options |= ONENAND_HAS_2PLANE;
		else if (numbufs == 1) {
			this->options |= ONENAND_HAS_4KB_PAGE;
			this->options |= ONENAND_HAS_CACHE_PROGRAM;
			/*
			 * There are two different 4KiB pagesize chips
			 * and no way to detect it by H/W config values.
			 *
			 * To detect the correct NOP for each chips,
			 * It should check the version ID as workaround.
			 *
			 * Now it has as following
			 * KFM4G16Q4M has NOP 4 with version ID 0x0131
			 * KFM4G16Q5M has NOP 1 with versoin ID 0x013e
			 */
			if ((this->version_id & 0xf) == 0xe)
				this->options |= ONENAND_HAS_NOP_1;
		}

	case ONENAND_DEVICE_DENSITY_2Gb:
		/* 2Gb DDP does not have 2 plane */
		if (!ONENAND_IS_DDP(this))
			this->options |= ONENAND_HAS_2PLANE;
		this->options |= ONENAND_HAS_UNLOCK_ALL;

	case ONENAND_DEVICE_DENSITY_1Gb:
		/* A-Die has all block unlock */
		if (process)
			this->options |= ONENAND_HAS_UNLOCK_ALL;
		break;

	default:
		/* Some OneNAND has continuous lock scheme */
		if (!process)
			this->options |= ONENAND_HAS_CONT_LOCK;
		break;
	}

	/* The MLC has 4KiB pagesize. */
	if (ONENAND_IS_MLC(this))
		this->options |= ONENAND_HAS_4KB_PAGE;

	if (ONENAND_IS_4KB_PAGE(this))
		this->options &= ~ONENAND_HAS_2PLANE;

	if (FLEXONENAND(this)) {
		this->options &= ~ONENAND_HAS_CONT_LOCK;
		this->options |= ONENAND_HAS_UNLOCK_ALL;
	}

	if (this->options & ONENAND_HAS_CONT_LOCK)
		dev_dbg(this->dev, "Lock scheme is Continuous Lock\n");
	if (this->options & ONENAND_HAS_UNLOCK_ALL)
		dev_dbg(this->dev, "Chip support all block unlock\n");
	if (this->options & ONENAND_HAS_2PLANE)
		dev_dbg(this->dev, "Chip has 2 plane\n");
	if (this->options & ONENAND_HAS_4KB_PAGE)
		dev_dbg(this->dev, "Chip has 4KiB pagesize\n");
	if (this->options & ONENAND_HAS_CACHE_PROGRAM)
		dev_dbg(this->dev, "Chip has cache program feature\n");
}

/**
 * onenand_erase - [MTD Interface] erase block(s)
 * @param mtd		MTD device structure
 * @param instr		erase instruction
 *
 * Erase one or more blocks
 */
static int s3c_onenand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	return -EINVAL;
}

/**
 * onenand_read - [MTD Interface] Read data from flash
 * @param mtd		MTD device structure
 * @param from		offset to read from
 * @param len		number of bytes to read
 * @param retlen	pointer to variable to store the number of read bytes
 * @param buf		the databuffer to put data
 *
 * Read with ecc
*/
static int s3c_onenand_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	return -EINVAL;
}

/**
 * onenand_write - [MTD Interface] write buffer to FLASH
 * @param mtd		MTD device structure
 * @param to		offset to write to
 * @param len		number of bytes to write
 * @param retlen	pointer to variable to store the number of written bytes
 * @param buf		the data to write
 *
 * Write with ECC
 */
static int s3c_onenand_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	return -EINVAL;
}

/**
 * onenand_read_oob - [MTD Interface] Read main and/or out-of-band
 * @param mtd:		MTD device structure
 * @param from:		offset to read from
 * @param ops:		oob operation description structure

 * Read main and/or out-of-band
 */
static int s3c_onenand_read_oob(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
	return -EINVAL;
}

/**
 * onenand_write_oob - [MTD Interface] NAND write data and/or out-of-band
 * @param mtd:		MTD device structure
 * @param to:		offset to write
 * @param ops:		oob operation description structure
 */
static int s3c_onenand_write_oob(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
	return -EINVAL;
}

/**
 * onenand_panic_write - [MTD Interface] write buffer to FLASH in a panic context
 * @param mtd		MTD device structure
 * @param to		offset to write to
 * @param len		number of bytes to write
 * @param retlen	pointer to variable to store the number of written bytes
 * @param buf		the data to write
 *
 * Write with ECC
 */
static int s3c_onenand_panic_write(struct mtd_info *mtd, loff_t to, size_t len,
			 size_t *retlen, const u_char *buf)
{
	return -EINVAL;
}

/**
 * onenand_sync - [MTD Interface] sync
 * @param mtd		MTD device structure
 *
 * Sync is actually a wait for chip ready function
 */
static void s3c_onenand_sync(struct mtd_info *mtd)
{
}

/**
 * onenand_lock - [MTD Interface] Lock block(s)
 * @param mtd		MTD device structure
 * @param ofs		offset relative to mtd start
 * @param len		number of bytes to unlock
 *
 * Lock one or more blocks
 */
static int s3c_onenand_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	return -EINVAL;
}

/**
 * onenand_unlock - [MTD Interface] Unlock block(s)
 * @param mtd		MTD device structure
 * @param ofs		offset relative to mtd start
 * @param len		number of bytes to unlock
 *
 * Unlock one or more blocks
 */
static int s3c_onenand_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	return -EINVAL;
}

/**
 * onenand_block_isbad - [MTD Interface] Check whether the block at the given offset is bad
 * @param mtd		MTD device structure
 * @param ofs		offset relative to mtd start
 *
 * Check whether the block is bad
 */
static int s3c_onenand_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	return -EINVAL;
}

/**
 * onenand_block_markbad - [MTD Interface] Mark the block at the given offset as bad
 * @param mtd		MTD device structure
 * @param ofs		offset relative to mtd start
 *
 * Mark the block as bad
 */
static int s3c_onenand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	return -EINVAL;
}

/**
 * onenand_probe - [OneNAND Interface] Probe the OneNAND device
 * @param mtd		MTD device structure
 */
static int s3c_onenand_setup(struct s3c_onenand *this)
{
	struct mtd_info *mtd = &this->mtd;
	int density;
	int i;

	/* Read manufacturer and device IDs from Register */
	this->manuf_id = readl(this->base + MANUFACT_ID_REG);
	this->device_id = readl(this->base + DEVICE_ID_REG);
	this->version_id = readl(this->base + FLASH_VER_ID_REG);
	this->technology = readl(this->base + TECH_REG);

	/* Flash device information */
	onenand_print_device_info(this);

	/* Check OneNAND features */
	onenand_check_features(this);

	density = onenand_get_density(this->device_id);

	mtd->size = (16 << density) << 20;

	/* OneNAND page size & block size */
	/* The data buffer size is equal to page size */
	mtd->writesize = readl(this->base + ONENAND_REG_DATA_BUFFER_SIZE);

	/* We use the full BufferRAM */
	if (ONENAND_IS_4KB_PAGE(this))
		mtd->writesize <<= 1;

	mtd->oobsize = mtd->writesize >> 5;

	/* Pages per a block are always 64 in OneNAND */
	mtd->erasesize = mtd->writesize << 6;

	switch (mtd->oobsize) {
	case 128:
		mtd->ecclayout = &onenand_oob_128;
		break;
	case 64:
		mtd->ecclayout = &onenand_oob_64;
		break;
	case 32:
		mtd->ecclayout = &onenand_oob_32;
		break;
	default:
		pr_err("%s: No OOB scheme defined for oobsize %d\n",
			__func__, mtd->oobsize);
		return -EINVAL;
	}

	/*
	 * The number of bytes available for a client to place data into
	 * the out of band area
	 */
	mtd->ecclayout->oobavail = 0;
	for (i = 0; i < MTD_MAX_OOBFREE_ENTRIES &&
	    mtd->ecclayout->oobfree[i].length; i++)
		mtd->ecclayout->oobavail +=
			mtd->ecclayout->oobfree[i].length;
	mtd->oobavail = mtd->ecclayout->oobavail;
	mtd->ecc_strength = 1;

	/* Fill in remaining MTD driver data */
	mtd->type = ONENAND_IS_MLC(this) ? MTD_MLCNANDFLASH : MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->_erase = s3c_onenand_erase;
	mtd->_read = s3c_onenand_read;
	mtd->_write = s3c_onenand_write;
	mtd->_read_oob = s3c_onenand_read_oob;
	mtd->_write_oob = s3c_onenand_write_oob;
	mtd->_panic_write = s3c_onenand_panic_write;
	mtd->_sync = s3c_onenand_sync;
	mtd->_lock = s3c_onenand_lock;
	mtd->_unlock = s3c_onenand_unlock;
	mtd->_block_isbad = s3c_onenand_block_isbad;
	mtd->_block_markbad = s3c_onenand_block_markbad;
	mtd->owner = THIS_MODULE;
	mtd->writebufsize = mtd->writesize;

	return 0;
}

static const struct s3c_onenand_variant s3c6400_onenand_variant = {
	.cmd_map_shift	= 24,
	.fba_shift	= 10,
	.fpa_shift	= 4,
	.fsa_shift	= 2,
};

static const struct s3c_onenand_variant s3c6410_onenand_variant = {
	.cmd_map_shift	= 24,
	.fba_shift	= 12,
	.fpa_shift	= 6,
	.fsa_shift	= 4,
};

static const struct s3c_onenand_variant s5pc100_onenand_variant = {
	.cmd_map_shift	= 26,
	.fba_shift	= 13,
	.fpa_shift	= 7,
	.fsa_shift	= 5,
};

static struct of_device_id s3c_onenand_of_matches[] = {
	{
		.compatible	= "samsung,s3c6400-onenand",
		.data		= &s3c6400_onenand_variant,
	}, {
		.compatible	= "samsung,s3c6410-onenand",
		.data		= &s3c6410_onenand_variant,
	}, {
		.compatible	= "samsung,s5pc100-onenand",
		.data		= &s5pc100_onenand_variant,
	}, { },
};
MODULE_DEVICE_TABLE(of, s3c_onenand_of_matches);

static int s3c_onenand_probe(struct platform_device *pdev)
{
	struct onenand_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	struct s3c_onenand *onenand;
	struct mtd_info *mtd;
	struct resource *res;
	int ret;

	onenand = devm_kzalloc(&pdev->dev, sizeof(*onenand), GFP_KERNEL);
	if (!onenand) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	if (np) {
		const struct of_device_id *match;

		match = of_match_node(s3c_onenand_of_matches, np);
		if (!match)
			return -ENODEV;

		onenand->variant = match->data;
	} else {
		const struct platform_device_id *id;

		id = platform_get_device_id(pdev);
		onenand->variant = (void *)id->driver_data;
	}

	mtd = &onenand->mtd;
	mtd->priv = onenand;
	mtd->dev.parent = &pdev->dev;
	mtd->owner = THIS_MODULE;
	mtd->subpage_sft = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		return -ENOENT;
	}

	onenand->base = devm_request_and_ioremap(&pdev->dev, res);
	if (!onenand->base) {
		dev_err(&pdev->dev, "failed to map memory resource\n");
		return -EFAULT;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "no buffer memory resource defined\n");
		return -ENOENT;
	}

	onenand->ahb_addr = devm_request_and_ioremap(&pdev->dev, res);
	if (!onenand->ahb_addr) {
		dev_err(&pdev->dev, "failed to map buffer memory resource\n");
		return -EFAULT;
	}

	ret = s3c_onenand_setup(onenand);
	if (ret) {
		dev_err(&pdev->dev, "OneNAND initialization failed\n");
		return -EFAULT;
	}

	ret = mtd_device_parse_register(mtd, NULL, NULL,
		pdata ? pdata->parts : NULL, pdata ? pdata->nr_parts : 0);
	if (ret) {
		dev_err(&pdev->dev, "failed to register mtd device\n");
		return ret;
	}

	platform_set_drvdata(pdev, onenand);

	return 0;
}

static int s3c_onenand_remove(struct platform_device *pdev)
{
	struct s3c_onenand *onenand = platform_get_drvdata(pdev);

	return mtd_device_unregister(&onenand->mtd);
}

#ifdef CONFIG_PM_RUNTIME
static int s3c_onenand_runtime_suspend(struct device *dev)
{
	return 0;
}

static int s3c_onenand_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int s3c_onenand_suspend(struct device *dev)
{
	return 0;
}

static  int s3c_onenand_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops s3c_onenand_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s3c_onenand_suspend, s3c_onenand_resume)
	SET_RUNTIME_PM_OPS(s3c_onenand_runtime_suspend,
					s3c_onenand_runtime_resume, NULL)
};

static struct platform_device_id s3c_onenand_driver_ids[] = {
	{
		.name		= "s3c6400-onenand",
		.driver_data	= (kernel_ulong_t)&s3c6400_onenand_variant,
	}, {
		.name		= "s3c6410-onenand",
		.driver_data	= (kernel_ulong_t)&s3c6410_onenand_variant,
	}, {
		.name		= "s5pc100-onenand",
		.driver_data	= (kernel_ulong_t)&s5pc100_onenand_variant,
	}, { },
};
MODULE_DEVICE_TABLE(platform, s3c_onenand_driver_ids);

static struct platform_driver s3c_onenand_driver = {
	.probe          = s3c_onenand_probe,
	.remove         = s3c_onenand_remove,
	.id_table	= s3c_onenand_driver_ids,
	.driver         = {
		.name	= "s3c-onenand",
		.pm	= &s3c_onenand_pm_ops,
		.of_match_table	= of_match_ptr(s3c_onenand_of_matches),
	},
};
module_platform_driver(s3c_onenand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("Samsung S3C64xx/S5PC100 OneNAND controller driver");

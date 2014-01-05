/*
 * Copyright (c)	2013 Samsung Electronics Co., Ltd.
 * Author: Mateusz Krawczuk <m.krawczuk@partner.samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for Samsung S5PV210 clock controller.
*/

#ifndef _DT_BINDINGS_CLOCK_SAMSUNG_S5PV210_CLOCK_H
#define _DT_BINDINGS_CLOCK_SAMSUNG_S5PV210_CLOCK_H

/* Core clocks. */
#define FIN_PLL	1
#define FOUT_APLL	2
#define FOUT_MPLL	3
#define FOUT_EPLL	4
#define FOUT_VPLL	5

/* Muxes. */
#define MOUT_FLASH	6
#define MOUT_PSYS	7
#define MOUT_DSYS	8
#define MOUT_MSYS	9
#define MOUT_VPLL	10
#define MOUT_EPLL	11
#define MOUT_MPLL	12
#define MOUT_APLL	13
#define MOUT_VPLLSRC	14
#define MOUT_CSIS	15
#define MOUT_FIMD	16
#define MOUT_CAM1	17
#define MOUT_CAM0	18
#define MOUT_DAC	19
#define MOUT_MIXER	20
#define MOUT_HDMI	21
#define MOUT_G2D	22
#define MOUT_MFC	23
#define MOUT_G3D	24
#define MOUT_FIMC2	25
#define MOUT_FIMC1	26
#define MOUT_FIMC0	27
#define MOUT_UART3	28
#define MOUT_UART2	29
#define MOUT_UART1	30
#define MOUT_UART0	31
#define MOUT_MMC3	32
#define MOUT_MMC2	33
#define MOUT_MMC1	34
#define MOUT_MMC0	35
#define MOUT_PWM	36
#define MOUT_SPI0	37
#define MOUT_SPI1	38
#define MOUT_DMC0	39
#define MOUT_PWI	40
#define MOUT_HPM	41
#define MOUT_SPDIF	42
#define MOUT_AUDIO2	43
#define MOUT_AUDIO1	44
#define MOUT_AUDIO0	45

/* Dividers. */
#define DOUT_PCLKP	46
#define DOUT_HCLKP	47
#define DOUT_PCLKD	48
#define DOUT_HCLKD	49
#define DOUT_PCLKM	50
#define DOUT_HCLKM	51
#define DOUT_A2M	52
#define DOUT_APLL	53
#define DOUT_CSIS	54
#define DOUT_FIMD	55
#define DOUT_CAM1	56
#define DOUT_CAM0	57
#define DOUT_TBLK	58
#define DOUT_G2D	59
#define DOUT_MFC	60
#define DOUT_G3D	61
#define DOUT_FIMC2	62
#define DOUT_FIMC1	63
#define DOUT_FIMC0	64
#define DOUT_UART3	65
#define DOUT_UART2	66
#define DOUT_UART1	67
#define DOUT_UART0	68
#define DOUT_MMC3	69
#define DOUT_MMC2	70
#define DOUT_MMC1	71
#define DOUT_MMC0	72
#define DOUT_PWM	73
#define DOUT_SPI1	74
#define DOUT_SPI0	75
#define DOUT_DMC0	76
#define DOUT_PWI	77
#define DOUT_HPM	78
#define DOUT_COPY	79
#define DOUT_FLASH	80
#define DOUT_AUDIO2	81
#define DOUT_AUDIO1	82
#define DOUT_AUDIO0	83
#define DOUT_DPM	84
#define DOUT_DVSEM	85

/* Gates */
#define SCLK_FIMC	86
#define CSIS	87
#define ROTATOR	88
#define FIMC2	89
#define FIMC1	90
#define FIMC0	91
#define MFC	92
#define G2D	93
#define G3D	94
#define IMEM	95
#define PDMA1	96
#define PDMA0	97
#define MDMA	98
#define DMC1	99
#define DMC0	100
#define NFCON	101
#define SROMC	102
#define CFCON	103
#define NANDXL	104
#define USB_HOST	105
#define USB_OTG	106
#define HDMI	107
#define TVENC	108
#define MIXER	109
#define VP	110
#define DSIM	111
#define FIMD	112
#define TZIC3	113
#define TZIC2	114
#define TZIC1	115
#define TZIC0	116
#define VIC3	117
#define VIC2	118
#define VIC1	119
#define VIC0	120
#define TSI	121
#define HSMMC3	122
#define HSMMC2	123
#define HSMMC1	124
#define HSMMC0	125
#define JTAG	126
#define MODEMIF	127
#define CORESIGHT	128
#define SDM	129
#define SECSS	130
#define PCM2	131
#define PCM1	132
#define PCM0	133
#define SYSCON	134
#define GPIO	135
#define TSADC	136
#define PWM	137
#define WDT	138
#define KEYIF	139
#define UART3	140
#define UART2	141
#define UART1	142
#define UART0	143
#define SYSTIMER	144
#define RTC	145
#define SPI1	146
#define SPI0	147
#define I2C_HDMI_PHY	148
#define I2C_HDMI_CEC	149
#define I2C2	150
#define I2C0	151
#define I2S1	152
#define I2S2	153
#define I2S0	154
#define AC97	155
#define SPDIF	156
#define TZPC3	157
#define TZPC2	158
#define TZPC1	159
#define TZPC0	160
#define SECKEY	161
#define IEM_APC	162
#define IEM_IEC	163
#define CHIPID	164
#define JPEG	163

/* Special clocks*/
#define SCLK_PWI	164
#define SCLK_SPDIF	165
#define SCLK_AUDIO2	166
#define SCLK_AUDIO1	167
#define SCLK_AUDIO0	168
#define SCLK_PWM	169
#define SCLK_SPI1	170
#define SCLK_SPI0	171
#define SCLK_UART3	172
#define SCLK_UART2	173
#define SCLK_UART1	174
#define SCLK_UART0	175
#define SCLK_MMC3	176
#define SCLK_MMC2	177
#define SCLK_MMC1	178
#define SCLK_MMC0	179
#define SCLK_FINVPLL	180
#define SCLK_CSIS	181
#define SCLK_FIMD	182
#define SCLK_CAM1	183
#define SCLK_CAM0	184
#define SCLK_DAC	185
#define SCLK_MIXER	186
#define SCLK_HDMI	187
#define SCLK_FIMC2	188
#define SCLK_FIMC1	189
#define SCLK_FIMC0	190
#define SCLK_HDMI27M	191
#define SCLK_HDMIPHY	192
#define SCLK_USBPHY0	193
#define SCLK_USBPHY1	194

/* S5P6442-specific clocks */
#define MOUT_D0SYNC	195
#define MOUT_D1SYNC	196

#define DOUT_MIXER	197

#define ETB		198
#define ETM		199
#define I2C1		200

/* CLKOUT */
#define FOUT_APLL_CLKOUT	201
#define FOUT_MPLL_CLKOUT	202
#define DOUT_APLL_CLKOUT	203
#define MOUT_CLKSEL		204
#define DOUT_CLKOUT		205
#define MOUT_CLKOUT		206

/* Total number of clocks. */
#define NR_CLKS		207

#endif /* _DT_BINDINGS_CLOCK_SAMSUNG_S5PV210_CLOCK_H */

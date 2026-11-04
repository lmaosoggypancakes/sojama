/**
  * @file    adau1372.h
  * @brief   Driver for the Analog Devices ADAU1372 codec (I2C control port).
  *
  * Register map and power-up sequence follow the Linux ASoC driver
  * (sound/soc/codecs/adau1372.c).
  *
  * Signal paths set up by ADAU1372_Init():
  *   Capture:  AIN0-3 -> PGA0-3 -> ADC0-3 -> decimators -> output ASRC0-3
  *             -> serial output slots 0-3
  *   Playback: serial input slots 0+1 -> input ASRC0/1 -> DAC0/1
  *             -> headphone/line output stage
  */
#ifndef ADAU1372_H
#define ADAU1372_H

#include "stm32u5xx_hal.h"

/* 7-bit I2C address range, selected by the ADDR0/ADDR1 pins. */
#define ADAU1372_I2C_ADDR_BASE      0x3C

/* Register addresses (16-bit subaddress, 8-bit data). */
#define ADAU1372_REG_CLK_CTRL       0x00
#define ADAU1372_REG_PLL(x)         (0x01 + (x))
#define ADAU1372_REG_DAC_SOURCE     0x11
#define ADAU1372_REG_SOUT_SOURCE_0_1 0x13
#define ADAU1372_REG_SOUT_SOURCE_2_3 0x14
#define ADAU1372_REG_SOUT_SOURCE_4_5 0x15
#define ADAU1372_REG_SOUT_SOURCE_6_7 0x16
#define ADAU1372_REG_ADC_SDATA_CH   0x17
#define ADAU1372_REG_ASRCO_SOURCE_0_1 0x18
#define ADAU1372_REG_ASRCO_SOURCE_2_3 0x19
#define ADAU1372_REG_ASRC_MODE      0x1A
#define ADAU1372_REG_ADC_CTRL0      0x1B
#define ADAU1372_REG_ADC_CTRL1      0x1C
#define ADAU1372_REG_ADC_CTRL2      0x1D
#define ADAU1372_REG_ADC_CTRL3      0x1E
#define ADAU1372_REG_ADC_VOL(x)     (0x1F + (x))
#define ADAU1372_REG_PGA_CTRL(x)    (0x23 + (x))
#define ADAU1372_REG_PGA_BOOST      0x28
#define ADAU1372_REG_MICBIAS        0x2D
#define ADAU1372_REG_DAC_CTRL       0x2E
#define ADAU1372_REG_DAC_VOL(x)     (0x2F + (x))
#define ADAU1372_REG_OP_STAGE_MUTE  0x31
#define ADAU1372_REG_SAI0           0x32
#define ADAU1372_REG_SAI1           0x33
#define ADAU1372_REG_SOUT_CTRL      0x34
#define ADAU1372_REG_MODE_MP(x)     (0x38 + (x))
#define ADAU1372_REG_OP_STAGE_CTRL  0x43
#define ADAU1372_REG_DECIM_PWR      0x44
#define ADAU1372_REG_INTERP_PWR     0x45
#define ADAU1372_REG_BIAS_CTRL0     0x46
#define ADAU1372_REG_BIAS_CTRL1     0x47

/* CLK_CTRL bits */
#define ADAU1372_CLK_CTRL_PLL_EN    (1U << 7)
#define ADAU1372_CLK_CTRL_XTAL_DIS  (1U << 4)
#define ADAU1372_CLK_CTRL_CLKSRC    (1U << 3)
#define ADAU1372_CLK_CTRL_CC_MDIV   (1U << 1)
#define ADAU1372_CLK_CTRL_MCLK_EN   (1U << 0)

/* SAI0 fields */
#define ADAU1372_SAI0_DELAY1        (0x0U << 6)
#define ADAU1372_SAI0_DELAY0        (0x1U << 6)
#define ADAU1372_SAI0_DELAY_MASK    (0x3U << 6)
#define ADAU1372_SAI0_SAI_I2S       (0x0U << 4)
#define ADAU1372_SAI0_SAI_TDM2      (0x1U << 4)
#define ADAU1372_SAI0_SAI_TDM4      (0x2U << 4)
#define ADAU1372_SAI0_SAI_TDM8      (0x3U << 4)
#define ADAU1372_SAI0_SAI_MASK      (0x3U << 4)
#define ADAU1372_SAI0_FS_MASK       0x0FU

/* SAI1 bits */
#define ADAU1372_SAI1_TDM_TS        (1U << 7)
#define ADAU1372_SAI1_BCLK_TDMC     (1U << 6)
#define ADAU1372_SAI1_LR_MODE       (1U << 5)
#define ADAU1372_SAI1_LR_POL        (1U << 4)
#define ADAU1372_SAI1_BCLKRATE      (1U << 2)
#define ADAU1372_SAI1_BCLKEDGE      (1U << 1)
#define ADAU1372_SAI1_MS            (1U << 0)

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t addr;               /* 7-bit address, 0x3C-0x3F */
} ADAU1372_HandleTypeDef;

typedef struct
{
  uint32_t mclk_hz;           /* MCLK input: 12288000 or 24576000 */
  uint32_t sample_rate;       /* 8000, 12000, 16000, 24000, 32000, 48000, 96000, 192000 */
  uint8_t  tdm_slots;         /* 0 = I2S stereo, or 2, 4, 8 TDM slots */
  uint8_t  slot_bits;         /* 16, 24 or 32 */
  uint8_t  clock_master;      /* 1 = codec drives BCLK/LRCLK, 0 = STM32 drives them */
  uint8_t  lrclk_pulse;       /* 1 = 1-BCLK frame sync pulse (DSP/TDM), 0 = 50% duty LRCLK */
  uint8_t  lrclk_invert;      /* SAI_1 LR_POL. Pulse mode: 0 = short positive pulse, 1 = short negative pulse */
  uint8_t  headphone_out;     /* 1 = headphone drivers (16 ohm or more), 0 = line outputs */
} ADAU1372_ConfigTypeDef;

HAL_StatusTypeDef ADAU1372_WriteReg(ADAU1372_HandleTypeDef *h, uint16_t reg, uint8_t val);
HAL_StatusTypeDef ADAU1372_ReadReg(ADAU1372_HandleTypeDef *h, uint16_t reg, uint8_t *val);
HAL_StatusTypeDef ADAU1372_UpdateBits(ADAU1372_HandleTypeDef *h, uint16_t reg, uint8_t mask, uint8_t val);

/* Needs MCLK running on the codec's MCLKIN pin: most registers are not
 * accessible until the internal clock is enabled. */
HAL_StatusTypeDef ADAU1372_Init(ADAU1372_HandleTypeDef *h, const ADAU1372_ConfigTypeDef *cfg);

/* ch = 0..1. vol = attenuation in 0.375 dB steps, 0x00 = 0 dB. */
HAL_StatusTypeDef ADAU1372_SetDacVolume(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t vol);
HAL_StatusTypeDef ADAU1372_SetDacMute(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t mute);

/* ch = 0..3. vol = attenuation in 0.375 dB steps, 0x00 = 0 dB. */
HAL_StatusTypeDef ADAU1372_SetAdcVolume(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t vol);
HAL_StatusTypeDef ADAU1372_SetAdcMute(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t mute);

/* ch = 0..3. gain = 0..0x3F, 0.75 dB steps: 0x00 = -12 dB, 0x10 = 0 dB, 0x3F = +35.25 dB. */
HAL_StatusTypeDef ADAU1372_SetPgaGain(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t gain);
HAL_StatusTypeDef ADAU1372_SetPgaBoost(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t on);

/* idx = 0..1 (MICBIAS0/MICBIAS1). Needed for electret mics. */
HAL_StatusTypeDef ADAU1372_SetMicBias(ADAU1372_HandleTypeDef *h, uint8_t idx, uint8_t on);

#endif /* ADAU1372_H */

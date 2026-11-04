/**
  * @file    adau1372.c
  * @brief   Driver for the Analog Devices ADAU1372 codec (I2C control port).
  */
#include "adau1372.h"

#define ADAU1372_I2C_TIMEOUT_MS 100

/* Readable from the debugger: the last step ADAU1372_Init() started, and the
 * value it read back. Step is 0 once init has succeeded. For an I2C failure,
 * also look at hi2c->ErrorCode.
 *   1 device ACK, 2 clock enable, 3 pin setup, 4 serial port,
 *   5 capture path, 6 playback path, 8 read-back check */
volatile uint8_t adau1372_init_step;
volatile uint8_t adau1372_init_readback;
volatile uint8_t adau1372_init_clk_ctrl;   /* expect 0x03 at 12.288 MHz */
volatile uint8_t adau1372_regs[0x48];      /* full register dump at step 8 */
volatile uint8_t adau1372_pll_lock;         /* 1 if the PLL locked to MCLK during the test */

/* Index = SAI0 FS field value. */
static const uint32_t adau1372_rates[] = {
  48000, 8000, 12000, 16000, 24000, 32000, 96000, 192000
};

HAL_StatusTypeDef ADAU1372_WriteReg(ADAU1372_HandleTypeDef *h, uint16_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(h->hi2c, h->addr << 1, reg, I2C_MEMADD_SIZE_16BIT,
                           &val, 1, ADAU1372_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef ADAU1372_ReadReg(ADAU1372_HandleTypeDef *h, uint16_t reg, uint8_t *val)
{
  return HAL_I2C_Mem_Read(h->hi2c, h->addr << 1, reg, I2C_MEMADD_SIZE_16BIT,
                          val, 1, ADAU1372_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef ADAU1372_UpdateBits(ADAU1372_HandleTypeDef *h, uint16_t reg, uint8_t mask, uint8_t val)
{
  uint8_t cur;
  HAL_StatusTypeDef st = ADAU1372_ReadReg(h, reg, &cur);
  if (st != HAL_OK)
  {
    return st;
  }
  return ADAU1372_WriteReg(h, reg, (cur & ~mask) | (val & mask));
}

static HAL_StatusTypeDef adau1372_config_serial_port(ADAU1372_HandleTypeDef *h,
                                                     const ADAU1372_ConfigTypeDef *cfg)
{
  uint8_t sai0 = ADAU1372_SAI0_DELAY1;
  uint8_t sai1 = 0;
  uint8_t fs;
  HAL_StatusTypeDef st;

  for (fs = 0; fs < sizeof(adau1372_rates) / sizeof(adau1372_rates[0]); fs++)
  {
    if (adau1372_rates[fs] == cfg->sample_rate)
    {
      break;
    }
  }
  if (fs == sizeof(adau1372_rates) / sizeof(adau1372_rates[0]))
  {
    return HAL_ERROR;
  }
  sai0 |= fs;

  switch (cfg->tdm_slots)
  {
    case 0: sai0 |= ADAU1372_SAI0_SAI_I2S;  break;
    case 2: sai0 |= ADAU1372_SAI0_SAI_TDM2; break;
    case 4: sai0 |= ADAU1372_SAI0_SAI_TDM4; break;
    case 8: sai0 |= ADAU1372_SAI0_SAI_TDM8; break;
    default: return HAL_ERROR;
  }

  switch (cfg->slot_bits)
  {
    case 16:
      sai1 |= ADAU1372_SAI1_BCLKRATE;
      if (cfg->tdm_slots != 0)
      {
        sai1 |= ADAU1372_SAI1_BCLK_TDMC;
      }
      break;
    case 24:
    case 32:
      break;
    default:
      return HAL_ERROR;
  }

  if (cfg->clock_master)
  {
    sai1 |= ADAU1372_SAI1_MS;
  }
  if (cfg->lrclk_pulse)
  {
    sai1 |= ADAU1372_SAI1_LR_MODE;
  }
  if (cfg->lrclk_invert)
  {
    sai1 |= ADAU1372_SAI1_LR_POL;
  }

  st = ADAU1372_WriteReg(h, ADAU1372_REG_SAI0, sai0);
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_SAI1, sai1);
  if (st != HAL_OK) return st;

  /* TDM only: disable the output slots beyond the ones in use. */
  if (cfg->tdm_slots != 0)
  {
    uint8_t tx_mask = (uint8_t)((1U << cfg->tdm_slots) - 1U);
    st = ADAU1372_WriteReg(h, ADAU1372_REG_SOUT_CTRL, (uint8_t)~tx_mask);
  }
  return st;
}

HAL_StatusTypeDef ADAU1372_Init(ADAU1372_HandleTypeDef *h, const ADAU1372_ConfigTypeDef *cfg)
{
  HAL_StatusTypeDef st;
  uint8_t clk_ctrl;
  uint8_t readback;

  /* The chip leaves its internal reset 12 ms after power-up. */
  HAL_Delay(15);

  adau1372_init_step = 1;
  if (HAL_I2C_IsDeviceReady(h->hi2c, h->addr << 1, 3, ADAU1372_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* The internal master clock must be 12.288 MHz: divide a 12.288 MHz MCLK by 1
   * and a 24.576 MHz MCLK by 2. Other rates need the PLL, which is not set up here. */
  if (cfg->mclk_hz == 12288000U)
  {
    clk_ctrl = ADAU1372_CLK_CTRL_CC_MDIV;
  }
  else if (cfg->mclk_hz == 24576000U)
  {
    clk_ctrl = 0;
  }
  else
  {
    return HAL_ERROR;
  }

  adau1372_init_step = 2;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_CLK_CTRL, clk_ctrl);
  if (st != HAL_OK) return st;

  /* Diagnostic: the PLL only locks if MCLK reaches the chip and its core runs.
   * Integer mode, R = 4 (PLL_CTRL4 = 0x20). The PLL is turned off afterwards. */
  st = ADAU1372_WriteReg(h, ADAU1372_REG_PLL(4), 0x20);
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_CLK_CTRL, clk_ctrl | ADAU1372_CLK_CTRL_PLL_EN);
  if (st != HAL_OK) return st;
  adau1372_pll_lock = 0;
  for (uint8_t i = 0; i < 10 && adau1372_pll_lock == 0; i++)
  {
    HAL_Delay(2);
    st = ADAU1372_ReadReg(h, ADAU1372_REG_PLL(5), &readback);
    if (st != HAL_OK) return st;
    adau1372_pll_lock = readback & 0x01;
  }
  st = ADAU1372_WriteReg(h, ADAU1372_REG_CLK_CTRL, clk_ctrl);
  if (st != HAL_OK) return st;

  st = ADAU1372_WriteReg(h, ADAU1372_REG_CLK_CTRL, clk_ctrl | ADAU1372_CLK_CTRL_MCLK_EN);
  if (st != HAL_OK) return st;
  HAL_Delay(2);

  adau1372_init_step = 3;
  /* Pin ADC_SDATA0/MP1 as Serial Output 0. */
  st = ADAU1372_WriteReg(h, ADAU1372_REG_MODE_MP(1), 0x00);
  if (st != HAL_OK) return st;

  adau1372_init_step = 4;
  st = adau1372_config_serial_port(h, cfg);
  if (st != HAL_OK) return st;

  adau1372_init_step = 5;
  /* Capture: ADCn -> output ASRCn -> serial output slot n. */
  st = ADAU1372_WriteReg(h, ADAU1372_REG_DECIM_PWR, 0xFF);   /* ASRC decimators + ADC filters */
  if (st != HAL_OK) return st;
  st = ADAU1372_UpdateBits(h, ADAU1372_REG_ADC_CTRL2, 0x03, 0x03);  /* ADC0, ADC1 */
  if (st != HAL_OK) return st;
  st = ADAU1372_UpdateBits(h, ADAU1372_REG_ADC_CTRL3, 0x03, 0x03);  /* ADC2, ADC3 */
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_ASRC_MODE, 0x02);   /* output ASRC on */
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_ASRCO_SOURCE_0_1, 0x54);
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_ASRCO_SOURCE_2_3, 0x76);
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_SOUT_SOURCE_0_1, 0x54);
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_SOUT_SOURCE_2_3, 0x76);
  if (st != HAL_OK) return st;
  for (uint8_t ch = 0; ch < 4; ch++)
  {
    /* Bit 7 = PGA_EN, bit 6 = PGA_MUTE. PGA_GAIN keeps its reset value (-12 dB). */
    st = ADAU1372_UpdateBits(h, ADAU1372_REG_PGA_CTRL(ch), 0xC0, 0x80);
    if (st != HAL_OK) return st;
    st = ADAU1372_SetAdcMute(h, ch, 0);
    if (st != HAL_OK) return st;
  }

  adau1372_init_step = 6;
  /* Playback: serial input slots 0+1 -> input ASRC0/1 -> DAC0/1 -> outputs. */
  st = ADAU1372_WriteReg(h, ADAU1372_REG_INTERP_PWR, 0x0F);  /* ASRC interpolators + DAC modulators */
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_ASRC_MODE, 0x03);   /* input ASRC on, slots 0+1 */
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_DAC_SOURCE, 0xDC);
  if (st != HAL_OK) return st;
  /* Bits 0-1 = DAC0/1 enable, bits 3-4 = DAC0/1 mute. Enable while still muted. */
  st = ADAU1372_UpdateBits(h, ADAU1372_REG_DAC_CTRL, 0x03, 0x03);
  if (st != HAL_OK) return st;
  /* Bits 4-5 = headphone mode, bits 0-3 = output stages powered down. */
  st = ADAU1372_WriteReg(h, ADAU1372_REG_OP_STAGE_CTRL, cfg->headphone_out ? 0x30 : 0x00);
  if (st != HAL_OK) return st;
  if (cfg->headphone_out)
  {
    HAL_Delay(7);   /* at least 6 ms before unmuting headphones */
  }
  st = ADAU1372_UpdateBits(h, ADAU1372_REG_DAC_CTRL, 0x18, 0x00);
  if (st != HAL_OK) return st;
  st = ADAU1372_WriteReg(h, ADAU1372_REG_OP_STAGE_MUTE, 0x00);
  if (st != HAL_OK) return st;

  adau1372_init_step = 8;
  for (uint16_t reg = 0; reg < sizeof(adau1372_regs); reg++)
  {
    /* Unmapped addresses may NACK, so a failed read here is not an error. */
    if (ADAU1372_ReadReg(h, reg, &readback) == HAL_OK)
    {
      adau1372_regs[reg] = readback;
    }
  }

  st = ADAU1372_ReadReg(h, ADAU1372_REG_CLK_CTRL, &readback);
  if (st != HAL_OK) return st;
  adau1372_init_clk_ctrl = readback;

  /* Check that writes past CLK_CTRL took effect (they are ignored without MCLK). */
  st = ADAU1372_ReadReg(h, ADAU1372_REG_ASRC_MODE, &readback);
  if (st != HAL_OK) return st;
  adau1372_init_readback = readback;
  if (readback != 0x03)
  {
    return HAL_ERROR;
  }

  adau1372_init_step = 0;
  return HAL_OK;
}

HAL_StatusTypeDef ADAU1372_SetDacVolume(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t vol)
{
  if (ch > 1) return HAL_ERROR;
  return ADAU1372_WriteReg(h, ADAU1372_REG_DAC_VOL(ch), vol);
}

HAL_StatusTypeDef ADAU1372_SetDacMute(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t mute)
{
  uint8_t bit;
  if (ch > 1) return HAL_ERROR;
  bit = (uint8_t)(1U << (3 + ch));
  return ADAU1372_UpdateBits(h, ADAU1372_REG_DAC_CTRL, bit, mute ? bit : 0);
}

HAL_StatusTypeDef ADAU1372_SetAdcVolume(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t vol)
{
  if (ch > 3) return HAL_ERROR;
  return ADAU1372_WriteReg(h, ADAU1372_REG_ADC_VOL(ch), vol);
}

HAL_StatusTypeDef ADAU1372_SetAdcMute(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t mute)
{
  /* ADC0/1 mutes are in ADC_CTRL0 bits 3/4, ADC2/3 in ADC_CTRL1 bits 3/4. */
  uint16_t reg;
  uint8_t bit;
  if (ch > 3) return HAL_ERROR;
  reg = (ch < 2) ? ADAU1372_REG_ADC_CTRL0 : ADAU1372_REG_ADC_CTRL1;
  bit = (uint8_t)(1U << (3 + (ch & 1)));
  return ADAU1372_UpdateBits(h, reg, bit, mute ? bit : 0);
}

HAL_StatusTypeDef ADAU1372_SetPgaGain(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t gain)
{
  if (ch > 3 || gain > 0x3F) return HAL_ERROR;
  return ADAU1372_UpdateBits(h, ADAU1372_REG_PGA_CTRL(ch), 0x3F, gain);
}

HAL_StatusTypeDef ADAU1372_SetPgaBoost(ADAU1372_HandleTypeDef *h, uint8_t ch, uint8_t on)
{
  uint8_t bit;
  if (ch > 3) return HAL_ERROR;
  bit = (uint8_t)(1U << ch);
  return ADAU1372_UpdateBits(h, ADAU1372_REG_PGA_BOOST, bit, on ? bit : 0);
}

HAL_StatusTypeDef ADAU1372_SetMicBias(ADAU1372_HandleTypeDef *h, uint8_t idx, uint8_t on)
{
  uint8_t bit;
  if (idx > 1) return HAL_ERROR;
  bit = (uint8_t)(1U << (4 + idx));
  return ADAU1372_UpdateBits(h, ADAU1372_REG_MICBIAS, bit, on ? bit : 0);
}

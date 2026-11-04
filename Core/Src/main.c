/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adau1372.h"
#include "audio_clip.h"
#include "audio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;

/* USER CODE BEGIN PV */
ADAU1372_HandleTypeDef codec = { .hi2c = &hi2c2, .addr = ADAU1372_I2C_ADDR_BASE };

/* Matches SAI1: 48 kHz, 4 x 32-bit TDM slots, STM32 drives BCLK and a
 * 1-bit frame sync pulse, MCLK from SAI1_MCLK_A (PE2). */
static const ADAU1372_ConfigTypeDef codec_cfg = {
  .mclk_hz      = 12288000,
  .sample_rate  = 48000,
  .tdm_slots    = 4,
  .slot_bits    = 32,
  .clock_master = 0,
  .lrclk_pulse  = 1,
  .lrclk_invert = 0,   /* datasheet Table 20 says 1 for TDM; try 1 if there is no sound */
  .headphone_out = 1,
};


/* What plays when not recording or playing back, changeable from the debugger:
 * 0 = song clip, 1 = mic passthrough, 2 = silence. */
volatile uint8_t audio_mode = 2;
/* Mic level on input 0, updated every 100 ms. Watch mic_meter.peak_pct while talking. */
volatile Audio_Meter mic_meter;
volatile uint32_t audio_frames;
static Audio_Clip clip = { audio_clip, AUDIO_CLIP_LEN, 0, 0 };

/* Record / play back. Record: B1 user button (PC13, high when pressed).
 * Play: button from PD0 to GND (internal pull-up). Red LD3 (PG2) flashes while
 * recording, green LD1 (PC7) is on during playback. */
#define REC_SECONDS     10U
#define REC_RATE        24000U
#define REC_LEN         (REC_SECONDS * REC_RATE)
#define PLAY_BTN_PORT   GPIOD
#define PLAY_BTN_PIN    GPIO_PIN_0
#define LED_FLASH_FRAMES 6000U   /* 125 ms at 48 kHz */
typedef enum { APP_IDLE, APP_RECORDING, APP_PLAYING } App_State;
volatile App_State app_state = APP_IDLE;
static int16_t rec_buf[REC_LEN];
static Audio_Recorder recorder = { rec_buf, REC_LEN, 0, 0, 0 };
static Audio_Clip rec_clip;
static Audio_Button rec_button, play_button;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SAI1_Init(void);
static void MX_ICACHE_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ADAU1372 7-bit I2C address is 0x3C-0x3F, set by the ADDR0/ADDR1 pins. */
#define ADAU1372_ADDR_FIRST 0x3C
#define ADAU1372_ADDR_LAST  0x3F

/* Results of the I2C check, readable from the debugger. */
volatile uint8_t i2c_found_addrs[128];
volatile uint8_t i2c_found_count;
volatile uint8_t adau1372_addr;      /* 0 if not found */
volatile uint8_t adau1372_reg0;      /* register 0x0000 (CLK_CONTROL) */
volatile HAL_StatusTypeDef adau1372_read_status = HAL_ERROR;

/* I2C connection test for the ADAU1372. Runs on the default 4 MHz MSI
 * clock, so it does not need the external 12.288 MHz HSE.
 * Result is shown on the Nucleo LEDs and never returns:
 *   green LD1 (PC7) slow blink  -> ADAU1372 ACKed and register read worked
 *   blue  LD2 (PB7) slow blink  -> some device ACKed, but not at 0x3C-0x3F
 *   blue  LD2 (PB7) fast blink  -> nothing on the bus responded */
__attribute__((unused)) static void I2C_ConnectionTest(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint16_t led_pin;
  GPIO_TypeDef *led_port;
  uint32_t period;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Same settings as MX_I2C2_Init(), but with a timing value for
   * ~100 kHz from a 4 MHz kernel clock (PRESC=0, SCLL=0x13, SCLH=0x0F). */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00420F13;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable the internal pull-ups on SDA/SCL in case the breakout has none.
   * They are weak (~40k) but fine at 100 kHz with short wires. */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* Scan the whole bus. */
  i2c_found_count = 0;
  adau1372_addr = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 2, 10) == HAL_OK)
    {
      i2c_found_addrs[i2c_found_count++] = addr;
      if (addr >= ADAU1372_ADDR_FIRST && addr <= ADAU1372_ADDR_LAST && adau1372_addr == 0)
      {
        adau1372_addr = addr;
      }
    }
  }

  /* ADAU1372 registers use a 16-bit subaddress. */
  if (adau1372_addr != 0)
  {
    uint8_t val = 0;
    adau1372_read_status = HAL_I2C_Mem_Read(&hi2c2, adau1372_addr << 1, 0x0000,
                                            I2C_MEMADD_SIZE_16BIT, &val, 1, 100);
    adau1372_reg0 = val;
  }

  if (adau1372_addr != 0 && adau1372_read_status == HAL_OK)
  {
    led_port = GPIOC; led_pin = GPIO_PIN_7; period = 500;
  }
  else if (i2c_found_count > 0)
  {
    led_port = GPIOB; led_pin = GPIO_PIN_7; period = 500;
  }
  else
  {
    led_port = GPIOB; led_pin = GPIO_PIN_7; period = 100;
  }

  while (1)
  {
    HAL_GPIO_TogglePin(led_port, led_pin);
    HAL_Delay(period);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SAI1_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  /* The STM32 generates the codec's MCLK, so run SAI1 before touching it. */
  __HAL_SAI_ENABLE(&hsai_BlockA1);

  /* Bus scan: results in i2c_found_addrs / i2c_found_count / adau1372_addr. */
  i2c_found_count = 0;
  adau1372_addr = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 2, 10) == HAL_OK)
    {
      i2c_found_addrs[i2c_found_count++] = addr;
      if (addr >= ADAU1372_ADDR_FIRST && addr <= ADAU1372_ADDR_LAST && adau1372_addr == 0)
      {
        adau1372_addr = addr;
      }
    }
  }

  if (ADAU1372_Init(&codec, &codec_cfg) != HAL_OK)
  {
    Error_Handler();
  }


  /* Start full-duplex audio. Block A (receive, slot 0 only) is already running to
   * supply MCLK, so drop whatever piled up in its FIFO during init. */
  while ((SAI1_Block_A->SR & SAI_xSR_FLVL) != SAI_FIFOSTATUS_EMPTY)
  {
    (void)SAI1_Block_A->DR;
  }
  __HAL_SAI_CLEAR_FLAG(&hsai_BlockA1, SAI_FLAG_OVRUDR);
  /* Block B (transmit, slots 0 and 1) starts on Block A's next frame sync. */
  for (uint32_t i = 0; i < 4; i++)
  {
    SAI1_Block_B->DR = 0;
  }
  __HAL_SAI_ENABLE(&hsai_BlockB1);
  uint8_t last_vol = 0xFF;
  uint32_t play_left = 0;
  uint32_t led_count = 0;

  /* Buttons and LEDs. */
  {
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = PLAY_BTN_PIN;
    HAL_GPIO_Init(PLAY_BTN_PORT, &gpio);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOG, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* DAC volume: -12 dB for mic passthrough to keep feedback down, 0 dB otherwise.
     * Each step above 0x00 is 0.375 dB quieter. Changing it takes ~1 ms of I2C,
     * so it only happens when the setting changes. */
    uint8_t vol = (app_state == APP_IDLE && audio_mode == 1) ? 0x20 : 0x00;
    if (vol != last_vol)
    {
      last_vol = vol;
      ADAU1372_SetDacVolume(&codec, 0, vol);
      ADAU1372_SetDacVolume(&codec, 1, vol);
    }

    if (Audio_ButtonPressed(&rec_button, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET))
    {
      Audio_RecStart(&recorder);
      app_state = APP_RECORDING;
      led_count = 0;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    if (Audio_ButtonPressed(&play_button, HAL_GPIO_ReadPin(PLAY_BTN_PORT, PLAY_BTN_PIN) == GPIO_PIN_RESET)
        && app_state != APP_RECORDING && recorder.len > 0)
    {
      rec_clip = (Audio_Clip){ rec_buf, recorder.len, 0, 0 };
      play_left = recorder.len * 2U;
      app_state = APP_PLAYING;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    }

    /* One mic sample arrives per 48 kHz frame; waiting for it paces the loop. */
    while ((SAI1_Block_A->SR & SAI_xSR_FLVL) == SAI_FIFOSTATUS_EMPTY)
    {
    }
    int32_t mic = Audio_FromSai24(SAI1_Block_A->DR);
    Audio_MeterAdd((Audio_Meter *)&mic_meter, mic);

    int32_t out = 0;
    if (app_state == APP_RECORDING)
    {
      if (++led_count == LED_FLASH_FRAMES)
      {
        led_count = 0;
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
      }
      if (Audio_RecAdd(&recorder, mic))
      {
        app_state = APP_IDLE;
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
      }
    }
    else if (app_state == APP_PLAYING)
    {
      out = Audio_ClipNext(&rec_clip);
      if (--play_left == 0U)
      {
        app_state = APP_IDLE;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
      }
    }
    else if (audio_mode == 0)
    {
      out = Audio_ClipNext(&clip);
    }
    else if (audio_mode == 1)
    {
      out = mic;
    }
    uint32_t word = Audio_ToSai24(out);

    /* Two words per frame (left, right); wait until the FIFO has room for both. */
    while ((SAI1_Block_B->SR & SAI_xSR_FLVL) >= SAI_FIFOSTATUS_3QUARTERFULL)
    {
    }
    SAI1_Block_B->DR = word;
    SAI1_Block_B->DR = word;
    audio_frames++;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInit.PLL2.PLL2Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLL2.PLL2M = 1;
  PeriphClkInit.PLL2.PLL2N = 30;
  PeriphClkInit.PLL2.PLL2P = 5;
  PeriphClkInit.PLL2.PLL2Q = 2;
  PeriphClkInit.PLL2.PLL2R = 2;
  PeriphClkInit.PLL2.PLL2RGE = RCC_PLLVCIRANGE_1;
  PeriphClkInit.PLL2.PLL2FRACN = 5898;
  PeriphClkInit.PLL2.PLL2ClockOut = RCC_PLL2_DIVP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x5060689D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_24;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MckOutput = SAI_MCK_OUTPUT_ENABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength = 128;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockA1.SlotInit.SlotNumber = 4;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000001;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_TX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_24;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MckOutput = SAI_MCK_OUTPUT_ENABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB1.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockB1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB1.FrameInit.FrameLength = 128;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockB1.SlotInit.SlotNumber = 4;
  hsai_BlockB1.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

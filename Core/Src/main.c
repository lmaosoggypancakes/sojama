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
#include "uart_comm.h"
#include <string.h>

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
 * 0 = song clip, 1 = mic passthrough (mic 1 left, mic 2 right), 2 = silence,
 * 3 = test tone on both speakers (TONE_HZ at TONE_LEVEL). */
volatile uint8_t audio_mode = 2;
#define TONE_HZ    1000.0f
#define TONE_LEVEL 0.25f     /* peak, fraction of full scale: -12 dBFS */
static Audio_Tone tone;
/* Mic levels, updated every 100 ms: [0] = AIN0 (TDM slot 0), [1] = AIN1 (slot 1).
 * Watch mic_meter[n].peak_pct while talking. */
volatile Audio_Meter mic_meter[2];
volatile uint32_t audio_frames;
/* Audio runs on SAI1 DMA: each DMA half-buffer is AUDIO_BLOCK_FRAMES frames (1 ms),
 * processed in the DMA interrupt by Audio_ProcessBlock(). Diagnostics:
 * audio_blocks: blocks processed. audio_isr_max_cycles: longest block processing time
 * (DWT cycles, 160 = 1 us; a block is 1 ms = 160000). audio_tx_underruns: times the
 * speaker FIFO ran empty (each one is a click). audio_sai_errors: SAI/DMA errors. */
#define AUDIO_BLOCK_FRAMES 48U
#define AUDIO_BUF_WORDS    (2U * AUDIO_BLOCK_FRAMES * 2U)   /* 2 halves x frames x 2 slots */
volatile uint32_t audio_blocks;
volatile uint32_t audio_isr_max_cycles;
volatile uint32_t audio_tx_underruns;
volatile uint32_t audio_sai_errors;
static uint32_t audio_rx_buf[AUDIO_BUF_WORDS];
static uint32_t audio_tx_buf[AUDIO_BUF_WORDS];
static DMA_HandleTypeDef hdma_sai1_a;
static DMA_HandleTypeDef hdma_sai1_b;
static DMA_NodeTypeDef dma_node_sai1_a;
static DMA_NodeTypeDef dma_node_sai1_b;
static DMA_QListTypeDef dma_queue_sai1_a;
static DMA_QListTypeDef dma_queue_sai1_b;
/* Data streamed to the PC over UART, changeable from the debugger. Either stream
 * turns off the UART test counter.
 * 1 = mic levels (Tests/mic_plot.py): every 50 ms "MIC,<peak1>,<peak2>\r\n", each peak
 *     the largest |sample - dc| of that mic in the period, 0..8388608 (full scale).
 * 2 = spectrum (Tests/fft_plot.py): "FFT,<mic 0|1>,<v0>,...,<v255>\r\n", bin k at
 *     k * 93.75 Hz, each value the bin level in -dBFS (0..140). Alternates mics. */
volatile uint8_t stream_mode = 2;
#define MIC_STREAM_FRAMES 2400U   /* 50 ms at 48 kHz */
/* Written by the audio interrupt, sent by the main loop. */
static uint32_t stream_peak[2];
static uint32_t stream_frames;
static uint32_t mic_report[2];
static volatile uint8_t mic_report_ready;

/* Spectrum stream: the audio interrupt collects SPEC_N samples of mic spec_ch while
 * spec_collecting is set; the main loop then runs the FFT, formats and sends it. */
static Audio_Spectrum spectrum;
static volatile uint8_t spec_collecting;
static volatile uint8_t spec_ch;
static uint8_t spec_line[UART_TX_BUF_SIZE];
static size_t spec_len;
static Audio_Clip clip = { audio_clip, AUDIO_CLIP_LEN, 0, 0, 2 };

/* Record / play back both mics. Record: button from PC8 to GND, or the B1 user button
 * (PC13, high when pressed). Play: button from PC9 to GND. PC8/PC9 use internal pull-ups. Red LD3 (PG2) flashes while
 * recording, green LD1 (PC7) is on during playback. Stored at 16 kHz (1/3 of 48 kHz)
 * so 10 s of both mics fits in RAM. */
#define REC_SECONDS     10U
#define REC_RATIO       3U
#define REC_LEN         (REC_SECONDS * 48000U / REC_RATIO)
#define REC_BTN_PORT    GPIOC
#define REC_BTN_PIN     GPIO_PIN_8
#define PLAY_BTN_PORT   GPIOC
#define PLAY_BTN_PIN    GPIO_PIN_9
#define LED_FLASH_FRAMES 6000U   /* 125 ms at 48 kHz */
typedef enum { APP_IDLE, APP_RECORDING, APP_PLAYING } App_State;
volatile App_State app_state = APP_IDLE;
static int16_t rec_buf[2][REC_LEN];
static Audio_Recorder recorder[2] = {
  { rec_buf[0], REC_LEN, 0, 0, 0, REC_RATIO },
  { rec_buf[1], REC_LEN, 0, 0, 0, REC_RATIO },
};
static Audio_Clip rec_clip[2];
/* Normal recording (0): mic 1 -> speaker 1 (left, HPOUTL), mic 2 -> speaker 2
 * (right, HPOUTR), 10 s each. Test mode (1): one track, mic 1 for the first half and
 * mic 2 for the second, played on both speakers; red LD3 flashes during the mic 1
 * half, blue LD2 (PB7) during the mic 2 half. */
volatile uint8_t rec_split_test = 0;
#define REC_SPLIT_FRAMES (REC_SECONDS * 48000U / 2U)
static Audio_Button rec_button, play_button;
/* Record / playback state, used only by the audio interrupt. */
static uint32_t play_left;
static uint32_t led_count;
static uint32_t rec_frames;
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

/* Stop both SAI1 blocks at the end of the current frame and empty their FIFOs. */
static void Audio_StopSai(void)
{
  __HAL_SAI_DISABLE(&hsai_BlockB1);
  __HAL_SAI_DISABLE(&hsai_BlockA1);
  uint32_t start = HAL_GetTick();
  while (((SAI1_Block_A->CR1 | SAI1_Block_B->CR1) & SAI_xCR1_SAIEN) && (HAL_GetTick() - start < 2U))
  {
  }
  SAI1_Block_A->CR2 |= SAI_xCR2_FFLUSH;
  SAI1_Block_B->CR2 |= SAI_xCR2_FFLUSH;
  __HAL_SAI_CLEAR_FLAG(&hsai_BlockA1, SAI_FLAG_OVRUDR);
  __HAL_SAI_CLEAR_FLAG(&hsai_BlockB1, SAI_FLAG_OVRUDR);
}

/* Circular GPDMA channel with one linked-list node covering the whole buffer.
 * The SAI HAL fills in the node's addresses and size when the transfer starts. */
static HAL_StatusTypeDef Audio_DmaSetup(DMA_HandleTypeDef *hdma, DMA_Channel_TypeDef *channel,
                                        DMA_NodeTypeDef *node, DMA_QListTypeDef *queue,
                                        uint32_t request, uint32_t direction)
{
  DMA_NodeConfTypeDef conf = {0};
  uint8_t to_periph = (direction == DMA_MEMORY_TO_PERIPH);

  conf.NodeType = DMA_GPDMA_LINEAR_NODE;
  conf.Init.Request = request;
  conf.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  conf.Init.Direction = direction;
  conf.Init.SrcInc = to_periph ? DMA_SINC_INCREMENTED : DMA_SINC_FIXED;
  conf.Init.DestInc = to_periph ? DMA_DINC_FIXED : DMA_DINC_INCREMENTED;
  conf.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  conf.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  conf.Init.SrcBurstLength = 1;
  conf.Init.DestBurstLength = 1;
  conf.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  conf.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  conf.Init.Mode = DMA_NORMAL;
  conf.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  conf.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  conf.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  if (HAL_DMAEx_List_BuildNode(&conf, node) != HAL_OK
      || HAL_DMAEx_List_InsertNode(queue, NULL, node) != HAL_OK
      || HAL_DMAEx_List_SetCircularMode(queue) != HAL_OK)
  {
    return HAL_ERROR;
  }

  hdma->Instance = channel;
  hdma->InitLinkedList.Priority = DMA_HIGH_PRIORITY;
  hdma->InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  hdma->InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  hdma->InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  hdma->InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(hdma) != HAL_OK
      || HAL_DMAEx_List_LinkQ(hdma, queue) != HAL_OK
      || HAL_DMA_ConfigChannelAttributes(hdma, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/* Start full-duplex audio on SAI1 DMA: Block A receives slots 0 and 1 (the mics),
 * Block B sends slots 0 and 1 (the speakers). The synchronous slave (B) starts before
 * the master (A), so the first word of each buffer is TDM slot 0. */
static HAL_StatusTypeDef Audio_Start(void)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();
  if (Audio_DmaSetup(&hdma_sai1_a, GPDMA1_Channel1, &dma_node_sai1_a, &dma_queue_sai1_a,
                     GPDMA1_REQUEST_SAI1_A, DMA_PERIPH_TO_MEMORY) != HAL_OK
      || Audio_DmaSetup(&hdma_sai1_b, GPDMA1_Channel2, &dma_node_sai1_b, &dma_queue_sai1_b,
                        GPDMA1_REQUEST_SAI1_B, DMA_MEMORY_TO_PERIPH) != HAL_OK)
  {
    return HAL_ERROR;
  }
  __HAL_LINKDMA(&hsai_BlockA1, hdmarx, hdma_sai1_a);
  __HAL_LINKDMA(&hsai_BlockB1, hdmatx, hdma_sai1_b);

  /* Above the UART (14) so audio is never held up by it. */
  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);

  /* Block A was running only to supply MCLK during codec setup. */
  Audio_StopSai();
  if (HAL_SAI_Transmit_DMA(&hsai_BlockB1, (uint8_t *)audio_tx_buf, AUDIO_BUF_WORDS) != HAL_OK
      || HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)audio_rx_buf, AUDIO_BUF_WORDS) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/* Process one 1 ms block: rx holds AUDIO_BLOCK_FRAMES frames of mic samples (slot 0,
 * slot 1), tx receives the matching speaker samples. Runs in the DMA interrupt. While
 * this block of rx was being received, the DMA was playing the other half of tx, so
 * this tx half is played about 1 ms from now. */
static void Audio_ProcessBlock(const uint32_t *rx, uint32_t *tx)
{
  uint32_t t_start = DWT->CYCCNT;

  for (uint32_t f = 0; f < AUDIO_BLOCK_FRAMES; f++)
  {
    int32_t mic[2];
    for (uint32_t ch = 0; ch < 2; ch++)
    {
      mic[ch] = Audio_FromSai24(rx[2 * f + ch]);
      Audio_MeterAdd((Audio_Meter *)&mic_meter[ch], mic[ch]);
      int32_t dev = mic[ch] - mic_meter[ch].dc;
      uint32_t mag = (dev < 0) ? (uint32_t)(-(int64_t)dev) : (uint32_t)dev;
      if (mag > stream_peak[ch])
      {
        stream_peak[ch] = mag;
      }
    }

    uint8_t rec_raw = (HAL_GPIO_ReadPin(REC_BTN_PORT, REC_BTN_PIN) == GPIO_PIN_RESET)
                      || (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
    if (Audio_ButtonPressed(&rec_button, rec_raw))
    {
      Audio_RecStart(&recorder[0]);
      Audio_RecStart(&recorder[1]);
      app_state = APP_RECORDING;
      led_count = 0;
      rec_frames = 0;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    if (Audio_ButtonPressed(&play_button, HAL_GPIO_ReadPin(PLAY_BTN_PORT, PLAY_BTN_PIN) == GPIO_PIN_RESET)
        && app_state != APP_RECORDING && recorder[0].len > 0)
    {
      for (uint32_t ch = 0; ch < 2; ch++)
      {
        /* In the split test both outputs play the single track in rec_buf[0]. */
        uint32_t src = rec_split_test ? 0U : ch;
        rec_clip[ch] = (Audio_Clip){ rec_buf[src], recorder[src].len, 0, 0, REC_RATIO };
      }
      play_left = recorder[0].len * REC_RATIO;
      app_state = APP_PLAYING;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    }

    int32_t out[2] = { 0, 0 };
    if (app_state == APP_RECORDING)
    {
      uint8_t second_half = (rec_split_test && rec_frames >= REC_SPLIT_FRAMES) ? 1U : 0U;
      if (rec_frames == REC_SPLIT_FRAMES && rec_split_test)
      {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
        led_count = 0;
      }
      rec_frames++;
      if (++led_count == LED_FLASH_FRAMES)
      {
        led_count = 0;
        if (second_half)
        {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        }
        else
        {
          HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
        }
      }
      uint8_t full;
      if (rec_split_test)
      {
        full = Audio_RecAdd(&recorder[0], second_half ? mic[1] : mic[0]);
      }
      else
      {
        full = Audio_RecAdd(&recorder[0], mic[0]);
        full |= Audio_RecAdd(&recorder[1], mic[1]);
      }
      if (full)
      {
        app_state = APP_IDLE;
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
      }
    }
    else if (app_state == APP_PLAYING)
    {
      out[0] = Audio_ClipNext(&rec_clip[0]);
      out[1] = Audio_ClipNext(&rec_clip[1]);
      if (--play_left == 0U)
      {
        app_state = APP_IDLE;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
      }
    }
    else if (audio_mode == 0)
    {
      out[0] = Audio_ClipNext(&clip);
      out[1] = out[0];
    }
    else if (audio_mode == 1)
    {
      out[0] = mic[0];
      out[1] = mic[1];
    }
    else if (audio_mode == 3)
    {
      out[0] = Audio_ToneNext(&tone);
      out[1] = out[0];
    }
    tx[2 * f] = Audio_ToSai24(out[0]);
    tx[2 * f + 1] = Audio_ToSai24(out[1]);

    /* Streams: hand data to the main loop, which does the slow work. */
    if (++stream_frames >= MIC_STREAM_FRAMES)
    {
      mic_report[0] = stream_peak[0];
      mic_report[1] = stream_peak[1];
      mic_report_ready = 1;
      stream_peak[0] = 0;
      stream_peak[1] = 0;
      stream_frames = 0;
    }
    if (spec_collecting && Audio_SpectrumAdd(&spectrum, mic[spec_ch]))
    {
      spec_collecting = 0;
    }
  }

  if (SAI1_Block_B->SR & SAI_xSR_OVRUDR)
  {
    audio_tx_underruns++;
    __HAL_SAI_CLEAR_FLAG(&hsai_BlockB1, SAI_FLAG_OVRUDR);
  }
  audio_frames += AUDIO_BLOCK_FRAMES;
  audio_blocks++;
  uint32_t cycles = DWT->CYCCNT - t_start;
  if (cycles > audio_isr_max_cycles)
  {
    audio_isr_max_cycles = cycles;
  }
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai == &hsai_BlockA1)
  {
    Audio_ProcessBlock(&audio_rx_buf[0], &audio_tx_buf[0]);
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai == &hsai_BlockA1)
  {
    Audio_ProcessBlock(&audio_rx_buf[AUDIO_BUF_WORDS / 2U], &audio_tx_buf[AUDIO_BUF_WORDS / 2U]);
  }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  (void)hsai;
  audio_sai_errors++;
}

void GPDMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_sai1_a);
}

void GPDMA1_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_sai1_b);
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


  /* UART to the PC (ST-LINK virtual COM port). Audio still runs if this fails. */
  uart_comm_init();
  if (stream_mode != 0)
  {
    uart_test_periodic = 0;
  }
  Audio_SpectrumInit();
  Audio_ToneSet(&tone, TONE_HZ, 48000.0f, TONE_LEVEL);
  Audio_SpectrumStart(&spectrum);

  /* Cycle counter for the audio interrupt timing measurement. */
  DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  uint8_t last_vol = 0xFF;

  /* Buttons and LEDs. */
  {
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = REC_BTN_PIN | PLAY_BTN_PIN;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOG, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  }

  /* Start the spectrum stream's first collection, then audio. */
  spec_collecting = (stream_mode == 2);
  if (Audio_Start() != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Audio runs in the SAI DMA interrupt; this loop does the slow work, and nothing
     * here can make the audio late. */
    uart_comm_poll();

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

    if (mic_report_ready)
    {
      mic_report_ready = 0;
      if (stream_mode == 1)
      {
        /* If UART is still busy this report is skipped, never waited for. */
        uint8_t msg[32];
        size_t n = 4;
        memcpy(msg, "MIC,", 4);
        n += uart_format_uint(&msg[n], mic_report[0]);
        msg[n++] = ',';
        n += uart_format_uint(&msg[n], mic_report[1]);
        msg[n++] = '\r';
        msg[n++] = '\n';
        uart_send(msg, n);
      }
    }

    if (stream_mode == 2 && !spec_collecting && spectrum.phase == 0U)
    {
      /* Stream just switched on: start collecting. */
      Audio_SpectrumStart(&spectrum);
      __DMB();
      spec_collecting = 1;
    }
    else if (stream_mode == 2 && !spec_collecting)
    {
      /* The interrupt has collected a full set of samples: run the FFT, format the
       * line, and send it once the UART is free. */
      if (spec_len == 0U)
      {
        while (!Audio_SpectrumStep(&spectrum))
        {
        }
        memcpy(spec_line, "FFT,", 4);
        spec_len = 4;
        spec_line[spec_len++] = (uint8_t)('0' + spec_ch);
        for (uint32_t k = 0; k < SPEC_BINS; k++)
        {
          spec_line[spec_len++] = ',';
          spec_len += uart_format_uint(&spec_line[spec_len], spectrum.db[k]);
        }
        spec_line[spec_len++] = '\r';
        spec_line[spec_len++] = '\n';
      }
      if (!uart_tx_busy() && uart_send(spec_line, spec_len))
      {
        spec_len = 0;
        spec_ch ^= 1U;
        Audio_SpectrumStart(&spectrum);
        __DMB();                 /* spectrum reset before the interrupt starts adding */
        spec_collecting = 1;
      }
    }
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
  hsai_BlockA1.SlotInit.SlotActive = 0x00000003;
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

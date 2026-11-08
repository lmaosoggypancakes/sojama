/**
  * @file    uart_comm.c
  * @brief   Nonblocking UART link to the PC over the ST-LINK virtual COM port.
  *
  * Owns USART1, GPDMA1 channel 0 and their interrupts. These are set up here rather
  * than in CubeMX; if USART1 or GPDMA1 channel 0 is later enabled in CubeMX, remove
  * the matching setup, MSP code and IRQ handlers from one of the two places.
  */
#include "uart_comm.h"
#include "stm32u5xx_hal.h"
#include <string.h>

/* Below everything else in the system except SysTick (15). The audio loop runs in
 * thread mode and polls the SAI FIFOs, so these short ISRs only delay it briefly. */
#define UART_IRQ_PRIORITY   14U
#define UART_RX_CHUNK       32U
#define UART_TEST_PERIOD_MS 100U

volatile UART_Stats uart_stats;
volatile uint8_t uart_test_periodic = 1;
volatile uint8_t uart_test_echo = 1;

static UART_HandleTypeDef huart1;
static DMA_HandleTypeDef hdma_usart1_tx;

/* TX: owned by DMA while tx_busy is set. */
static uint8_t tx_buf[UART_TX_BUF_SIZE];
static volatile uint8_t tx_busy;

/* RX: the HAL fills rx_chunk; the callback assembles lines in rx_line and hands a
 * finished line over in ready_line. */
static uint8_t rx_chunk[UART_RX_CHUNK];
static char rx_line[UART_RX_LINE_MAX];
static uint32_t rx_len;
static uint8_t rx_discarding;     /* current line overflowed; skip to the next newline */
static char ready_line[UART_RX_LINE_MAX];
static volatile uint8_t line_ready;

/* Main-loop state. */
static uint8_t echo_buf[UART_TX_BUF_SIZE];
static size_t echo_len;           /* nonzero while an echo is waiting for the TX buffer */
static uint32_t test_count;
static uint32_t test_last_ms;

static void uart_start_rx(void)
{
  if (HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_chunk, UART_RX_CHUNK) != HAL_OK)
  {
    uart_stats.uart_errors++;
  }
}

bool uart_comm_init(void)
{
  GPIO_InitTypeDef gpio = {0};
  RCC_PeriphCLKInitTypeDef clk = {0};

  clk.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  clk.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
  {
    return false;
  }
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* PA9 = USART1_TX, PA10 = USART1_RX, wired to the ST-LINK virtual COM port. */
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* TX DMA: memory -> USART1_TDR, one byte per request. */
  hdma_usart1_tx.Instance = GPDMA1_Channel0;
  hdma_usart1_tx.Init.Request = GPDMA1_REQUEST_USART1_TX;
  hdma_usart1_tx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.SrcInc = DMA_SINC_INCREMENTED;
  hdma_usart1_tx.Init.DestInc = DMA_DINC_FIXED;
  hdma_usart1_tx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_usart1_tx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_usart1_tx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_usart1_tx.Init.SrcBurstLength = 1;
  hdma_usart1_tx.Init.DestBurstLength = 1;
  hdma_usart1_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  hdma_usart1_tx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
  {
    return false;
  }
  if (HAL_DMA_ConfigChannelAttributes(&hdma_usart1_tx, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    return false;
  }
  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    return false;
  }

  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, UART_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, UART_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  uart_start_rx();
  test_last_ms = HAL_GetTick();
  return true;
}

bool uart_send(const uint8_t *data, size_t length)
{
  if (length == 0U || length > UART_TX_BUF_SIZE)
  {
    uart_stats.tx_errors++;
    return false;
  }
  if (tx_busy)
  {
    uart_stats.tx_busy++;
    return false;
  }

  memcpy(tx_buf, data, length);
  tx_busy = 1;
  if (HAL_UART_Transmit_DMA(&huart1, tx_buf, (uint16_t)length) != HAL_OK)
  {
    tx_busy = 0;
    uart_stats.tx_errors++;
    return false;
  }
  uart_stats.tx_started++;
  return true;
}

size_t uart_format_uint(uint8_t *buf, uint32_t value)
{
  char digits[10];
  size_t d = 0;
  size_t n = 0;

  do
  {
    digits[d++] = (char)('0' + value % 10U);
    value /= 10U;
  } while (value != 0U);
  while (d > 0U)
  {
    buf[n++] = (uint8_t)digits[--d];
  }
  return n;
}

bool uart_tx_busy(void)
{
  return tx_busy != 0U;
}

/* Write "<prefix><number>\r\n" to buf; returns the length. */
static size_t format_number_line(uint8_t *buf, const char *prefix, uint32_t value)
{
  size_t n = strlen(prefix);

  memcpy(buf, prefix, n);
  n += uart_format_uint(&buf[n], value);
  buf[n++] = '\r';
  buf[n++] = '\n';
  return n;
}

void uart_comm_poll(void)
{
  /* A finished command line: queue the echo (the only command handling for now). */
  if (line_ready)
  {
    if (uart_test_echo && echo_len == 0U)
    {
      size_t len = strlen(ready_line);
      memcpy(echo_buf, "echo: ", 6);
      memcpy(&echo_buf[6], ready_line, len);
      echo_buf[6 + len] = '\r';
      echo_buf[7 + len] = '\n';
      echo_len = len + 8U;
    }
    line_ready = 0;
  }

  /* Echoes are kept until the TX buffer is free; they are retried on later polls. */
  if (echo_len != 0U && !tx_busy)
  {
    if (uart_send(echo_buf, echo_len))
    {
      echo_len = 0;
    }
  }

  /* Periodic test message. If TX is busy this period is skipped, not delayed. */
  uint32_t now = HAL_GetTick();
  if (uart_test_periodic && (now - test_last_ms) >= UART_TEST_PERIOD_MS)
  {
    test_last_ms = now;
    uint8_t msg[40];
    size_t len = format_number_line(msg, "STM32 UART test: ", test_count + 1U);
    if (uart_send(msg, len))
    {
      test_count++;
    }
  }
}

/* ---- HAL callbacks (interrupt context: keep short) ------------------------- */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    tx_busy = 0;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  for (uint16_t i = 0; i < Size; i++)
  {
    char c = (char)rx_chunk[i];
    if (c == '\r')
    {
      continue;
    }
    if (c == '\n')
    {
      if (rx_discarding)
      {
        rx_discarding = 0;
      }
      else if (line_ready)
      {
        uart_stats.rx_dropped++;
      }
      else
      {
        memcpy(ready_line, rx_line, rx_len);
        ready_line[rx_len] = '\0';
        line_ready = 1;
        uart_stats.rx_lines++;
      }
      rx_len = 0;
    }
    else if (!rx_discarding)
    {
      if (rx_len < UART_RX_LINE_MAX - 1U)
      {
        rx_line[rx_len++] = c;
      }
      else
      {
        rx_discarding = 1;
        rx_len = 0;
        uart_stats.rx_overflows++;
      }
    }
  }

  /* Receive-to-idle in interrupt mode stops after each event; start the next one. */
  uart_start_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  uart_stats.uart_errors++;
  uart_stats.last_error = huart->ErrorCode;

  /* A TX DMA error ends the transfer: free the buffer. */
  if (huart->gState == HAL_UART_STATE_READY && tx_busy)
  {
    tx_busy = 0;
    uart_stats.tx_errors++;
  }
  /* Errors such as overrun stop reception: drop the partial line and restart. */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    rx_len = 0;
    rx_discarding = 0;
    uart_start_rx();
  }
}

/* ---- Interrupt handlers --------------------------------------------------- */

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void GPDMA1_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

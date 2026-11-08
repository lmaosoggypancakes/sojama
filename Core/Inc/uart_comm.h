/**
  * @file    uart_comm.h
  * @brief   Nonblocking UART link to the PC over the ST-LINK virtual COM port.
  *
  * USART1 (PA9 TX, PA10 RX) at 115200 8-N-1. TX uses GPDMA1 channel 0; RX uses
  * receive-to-idle interrupts and collects newline-terminated lines.
  *
  * Everything here is called from the main loop. Interrupt callbacks only move
  * bytes and set flags; lines are handled in uart_comm_poll().
  */
#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UART_TX_BUF_SIZE   1100U  /* largest message uart_send() accepts (fits one spectrum line) */
#define UART_RX_LINE_MAX   64U    /* longest command line, including the terminator */

typedef struct
{
  uint32_t tx_started;     /* DMA transfers started */
  uint32_t tx_busy;        /* uart_send() refused because a transfer was in progress */
  uint32_t tx_errors;      /* uart_send() failed to start, or a TX error ended a transfer */
  uint32_t rx_lines;       /* complete lines received */
  uint32_t rx_overflows;   /* lines longer than UART_RX_LINE_MAX, discarded */
  uint32_t rx_dropped;     /* lines discarded because the previous one was not handled yet */
  uint32_t uart_errors;    /* framing, noise, overrun and other UART errors */
  uint32_t last_error;     /* last HAL UART error code */
} UART_Stats;

extern volatile UART_Stats uart_stats;

/* Test features, changeable from the debugger:
 * uart_test_periodic sends "STM32 UART test: N" every 100 ms,
 * uart_test_echo answers each received line with "echo: <line>". */
extern volatile uint8_t uart_test_periodic;
extern volatile uint8_t uart_test_echo;

/* Set up USART1, its DMA channel and interrupts, and start receiving.
 * Returns false if the peripheral could not be initialized. */
bool uart_comm_init(void);

/* Copy data into the TX buffer and start a DMA transfer. Never waits.
 * Returns false if a transfer is in progress, length is 0 or larger than
 * UART_TX_BUF_SIZE, or the transfer could not be started. */
bool uart_send(const uint8_t *data, size_t length);

/* True while a DMA transfer is in progress (uart_send() would return false). */
bool uart_tx_busy(void);

/* Write value as decimal digits (no terminator) and return the number written (1..10). */
size_t uart_format_uint(uint8_t *buf, uint32_t value);

/* Call often from the main loop: handles received lines and the test messages. */
void uart_comm_poll(void);

#endif /* UART_COMM_H */

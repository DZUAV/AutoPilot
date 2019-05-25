#ifndef _MAVLINK_USART_FIFO_H_
#define _MAVLINK_USART_FIFO_H_
#include "stdint.h" 

#define UART_TX_BUFFER_SIZE        255
#define UART_RX_BUFFER_SIZE        255

#define true 1
#define false 0
typedef struct _fifo {
	uint8_t* buf;
	uint16_t length;
	uint16_t head;
	uint16_t tail;
} fifo_t;
extern fifo_t uart_rx_fifo, uart_tx_fifo;
extern uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE], uart_rx_buf[UART_RX_BUFFER_SIZE];

uint8_t fifo_read_ch(fifo_t* fifo, uint8_t* ch);
uint8_t fifo_write_ch(fifo_t* fifo, uint8_t ch);
uint16_t fifo_free(fifo_t* fifo);
uint16_t fifo_used(fifo_t* fifo);
void fifo_init(fifo_t* fifo, uint8_t* buf, uint16_t length);
uint8_t serial_write_buf(uint8_t* buf, uint16_t length);
uint8_t serial_read_ch(void);
uint16_t serial_free(void);
uint16_t serial_available(void);

#endif  












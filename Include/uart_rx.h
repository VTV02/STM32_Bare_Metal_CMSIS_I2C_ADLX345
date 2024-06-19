
#ifndef UART_RX_H_
#define UART_RX_H_

void uart2_txrx_init(void);
void uart2_write(int ch);
void write_string(char* word);
char uart2_read(void);
void uart2_rx_interrupt_init(void);
void dma1_stream6_init(uint32_t src, uint32_t dst, uint32_t len);

#define HISR_TCIF6	 (1U<<21)
#define HIFCR_CTCIF6 (1U<<21)

#endif /* UART_RX_H_ */

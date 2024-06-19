#include<stm32f411xe.h>
#include <uart_rx.h>
#include <string.h>

#define SYS_FREQ         16000000
#define APB1_CLK		 SYS_FREQ
#define UART_BAURATE	 115200
#define UART2EN			(1U<<17)
#define GPIOAEN			(1U<<0)
#define CR1_TE			(1U<<3)
#define CR1_RE			(1U<<2)
#define CR1_EN			(1U<<13)
#define SR_TXE			(1U<<7)
#define SR_RXNE			(1U<<5)
#define CR1_RXNEIE      (1U<<5)

#define DMA1EN			(1U<<21)
#define DMA_S_EN		(1U<<0)

void write_string(char* word);
static uint16_t compute_uart_bd(uint32_t PeriphClk,uint32_t BauRate);
static void uart_set_baurate(USART_TypeDef *USARTx,uint32_t PeriphClk, uint32_t BauRate);
char uart2_read(void);

void dma1_stream6_init(uint32_t src, uint32_t dst, uint32_t len)
{
	/*Enable clock access to DMA*/
	RCC->AHB1ENR|=DMA1EN;
	/*Disable DMA1 Stream6 */
	DMA1_Stream6->CR&=~DMA_S_EN;
	/*Clear all interrupt flag of Stream6*/
	DMA1->HIFCR|=(1U<<16);
	DMA1->HIFCR|=(1U<<18);
	DMA1->HIFCR|=(1U<<19);
	DMA1->HIFCR|=(1U<<20);
	DMA1->HIFCR|=(1U<<21);
	/*Set the destination buffer */
	DMA1_Stream6->PAR|=dst;
	/*Set the source buffer*/
	DMA1_Stream6->M0AR|=src;
	/*Set the length*/
	DMA1_Stream6->NDTR|=len;
	/*Select Stream6 CH4*/
	DMA1_Stream6->CR|=(1U<<27);
	DMA1_Stream6->CR&=~(1U<<26);
	DMA1_Stream6->CR&=~(1U<<25);
	/*Enable memory increment */
	DMA1_Stream6->CR|=(1U<<10);
	/*Configure transfer direction*/
	DMA1_Stream6->CR&=~(1U<<6);
	DMA1_Stream6->CR|=(1U<<7);
	/*Enable DMA transfer complete interrupt*/
	DMA1_Stream6->CR|=(1U<<4);
	/*Enable direct mode and disable FIFO*/
	DMA1_Stream6->FCR|=0;
	/*Enable DMA1 Stream6 */
	DMA1_Stream6->CR|=DMA_S_EN;
	/*Enable UART2 transmitter DMA */
	USART2->CR3|=(1U<<7);
	/*DMA interrupt enable in NVIC*/
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);


}


void uart2_txrx_init(void)
{
	/***************************Configure uart gpio pin*****************************/
	/*Enable clock access to gpioa*/
	RCC->AHB1ENR|=GPIOAEN;
	/*Set PA2 as alternate function mode*/
	GPIOA->MODER|=(1U<<5);
	GPIOA->MODER&=~(1U<<4);
	/*Set PA2 as alternate function type UART_TX (AF07)*/
	/*PA2 configure AFRL2 with AF07*/
	GPIOA->AFR[0]|=(1U<<8);
	GPIOA->AFR[0]|=(1U<<9);
	GPIOA->AFR[0]|=(1U<<10);
	GPIOA->AFR[0]&=~(1U<<11);

	/*Set PA3 as alternate function */
	GPIOA->MODER|=(1U<<7);
	GPIOA->MODER&=~(1U<<6);
	/*Set PA3 as alternate function type UART_RX(AF07)*/
	GPIOA->AFR[0]|=(1U<<12);
	GPIOA->AFR[0]|=(1U<<13);
	GPIOA->AFR[0]|=(1U<<14);
	GPIOA->AFR[0]&=~(1U<<15);

	/***************************Configure uart module*******************************/
	/*Enable clock access to uart2*/
	RCC->APB1ENR|=UART2EN;
	/*Configure baudrate*/
	uart_set_baurate(USART2,APB1_CLK, UART_BAURATE);

	/*Configure transfer direction*/
	USART2->CR1|=CR1_TE;
	/*Configure receiver direction*/
	USART2->CR1|=CR1_RE;
	/*Enable uart module*/
	USART2->CR1|=CR1_EN;

}

void uart2_rx_interrupt_init(void)
{
	/***************************Configure uart gpio pin*****************************/
	/*Enable clock access to gpioa*/
	RCC->AHB1ENR|=GPIOAEN;
	/*Set PA2 as alternate function mode*/
	GPIOA->MODER|=(1U<<5);
	GPIOA->MODER&=~(1U<<4);
	/*Set PA2 as alternate function type UART_TX (AF07)*/
	/*PA2 configure AFRL2 with AF07*/
	GPIOA->AFR[0]|=(1U<<8);
	GPIOA->AFR[0]|=(1U<<9);
	GPIOA->AFR[0]|=(1U<<10);
	GPIOA->AFR[0]&=~(1U<<11);

	/*Set PA3 as alternate function */
	GPIOA->MODER|=(1U<<7);
	GPIOA->MODER&=~(1U<<6);
	/*Set PA3 as alternate function type UART_RX(AF07)*/
	GPIOA->AFR[0]|=(1U<<12);
	GPIOA->AFR[0]|=(1U<<13);
	GPIOA->AFR[0]|=(1U<<14);
	GPIOA->AFR[0]&=~(1U<<15);

	/***************************Configure uart module*******************************/
	/*Enable clock access to uart2*/
	RCC->APB1ENR|=UART2EN;
	/*Configure baudrate*/
	uart_set_baurate(USART2,APB1_CLK, UART_BAURATE);

	/*Configure transfer direction*/
	USART2->CR1|=CR1_TE;
	/*Configure receiver direction*/
	USART2->CR1|=CR1_RE;
	/*Enable RXNE Interrupt */
	USART2->CR1|=CR1_RXNEIE;
	/*Enable UART2 interrupt NVIC*/
	NVIC_EnableIRQ(USART2_IRQn);
	/*Enable uart module*/
	USART2->CR1|=CR1_EN;

}



char uart2_read(void)
{
	/*Make sure the receiver data register is not empty*/
	while(!(USART2->SR&SR_RXNE));
	return USART2->DR;
}

void uart2_write(int ch)
{
	/*Make sure the transmit data register is empty*/
	while(!(USART2->SR & SR_TXE));
	/*Write to transmit data register*/
	USART2->DR=(ch & 0xFF);
}

void write_string(char* word)
{
	for(int i=0;i<strlen(word);i++)
	{
		uart2_write(word[i]);
	}

}

static void uart_set_baurate(USART_TypeDef *USARTx,uint32_t PeriphClk, uint32_t BauRate)
{
	USARTx->BRR=compute_uart_bd(PeriphClk,BauRate);

}
static uint16_t compute_uart_bd(uint32_t PeriphClk,uint32_t BauRate)
{
	return (PeriphClk+(BauRate/2U))/BauRate;
}

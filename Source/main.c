#include <stdio.h>
#include<stdint.h>
#include <uart_rx.h>
#include<stm32f411xe.h>
#include <uart_rx.h>
#include <i2c.h>
#define GPIOAEN			(1U<<0)
#define PIN5			(1U<<5)
#define LED				PIN5
#define SR_RXNE			(1U<<5)


int main(void)
{
	RCC->AHB1ENR|=(1U<<0);
	GPIOA->MODER|=(1U<<10);
	GPIOA->MODER&=~(1U<<11);

	while(1)
	{

	}
}



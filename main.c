#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <inttypes.h>

void u_delay(uint32_t secs);

void u_delay(uint32_t secs) {
	for (uint32_t x = 0; x < secs*3150000; x++);
}


int main(void)
{
	uint32_t i = 0;
	
	mcpr_SetSystemCoreClock();
	
	// Peripheral GPIOD einschalten
	RCC->AHB1ENR |= 1<<3|1;

	// Orange LED (Port D13) auf Ausgang schalten
	GPIOD->MODER |= 0x05000000; //1<<26;
	GPIOD->ODR |= 1<<12;
	
	while( 1 ) {
		
		if( (GPIOA->IDR & 1) != 0) { 
			GPIOD->ODR |= 1<<12; 
			u_delay(1);
			GPIOD->ODR &= 0xEFFF; // ~(1<<13); 
			u_delay(1);
		} else { 
			GPIOD->ODR &= 0xEFFF; // ~(1<<13); 
		}
}
}
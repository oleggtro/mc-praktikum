#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <inttypes.h>


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
		
			GPIOD->ODR |= 1<<12; 
			for (uint32_t x = 0; x < 5000000; x++);
			GPIOD->ODR &= 0xEFFF; // ~(1<<13); 
			for (uint32_t y = 0; y < 5000000; y++);
		}
	
		
}


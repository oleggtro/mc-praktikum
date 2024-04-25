#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <inttypes.h>


//Forward declaration
void u_delay(uint32_t umil);
void LEDs_InitPorts(void);
void LCD_Output16BitWord(uint16_t data);
void LEDs_Write(uint16_t data);
void Run_LEDs(void);
void init_TIM4(void);
void wait_ms(uint16_t ms);
void TIM7_IRQHandler(void);
void Timer_init(void);

uint32_t ms_counter = 0;
const uint16_t delay = 50;
int brightness = 100; //Helligkeit von 000 - 999 (ARR)

void Timer_init(void)
{ 	
    TIM7->CNT = 0;
    TIM7->PSC = 84-1;
    TIM7->ARR = 1000-1;
    TIM7->CR1 |= 1;
    TIM7->DIER |= 1;

    
        NVIC_SetPriority(TIM7_IRQn, 1);
        NVIC_EnableIRQ(TIM7_IRQn);

    return;
}
void TIM7_IRQHandler(void)
{
		TIM7->SR =0;
    
    /*if(TIM7->SR & TIM_SR_UIF){
        TIM7->SR &= ~TIM_SR_UIF;
    }*/
    ms_counter += 1;
}

void wait_ms(uint16_t ms) {
    uint32_t target = ms_counter + ms;

    while (ms_counter < target) {}
			
}


void init_TIM4(void)
{
	RCC->APB1ENR |= (1<<2);
	TIM4->CCMR1 |= (3<<13);
	TIM4->CCER |= (1<<4);
	TIM4->CNT = 0;
	TIM4->PSC = 419;	//200Hz
	TIM4->ARR = 999;
	TIM4->CCR2 = 100; //10% Helligkeit
	TIM4->CR1 |= 1;
	
	return;
	}

int main(void)
{
	
	
	uint32_t i = 0;
	
	mcpr_SetSystemCoreClock();
	
	
	
	
	//LEDs_InitPorts();
	Timer_init();
		init_TIM4();
	
	//GPIOA->MODER |= (1u<<0);
	//timer 7 enable
	RCC->APB1ENR |= 1<<5;
	RCC->AHB1ENR |= 1<<3| 1<<4| 1;
		
	GPIOD->MODER |= (1<<27);
	GPIOD->AFR[1] |= (2<<20); 
	
    // initialize ports{[
	while(1) {
		if( (GPIOA->IDR & 1) != 0)
		{
			while( (GPIOA->IDR & 1) != 0)
			{
				TIM4->CCR2 = 999;
			}
			wait_ms(10000);
			
			while (TIM4->CCR2 > 100)
			{
				wait_ms(1);
				TIM4->CCR2--;
			
			}	
	
	}
		}
	
	/*	LEDs_InitPorts();
	
	while( 1 ) {
		
    // LEDs_Write(0xFFFF); // binary full on 16bit word
        
		if( (GPIOA->IDR & 1) != 0) { 
           Run_LEDs();
		} else { 
				LEDs_Write(0);
		}
		//u_delay(50);
        
	}*/
    return 0;
}

#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include "display.h"
#include <stdio.h>
#include <inttypes.h>


volatile uint16_t tc1 = 0;
volatile uint16_t deltat = 0;
volatile uint32_t ticks = 0;
volatile uint16_t deltatchange = 0;
uint32_t frequency = 0;



void LCD_Output16BitWord(uint16_t data){
  
    GPIOD->ODR &= ~(0xC000); //clear
    GPIOD->ODR |= (data<<14); // add

    // LED 2/3
    GPIOD->ODR &= ~(0x0003);
    GPIOD->ODR |= (data>> 2)& ~(0xFFFC); 

    //LED 4-12
    GPIOE->ODR &= ~(0xFF80);
    GPIOE->ODR |= (data<<3)& ~(0x007F); 

    //LED 13-15
    GPIOD->ODR &= ~(0x0700);
    GPIOD->ODR |= (data>>5)&~(0xF8FF);

    return;
}

void TIM8_BRK_TIM12_IRQHandler(void){
	// falls nicht gecaptured wird, pr¸fen ob Channel 1 getriggert wird TIM12->SR & TIM_SR_CC1IF
	if (TIM12->SR & TIM_SR_CC1IF){
		TIM12->SR &= 0;  // Capture-Interrupt Flag reset
		uint16_t tc2;
		tc2 = TIM12->CCR1;
		deltat = tc2 - tc1;  // deltat = 2 flanks
		tc1 = tc2;
		ticks = deltat;
		deltatchange = 1;
	}
	// ticks += deltat;
	
}

void GPIO_Init(void){
	RCC->AHB1ENR |= 1<<3|1<<4|1<<1|1;
	GPIOB->MODER |= (1<<29);  // GPIOB MODER 14 auf 10
	GPIOB->AFR[1] |= 0x09000000;   // set AF9 in AFRH14 for PB14
}

void TIM12_Init(void){
	RCC->APB1ENR |= (1 << 6);   
	
	TIM12->PSC = 0;  
	TIM12->ARR = 0xFFFF;  
	TIM12->SMCR = 0;
	TIM12->CCMR1 |= 1;  
	TIM12->CCER |= 0x01;  
	TIM12->DIER |= 2;  
	TIM12->CR1 |= 1;  
	
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 2);
}

void LCD_ON(void){
		GPIOD->ODR |= 0x2000;  // set PD13 to 1
}

int main(void)
{
	// LCD variables
	char fq[32];
	char ts[32];
	
	// inits
	mcpr_SetSystemCoreClock();
	GPIO_Init();
	TIM12_Init();
  LCD_Init();
	GPIOD->ODR= ~(1<<13);
  LCD_ClearDisplay (0xFE00);
	LCD_ON();

	
	while(1){
	   if (deltat != 0 && deltatchange) {   //no div by 0
		   frequency = 84000000 / deltat;  
			 deltatchange = 0;
     }
			
		 sprintf(fq, "f: %u Hz", frequency);
		 sprintf(ts, "ticks: %u", ticks);
		 LCD_WriteString(10, 10, 0xFFFF, 0x0000, fq);
		 LCD_WriteString(10, 30, 0xFFFF, 0x0000, ts);
		  
	}
}
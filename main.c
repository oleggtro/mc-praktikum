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
void userbutton(void);

const uint16_t delay = 50;


void userbutton(void)
{
	RCC->AHB1ENR |= (1u<<0);
	//set userbutton input mode (default)
	GPIOA->MODER &= ~(1u<<0) & ~(1u<<1);
	GPIOA->ODR  ^= ()
	
}
//init ports
void LEDs_InitPorts(void)

{
    //Turn on peripheral GPIOD and GPIOE 
    RCC->AHB1ENR |= 1<<3| 1<<4| 1;

    // GPIOD config for LED selection and write signals
    // configure PD7, PD11, PD5 as output
    GPIOD->MODER |= (1 << (7 * 2)) | (1 << (5 * 2)) | (1 << (11 * 2));

    // ensure PD7, PD11 and PD5 are set to high initially
    GPIOD->ODR |= (1 << 7) | (1 << 5) | (1 << 11) ;

    //// orange LED (D13) set output
    //GPIOD->MODER |= 1 << (12*2); // equiv to 0x04000000
    //GPIOD->ODR |= 1<<12;

    // Configure data lines
    GPIOD->MODER |= (1 << (15*2))   | (1 << (9*2))  | (1 << (8*2)) | (1 << (10*2)) | (1 << (14*2))| (1 << (1*2)) | 1;
    GPIOE->MODER |= (1 << (15*2)) | (1 << (14*2)) | (1 << (13*2)) | (1 << (12*2)) | (1 << (11*2)) | (1 << (10*2))
                                  | (1 << (9*2))  | (1 << (8*2))  | (1 << (7*2));
 
    return;
}






// write data to register and activate LED running light
void LEDs_Write(uint16_t data)
{
    // PD11 to high: select ic for LEDs
    GPIOD->ODR |= (1 << 11);
	
    // PD7 to low to address external hardware
    GPIOD->ODR &= ~(1 << 7);


    // write 16bit word to lcd
    LCD_Output16BitWord(data);

    // PD5 to low + then high to save data
    GPIOD->ODR &= ~(1 << 5);
	
    GPIOD->ODR |= (1 << 5);

    // PD7 to high to deactivate external hardware
    GPIOD->ODR |= (1 << 7);
}

// Function to write a 16-bit word to the LCD in the correct order
void LCD_Output16BitWord(uint16_t data)
{


	// data bits 2 and 3 to D 0 and 1
    GPIOD->ODR = (GPIOD->ODR & ~(0x3)) | ((data & 0x000C) >> 2);

	// set bits 4 to 12 to E, 7 to 15
    GPIOE->ODR = (GPIOE->ODR & ~(0x1FF << 7)) | ((data & 0x1FF0) << 3);

	// set bits 13 to 15 to D 8 to 10
    GPIOD->ODR = (GPIOD->ODR & ~(0x7 << 8)) | ((data & 0xE000) >> 5);

    // set bits 0 und 1 to D, Bits 14 and 15
    GPIOD->ODR = (GPIOD->ODR & ~(0x3 << 14)) | ((data & 0x0003) << 14);	

    return;
}

void Run_LEDs(void)
{
	
    for(uint16_t i = 0; i < 16; i++) {
        LEDs_Write(1 << i);  
        u_delay(delay);     
    }
    
    for(uint16_t i = 0; i < 16; i++) {
        LEDs_Write(~(1 << i)); 
        u_delay(delay);
    }
}


void u_delay(uint32_t umil) {
	for (uint32_t x = 0; x < umil*3150; x++);
}


/*void TIM8_BRK_TIM12_IRQHandler()
{
	uint16_t t2 = TIM12->CCR1;
	uint16_t anzTakte = t2 - t;
	uint16_t Hz = 1/(anzTake/20000000);
	LCD_ClearDisplay(0x0000);
	
	LCD_WriteString(100, 10, 0xFFFF, 0x0000, "Takte");
	LCD_WriteString(100, 30, 0xFFFF, 0x0000, "Hertz");

	sprintf(ts, "%u", anzTakte);
	LCD_WriteString(100, 10, 0xFFFF, 0x0000, ts);
	sprintf(ts, "%u", Hz);
	LCD_WriteString(100, 30, 0xFFFF, 0x0000, ts);

	t = t2;
}*/
void init_TIM4(void)
{
	int brightness = 100; //Helligkeit von 000 - 999 (ARR)

	RCC->APB1ENR |= (1<<2);
	TIM4->CCMR1 &= ~(1<<12) | (11<<13);
	TIM4->CCER |= (1<<4);
	TIM4->CNT = 0;
	TIM4->PSC = 419;	//200Hz
	TIM4->ARR = 999;
	TIM4->CCR2 = brightness; //10% Helligkeit
	TIM4->CR1 |= 1;
	
	return;
	}

int main(void)
{
	GPIOA->MODER |= (1u<<0);
	
	uint32_t i = 0;
	
	mcpr_SetSystemCoreClock();
	
	// init Alternate Function
	
	GPIOD->MODER |= (1<<27);
	GPIOD->AFR[1] |= (2<<20); 

    // initialize ports

	if (TASTEGEDRÜCKT)
	{
		while(TASTEGEDRÜCKT)
		{
			TIM4->CCR2 = 999;
		}
		for (10SEKUNDEN){}
		while (brightness > 100)
		{
			//Jeden zweiten Interrupt = 1s Dimmzeit
			TIM4->CCR2 --;
			
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

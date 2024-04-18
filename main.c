#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include "display.h"
#include <math.h>


// switch modes between LEDs and LCD display
#define run_mode 1
#define show_mode 0

#define LED_ON 1
#define LED_OFF 0

uint8_t mode = show_mode;

//Forward declaration
void u_delay(uint32_t ms);
void LEDs_InitPorts (void);
void LCD_Output16BitWord (uint16_t data);
void LEDs_Write (uint16_t data);
void Run_LEDs (void);
void Timer_init(void);
void wait_ms(uint16_t ms);
uint8_t switch_int(int i);
void TIM7_IRQHandler(void);


uint32_t ms_counter;
uint32_t ms_since_last_press;
uint16_t ms_led_counter;
uint8_t led_status;
char str1[50];
char str2[50];



static volatile uint16_t last_capture = 0;
static volatile uint8_t capture_flag = 0;
static volatile uint64_t capture_time = 0;
static volatile float freq = 0;



//init ports
void LEDs_InitPorts(void)

{
    //Turn on peripheral GPIOD and GPIOE 
    RCC->AHB1ENR |=  1<<3| 1<<4| 1;

		RCC->APB1ENR |= 1<<5;
	
    // GPIOD Konfiguration for LED selection and write signals
    // configure PD7, PD11, PD5 as output
    GPIOD->MODER |= (1 << (7 * 2)) | (1 << (5 * 2)) | (1 << (11 * 2));


    // orange LED (D13) set output
    GPIOD->MODER |= (1 << (12*2)) | (1 << (13*2)) | (1 << (11*2)); 
    //GPIOD->ODR |= 1<<12 | 1<<13 | 1<<11;
 
    return;
}

void TIM7_IRQHandler(void)
{
	
	if(TIM7->SR & TIM_SR_UIF){
		TIM7->SR &= ~TIM_SR_UIF;
	}
    ms_counter += 1;
}




void TIM8_BRK_TIM12_IQR(){
    if(TIM12->SR & 0x2) // check if capture/compare 1 interrupt is set
    {
        uint16_t recent_capture;
        TIM12->SR &= ~0x2; // reset bit
        recent_capture = TIM12->CCR1; // read reg
        capture_time = recent_capture - last_capture; // calc capture timeframe
        last_capture = recent_capture; // set capture reg to last read time
        capture_flag = 1; // set capture flag
    }
}

void Timer_init(void)
{
    // turn on TIM12
    RCC->APB1ENR |= (1<<6);
    //init tim12
    // (page 50)
    // 101: select TI1FP1 as input
    // 111: use source as counter reference
    //TIM12->SMCR |= ((101 << 4) |  111);
    TIM12->CR1 |= 1;

    // leave CC1NP and CC1P as they are (only count on raising voltage)





    GPIOB->MODER &= ~(3UL << 2*14); // set PB14 input
    GPIOB->MODER |= 2 << 2*14; // set PB14 alternate fn
    GPIOB->AFR[1] |= 9 << (4 * (14-8)); // set alternate fn 9 for PB14  
    
    TIM12->SMCR = 0; // (MSM = 0): Slave mode control register ==> unused
    TIM12->PSC = 0; // prescaler 0
    TIM12->ARR = 0xFFFF; // CCMR1 for capture+compare
    // set internal counter 1 to count on its own channel (1)
    TIM12->CCMR1 = 1; // TI1 is input without filters etc
    TIM12->CCER |= 1; // activate capture+compare + raising flank 
    TIM12->DIER = 0x0003; // capture event (ch 1) und update event enable
    TIM12->SR = 0; // del statusreg

    TIM12->CR1 |= 1; // activate TIM12

    //Ich denk so wie bei TIM7
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 2); //Prio festlegen
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn); // Enable Timer 12 Interrupt

    // do we need to set CKD??
    // CKD=00 fDTS = internal clock
    // 01 half, 10 quarter, 11 not defined



    TIM7->CNT = 0;
    TIM7->PSC = 84-1;
    TIM7->ARR = 1000-1;
    TIM7->CR1 |= 1;
    TIM7->DIER |= 1;

	
		NVIC_SetPriority(TIM7_IRQn, 1);
		NVIC_EnableIRQ(TIM7_IRQn);

    return;
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


// write data to register and activate LED running light
void LEDs_Write(uint16_t data)
{
    // PD7 to low to address external hardware
    GPIOD->ODR &= ~(1 << 7);

    // PD11 to high: select ic for LEDs
    GPIOD->ODR |= (1 << 11);

    // write 16bit word to lcd
    LCD_Output16BitWord(data);

    // PD5 to high to freeze data
    GPIOD->ODR |= (1 << 5);
    // short delay (maybe a little to long)
    u_delay(1000);
    // PD5 to low + then high to save data
    GPIOD->ODR &= ~(1 << 5);
    u_delay(1000);
    GPIOD->ODR |= (1 << 5);

    // PD7 to high to deactivate external hardware
    GPIOD->ODR |= (1 << 7);
}


// Funktion, um das Lauflicht zu steuern
void Run_LEDs(void)
{
    // Lauflicht von Bit 0 bis 15
    for(uint16_t i = 0; i < 16; i++) {
        LEDs_Write(1 << i);  // Eine '1' durchschieben
        u_delay(500000);     // 0,5 Sekunden warten
    }
    
    // Lauflicht von Bit 15 bis 0 mit einer '0'
    for(uint16_t i = 0; i < 16; i++) {
        LEDs_Write(~(1 << i));  // Eine '0' durchschieben
        u_delay(500000);        // 0,5 Sekunden warten
    }
}


void wait_ms(uint16_t ms) {
    uint32_t target = ms_counter + ms;

    while (ms_counter < target) {}
}

void u_delay(uint32_t ms) {
	for (uint32_t x = 0; x < ms*3150; x++);
}


uint8_t switch_int(int i) {
	if (i == 1) {
		return 0;
	} else {
		return 1;
	}
}


int main(void)
{
		ms_since_last_press = 12000;
		ms_led_counter = 0;
		led_status = LED_OFF;
		uint32_t i = 0;
	
		mcpr_SetSystemCoreClock();

    // initialize ports
		LEDs_InitPorts();
		Timer_init();

    LCD_Init();
    // clear display with color red
    LCD_ClearDisplay(0xFE00);
	
	uint32_t tmp = ms_counter;
	while( 1 ) {
		
		// check led status and activate / deactivate LED
        
				
        if( (GPIOA->IDR & 1) != 0) {
					// turn on display		
					GPIOD->ODR |= 1<<13;
            ms_since_last_press = 0;
					
					if (ms_led_counter >= 500) {
						ms_led_counter = 0;
						led_status = switch_int(led_status);
					}
        }
				
        if (led_status == LED_ON) {
            GPIOD->ODR |= 1<<12;
        } else {
            GPIOD->ODR &= ~(1<<12);
        }
        
        if (ms_since_last_press < 10000) {
            // write time
            uint32_t x = (ms_since_last_press / 1000);
            sprintf(str1, "time: %u", x);
            LCD_WriteString(10, 10, 0xFFFF, 0x0000, &str1);


            if (capture_flag) {
                freq = 84000000.0 / capture_time;
              capture_flag = 0; // reset flag
            }
            //output freq
            sprintf(str2, "freq: %.2f Hz", freq); 
            LCD_WriteString( 10, 30, 0x7E00, 0x0000, str2); 


        } else {
					LCD_ClearDisplay(0x0000);
					GPIOD->ODR &= ~(1<<13); 
				}


				while(tmp + 50 > ms_counter){}
					tmp+=50;
        ms_since_last_press = ms_since_last_press + 50;
        ms_led_counter = ms_led_counter + 50;

		
        
	}
    return 0;
}

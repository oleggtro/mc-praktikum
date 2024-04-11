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


uint32_t ms_counter;
uint32_t ms_since_last_press;
uint16_t ms_led_counter;
uint8_t led_status;
char disp_sec;



//init ports
void LEDs_InitPorts(void)

{
    //Turn on peripheral GPIOD and GPIOE 
    RCC->AHB1ENR |= 1<<3| 1<<4| 1;

    // GPIOD Konfiguration for LED selection and write signals
    // configure PD7, PD11, PD5 as output
    GPIOD->MODER |= (1 << (7 * 2)) | (1 << (5 * 2)) | (1 << (11 * 2));


    // orange LED (D13) set output
    GPIOD->MODER |= (1 << (12*2)) | (1 << (13*2)); 
    GPIOD->ODR |= 1<<12 | 1<<13;
 
    return;
}

void TIM7_IRQHandler()
{
    ms_counter += 1;
}


void Timer_init(void)
{
    TIM7->CNT = 0;
    TIM7->PSC = 20;
    TIM7->ARR = 4000;
    TIM7->CR1 |= 1;
    TIM7->DIER |= 1;


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


int main(void)
{
    led_status = 1;
	uint32_t i = 0;
	
	mcpr_SetSystemCoreClock();

    // initialize ports
	//LEDs_InitPorts();


    LCD_Init();
    // clear display with color red
    LCD_ClearDisplay(0xFE00);
	
	while( 1 ) {
        if( (GPIOA->IDR & 1) != 0) {
            ms_since_last_press = 0;

            if (ms_led_counter >= 1000) {
            ms_led_counter = 0;
            led_status = 1 - led_status;
          }

        }
        
        
        if (ms_since_last_press < 10000) {
            uint32_t x = (ms_since_last_press / 1000);
            snprintf(&disp_sec, sizeof(disp_sec), "%u", x);
            LCD_WriteString(10, 10, 0xFFFF, 0x0000, &disp_sec);
            led_status = LED_ON;
        } 


        // check led status and activate / deactivate LED
        if (led_status == LED_ON) {
            GPIOD->ODR |= 1<<12;
        } else {
            GPIOD->ODR &= 0xEFFF;
        }





        wait_ms(50);
        ms_since_last_press += 50;
        ms_led_counter += 50;

		
        
	}
    return 0;
}

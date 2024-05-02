#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "_mcpr_stm32f407.h"
#include "display.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>


static uint8_t flag_50ms = 0;
static uint8_t user_button = 0;
static uint8_t flag_LEDs = 0;
static uint8_t display_out = 0;
static uint32_t last_capture = 0;
static uint8_t capture_flag = 0;
static uint32_t capture_zeitraum = 0;
static float frequency = 0;
static uint32_t brightness = 0;
static uint8_t button_pressed = 0;
static uint32_t button_pressed_time = 0;
static uint8_t brightness_flag = 0;

//Forward declaration
void LEDs_InitPorts (void);
void LCD_Output16BitWord (uint16_t data);
void LEDs_Write (uint16_t data);
void Run_LEDs (void);
void timer12_init(void);
void timer7_init(void);
void timer4_init(void);
void TIM7_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);
void display_output(void);

void timer4_init(void)
{
    // Timer 4
    RCC->APB1ENR |= (1 << 2); 

    GPIOD->MODER &= ~(3 << (2 * 13));
    GPIOD->MODER |= (2 << (2 * 13));

    // alt fn 2 for pd13 (TIM4_CH2)
    GPIOD->AFR[1] |= (2 << (4 * (13 - 8)));

    // Timer 4 konfigurieren
    TIM4->PSC = (84000000 / 200000) - 1; // 200hz
    TIM4->ARR = 999; 
	TIM4->EGR = 1;
    TIM4->CCMR1 &= (2u << 8); 
    TIM4->CCMR1 = (6 << 12); 
    TIM4->CCER = TIM_CCER_CC2E;
	  TIM4->CR1 = 0x1; 
	 

    TIM4->DIER = 1; 
	  NVIC_SetPriority(TIM4_IRQn, 3); 
    NVIC_EnableIRQ(TIM4_IRQn);
		 TIM4->CCR2 = 100;
}


void timer7_init(){
    RCC->APB1ENR |= (1<<5);
    TIM7->PSC = 84 - 1;
    TIM7->ARR = 1000 -1; 
    TIM7->CR1 |= 1;
    TIM7->DIER |= 1; 

    NVIC_SetPriority(TIM7_IRQn, 1); 
    NVIC_EnableIRQ(TIM7_IRQn); 

}

void timer12_init(){
	RCC->APB1ENR |= (1<<6);
	TIM12->PSC = 0;
	TIM12->ARR = 0xFFFF;
	TIM12->CR1 = 1;
	
	TIM12->CCMR1 = 1; 
	TIM12->CCER = 1; 						
	TIM12->CCER &= ~(0x000Au); 		 
	TIM12->DIER = 2; 
	
	GPIOB->MODER |= (1<< 29); // PB14 input
	GPIOB->MODER &= ~(1u<<28); // PB14 alt fn
	GPIOB->AFR[1] = (9 << 24); // alternative funktion 9 für PB14 setzen

	TIM12->SR = 0;  // Statusregister löschen
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 2); 
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn); 

}

void TIM4_IRQHandler(void)
{
    if(TIM4->SR & 0x1UL) 
    
        TIM4->SR &= ~0x1UL; 
        if(brightness_flag)
        {
            brightness = 1000; 
        }
        else if(brightness > 100)
        {
						
            brightness -= 5; // Pro Sekunde um 25% reduzieren
        }
        else
        {
            brightness = 100; 
        }
        TIM4->CCR2 = brightness;
    }

void TIM7_IRQHandler(void) {

static volatile uint32_t counter_1Hz = 0;
static volatile uint32_t counter_1s = 0;
static volatile uint32_t counter_10s = 0;

    if (TIM7->SR & 1) { 
        TIM7->SR &= ~1UL; 

        counter_1Hz++; 
        counter_1s++;

        // Counter variable
        static uint8_t counter_50ms = 0;
        counter_50ms++;
        if (counter_50ms >= 50) {
            counter_50ms = 0;
            flag_50ms = 1;
        }


        // Überprüfen ob 10s vergangen sind für das ausgehen des Displays
        if (button_pressed && (button_pressed_time >= 10000)) {
            button_pressed = 0;
            button_pressed_time = 0;
            brightness_flag = 0;
        }
        else if (button_pressed) {
            button_pressed_time++;
            brightness_flag = 1;
        }
    }
}

// Timer 12 Interrupt Servicefunktion, Interrupts Service Routine
void TIM8_BRK_TIM12_IRQHandler(){
    if(TIM12->SR & 2) // Check if the Capture/Compare 1 Interrupt flag is set
    {
        uint32_t recent_capture;
        TIM12->SR &= ~0x2UL; // Bit für das Capture/Compare 1 Interrupt zurücksetzen
        recent_capture = TIM12->CCR1; // Capture/Compare Register auslesen
        capture_zeitraum = recent_capture - last_capture; // Capture Zeitraum berechnen
        last_capture = recent_capture; // Capture Register auf den letzten gelesenen Wert setzen
        capture_flag = 1; // Capture the Flag setzen
    }
}

// Funktion zur Initialisierung der Ports für die LEDs
void LEDs_InitPorts(void)

{
    // Peripheral GPIOD und GPIOE einschalten
    RCC->AHB1ENR |= 1<<3| 1<<4| 1; 

    // GPIOD Konfiguration für die LED-Auswahl- und Schreibsignale
    // PD7, PD11, PD5 als Ausgang konfigurieren und PD12 fürs Display (und orange LED)
    GPIOD->MODER |= (1 << (7 * 2)) | (1 << (11 * 2)) | (1 << (5 * 2)) | (1 << (12 * 2)) | (1<< (13 * 2));

    // Sicherstellen, dass PD7, PD11 und PD5 and PD12 (for the Display and the orange LED) initial auf High gesetzt sind
    GPIOD->ODR |= (1 << 7) | (1 << 11) | (1 << 5)  | (1 << 13);

    // Configure data lines
    GPIOD->MODER |= (1 << (15*2)) | (1 << (14*2)) | (1 << (10*2)) | (1 << (9*2))  | (1 << (8*2))  | (1 << (1*2)) | 1;
    GPIOE->MODER |= (1 << (15*2)) | (1 << (14*2)) | (1 << (13*2)) | (1 << (12*2)) | (1 << (11*2)) | (1 << (10*2))
                                  | (1 << (9*2))  | (1 << (8*2))  | (1 << (7*2));
 
    return;
}



// Function to write a 16-bit word to the LCD in the correct order
void LCD_Output16BitWord(uint16_t data)
{
    // Setzen der Bits 0 und 1 auf Port D, Bits 14 und 15
    GPIOD->ODR = (GPIOD->ODR & ~(0x3UL << 14)) | ((data & 0x0003UL) << 14);

    // Setzen der Bits 2 und 3 auf Port D, Bit 0 und 1
    GPIOD->ODR = (GPIOD->ODR & ~(0x3UL)) | ((data & 0x000C) >> 2);

    // Setzen der Bits 13 bis 15 auf Port D, Bit 8 bis 10
    GPIOD->ODR = (GPIOD->ODR & ~(0x7UL << 8)) | ((data & 0xE000UL) >> 5);

    // Setzen der Bits 4 bis 12 auf Port E, Bit 7 bis 15
    GPIOE->ODR = (GPIOE->ODR & ~(0x1FFUL << 7)) | ((data & 0x1FF0UL) << 3);

    return;
}




int main(void)

{

    // System Core Clock initialisieren
	mcpr_SetSystemCoreClock();
    // Initialisierung aller Ports
	LEDs_InitPorts();
    // Timer 7&12&4 initialisieren
    timer7_init();
    timer12_init();
    
    // Display initialisieren
    LCD_Init();
		timer4_init();
		LCD_ClearDisplay(0x0000);
	
	while( 1 ) {

		if( (GPIOA->IDR & 1) != 0) { 
            user_button = 1;
            button_pressed = 1;
        } else {
            user_button = 0;
        }
				
    if (capture_flag) {
        frequency = 84000000.0 / capture_zeitraum; 
        capture_flag = 0; 
    }
		
		
		
    char str1[50]; 
    sprintf(str1, "t: %u seconds", display_out); 
    LCD_WriteString( 10, 10, 0xF0F0, 0x0000, str1); 

   
    char str2[50]; 
    sprintf(str2, "freq: %.2f Hz", frequency); 
    LCD_WriteString( 10, 30, 0xFFFF, 0x0000, str2); 

        // So wird die Main nur alle 50ms aufgerufen
        timer12_init();
        while (!flag_50ms){}
        flag_50ms = 0;
	}
}

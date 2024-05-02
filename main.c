#include "_mcpr_stm32f407.h"
#include "stm32f4xx.h"
#include <inttypes.h>

// Forward declaration
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
int brightness = 100; // Helligkeit von 000 - 999 (ARR)

static volatile float frequency = 0;
static volatile uint32_t capture_zeitraum = 0;
static volatile uint8_t display_out = 0;
static volatile uint8_t capture_flag = 0;
static volatile uint8_t brightness_flag = 0;


void Timer_init(void) {
    RCC->APB1ENR |= (1<<5);

  TIM7->CNT = 0;
  TIM7->PSC = 84 - 1;
  TIM7->ARR = 1000 - 1;
  TIM7->CR1 |= 1;
  TIM7->DIER |= 1;

  NVIC_SetPriority(TIM7_IRQn, 1);
  NVIC_EnableIRQ(TIM7_IRQn);

  return;
}
void TIM7_IRQHandler(void) {
  TIM7->SR = 0;

  /*if(TIM7->SR & TIM_SR_UIF){
      TIM7->SR &= ~TIM_SR_UIF;
  }*/
  ms_counter += 1;
}

void wait_ms(uint16_t ms) {
  uint32_t target = ms_counter + ms;

  while (ms_counter < target) {
  }
}

void TIM4_IRQHandler(void)
{
    if(TIM4->SR & 0x1UL) //TIM4->SR = TIM_IT_Update
    {
        TIM4->SR &= ~0x1UL; // Flag reset // TIM4->SR = ~TIM_IT_Update
        if(brightness_flag)
        {
            brightness = 1000; // Volle Helligkeit => 1000 = 100%
        }
        else if(brightness > 100)
        {
						
            brightness -= 5; // per s reduce by 25%
        }
        else
        {
            brightness = 100; 
        }
        TIM4->CCR2 = brightness;
    }
}

void init_TIM4(void) {
  RCC->APB1ENR |= (1 << 2);

  GPIOD->MODER &= ~(3 << (2 * 13));
  GPIOD->MODER |= (2 << (2 * 13));
  GPIOD->AFR[1] |= (2 << (4 * (13 - 8)));

  TIM4->PSC = (84000000 / 200000) - 1; // 200Hz
  TIM4->ARR = 1000;
  TIM4->EGR = 1;
  TIM4->CCMR1 &= (2u << 8);
  TIM4->CCMR1 = (6 << 12);
  TIM4->CCER = TIM_CCER_CC2E;
  TIM4->CR1 = 0x1;

  TIM4->DIER = 1;
  NVIC_SetPriority(TIM4_IRQn, 3);
  NVIC_EnableIRQ(TIM4_IRQn);
  TIM4->CCR2 = 100;

  return;
}

int main(void) {

  uint32_t i = 0;

  mcpr_SetSystemCoreClock();

  // LEDs_InitPorts();
  Timer_init();
  init_TIM4();

  // GPIOA->MODER |= (1u<<0);
  // timer 7 enable
  RCC->APB1ENR |= 1 << 5;
  RCC->AHB1ENR |= 1 << 3 | 1 << 4 | 1;

  GPIOD->MODER |= (1 << 27);
  GPIOD->AFR[1] |= (2 << 20);

  // initialize ports{[
  while (1) {
    if ((GPIOA->IDR & 1) != 0) {
      while ((GPIOA->IDR & 1) != 0) {
        TIM4->CCR2 = 999;
      }
      wait_ms(10000);

      while (TIM4->CCR2 > 100) {
        wait_ms(1);
        TIM4->CCR2--;
      }
    }

    // Frequenz berechnen und auf dem Display anzeigen
    if (capture_flag) {
      frequency = 84000000.0 / capture_zeitraum; // Frequenz berechnen
      capture_flag = 0; // Capture the Flag zurücksetzen
    }

    char str1[50];
    sprintf(str1, "Time: %u seconds", display_out);
    LCD_WriteString(10, 10, 0x001F, 0x0000, str1);

    char str2[50];
    sprintf(str2, "Frequency: %.2f Hz", frequency);
    LCD_WriteString(10, 30, 0x07E0, 0x0000, str2);
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

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "_mcpr_stm32f407.h"
#include "display.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>


// Modes wechseln zwischen Lauflicht und 16 Bit Anzeige
#define LAUFLICHT_MODE 1
#define ANZEIGE_MODE 0
static uint8_t modus = LAUFLICHT_MODE;

//Flag of Timer 7 and the User Button (PA0)
static volatile uint8_t flag_50ms = 0;
static volatile uint8_t user_button = 0;
static volatile uint8_t flag_LEDs = 0;
static volatile uint8_t display_out = 0;
static volatile uint32_t last_capture = 0;
static volatile uint8_t capture_flag = 0;
static volatile uint32_t capture_zeitraum = 0;
static volatile float frequency = 0;
static volatile uint32_t brightness = 0;
static volatile uint8_t button_pressed = 0;
static volatile uint32_t button_pressed_time = 0;
static volatile uint8_t brightness_flag = 0;

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

////////////////////////// FUNCTIONS /////////////////////////////////
void timer4_init(void)
{
    // Timer 4
    RCC->APB1ENR |= (1 << 2); // RCC_APB1ENR_TIM4EN einschalten

    // PD13 auf Alternate Function setzen
    GPIOD->MODER &= ~(3 << (2 * 13));
    GPIOD->MODER |= (2 << (2 * 13));

    // Alternative Funktion 2 für PD13 setzen (TIM4_CH2)
    GPIOD->AFR[1] |= (2 << (4 * (13 - 8)));

    // Timer 4 konfigurieren
    TIM4->PSC = (84000000 / 200000) - 1; // Vorteiler für 200 Hz 
    TIM4->ARR = 1000; // Autoreload-Wert für 100% Tastverhältnis
	TIM4->EGR = 1;
    TIM4->CCMR1 &= (2u << 8); // Preload aktivieren
    TIM4->CCMR1 = (6 << 12); // PWM-Modus 1 auf Kanal 2 
    //TIM4->CCER &= ~(1u << 4); // Capture/Compare 2 Ausgang aktivieren //TIM4->CCER = TIM_CCER_CC2E;
    //TIM4->CCER &= ~(1u << 7); 
    TIM4->CCER = TIM_CCER_CC2E;
	  TIM4->CR1 = 0x1; // Timer 4 einschalten // TIM4->CR1 = TIM_CR1_CEN;
	 

    // Interrupt konfigurieren
    TIM4->DIER = 1; // Update-Interrupt aktivieren // TIM4->DIER = TIM_DIER_UIE;
	  NVIC_SetPriority(TIM4_IRQn, 3); // Priorität festlegen
    NVIC_EnableIRQ(TIM4_IRQn); // NVIC IRQ für Timer 4 aktivieren
		 TIM4->CCR2 = 100;
}


// Init for timer7
void timer7_init(){
    // Timer 7 einschalten
    RCC->APB1ENR |= (1<<5);
    // Timer 7 konfigurieren
    TIM7->PSC = 84 - 1; // Prescaler 
    TIM7->ARR = 1000 -1; // Auto-Reload Register every 1ms // error: called object type 'int' is not a function or function pointer
    TIM7->CR1 |= 1; // Enable Timer 7
    TIM7->DIER |= 1; // Enable Interrupt

    // Update-Interrupt für TIM7 aktivieren und Priorität festlegen
    NVIC_SetPriority(TIM7_IRQn, 1); // Priorität festlegen
    NVIC_EnableIRQ(TIM7_IRQn); // Enable Timer 7 Interrupt

}

// Init for Timer 12
// Timer 12 input sind CH1/2 -> Pins PB14/15
void timer12_init(){
    // Timer 12 einschalten
	RCC->APB1ENR |= (1<<6);
	TIM12->PSC = 0;
	TIM12->ARR = 0xFFFF;
	TIM12->CR1 = 1; // Timer 12 einschalten
	
	TIM12->CCMR1 = 1; // TI1 ist somit Eingang, sosnt keine Filter o. sonstiges
	TIM12->CCER = 1; // Aktivieren des Capture&Compare + Steigende Flanke 								
	TIM12->CCER &= ~(0x000Au); 			// Set CC1NP and CC1P to 0, get reaction on rising edge 
	TIM12->DIER = 2; // Capture Ereignis (Kanal 1) und Update-Event enable 1
	
	GPIOB->MODER |= (1<< 29); // PB14 auf Input setzen
	GPIOB->MODER &= ~(1u<<28); // PB14 auf Alternate Function setzen
	GPIOB->AFR[1] = (9 << 24); // alternative funktion 9 für PB14 setzen // 32 Bit Register auf 4 Bit pro Pin und 8 Pins pro AFR Register

	TIM12->SR = 0;  // Statusregister löschen
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 2); //Prio festlegen
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn); // Enable Timer 12 Interrupt

}

void TIM4_IRQHandler(void)
{
    if(TIM4->SR & 0x1UL) //TIM4->SR = TIM_IT_Update
    {
        TIM4->SR &= ~0x1UL; // Flag zurücksetzen // TIM4->SR = ~TIM_IT_Update
        if(brightness_flag)
        {
            brightness = 1000; // Volle Helligkeit // 1000 = 100%, da ARR = 1000 und somit 1000/1000 = 1 = 100% // SomIt sind 30 =
        }
        else if(brightness > 100)
        {
						
            brightness -= 5; // Pro Sekunde um 25% reduzieren
        }
        else
        {
            brightness = 100; // Reduzierte Helligkeit
        }
        TIM4->CCR2 = brightness;
    }
}

// Timer 7 Interrupt Servicefunktion
void TIM7_IRQHandler(void) {

static volatile uint32_t counter_1Hz = 0;
static volatile uint32_t counter_1s = 0;
static volatile uint32_t counter_10s = 0;

    if (TIM7->SR & 1) { // If Update-interrupt flag is set // For 1 you can write TIM_SR_UIF
        TIM7->SR &= ~1UL; // Flag löschen

        counter_1Hz++; // Counter für 2 * 0,5 Sekunde erhöhen
        counter_1s++; // Counter für 1 Sekunde erhöhen

        // Counter für 0,5 Sekunde 
        if (counter_1Hz >= 500) {
            counter_1Hz = 0; // Reset
        
            if(user_button) { // Wenn der Taster gedrückt wird, also = 1
                GPIOD->ODR ^= 1UL<<12; // Schaltet die grüne LED alle 1s aus und wieder ein, da XOR: 1^1 = 0 und 0^1 = 1
            }

        }
        /*
        // Wenn die User-Taste losgelassen wird und das Display an ist
        if (!user_button && (GPIOD->ODR & 1UL<<13)) {

            counter_10s++; // Counter für 10 Sekunden erhöhen

            if (counter_10s >= 10000){
                counter_10s = 0; // Reset
            
                GPIOD->ODR &= ~(1UL<<13); // Schaltet die Hintergrundbeleuchtung aus (PD13)
            }
        }
        else {
            counter_10s = 0; // Reset
        }*/

        // Counter variable
        static uint8_t counter_50ms = 0;
        counter_50ms++;
        if (counter_50ms >= 50) {
            counter_50ms = 0;
            flag_50ms = 1;
        }

        // Call Run_LEDs function every x-ms
        // In this version we call the Run_LEDs function every 500ms and call the Run_LEDs function which is increases the position of the LEDs by 1
        // This way we can control the speed of the LEDs by calling the function more or less often
        static uint32_t counter_LEDs = 0;
        counter_LEDs++;
        if (counter_LEDs >= 50) {
            counter_LEDs = 0;
            flag_LEDs = 1;

        }

        // Counter für die Anzeige im Display
        // Die Zeit seit Reset wird im Display angezeigt (in Sekunden)
        if (counter_1s >= 1000){
            counter_1s = 0; // Reset
            display_out++; // For every second
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


// Funktion, um Daten an die LEDs zu schreiben und ein Lauflicht zu erzeugen
void LEDs_Write(uint16_t data)
{
    // PD11 auf High setzen, um den IC3 für die LEDs auszuwählen
    GPIOD->ODR |= (1 << 11);

    // PD7 auf Low setzen, um die externen Bausteine anzusteuern
    GPIOD->ODR &= ~(1UL << 7);

    // Das 16-Bit-Wort an das LCD schreiben
    LCD_Output16BitWord(data);

    // PD5 auf Low und dann wieder auf High setzen, um die Daten zu speichern
    GPIOD->ODR &= ~(1UL << 5);
    // Kein delay nötig
    GPIOD->ODR |= (1 << 5);

    // PD7 auf High setzen, um die externen Bausteine zu deaktivieren
    GPIOD->ODR |= (1 << 7);
}


// Funktion, um das Lauflicht zu steuern
void Run_LEDs(void)
{
    // Static variable to keep track of the current position
    static uint16_t current_led_pos = 0;

    // Lauflicht von Bit 0 bis 15
    if(current_led_pos < 16) {
        LEDs_Write(1 << current_led_pos);  // Eine '1' durchschieben
        current_led_pos++;
    }
    // Lauflicht von Bit 15 bis 0 mit einer '0'
    else if(current_led_pos < 32) {
        LEDs_Write(~(1 << (current_led_pos - 16)));  // Eine '0' durchschieben
        current_led_pos++;
    }
    else {
        current_led_pos = 0;
    }
}

// Auslagern der Display-Ausgabe in eine Funktion
void display_output()
{
    // Frequenz berechnen und auf dem Display anzeigen
    if (capture_flag) {
        frequency = 84000000.0 / capture_zeitraum; // Frequenz berechnen
        capture_flag = 0; // Capture the Flag zurücksetzen
    }
		//LCD_ClearDisplay( 0xFE00 );
    // Zeit seit Reset ins Display schreiben
    char str1[50]; // Erstellen eines Puffers für den String
    sprintf(str1, "Time: %u seconds", display_out); // Konvertieren des display_out s in einen String
    LCD_WriteString( 10, 10, 0x001F, 0x0000, str1); // Zeigen Sie den String auf dem Display an // Vorher: 0xFFFF, 0x0000 // Jetzt hopffentlich blau

    // Ausgaben auf dem Display
    char str2[50]; // Erstellen eines Puffers für den String
    sprintf(str2, "Frequency: %.2f Hz", frequency); // Konvertieren des display_out s in einen String
    LCD_WriteString( 10, 30, 0x07E0, 0x0000, str2); // Zeigen Sie den String auf dem Display an // Hoffentlich grün
}

////////////////////////// MAIN /////////////////////////////////
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
	
	while( 1 ) {

        // Taster auf Port A0 abfragen wodurch die LED auf Port D12 in 1s Intervallen blinkt
		if( (GPIOA->IDR & 1) != 0) { 
            user_button = 1;
            button_pressed = 1;
        } else {
            user_button = 0;
        }
                // Modus wechseln zwischen Lauflicht und 16 Bit Anzeige
        if( modus == LAUFLICHT_MODE ) {
            // Lauflicht von Bit 0 bis 15m
            Run_LEDs();
        }
        else if ( modus == ANZEIGE_MODE ) {
            // 16 Bit Anzeige für die LEDs
            LEDs_Write(0xFFFF); // Binär: 1111 1111 1111 1111
        }

        // Display-Ausgabe
        display_output();
		    timer12_init(); 

        // So wird die Main nur alle 50ms aufgerufen
        timer12_init();
        while (!flag_50ms){}
        flag_50ms = 0;
	}
}

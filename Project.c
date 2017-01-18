// --------------------------------------------------------
//  School: University of Victoria, Canada.
//  Course: CENG 355 "Microprocessor-Based Systems".
//  Developed by: Justin Richard
//  Project: CENG 355 Lab Project
// --------------------------------------------------------
//
// --------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

// Defines
#define myTIM2_PRESCALER ((uint16_t) 0x0000) 	// Clock prescaler for TIM2 timer: no prescaling
#define myTIM2_PERIOD ((uint32_t) 0xFFFFFFFF) 	// Maximum possible setting for overflow
#define myTIM3_PRESCALER ((uint16_t) 0xBB80) 	// Clock prescaler for TIM3 timer: 48Mhz/48K (0xBB80 = 48000) = 1Khz prescaling
#define myTIM3_PERIOD ((uint32_t) 0xFFFFFFFF) 	// Maximum possible setting for overflow

// SPI
SPI_InitTypeDef SPI_InitStructInfo;
SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

// Declare Functions
void myGPIO_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myTIM3_Wait(int);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void myLCD_Init(void);

void mySendToLCD(int, int);
void myLCD_SendCommand(char);
void myLCD_SendData(char);
void mySPI_SendData(uint8_t);

// Declare global vars
float timerEnabled = 0;
float timerPulses = 0;
float frequency = 0;

int main(int argc, char* argv[]){
    trace_printf("Welcome to the project demo.\n");
    myGPIO_Init();	// Initialize I/O port PA
    myTIM2_Init();  // Initialize timer TIM2
    myTIM3_Init();  // Initialize timer TIM3
    myEXTI_Init();  // Initialize EXTI
    myADC_Init();   // Initialize ADC
    myDAC_Init();   // Initialize DAC
    myLCD_Init();   // Initialize LCD
    while (1){
        ADC1 -> CR = ((uint32_t) 0x00000004); 	// Start of ADC conversion
        while((ADC1 -> ISR & 0x00000004) ==0); 	// Wait for end of conversion
        DAC->DHR12R1 = ADC1->DR;				// DAC input from ADC last conversion result
        mySendToLCD(frequency, ((ADC1->DR)*5000/4095)); // Resistance = ADC value * 5000 / 4095 (Max adc value)
    }
    return 0;
}

// Initialize General Purpose I/O pins
void myGPIO_Init(){
    RCC->AHBENR |= 0x00060000;                  // Enable clock for GPIOA and GPIOB
    // PA1 - Input
    GPIOA->MODER &= ~((uint32_t) 0x0000000C);   // Set as input
    GPIOA->PUPDR &= ~((uint32_t) 0x0000000C);   // Ensure no pull-up/pull-down
    // PA2 - ADC
    GPIOA->MODER |= ((uint32_t) 0x00000030);    // Set as analog
    GPIOA->PUPDR &= ~((uint32_t) 0x00000030);   // Ensure no pull-up/pull-down
    // PA4 - DAC
    GPIOA->MODER |= ((uint32_t) 0x00000300);    // Set as analog
    GPIOA->PUPDR &= ~((uint32_t) 0x00000300);   // Ensure no pull-up/pull-down
    // PB3 - SPI SCK
    GPIOB->MODER |= (GPIO_MODER_MODER3_1);		// Set as output
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);		// Ensure no pull-up/pull-down
    // PB5 - SPI MOSI
    GPIOB->MODER |= (GPIO_MODER_MODER5_1);		// Set as output
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);		// Ensure no pull-up/pull-down
    // PB4 - SPI LCK
    GPIOB->MODER |= (GPIO_MODER_MODER4_0);		// Alternating function
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		// Ensure no pull-up/pull-down
}

// Initialize TIM2 timer
void myTIM2_Init(){
    RCC->APB1ENR |= ((uint32_t) 0x00000001);// Enable clock for TIM2 peripheral - Relevant register: RCC->APB1ENR
    TIM2->CR1 = ((uint16_t) 0x006C);        // Configure TIM2: buffer auto-reload, count up, stop on overflow, enable update events, interrupt on overflow only
    TIM2->PSC = myTIM2_PRESCALER;           // Set clock prescaler value
    TIM2->ARR = myTIM2_PERIOD;              // Set auto-reloaded delay
    TIM2->EGR = ((uint16_t) 0x0001);        // Update timer registers - Relevant register: TIM2->EGR
    NVIC_SetPriority(TIM2_IRQn, 0);         // Assign TIM2 interrupt priority = 0 in NVIC - Relevant register: NVIC->IP[3], or use NVIC_SetPriority
    NVIC_EnableIRQ(TIM2_IRQn);              // Enable TIM2 interrupts in NVIC - Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    TIM2->DIER |= ((uint16_t) 0x0001);      // Enable update interrupt generation - Relevant register: TIM2->DIER
}

// Initialize TIM3 timer
void myTIM3_Init(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     // Enable clock for TIM3 peripheral - Relevant register: RCC->APB1ENR
    TIM3->CR1 = ((uint16_t) 0x006C);        // Configure TIM3: buffer auto-reload, count up, stop on overflow, enable update events, interrupt on overflow only
    TIM3->PSC = myTIM3_PRESCALER;           // Set clock prescaler value
    TIM3->EGR |= ((uint16_t) 0x0001);       // Update timer registers - Relevant register: TIM3->EGR
}

// Use tim3 to wait for a certain amount of clock cycles
void myTIM3_Wait(int ms){
    TIM3->CNT = ((uint32_t) 0x00000000);	// Clear the timer
    TIM3->ARR =  ms;       					// Set the timeout value from parameter
    TIM3->EGR |= ((uint16_t) 0x0001);       // Update registers
    TIM3->CR1 |= ((uint16_t) 0x0001);       // Start the timer
    while((TIM3->SR & 0x0001) == 0);   	    // Wait until timer hits the desired count
    TIM3->SR &= ~((uint16_t) 0x0001);       // Clear update interrupt flag
    TIM3->CR1 &= ~0x0001;                   // Stop timer (TIM3->CR1)
}

// Initialize extended interrupt and events controller
void myEXTI_Init(){
    SYSCFG->EXTICR[0] = ((uint32_t) 0x0000);// Map EXTI1 line to PA1 - Relevant register: SYSCFG->EXTICR[0] (PA2 = ADC, PA4 = DAC)
    EXTI->RTSR =        ((uint16_t) 0x0002);// EXTI1 line interrupts: set rising-edge trigger - Relevant register: EXTI->RTSR
    EXTI->IMR =         ((uint16_t) 0x0002);// Unmask interrupts from EXTI1 line - Relevant register: EXTI->IMR
    NVIC_SetPriority(EXTI0_1_IRQn, 0);      // Assign EXTI1 interrupt priority = 0 in NVIC
    NVIC_EnableIRQ(EXTI0_1_IRQn);           // Enable EXTI1 interrupts in NVIC
}

// Notes: ADC supply requirements: 2.4 V to 3.6 V
// Initalize analog to digital converter
void myADC_Init(){
    RCC->APB2ENR |= ((uint16_t) 0x0200);  	// Enabled ADC clock
    ADC1->CFGR1 |= ((uint32_t) 0x00002000);	// Enable continuous conversion
    ADC1->CHSELR |= ((uint32_t) 0x00000004);// Select channel to use to 2 - PA2 (PA1 = EXT1 line, PA4 = DAC)
    ADC1->SMPR |= ((uint32_t) 0x00000007); 	// Set sampling time
    ADC1->CR |= ((uint32_t) 0x00000002); 	// Ensure ADC disabled before calibration
    ADC1->CR |= ((uint32_t) 0x80000000); 	// Start calibration
    while((ADC1->CR & 0x8000000) != 0x0); 	// Wait for ADC calibration to complete
    ADC1->CR |= ((uint32_t) 0x00000001); 	// Enable ADC
}

// Initialize digital to analog converter - Outputs to PA4 (PA1 = EXT1, PA4 = DAC)
void myDAC_Init(){
    RCC->APB1ENR |= ((uint32_t) 0x20000000);// Enabled DAC clock
    DAC->CR = ((uint32_t) 0x00000001); 		// Enable DAC on channel 1
    DAC->SWTRIGR |= ((uint32_t)0x1);		// DAC channel1 software trigger
}

// Initialize the LCD screen
void myLCD_Init(){
    trace_printf("Initializing LCD...\n");
    // Initialize SPI
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // Test for appropriate values
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct->SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, SPI_InitStruct);
    SPI_Cmd(SPI1, ENABLE);

    // Initialize the LCD screen step 1
    mySPI_SendData(0x02);  // 0000 0010 = 0x02 -> LCD Initialization 1 - H = 0010, RS = 0, EN = 0
    mySPI_SendData(0x82);	// 1000 0010 = 0x82 -> LCD Initialization 1 - H = 0010, RS = 0, EN = 1
    mySPI_SendData(0x02);	// 0000 0010 = 0x02 -> LCD Initialization 1 - H = 0010, RS = 0, EN = 0

    // Initialize steps 2-5 from slides
    myLCD_SendCommand(0x28); // 00 0010 1000 = 0x -> 4-bit interface = true, Two-lines = true, 5x8 Dots = true
    myLCD_SendCommand(0x0C); // 00 0000 1100 = 0x -> Display on = true, Cursor on = false, Cursor blink = false
    myLCD_SendCommand(0x06); // 00 0000 0110 = 0x -> Increment cursor position = true, display shift = false
    myLCD_SendCommand(0x01); // 00 0000 0001 = 0x -> Clear display
    trace_printf("Initialized LCD.\n");
}

// Print a given frequency and resistance to the screen
void mySendToLCD(int frequency, int resistance){
    // For debug:
    trace_printf("Frequency: %d hz, Resistance: %d Ohms\n", frequency, resistance);

    // Output to actual screen:
    myLCD_SendCommand(0x80);        // Select first line
    myLCD_SendData(0x46);           // Print F
    myLCD_SendData(0x3A);           // Print :
    myLCD_SendData((uint8_t) 48+((frequency/1000)%10));
    myLCD_SendData((uint8_t) 48+((frequency/100)%10));
    myLCD_SendData((uint8_t) 48+((frequency/10)%10));
    myLCD_SendData((uint8_t) 48+((frequency)%10));
    myLCD_SendData(0x48);           // Print O
    myLCD_SendData(0x7A);           // Print h

    myLCD_SendCommand(0xC0);        // Select second line
    myLCD_SendData(0x52);           // Print R
    myLCD_SendData(0x3A);           // Print :
    myLCD_SendData((uint8_t) 48+((resistance/1000)%10));
    myLCD_SendData((uint8_t) 48+((resistance/100)%10));
    myLCD_SendData((uint8_t) 48+((resistance/10)%10));
    myLCD_SendData((uint8_t) 48+((resistance)%10));
    myLCD_SendData(0x4F);           // Print H
    myLCD_SendData(0x68);           // Print z
}

// Takes a char and sends it to the LCD as a command for you - (RS = 0, EN 0/1/0)
void myLCD_SendCommand(char command){
    char h = ((command >> 4) & 0x0f);	// command = xxxx yyyy >> 4 = 0000 xxxxx, & 0x0f = 0000 xxxx = h
    char l = (command & 0x0f);			// command = xxxx yyyy & 0x0f 					 = 0000 yyyy = l
    mySPI_SendData(0x00 | h); // 0000 0000 | h = 0000 xxxx
    mySPI_SendData(0x80 | h); // 1000 0000 | h = 1000 xxxx
    mySPI_SendData(0x00 | h); // 0000 0000 | h = 0000 xxxx
    mySPI_SendData(0x00 | l); // 0000 0000 | l = 0000 yyyy
    mySPI_SendData(0x80 | l); // 1000 0000 | l = 1000 yyyy
    mySPI_SendData(0x00 | l); // 0000 0000 | l = 0000 yyyy
}

// Takes a char and sends it to the LCD as data (RS = 1, EN 0/1/0)
void myLCD_SendData(char data){
    char h = ((data >> 4) & 0x0f);	// data = xxxx yyyy >> 4 = 0000 xxxxx, & 0x0f = 0000 xxxx = h
    char l = (data & 0x0f);			// data = xxxx yyyy & 0x0f 					  = 0000 yyyy = l
    mySPI_SendData(0x40 | h); // 0100 0000 | h = 0100 xxxx
    mySPI_SendData(0xC0 | h); // 1100 0000 | h = 1100 xxxx
    mySPI_SendData(0x40 | h); // 0100 0000 | h = 0100 xxxx
    mySPI_SendData(0x40 | l); // 0100 0000 | l = 0100 yyyy
    mySPI_SendData(0xC0 | l); // 1100 0000 | l = 1100 yyyy
    mySPI_SendData(0x40 | l); // 0100 0000 | l = 0100 yyyy
}

// Sends data via SPI to LCD as per slide 24 of interfaces slides
void mySPI_SendData(uint8_t Data) {
    GPIOB->BRR |= GPIO_BRR_BR_4;        // Force LCK signal to 0
    while((SPI1->SR & SPI_SR_BSY) !=0); // Check if BSY is 0/SPI ready
    SPI_SendData8(SPI1, Data);			// Send the data via SPI
    while((SPI1->SR & SPI_SR_BSY) !=0); // Check if BSY is 0/SPI ready
    GPIOB->BSRR = GPIO_BSRR_BS_4;       // Force LCK signal to 1
    myTIM3_Wait(1); 					// Wait between LCD accesses since software too fast
}

// This handler is declared in system/src/cmsis/vectors_stm32f0xx.c - From template code
void TIM2_IRQHandler(){
    // Check if update interrupt flag is indeed set
    if ((TIM2->SR & TIM_SR_UIF) != 0){
        trace_printf("\n*** Overflow! ***\n"); 	// Print message to console
        TIM2->SR &= ~(0x0001);              	// Clear update interrupt flag
        TIM2->CR1 |= 0x0001;                	// Restart stopped timer
    }
}

// This handler is declared in system/src/cmsis/vectors_stm32f0xx.c - From lab 1 part 2
void EXTI0_1_IRQHandler(){
    if ((EXTI->PR & EXTI_PR_PR1) != 0){
        if (timerEnabled == 0){                     //  If this is the first edge:
            TIM2->CNT = 0x0000;                     //  Clear count register (TIM2->CNT)
            TIM2->CR1 |= 0x0001;                    //  Start timer (TIM2->CR1)
            timerPulses = 0;                        //  Clear pulse count
            timerEnabled = 1;                       //  Set triggered for next interrupt
        } else {                                    //  Else (this is the second edge):
            TIM2->CR1 &= ~0x0001;                   //  Stop timer (TIM2->CR1)
            timerEnabled = 0;                       //  Toggle triggered variable
            timerPulses = TIM2->CNT;                //  Read out count register (TIM2->CNT)
            frequency = SystemCoreClock / timerPulses;  //  Calculate signal frequency
        }
        EXTI->PR |= EXTI_PR_PR1;                    // Clear EXTI1 interrupt pending flag (EXTI->PR)
    }
}

#pragma GCC diagnostic pop
// ----------------------------------------------------------------------------

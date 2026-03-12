#include "stm32f4xx.h"

// ===== Global variables =====
volatile uint32_t system_ticks = 0; // Milliseconds counter
uint32_t SysCoreClock = 16000000; // System clock frequency (16 MHz)

// ===== Functions =====
void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void LED_Init(void);
void UART1_Init(void);
void UART1_SendChar(char c);
void UART1_SendString(const char* str);

// ===== Main =====
int main(void) {
    SysTick_Config(SysCoreClock / 1000); // Configure SysTick for 1ms interrupts
    LED_Init(); // Initialize the LED
    UART1_Init(); // Initialize UART1
    UART1_SendString("UART OK\r\n"); // Send a test message over UART

    while (1) {
        GPIOC->ODR ^= (1 << 13); // Toggle LED on PC13
        delay_ms(1000); // Delay for 1000ms
        UART1_SendString("\xF0\x9F\x91\x8D\r\n"); // Send a thumbs up emoji over UART
    }
}

// ========== Functions implementations ==========
// ===== SysTick Handler =====
void SysTick_Handler(void) {
    system_ticks++; // Increment the system tick counter
}

// ===== Delay function =====
void delay_ms(uint32_t ms) {
    uint32_t start_tick = system_ticks; // Capture the current tick count
    while ((system_ticks - start_tick) < ms);
}

// ===== LED Initialization =====
void LED_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    GPIOC->MODER &= ~(3 << 26); // Clear mode bits for PC13
    GPIOC->MODER |= (1 << 26); // Set PC13 as output
}

// ===== UART1 Initialization =====
void UART1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable USART1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
    GPIOA->MODER &= ~((3 << 18) | (3 << 20)); // Clear mode bits for PA9 and PA10
    GPIOA->MODER |= (2 << 18) | (2 << 20); // Set PA9 and PA10 to alternate function
    GPIOA->AFR[1] &= ~((0xF << 4) | (0xF << 8)); // Clear alternate function bits for PA9 and PA10
    GPIOA->AFR[1] |= (7 << 4) | (7 << 8); // Set alternate function to USART1 for PA9 and PA10
    USART1->BRR = SysCoreClock / 115200; // Set baud rate to 115200
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable transmitter, receiver and USART
}

void UART1_SendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE)); // Wait until transmit data register is empty
    USART1->DR = c; // Send the character
}

void UART1_SendString(const char* str) {
    while (*str) {
        UART1_SendChar(*str++); // Send each character in the string
    }
}
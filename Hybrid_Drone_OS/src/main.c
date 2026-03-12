#include "stm32f4xx.h"

// ===== Global variables =====
volatile uint32_t system_ticks = 0; // Milliseconds counter

// ===== Functions =====
void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void LED_Init(void);

// ===== Main =====
int main(void) {
    SysTick_Config(16000); // Configure SysTick for 1ms interrupts
    LED_Init(); // Initialize the LED

    while (1) {
        GPIOC->ODR ^= (1 << 13); // Toggle LED on PC13
        delay_ms(1000); // Delay for 1000ms
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
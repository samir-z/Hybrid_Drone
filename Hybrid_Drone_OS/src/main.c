#include "stm32f4xx.h"
#include <stdbool.h>
#include "mpu6050.h"

// ===== Global variables =====
volatile uint32_t system_ticks = 0; // Milliseconds counter
uint32_t SystemCoreClock = 16000000; // System clock frequency (16 MHz)

// ===== Functions =====
void SysTick_Handler(void);
void SystemClock_Config(void);
void delay_ms(uint32_t ms);
void LED_Init(void);
void UART1_Init(void);
void UART1_SendChar(char c);
void UART1_SendString(const char* str);
void UART1_SendInt(int32_t num);
void I2C1_Init(void);
uint8_t I2C1_Ping(uint8_t address);
uint8_t I2C1_Write_Reg(uint8_t address, uint8_t reg, uint8_t data);
uint8_t I2C1_Read_Reg(uint8_t address, uint8_t reg);
uint8_t I2C1_Read_Burst(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t length);

// ===== Main =====
int main(void) {
    // Configure the system clock to 100 MHz and SysTick for 1ms interrupts
    SystemClock_Config(); // Configure the system clock
    SysTick_Config(SystemCoreClock / 1000); // Configure SysTick for 1ms interrupts

    // Initialize peripherals
    LED_Init(); // Initialize the LED
    UART1_Init(); // Initialize UART1
    UART1_SendString("UART OK\r\n"); // Send a test message over UART
    I2C1_Init(); // Initialize I2C1
    while (!(I2C1_Ping(MPU6050_I2C_ADDR)))
    {
        UART1_SendString("MPU6050 not found\r\n");
        delay_ms(1000); // Wait before retrying
    }
    UART1_SendString("MPU6050 found\r\n");
    while (MPU6050_Init() == MPU6050_ERR)
    {
        UART1_SendString("MPU6050 initialization failed\r\n");
        delay_ms(1000); // Wait before retrying
    }
    UART1_SendString("MPU6050 initialized\r\n");

    // Main loop
    while (1) {
        MPU6050_Data_t sensor_data;
        if (MPU6050_Read(&sensor_data) == MPU6050_OK) {
            MPU6050_Scale(&sensor_data); // Scale raw data to physical units
            // Send scaled data over UART (for demonstration)
            UART1_SendString("Accel (m/s²): X=");
            UART1_SendInt((int32_t)(sensor_data.accel_x_ms2 * 100)); // Send as integer with 2 decimal places
            UART1_SendString(" Y=");
            UART1_SendInt((int32_t)(sensor_data.accel_y_ms2 * 100));
            UART1_SendString(" Z=");
            UART1_SendInt((int32_t)(sensor_data.accel_z_ms2 * 100));
            UART1_SendString("\r\n");
        } else {
            UART1_SendString("Failed to read from MPU6050\r\n");
        }
        delay_ms(500); // Wait before the next read
    }
}

// ========== Functions implementations ==========
// ===== SysTick Handler =====
void SysTick_Handler(void) {
    system_ticks++; // Increment the system tick counter
}

// ===== System Clock Configuration =====
void SystemClock_Config(void) {
    // Enable HSE (High-Speed External) oscillator
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait until HSE is ready

    // Configure FLASH for 100 MHz system clock
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;

    // Configure APB1 prescaler to divide by 2 (max 50 MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // Configure PLL (Phase-Locked Loop) to multiply HSE frequency to 100 MHz
    RCC->PLLCFGR &= ~((1 << 22) | (0x7FFF << 0)); // Clear PLL source bit and PLL M/N bits
    RCC->PLLCFGR = (25 << 0) | (200 << 6) | (1 << 22); // M=25, N=200, Source=HSE
    RCC->PLLCFGR &= ~(3 << 16); // Clear PLL P bits and P=2
    RCC->CR |= RCC_CR_PLLON; // Enable PLL
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait until PLL is ready
    RCC->CFGR |= (2 << 0); // Select PLL as system clock
    while ((RCC->CFGR & (3 << 2)) != (2 << 2)); // Wait until PLL is used as system clock
    SystemCoreClock = 100000000; // Update SystemCoreClock variable
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
    USART1->BRR = SystemCoreClock / 115200; // Set baud rate to 115200
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable transmitter, receiver and USART
}

// ===== UART1 Send Character =====
void UART1_SendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE)); // Wait until transmit data register is empty
    USART1->DR = c; // Send the character
}

// ===== UART1 Send String =====
void UART1_SendString(const char* str) {
    while (*str) {
        UART1_SendChar(*str++); // Send each character in the string
    }
}

// ===== UART1 Send Integer =====
void UART1_SendInt(int32_t num) {
    char str[12]; // Buffer to hold the string representation of the integer (max 10 digits + sign + null terminator)
    int i = 0;  // Index for the string buffer
    bool is_negative = false;   // Flag to indicate if the number is negative

    if (num < 0) {  // Handle negative numbers
        is_negative = true; // Set the negative flag
        num = -num; // Convert to positive for processing
    }
    if (num == 0) { // Handle zero explicitly
        UART1_SendChar('0');    // Send '0' character for zero
        return; // Exit the function after sending zero
    }
    while (num > 0) {   // Convert the integer to a string in reverse order
        str[i++] = (num % 10) + '0';    // Get the last digit and convert to character
        num /= 10;  // Remove the last digit
    }
    if (is_negative) str[i++] = '-';    // Add the negative sign if the number was negative
    
    // Send the string in correct order
    while (i > 0) {
        UART1_SendChar(str[--i]);
    }
}

// ===== I2C1 Initialization =====
void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C1 clock
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST; // Reset I2C1
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST; // Exit reset state
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock
    GPIOB->MODER &= ~((3 << 12) | (3 << 14)); // Clear mode bits for PB6 and PB7
    GPIOB->MODER |= (2 << 12) | (2 << 14); // Set PB6 and PB7 to alternate function
    GPIOB->OTYPER |= (1 << 6) | (1 << 7); // Set PB6 and PB7 to open-drain
    GPIOB->PUPDR &= ~((3 << 12) | (3 << 14)); // Clear pull-up/pull-down bits for PB6 and PB7
    GPIOB->PUPDR |= (1 << 12) | (1 << 14); // Enable pull-up resistors for PB6 and PB7
    GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28)); // Clear alternate function bits for PB6 and PB7
    GPIOB->AFR[0] |= (4 << 24) | (4 << 28); // Set alternate function to I2C1 for PB6 and PB7
    I2C1->CR2 = 50; // Set APB1 clock frequency to 50 MHz
    I2C1->CCR = 250; // Set I2C clock speed to 100 kHz
    I2C1->TRISE = 51; // Set maximum rise time for 100 kHz
    I2C1->CR1 = I2C_CR1_PE; // Enable I2C1
}

// ===== I2C1 Ping =====
uint8_t I2C1_Ping(uint8_t address) {
    uint32_t timeout = 10000;
    I2C1->CR1 |= I2C_CR1_START; // Generate START condition
    while (!(I2C1->SR1 & I2C_SR1_SB)) { // Wait for START condition to be generated
        if (--timeout == 0) {
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // Timeout occurred
        }
    }
    I2C1->DR = (address << 1) | 0; // Send the address with write bit
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { // Wait for address to be sent
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) {
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // Timeout occurred
        }
    }
    (void)I2C1->SR1; // Clear ADDR flag by reading SR1 and SR2
    (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
    return 1; // Device responded successfully
}

// ===== I2C1 Write Register =====
uint8_t I2C1_Write_Reg(uint8_t address, uint8_t reg, uint8_t data) {
    uint32_t timeout;

    // START
    I2C1->CR1 |= I2C_CR1_START; // Generate START condition
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { // Wait for START condition to be generated
        if (--timeout == 0) return 0; // Timeout occurred, bus might be stuck
    }

    // Send address with write bit
    I2C1->DR = (address << 1) | 0; // Send the address with write bit
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { // Wait for address to be sent
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }
    
    (void)I2C1->SR1; // Clear ADDR flag by reading SR1 and SR2
    (void)I2C1->SR2;

    // Send register address
    I2C1->DR = reg; // Send the register address
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) { // Wait for data register to be empty
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }

    // Send the data byte
    I2C1->DR = data; // Send the data byte
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) { // Wait for data register to be empty
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }

    // Generate STOP condition
    I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
    return 1; // Data written successfully
}

// ===== I2C1 Read Register =====
uint8_t I2C1_Read_Reg(uint8_t address, uint8_t reg) {
    uint32_t timeout;

    // START
    I2C1->CR1 |= I2C_CR1_START; // Generate START condition
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { // Wait for START condition to be generated
        if (--timeout == 0) return 0; // Timeout occurred, bus might be stuck
    }

    // Send address with write bit to specify register address
    I2C1->DR = (address << 1) | 0; // Send the address with write bit
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { // Wait for address to be sent
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }
    
    (void)I2C1->SR1; // Clear ADDR flag by reading SR1 and SR2
    (void)I2C1->SR2;

    // Send register address
    I2C1->DR = reg; // Send the register address
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) { // Wait for data register to be empty
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }

    // Generate repeated START condition for read operation
    I2C1->CR1 |= I2C_CR1_START; // Generate repeated START condition
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { // Wait for repeated START condition to be generated
        if (--timeout == 0) return 0; // Timeout occurred
    }

    // Send address with read bit
    I2C1->DR = (address << 1) | 1; // Send the address with read bit
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { // Wait for address to be sent
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }

    I2C1->CR1 &= ~I2C_CR1_ACK; // Disable ACK for single byte reception
    (void)I2C1->SR1; // Clear ADDR flag by reading SR1 and SR2
    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition after receiving the byte
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)) { // Wait for data to be received
        if (--timeout == 0) return 0; // Timeout occurred
    }
    return I2C1->DR; // Return the received byte
}

// ===== I2C1 Read Burst =====
uint8_t I2C1_Read_Burst(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t length) {
    uint32_t timeout;

    // START
    I2C1->CR1 |= I2C_CR1_START; // Generate START condition
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { // Wait for START condition to be generated
        if (--timeout == 0) return 0; // Timeout occurred, bus might be stuck
    }

    // Send address with write bit to specify register address
    I2C1->DR = (address << 1) | 0; // Send the address with write bit
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { // Wait for address to be sent
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }
    
    // Clear ADDR flag by reading SR1 and SR2
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // Send register address
    I2C1->DR = reg; // Send the register address
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) { // Wait for data register to be empty
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }

    // Generate repeated START condition for read operation
    I2C1->CR1 |= I2C_CR1_START; // Generate repeated START condition
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { // Wait for repeated START condition to be generated
        if (--timeout == 0) return 0; // Timeout occurred
    }

    // Send address with read bit
    I2C1->DR = (address << 1) | 1; // Send the address with read bit
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { // Wait for address to be sent
        if (I2C1->SR1 & I2C_SR1_AF) { // Check for ACK failure
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure flag
            I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
            return 0; // ACK failure occurred
        }
        if (--timeout == 0) return 0; // Timeout occurred
    }

    // Special handling for single byte read, as the hardware requires a specific sequence
    if (length == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;     // Disable ACK for single byte reception
        (void)I2C1->SR1; // Clear ADDR flag by reading SR1 and SR2
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;     // Generate STOP condition after receiving the byte
        
        timeout = 10000;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return 0; }
        buf[0] = I2C1->DR;
    } 
    else {  // For multiple bytes, we can read in a loop and manage ACK/STOP accordingly
        I2C1->CR1 |= I2C_CR1_ACK;      // Enable ACK for multiple byte reception
        (void)I2C1->SR1; // Clear ADDR flag by reading SR1 and SR2
        (void)I2C1->SR2;

        for (uint8_t i = 0; i < length; i++) {  // Read each byte in the burst
            if (i == length - 1) {  // Last byte
                timeout = 10000;
                while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return 0; }
                buf[i] = I2C1->DR;
            } 
            else if (i == length - 2) {  // Second to last byte: Disable ACK and generate STOP after reading the next byte
                timeout = 10000;
                while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return 0; }
                I2C1->CR1 &= ~I2C_CR1_ACK;  // Disable ACK for the next byte
                I2C1->CR1 |= I2C_CR1_STOP;  // Generate STOP after the next byte
                buf[i] = I2C1->DR;          // Read the second to last byte
            } 
            else {  // For all other bytes, just read normally
                timeout = 10000;
                while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return 0; }
                buf[i] = I2C1->DR;
            }
        }
    }
    return 1; // All bytes read successfully
}
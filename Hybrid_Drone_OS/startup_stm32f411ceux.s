    .syntax unified
    .cpu cortex-m4
    .thumb

    /* Exponemos estas etiquetas para que el Linker las vea */
    .global g_pfnVectors
    .global Default_Handler
    .global SysTick_Handler

    /* --- EL RESET HANDLER (La primera función que se ejecuta) --- */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* 1. Configurar el puntero de la pila (Stack Pointer) */
    ldr   sp, =_estack

    /* 2. Copiar la sección .data de la Flash a la RAM */
    ldr r0, =_sdata      /* Destino en RAM */
    ldr r1, =_edata      /* Fin del destino */
    ldr r2, =_sidata     /* Origen en Flash */
    movs r3, #0          /* Índice = 0 */
    b LoopCopyDataInit

CopyDataInit:
    ldr r4, [r2, r3]     /* Leer 4 bytes de Flash */
    str r4, [r0, r3]     /* Escribir 4 bytes en RAM */
    adds r3, r3, #4      /* Avanzar 4 bytes */

LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1           /* ¿Llegamos al final de .data? */
    bcc CopyDataInit     /* Si no, repetir */

    /* 3. Llenar de ceros la sección .bss en la RAM */
    ldr r2, =_sbss
    ldr r4, =_ebss
    movs r3, #0
    b LoopFillZerobss

FillZerobss:
    str  r3, [r2]        /* Escribir un 0 */
    adds r2, r2, #4      /* Avanzar 4 bytes */

LoopFillZerobss:
    cmp r2, r4           /* ¿Llegamos al final de .bss? */
    bcc FillZerobss      /* Si no, repetir */

    /* 4. Llamar a tu código en C */
    bl main

    /* 5. Si por algún motivo main() termina, atrapar al procesador aquí */
LoopForever:
    b LoopForever
    .size Reset_Handler, . - Reset_Handler

    /* --- LA TABLA DE VECTORES DE INTERRUPCIÓN --- */
    /* Esta es la sección que el Linker pone obligatoriamente en 0x08000000 */
    .section .isr_vector,"a",%progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word _estack         /* 0x0000: Top of Stack */
    .word Reset_Handler   /* 0x0004: Reset Handler */
    .word Default_Handler /* 0x0008: NMI Handler */
    .word Default_Handler /* 0x000C: Hard Fault Handler */
    .word Default_Handler         /* 0x0010: MemManage Handler */
    .word Default_Handler         /* 0x0014: BusFault Handler */
    .word Default_Handler         /* 0x0018: UsageFault Handler */
    .word 0                       /* 0x001C: Reservado */
    .word 0                       /* 0x0020: Reservado */
    .word 0                       /* 0x0024: Reservado */
    .word 0                       /* 0x0028: Reservado */
    .word Default_Handler         /* 0x002C: SVCall Handler */
    .word Default_Handler         /* 0x0030: Debug Monitor Handler */
    .word 0                       /* 0x0034: Reservado */
    .word Default_Handler         /* 0x0038: PendSV Handler */
    .word SysTick_Handler         /* 0x003C: SysTick Handler */
    .word Default_Handler         /* IRQ0: WWDG */
    .word Default_Handler         /* IRQ1: PVD */
    .word Default_Handler         /* IRQ2: TAMP_STAMP */
    .word Default_Handler         /* IRQ3: RTC_WKUP */
    .word Default_Handler         /* IRQ4: FLASH */
    .word Default_Handler         /* IRQ5: RCC */
    .word Default_Handler         /* IRQ6: EXTI0 */
    .word Default_Handler         /* IRQ7: EXTI1 */
    .word Default_Handler         /* IRQ8: EXTI2 */
    .word Default_Handler         /* IRQ9: EXTI3 */
    .word Default_Handler         /* IRQ10: EXTI4 */
    .word Default_Handler         /* IRQ11: DMA1_Stream0 */
    .word Default_Handler         /* IRQ12: DMA1_Stream1 */
    .word Default_Handler         /* IRQ13: DMA1_Stream2 */
    .word Default_Handler         /* IRQ14: DMA1_Stream3 */
    .word Default_Handler         /* IRQ15: DMA1_Stream4 */
    .word Default_Handler         /* IRQ16: DMA1_Stream5 */
    .word Default_Handler         /* IRQ17: DMA1_Stream6 */
    .word Default_Handler         /* IRQ18: ADC */
    .word 0                       /* IRQ19: Reservado */
    .word 0                       /* IRQ20: Reservado */
    .word 0                       /* IRQ21: Reservado */
    .word 0                       /* IRQ22: Reservado */
    .word Default_Handler         /* IRQ23: EXTI9_5 */
    .word Default_Handler         /* IRQ24: TIM1_BRK_TIM9 */
    .word Default_Handler         /* IRQ25: TIM1_UP_TIM10 */
    .word Default_Handler         /* IRQ26: TIM1_TRG_COM_TIM11 */
    .word Default_Handler         /* IRQ27: TIM1_CC */
    .word Default_Handler         /* IRQ28: TIM2 */
    .word Default_Handler         /* IRQ29: TIM3 */
    .word Default_Handler         /* IRQ30: TIM4 */
    .word Default_Handler         /* IRQ31: I2C1_EV */
    .word Default_Handler         /* IRQ32: I2C1_ER */
    .word Default_Handler         /* IRQ33: I2C2_EV */
    .word Default_Handler         /* IRQ34: I2C2_ER */
    .word Default_Handler         /* IRQ35: SPI1 */
    .word Default_Handler         /* IRQ36: SPI2 */
    .word USART1_IRQHandler       /* IRQ37: USART1 */
    .word Default_Handler         /* IRQ38: USART2 */
    .word Default_Handler         /* IRQ39: USART6 */
    .word Default_Handler         /* IRQ40: EXTI15_10 */
    .word Default_Handler  /* IRQ41: DMA1_Stream7 */
    .word Default_Handler  /* IRQ42: Reservado */
    .word Default_Handler  /* IRQ43: SDIO */
    .word Default_Handler  /* IRQ44: TIM5 */
    .word Default_Handler  /* IRQ45: SPI3 */
    .word Default_Handler  /* IRQ46: Reservado */
    .word Default_Handler  /* IRQ47: Reservado */
    .word Default_Handler  /* IRQ48: Reservado */
    .word Default_Handler  /* IRQ49: Reservado */
    .word Default_Handler  /* IRQ50: TIM6_DAC */
    .word Default_Handler  /* IRQ51: TIM7 */
    .word Default_Handler  /* IRQ52: DMA2_Stream0 */
    .word Default_Handler  /* IRQ53: DMA2_Stream1 */
    .word Default_Handler  /* IRQ54: DMA2_Stream2 */
    .word Default_Handler  /* IRQ55: DMA2_Stream3 */
    .word Default_Handler  /* IRQ56: DMA2_Stream4 */
    .word Default_Handler  /* IRQ57: Reservado */
    .word Default_Handler  /* IRQ58: Reservado */
    .word Default_Handler  /* IRQ59: Reservado */
    .word Default_Handler  /* IRQ60: Reservado */
    .word Default_Handler  /* IRQ61: Reservado */
    .word Default_Handler  /* IRQ62: ETH_WKUP */
    .word Default_Handler  /* IRQ63: Reservado */
    .word Default_Handler  /* IRQ64: Reservado */
    .word Default_Handler  /* IRQ65: Reservado */
    .word Default_Handler  /* IRQ66: Reservado */
    .word Default_Handler  /* IRQ67: DMA2_Stream5 */
    .word Default_Handler  /* IRQ68: DMA2_Stream6 */
    .word Default_Handler  /* IRQ69: DMA2_Stream7 */
    .word Default_Handler  /* IRQ70: USART6 */
    .word Default_Handler  /* IRQ71: I2C3_EV */
    .word Default_Handler  /* IRQ72: I2C3_ER */
    .word Default_Handler  /* IRQ73: Reservado */
    .word Default_Handler  /* IRQ74: Reservado */
    .word Default_Handler  /* IRQ75: Reservado */
    .word Default_Handler  /* IRQ76: Reservado */
    .word Default_Handler  /* IRQ77: Reservado */
    .word Default_Handler  /* IRQ78: Reservado */
    .word Default_Handler  /* IRQ79: Reservado */
    .word Default_Handler  /* IRQ80: FPU */
    .word Default_Handler  /* IRQ81: Reservado */
    .word Default_Handler  /* IRQ82: Reservado */
    .word Default_Handler  /* IRQ83: SPI4 */
    .word Default_Handler  /* IRQ84: SPI5 */

    /* Handler por defecto para atrapar errores */
    .weak USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
    b Default_Handler
    .size Default_Handler, . - Default_Handler
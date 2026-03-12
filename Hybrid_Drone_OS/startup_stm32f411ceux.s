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

    /* Handler por defecto para atrapar errores */
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
    b Default_Handler
    .size Default_Handler, . - Default_Handler
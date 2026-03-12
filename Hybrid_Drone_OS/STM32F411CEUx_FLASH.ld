/* Punto de entrada, le decimos al STM32 que función ejecutar primero */
ENTRY(Reset_Handler)

/* Definimos la cima del Stack */
_estack = 0x20020000;

/* Definimos la memoria fisica del STM32 (512KB FLASH, 128KB RAM) */
MEMORY
{
    RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 128K
    FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
}

/* Definimos las secciones */
SECTIONS
{
    /* La tabla de Vectores de Interrupción va al inicio de la FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* El código (.text) y constantes (.rodata) van en la Flash */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;     /* Etiqueta que marca el final del código */
    } >FLASH

    /* Guarda la dirección en Flash donde están los datos inicializados */
    _sidata = LOADADDR(.data);

    /* Variables inicializadas (.data). Viven en la RAM, pero se cargan desde la Flash */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;        /* Etiqueta: Inicio de .data en RAM */
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;        /* Etiqueta: Fin de .data en RAM */
    } >RAM AT> FLASH

    /* Variables No Inicializadas (.bss). Viven en RAM. */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;         /* Etiqueta: Inicio de .bss en RAM */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;         /* Etiqueta: Fin de .bss en RAM */
        __bss_end__ = _ebss;
    } >RAM
}
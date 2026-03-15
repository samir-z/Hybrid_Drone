cmake_minimum_required(VERSION 3.16)

# DEFINICIÓN DEL ENTORNO CROSS-COMPILING (Crucial para Bare Metal)
# Le decimos a CMake que estamos en un sistema genérico sin Sistema Operativo
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# Forzamos los compiladores de las librerias de apt UBUNTU 24.04
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# FIX #1: CMAKE_OBJCOPY y CMAKE_SIZE NO son variables nativas de CMake.
# Si se usan directamente, CMake las deja vacías y el POST_BUILD falla en silencio.
# find_program() las busca en el PATH y garantiza que existan antes de usarlas.
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(CMAKE_SIZE    arm-none-eabi-size    REQUIRED)

# Nombre del proyecto y lenguajes habilitados
project(Hybrid_Drone_OS C CXX ASM)

# FIX #4: Genera compile_commands.json en build/ automáticamente.
# Esto permite que clangd/IntelliSense te dé autocompletado y errores en tiempo real.
# El archivo en la raíz del proyecto debe ser un symlink a build/compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Banderas de Hardware
# mcpu: El cerebro exacto
# mthumb: El set de instrucciones compactas de ARM.
# mfloat-abi=hard y mfpu: Activamos la FPU por hardware para matemáticas de vuelo
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")

# Banderas del compilador (C y C++)
# -O0: Sin optimización (máxima información de debug, sin transformaciones)
# -g3: Máxima información de depuración
# -Wall -Wextra: El compilador logeará cualquier mala práctica
set(C_FLAGS "${CPU_FLAGS} -O0 -g3 -Wall -Wextra -ffunction-sections -fdata-sections")
set(CMAKE_C_FLAGS   "${C_FLAGS}"                        CACHE INTERNAL "C Flags")
set(CMAKE_CXX_FLAGS "${C_FLAGS} -fno-exceptions -fno-rtti" CACHE INTERNAL "C++ Flags")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp"   CACHE INTERNAL "ASM Flags")

# BANDERAS DEL ENLAZADOR (Linker)
# --specs=nano.specs: Usa newlib-nano (librería C reducida para embedded)
# --specs=nosys.specs: Stubs para syscalls de newlib que no existen en bare-metal
# -Wl,--gc-sections: Elimina el código muerto que no uses
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F411CEUx_FLASH.ld")
set(LINKER_FLAGS "${CPU_FLAGS} -T ${LINKER_SCRIPT} --specs=nano.specs --specs=nosys.specs -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS}" CACHE INTERNAL "Linker Flags")

# FIX #2: Rutas de fuentes con CMAKE_SOURCE_DIR para ser explícitos.
# Sin esto, las rutas relativas se resuelven desde donde se llama cmake,
# lo que puede fallar si el directorio de build cambia de lugar.
set(SOURCES
    "${CMAKE_SOURCE_DIR}/startup_stm32f411ceux.s"
    "${CMAKE_SOURCE_DIR}/src/main.c"
    "${CMAKE_SOURCE_DIR}/Drivers/mpu6050/mpu6050.c"
    "${CMAKE_SOURCE_DIR}/src/clock.c"
    "${CMAKE_SOURCE_DIR}/src/uart.c"
    "${CMAKE_SOURCE_DIR}/src/i2c.c"
    "${CMAKE_SOURCE_DIR}/src/pwm.c"
    "${CMAKE_SOURCE_DIR}/src/imu_filter.c"
)

# CREACIÓN DEL EJECUTABLE (El archivo .elf)
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Le decimos dónde están los archivos de cabecera (.h)
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    "${CMAKE_SOURCE_DIR}/inc"
    "${CMAKE_SOURCE_DIR}/Drivers/cmsis_core/Include"
    "${CMAKE_SOURCE_DIR}/Drivers/cmsis_device_f4/Include"
    "${CMAKE_SOURCE_DIR}/Drivers/mpu6050"
)

# Definir el modelo exacto del microcontrolador para este target.
# target_compile_definitions (no add_compile_definitions) → scoped al target,
# no contamina otros targets que puedas agregar en el futuro (tests, bootloader, libs).
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE STM32F411xE)

# Enlazamos con la librería matemática (libm) para funciones como atan2f, etc.
target_link_libraries(${PROJECT_NAME}.elf m)

# POST-CONSTRUCCIÓN (Convertir .elf a .bin)
# FIX #3: Usar $<TARGET_FILE:...> en vez de ${PROJECT_NAME}.elf relativo.
# Esto garantiza que el .elf y el .bin se generan en CMAKE_BINARY_DIR (build/)
# y no en un directorio ambiguo relativo al proceso de cmake.
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${PROJECT_NAME}.elf>
            $<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generando .bin y calculando tamaño de memoria..."
)

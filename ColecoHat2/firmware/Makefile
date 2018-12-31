FIRMWARE_VERSION    := $(shell date)

PROJECT         = cvhat

OPT             =       -Os -g

# macos TOOLROOT        ?=      $(HOME)/packages/gcc-arm-none-eabi-7-2018-q2-update
TOOLROOT        ?=      $(HOME)/x-tools/arm-unknown-eabi/
TUPLE		?= arm-unknown-eabi
STM32LCUBEROOT  ?=      $(HOME)/packages/STM32Cube_FW_L0_V1.10.0/
# macos GCC_LIB         ?=      $(TOOLROOT)/lib/gcc/arm-none-eabi/7.3.1
GCC_LIB         ?=      $(TOOLROOT)/lib/gcc/arm-none-eabi/8.2.0

CORTEX_M0_CC_FLAGS = -mthumb -mcpu=cortex-m0 -mfloat-abi=soft -mlittle-endian
TARGET = STM32L011xx
CMSIS_DEVICE_INC_PATH          = $(STM32LCUBEROOT)/Drivers/CMSIS/Device/ST/STM32L0xx/Include
HAL_INC_PATH            = $(STM32LCUBEROOT)/Drivers/STM32L0xx_HAL_Driver/Inc
CMSIS_CORE_INC_PATH         = $(STM32LCUBEROOT)/Drivers/CMSIS/Include/

INC_PATHS                = -I$(CMSIS_DEVICE_INC_PATH) -I$(HAL_INC_PATH) -I$(CMSIS_CORE_INC_PATH) -I.

NOPE = \
        logprintf.o \
        $(NULL)

OBJECTS = \
        main.o \
        startup_stm32l011xx.o \
        stm32l0xx_it.o \
        HAL/system_stm32l0xx.o \
        HAL/stm32l0xx_hal_cortex.o \
        HAL/stm32l0xx_hal_rcc.o \
        HAL/stm32l0xx_hal_gpio.o \
        HAL/stm32l0xx_hal_i2c.o \
        HAL/stm32l0xx_hal_i2c_ex.o \
        HAL/stm32l0xx_hal.o \
        syscalls.o \
        delay.o \
        leds.o \
        utility.o \
        uart.o \
        HAL/stm32l0xx_hal_uart.o \
        $(NULL)

SOURCES = $(OBJECTS:.o=.c)
DEPS = $(OBJECTS:.o=.d)

$(PROJECT).hex: $(PROJECT).elf
	$(TOOLROOT)/bin/$(TUPLE)-objcopy -O ihex $< $@

-include $(DEPS)

# --no-undefined
$(PROJECT).elf:  $(OBJECTS)
	$(TOOLROOT)/bin/$(TUPLE)-g++ $(OPT) $(CORTEX_M0_CC_FLAGS) -D$(TARGET) -TSTM32L011G4U6.ld -lm -Wl,--gc-sections $^ -o $@

startup_stm32l011xx.o: startup_stm32l011xx.s
	$(TOOLROOT)/bin/$(TUPLE)-gcc -Wall $(OPT) $(CORTEX_M4_HWFP_CC_FLAGS)  $< -c -o $@

%.o: %.c
	$(TOOLROOT)/bin/$(TUPLE)-gcc -std=c99 -Wall $(OPT) -ISTM32Cube4 -DFIRMWARE_VERSION="$(FIRMWARE_VERSION)" -fno-builtin $(INC_PATHS) -D$(TARGET) $(CORTEX_M0_CC_FLAGS)  $< -c -o $@ -MMD

%.o: %.cpp
	$(TOOLROOT)/bin/$(TUPLE)-g++ -std=c++14 -Wall $(OPT) -ISTM32Cube4 -DFIRMWARE_VERSION="$(FIRMWARE_VERSION)" -fno-builtin $(INC_PATHS) -D$(TARGET) $(CORTEX_M0_CC_FLAGS)  $< -c -o $@ -MMD

%.annotated: %.cpp
	$(TOOLROOT)/bin/$(TUPLE)-gcc -Wa,-adhln -std=c++14 -Wall $(OPT) -ISTM32Cube4 -DFIRMWARE_VERSION="$(FIRMWARE_VERSION)" -fno-builtin $(INC_PATHS) -D$(TARGET) $(CORTEX_M0_CC_FLAGS)  $< -c > $@

%.annotated: %.c
	$(TOOLROOT)/bin/$(TUPLE)-gcc -Wa,-adhln -std=c99 -Wall $(OPT) -ISTM32Cube4 -DFIRMWARE_VERSION="$(FIRMWARE_VERSION)" -fno-builtin $(INC_PATHS) -D$(TARGET) $(CORTEX_M0_CC_FLAGS)  $< -c > $@

clean:
	rm -f *.hex *.elf *.o $(OBJECTS) $(DEPS)

burn: $(PROJECT).hex
	../../../stm32flash/stm32flash -e 128 -w cvhat.hex -v /dev/tty.usbserial-AL00EUTE

# DO NOT DELETE

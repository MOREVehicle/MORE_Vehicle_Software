######
# THIS IS ONLY TESTED ON LINUX (probably will work on windows however std libaries will be needed to linked diffrently)
# this is made for the STM32G474RE
# only one board can be used at a time right now for debugging
#
# dependencies: 
# 	- arm-none-eabi-gcc, for cross compiling
# 	- openocd, for flashing
# 	- arm-none-eabi-newlib, for stdlibrary
# 	- arm-none-eabi-gdc, for debugging
# 	- bear, for generating compile commands.json for LSP
#
# needed files:
# 	- startup.s
# 	- linker.ld
# 	- BSP 
# 	- CMSIS (optional?)
# 	- HAL (optional)
######

TARGET = STM32_AUTONOMOUS_PROJECT
DEBUG = 1
OPT = -Og
BUILD_DIR = output

C_SOURCES =  \
$(wildcard src/*.c) \
$(wildcard platform/HAL/*.c) \
$(wildcard platform/HAL/src/*.c) \
$(wildcard platform/BSP/*.c) \

C_INCLUDES =  \
-Iinc \
-Iplatform/BSP \
-Iplatform/HAL\
-Iplatform/HAL/inc \
-Iplatform/HAL/inc/Legacy \
-Iplatform/CMSIS/Device/ST/STM32G4xx/Include \
-Iplatform/CMSIS/Include \
-I/usr/arm-none-eabi/include 					#this could be somewhere else depending on your instalation

ASM_SOURCES =  \
platform/startup_stm32g474xx.s

ASMM_SOURCES = 

AS_INCLUDES = 

# EDIT IF ANOTHER MODULE IS SUED FOR MODULE ENABLED
C_DEFS =  \
-DUSE_NUCLEO_64 \
-DUSE_HAL_DRIVER \
-DSTM32G474xx \
-DHAL_ADC_MODULE_ENABLED \
-DHAL_TIM_MODULE_ENABLED \
-DHAL_FDCAN_MODULE_ENABLED \
-DHAL_I2C_MODULE_ENABLED

PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
AS_DEFS = 

ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
LDSCRIPT = platform/STM32G474XX_FLASH.ld

LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

all: build flash

build: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

flash: $(BUILD_DIR)/$(TARGET).elf
	pkill openocd || true
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
		-c "program $< verify reset exit"

debug: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg & \
	sleep 1; \
	$(PREFIX)gdb -ex "target remote localhost:3333" $<

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

clean:
	-rm -fR $(BUILD_DIR)
  
-include $(wildcard $(BUILD_DIR)/*.d)

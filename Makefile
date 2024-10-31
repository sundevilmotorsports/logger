##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.5.0-B34] date: [Wed Oct 30 20:48:03 MST 2024] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = logger


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
BIN_DIR = $(BUILD_DIR)/bin

# Detect the number of processors
NUM_CORES := $(shell powershell -Command "(Get-WmiObject -Query 'SELECT NumberOfCores FROM Win32_Processor').NumberOfCores -join ''")


######################################
# source
######################################

#C Sources
C_SOURCES = $(shell python findFileC.py)
# Core/Src/main.c \
# Core/Src/stm32h7xx_it.c \
# Core/Src/stm32h7xx_hal_msp.c \
# FATFS/Target/bsp_driver_sd.c \
# FATFS/Target/sd_diskio.c \
# FATFS/Target/fatfs_platform.c \
# FATFS/App/fatfs.c \
# USB_DEVICE/App/usb_device.c \
# USB_DEVICE/App/usbd_desc.c \
# USB_DEVICE/App/usbd_cdc_if.c \
# USB_DEVICE/Target/usbd_conf.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_sdmmc.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_delayblock.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
# Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c \
# Core/Src/system_stm32h7xx.c \
# Middlewares/Third_Party/FatFs/src/diskio.c \
# Middlewares/Third_Party/FatFs/src/ff.c \
# Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
# Middlewares/Third_Party/FatFs/src/option/syscall.c \
# Middlewares/Third_Party/FatFs/src/option/ccsbcs.c \
# Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
# Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
# Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
# Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
# Core/Src/sysmem.c \
# Core/Src/syscalls.c

# ASM sources
ASM_SOURCES =  \
Core/Startup/startup_stm32h723vetx.s

# ASM sources
ASMM_SOURCES = 


#######################################
# binaries
#######################################
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
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m7

# fpu
FPU = -mfpu=fpv5-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32H723xx


# AS includes
AS_INCLUDES = 

# C includes
# C_INCLUDES =  \
# -IFATFS/Target \
# -IFATFS/App \
# -IUSB_DEVICE/App \
# -IUSB_DEVICE/Target \
# -ICore/Inc \
# -IDrivers/STM32H7xx_HAL_Driver/Inc \
# -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
# -IMiddlewares/Third_Party/FatFs/src \
# -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
# -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
# -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
# -IDrivers/CMSIS/Include
# -ICore/Inc \
# -ICore/Inc \# -IDrivers/STM32H7xx_HAL_Driver/Inc \
# -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
# -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
# -IDrivers/CMSIS/Include \
# -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
# -IMiddlewares/ST/STM32_USB_Device_Library/Class/Inc \
# -IMiddlewares/Third_Party/FatFs/src \
# -IUSB_DEVICE/App \
# -IUSB_DEVICE/Target \
# -IFATFS/App \
# -IFATFS/Target \
# -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc 
C_INCLUDES = \
$(shell python findHeaderDir.py)

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += -w


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32H723VETx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
# all: $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex $(BIN_DIR)/$(TARGET).bin
#######################################
.PHONY: all fast
fast:
	$(MAKE) -j$(NUM_CORES) all

all: $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex $(BIN_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
#Ensure output directory exists
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)	

$(BIN_DIR): $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
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

$(BIN_DIR)/$(TARGET).elf: $(OBJECTS) Makefile | $(BIN_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BIN_DIR)/%.hex: $(BIN_DIR)/%.elf | $(BIN_DIR)
	$(HEX) $< $@
	
$(BIN_DIR)/%.bin: $(BIN_DIR)/%.elf | $(BIN_DIR)
	$(BIN) $< $@	
	


#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***

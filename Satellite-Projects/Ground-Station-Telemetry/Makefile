# Ground Station Telemetry Processing Unit - Makefile
# Simple Makefile for building the telemetry processing unit

# Project settings
TARGET = ground_station_telemetry
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# Directories
SRC_DIR = src
INC_DIR = inc
BUILD_DIR = build

# Source files
SRCS = \
	$(SRC_DIR)/main.c \
	$(SRC_DIR)/app_tasks.c \
	$(SRC_DIR)/frame_parser.c \
	$(SRC_DIR)/telemetry_protocol.c \
	$(SRC_DIR)/telemetry_types.c \
	$(SRC_DIR)/lcd_driver.c \
	$(SRC_DIR)/crc16.c \
	$(SRC_DIR)/debug_uart.c \
	$(SRC_DIR)/stm32f4xx_hal.c

# Object files
OBJS = $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

# Include paths
INCLUDES = -I$(INC_DIR)

# Compiler flags
CFLAGS = -Wall -Wextra -Werror -O0 -g3 \
	-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	-DSTM32F407xx -DUSE_HAL_DRIVER \
	$(INCLUDES)

# Linker flags
LDFLAGS = -TSTM32F407VGTx_FLASH.ld \
	-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	-specs=nosys.specs -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
	-Wl,--gc-sections

# Build targets
all: $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	@echo "Building complete:"
	@$(SIZE) $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	@rm -rf $(BUILD_DIR)
	@echo "Cleaned build directory"

flash: all
	@echo "Flashing not implemented - use STM32CubeIDE or ST-Link utility"

.PHONY: all clean flash
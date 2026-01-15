TARGET_MCU        := RP2350A
TARGET_MCU_FAMILY := RP2350


# MCU_FLASH_SIZE may be defined in board-specific config.mk
ifeq ($(MCU_FLASH_SIZE),)
# default to 4MB
MCU_FLASH_SIZE  = 4096
endif

DEVICE_FLAGS    += -DPICO_RP2350A=1

# For pico-sdk, define flash-related attributes
# TARGET_DEVICE_FLAGS may be defined in board-specific config.mk
ifeq ($(TARGET_DEVICE_FLAGS),)
# default to 4194304 = 4 * 1024 * 1024 on W25Q080
TARGET_DEVICE_FLAGS  = \
                   -DPICO_FLASH_SPI_CLKDIV=2 \
                   -DPICO_FLASH_SIZE_BYTES=4194304 \
                   -DPICO_BOOT_STAGE2_CHOOSE_W25Q080=1
endif

DEVICE_FLAGS += $(TARGET_DEVICE_FLAGS)

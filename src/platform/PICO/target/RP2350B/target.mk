TARGET_MCU        := RP2350B
TARGET_MCU_FAMILY := RP2350

# MCU_FLASH_SIZE may be defined in board-specific config.mk
ifeq ($(MCU_FLASH_SIZE),)
# default to 8MB
MCU_FLASH_SIZE  = 8192
endif

# In pico-sdk, PICO_RP2350A=0 means RP2350B family.
DEVICE_FLAGS    += -DPICO_RP2350A=0

# For pico-sdk, define flash-related attributes
# TARGET_DEVICE_FLAGS may be defined in board-specific config.mk
ifeq ($(TARGET_DEVICE_FLAGS),)
# default to 8388608 = 8 * 1024 * 1024 on W25Q080
TARGET_DEVICE_FLAGS  = \
                   -DPICO_FLASH_SPI_CLKDIV=2 \
                   -DPICO_FLASH_SIZE_BYTES=8388608 \
                   -DPICO_BOOT_STAGE2_CHOOSE_W25Q080=1
endif

DEVICE_FLAGS += $(TARGET_DEVICE_FLAGS)

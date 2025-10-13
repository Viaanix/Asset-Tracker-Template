SHELL = /bin/bash

PROJECT_NAME = tracker
BOARD_NS = nrf9151dk_nrf9151_ns
BOARD = nrf9151dk/nrf9151/ns

ZEPHYR_PROJECT_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ZEPHYR_VIAANIX_BOARD_ROOT ?= $(realpath $(ZEPHYR_PROJECT_PATH)/../../zephyr_boards)
ZEPHYR_BUILD_PATH = $(ZEPHYR_PROJECT_PATH)/_build_$(PROJECT_NAME)

# TARGET intentionally empty
TARGET =

EXTRA_CONF_FILES += ../examples/modules/cloud/overlay-mqtt.conf
EXTRA_CONF_FILES += ../overlay-mqtt.conf

BUILD_FW = \
	west build -p always ./app \
	$(TARGET) \
	--build-dir $(ZEPHYR_BUILD_PATH) \
	-DNCS_TOOLCHAIN_VERSION=NONE \
	-DBOARD_ROOT=$(ZEPHYR_VIAANIX_BOARD_ROOT) \
	-DOVERLAY_CONFIG="boards/$(BOARD_NS).conf" \
	-DDTC_OVERLAY_FILE="boards/$(BOARD_NS).overlay" \
	-DEXTRA_CONF_FILE="$(EXTRA_CONF_FILES)" \
	-DBOARD=$(BOARD); \
	echo ""; \
	printf "%s %02d:%02d:%02d\n" "Total Build Time:" "$$(( $$SECONDS / 3600 ))" "$$(( ( $$SECONDS / 60 ) % 60 ))" "$$(( $$SECONDS % 60 ))"; \
	echo ""

report:
	$(eval TARGET = -t partition_manager_report)
	$(BUILD_FW)

build:
	$(BUILD_FW)

# Flash bootloader and application
flash:
	west flash --skip-rebuild -d $(ZEPHYR_BUILD_PATH) --runner nrfjprog

start-gdb-server:
	JLinkGDBServer -device nrf9160_xxaa \
	-if SWD -speed 8000 \
	-rtos $(RTOS_AWARENESS) \
	-nosinglerun -nosilent -vd

debug:
	arm-none-eabi-gdb-py \
	-ex="cd $(ZEPHYR_BUILD_PATH)" \
	-x $(ZEPHYR_BUILD_PATH)/../gdbinit-application \
	$(ZEPHYR_BUILD_PATH)/tracker/zephyr/zephyr.elf

rtt:
	JLinkRTTClient -RTTTelnetPort 19021

.PHONY: build flash rtt
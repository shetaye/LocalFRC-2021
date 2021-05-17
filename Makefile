ARDUINO_CLI=arduino-cli
BUILD_DIR=_build
BUILD_PATH=$(PWD)/$(BUILD_DIR)
SOURCE_DIR=robot
SOURCE_PATH=$(PWD)/$(SOURCE_DIR)
BOARD_TYPE=arduino:avr:mega
	SERIAL_PORT=/dev/cu.usbmodem1411401

.PHONY: all build program clean

all: build

build:
	$(ARDUINO_CLI) compile --build-path=$(BUILD_PATH) --build-cache-path=$(BUILD_PATH) -b $(BOARD_TYPE) $(SOURCE_PATH)

program:
	$(ARDUINO_CLI) upload -p $(SERIAL_PORT) --fqbn $(BOARD_TYPE) --input-dir=${BUILD_PATH}

clean:
	rm -rf $(BUILD_PATH)
	rm -f $(SOURCE_PATH)/*.elf
	rm -f $(SOURCE_PATH)/*.hex

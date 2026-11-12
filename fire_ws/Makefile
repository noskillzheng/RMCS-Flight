BUILD_DIR := build
SCRIPT_DIR := ../.script

.PHONY: all build flash led clean

all: build

build:
	@$(SCRIPT_DIR)/build-fire

flash: build
	@$(SCRIPT_DIR)/flash-fire

led:
	@$(SCRIPT_DIR)/fire-led

clean:
	@rm -rf $(BUILD_DIR)

CC ?= gcc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Werror -pedantic
BUILD_DIR := build
TEST_BIN := $(BUILD_DIR)/test_bt_uart_protocol

.PHONY: all test clean

all: test

$(TEST_BIN): mcu/src/bt_uart_protocol.c tests/test_bt_uart_protocol.c
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -Imcu/include $^ -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(TEST_BIN)

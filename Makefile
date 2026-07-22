# ZDT_X_V2 闭环步进电机驱动SDK Makefile
#
# Targets:
#   make dll     - 编译 Windows DLL (MinGW)
#   make linux   - 编译 Linux 可执行文件
#   make clean   - 清理

CC      ?= gcc
CFLAGS  ?= -Wall -O2 -I sdk/include

SDK_SRC  = sdk/src/zdt_stepper.c
PLATFORM = examples/platform_win32.c

.PHONY: all dll linux clean

all: dll

dll: sdk/zdt_stepper.dll
sdk/zdt_stepper.dll: $(SDK_SRC) $(PLATFORM)
	$(CC) $(CFLAGS) -DZDT_BUILD_DLL -shared -o $@ $^ -lsetupapi
	@echo "=== DLL built: $@ ==="

linux: build/stepper_test
build/stepper_test: $(SDK_SRC) examples/example_simple.c
	$(CC) $(CFLAGS) -o $@ $^
	@echo "=== Built: $@ ==="

clean:
	rm -f sdk/zdt_stepper.dll build/stepper_test

.PHONY: build clean flash erase	# make sure that these commands are not files


# Settins for target device
CPU := cortex-m4
DEVICE_FAMILY := STM32F4
DEVICE := STM32F411RE

DRIVERS_PATH = /Users/alexivanov/STM32/drivers/$(DEVICE_FAMILY)
CMSIS_INC := $(DRIVERS_PATH)/CMSIS/include
CMSIS_DEVICE_INC := $(DRIVERS_PATH)/CMSIS/Device/ST/STM32F4xx/include

STARTUP_FILE := startup/startup_stm32f411xe.s
LD_FILE := startup/STM32F411RETx_FLASH.ld 

CC_FLAGS := -mcpu=$(CPU) -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
			-fdata-sections -ffunction-sections --specs=nano.specs \
			-Wl,--gc-sections # basic flags
CC_FLAGS += -g3 -O0 # add debug symbols, optimization
CC_FLAGS += -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
			-Wformat-truncation -Wconversion -Wno-deprecated	# warnings
# CC_FLAGS += -v # debug info to terminal 

CPP_FLAGS := $(CC_FLAGS) 


LIBS := m

LD_FLAGS := --print-memory-usage --no-warn-rwx-segment -u _printf_float

INCLUDES := -Iinc -I$(CMSIS_DEVICE_INC) -I$(CMSIS_INC)


# Build binary files tools 
CC := arm-none-eabi-gcc
CPP := arm-none-eabi-g++
AS := arm-none-eabi-as


define build-obj
	@echo "\033[1;33mCompiling "$<" -> "$(notdir $@) "\033[0m"
	@$(CC) $(CC_FLAGS) $(INCLUDES) -c $< -o $@
endef

define build-cpp
	@echo "\033[1;33mCompiling "$<" -> "$(notdir $@) "\033[0m"
	@$(CPP) $(CC_FLAGS) $(INCLUDES) -c $< -o $@
endef

inc/%.h: 
	@echo Missing $@ file 

build/%.o: src/%.cpp inc/%.h 
	$(call build-cpp)

build/%.o: src/%.c inc/%.h 
	$(call build-obj)

build/%.o: startup/%.c 
	$(call build-obj)


build/startup.o: $(STARTUP_FILE)
	@echo "\033[1;33mCompiling $(STARTUP_FILE)\033[0m"
	@$(AS) $(STARTUP_FILE) -o build/startup.o


# Create object files
OBJ_FILES := $(addprefix build/, $(addsuffix .o, $(basename $(notdir $(wildcard src/*))) $(basename $(notdir $(wildcard startup/*.c*))))) 
LD_FLAGS := $(addprefix -Xlinker , $(LD_FLAGS)) 
LIBS := $(addprefix -l, $(LIBS))

build/app.elf: $(OBJ_FILES) build/startup.o $(LD_FILE)
	@echo $(OBJ_FILES)
	@echo "\033[1;33mBuild app.elf\033[0m"
	@$(CC) -Wl,-T,$(LD_FILE) $(LD_FLAGS) $(INCLUDES) $(CC_FLAGS) $(OBJ_FILES) -o build/app.elf build/startup.o $(LIBS)

build/app.bin: build/app.elf
	@echo "\033[1;33mBuild app.bin\033[0m"
	@arm-none-eabi-objcopy -O binary build/app.elf build/app.bin

build: build/app.bin
	@echo "\033[0;32mBuild finished \033[0m"

erase:
	@echo "\033[1;33mErasing\033[0m"
	@st-flash erase 

flash: build
	@st-flash --reset write build/app.bin 0x08000000 

clean:
	@echo "\033[1;33mCleaning\033[0m"
	@rm -rf build/*;

info:
	@st-info --probe
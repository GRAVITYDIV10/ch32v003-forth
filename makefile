SRCS += forth.s

FW_NAME ?= ch32v003-forth

CROSS_COMPILE ?= riscv64-unknown-elf-
CC = $(CROSS_COMPILE)gcc
OD = $(CROSS_COMPILE)objdump
OC = $(CROSS_COMPILE)objcopy
SZ = $(CROSS_COMPILE)size

LINK_SCRIPT ?= link.ld

CFLAGS += \
	-march=rv32ec_zicsr -mabi=ilp32e \
	-mno-relax -nostdlib \
	-x assembler-with-cpp -ggdb \
	-T $(LINK_SCRIPT)

#CFLAGS += -DTEST

all: $(FW_NAME).elf $(FW_NAME).bin $(FW_NAME).hex $(FW_NAME).dis
	$(SZ) $(FW_NAME).elf

clean:
	rm -f *.out $(FW_NAME).dis $(FW_NAME).elf $(FW_NAME).bin $(FW_NAME).hex

$(FW_NAME).hex:
	$(OC) -O ihex $(FW_NAME).elf $(FW_NAME).hex

$(FW_NAME).bin:
	$(OC) -O binary $(FW_NAME).elf $(FW_NAME).bin

$(FW_NAME).elf:
	$(CC) $(CFLAGS) $(SRCS) -o $(FW_NAME).elf

$(FW_NAME).dis: $(FW_NAME).elf
	$(OD) -d -s $(FW_NAME).elf > $(FW_NAME).dis

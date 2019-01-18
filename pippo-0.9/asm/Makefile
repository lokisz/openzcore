########################################################
#
# Notes for windows
#   1, it's not case-sensitive for filename
#   2, using del or cs-rm, instead of rm for linux/unix
#
########################################################

# ELDK@Linux 
#PREFIX =/localhome/fqiu/eldk/usr
#TARGET = powerpc-eabi
#CC = $(PREFIX)/bin/$(TARGET)-gcc
#LD = $(PREFIX)/bin/$(TARGET)-gcc
#OBJCOPY = $(PREFIX)/bin/$(TARGET)-objcopy
#OBJDUMP = $(PREFIX)/bin/$(TARGET)-objdump

# Codesoucery@Windows
TARGET = powerpc-eabi
CC = $(TARGET)-gcc
LD = $(TARGET)-gcc
OBJCOPY = $(TARGET)-objcopy
OBJDUMP = $(TARGET)-objdump

#
# Define locations for the text and data code sections. The bss
# gets tacked on to the end of the data by the linker script,
# don't worry about it.
#

# define this to move from the default of 0xFFF00000
IMAGE_TEXT_START = 0xFFFFE000

# where do you want the text to execute? Define this to move
# from 0x00000000
TEXT_START = 0xFFFFE000

# the data section location defaults to the end of the text section,
# so define these only if you want it in a specific place
# ex. If you're using a real ROM, you need to specify a DATA_START
# that is in RAM so you can actually write to the data space.
#
IMAGE_DATA_START = 0xFFFFC000
DATA_START = 0xFFFFC000

# define options for compilation
# add -gdwarf for debug
# CFLAGS = -g

# define options for linkage. Prevent the inclusion of standard start
# code and libraries.
LDFLAGS = -fno-builtin -nostartfiles -nodefaultlibs -T ld.script

ifdef IMAGE_TEXT_START
LDFLAGS += -Wl,--defsym,TEXT_START=$(TEXT_START) \
	-Wl,--defsym,IMAGE_TEXT_START=$(IMAGE_TEXT_START)
endif
ifdef IMAGE_DATA_START
LDFLAGS += -Wl,--defsym,DATA_START=$(DATA_START) \
	-Wl,--defsym,IMAGE_DATA_START=$(IMAGE_DATA_START)
endif

# define options for the objdump
DUMPFLAGS = --syms --disassemble-all

# list C modules to link with the init code here
# C_SRC = test.c
# C_OBJS = $(C_SRC:.c=.o)

# use variables to refer to init code in case it changes
ASM_TEST = asm_test.o
ASM_TEST_DEP = asm_test.h processor.h asm_test.S

#
# define build targets
#
all: go.srec 

clean:
	del *.o *.elf *.srec *.dump *.i *.bin

# build s-record with init code and c files linked together
go.srec: $(ASM_TEST)
	$(LD) $(LDFLAGS) -o go.elf $(ASM_TEST) 
	$(OBJDUMP) $(DUMPFLAGS) go.elf > go.dump
	$(OBJCOPY) -O binary -R .comment go.elf go.bin
	$(OBJCOPY) -O srec -R .comment go.elf go.srec

# compile init code
$(ASM_TEST): $(ASM_TEST_DEP)
	$(CC) $(CFLAGS) -c -x assembler-with-cpp $*.S
	$(CC) -E -o test.s $*.S

######################################################################
# build s-record with init code and c files linked together
#go.srec: $(C_OBJS) $(ASM_TEST)
#	$(LD) $(LDFLAGS) -o go.elf $(ASM_TEST) $(C_OBJS)
#	$(OBJDUMP) $(DUMPFLAGS) go.elf > go.dump
#	$(OBJCOPY) -O srec -R .comment go.elf go.srec
#
# compile init code
#$(ASM_TEST): $(ASM_TEST_DEP)
#	$(CC) $(CFLAGS) -c -x assembler-with-cpp $*.S
#	$(CC) -E -o asm_test.s $*.S
#
# handle compilation of C files
#%.o:%.c
#	$(CC) $(CFLAGS) -c $<

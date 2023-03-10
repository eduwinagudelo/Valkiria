#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!   
#
##############################################################################################
#
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#
# Included originally in the yagarto projects. Original Author : Michael Fischer
# Modified to suit our purposes by Hussam Al-Hertani
# Use at your own risk!!!!!
##############################################################################################
# Start of default section
#
#TRGT = arm-none-eabi-
#TRGT = arm-linux-gnu-
TRGT = /opt/gcc-arm-none-eabi-5_4-2016q2/bin/arm-none-eabi-
CC   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary -S
MCU  = cortex-m0
 
# List all default C defines here, like -D_DEBUG=1
DDEFS = -DSTM32F0XX -DUSE_STDPERIPH_DRIVER
# List all default ASM defines here, like -D_DEBUG=1
DADEFS =
 
# List all default directories to look for include files here
DINCDIR =
 
# List the default directory to look for the libraries here
DLIBDIR = /usr/lib64/
 
# List all default libraries here
DLIBS = 
#/usr/lib64/
 
#
# End of default section
##############################################################################################
 
##############################################################################################
# Start of user section
#
 
#
# Define project name and Ram/Flash mode here
PROJECT        = valkiria
RUN_FROM_FLASH = 1
 
# List all user C define here, like -D_DEBUG=1
UDEFS =
 
# Define ASM defines here
UADEFS =
 
# List C source files here
LIBSDIR    = /home/edwin/Development/STM32F0-Discovery_FW_V1.0.0/Libraries
CORELIBDIR = $(LIBSDIR)/CMSIS/Include
DEVDIR  = $(LIBSDIR)/CMSIS/ST/STM32F0xx
STMSPDDIR    = $(LIBSDIR)/STM32F0xx_StdPeriph_Driver
STMSPSRCDDIR = $(STMSPDDIR)/src
STMSPINCDDIR = $(STMSPDDIR)/inc
DISCOVERY    = /home/edwin/Development/STM32F0-Discovery_FW_V1.0.0/Utilities/STM32F0-Discovery
LINKER = ./linker
SRC  = ./src/main.c
SRC += ./src/mpu6050.c
SRC += ./src/stm32f0xx_it.c
#SRC += $(DEVDIR)\Source\Templates\system_stm32f0xx.c
SRC += ./src/system_stm32f0xx.c
#SRC += startup_stm32f0xx.S
#SRC += $(DISCOVERY)\stm32f0_discovery.c
## used parts of the STM-Library
#SRC += $(STMSPSRCDDIR)\stm32f0xx_adc.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_cec.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_crc.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_comp.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_dac.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_dbgmcu.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_dma.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_exti.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_flash.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_gpio.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_syscfg.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_i2c.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_iwdg.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_pwr.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_rcc.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_rtc.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_spi.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_tim.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_usart.c
#SRC += $(STMSPSRCDDIR)\stm32f0xx_wwdg.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_misc.c
# List ASM source files here
ASRC = ./startup/startup_stm32f0xx.s
 
# List all user directories here
UINCDIR = $(DEVDIR)/Include \
          $(CORELIBDIR) \
          $(STMSPINCDDIR) \
          $(DISCOVERY)    \
          ./inc     
# List the user directory to look for the libraries here
ULIBDIR = 
#/usr/lib/
 
# List all user libraries here
ULIBS =
#/usr/lib/
 
# Define optimisation level here
OPT = -Os
 
#
# End of user defines
##############################################################################################
#
# Define linker script file here
#
ifeq ($(RUN_FROM_FLASH), 0)
LDSCRIPT = $(LINKER)/stm32f0_linker.ld
FULL_PRJ = $(PROJECT)_ram
else
LDSCRIPT = ./linker/stm32f0_linker.ld
FULL_PRJ = $(PROJECT)_rom
endif
 
INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))
 
ifeq ($(RUN_FROM_FLASH), 0)
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM
else
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=1
endif
 
ADEFS   = $(DADEFS) $(UADEFS)
OBJS  = $(ASRC:.s=.o) $(SRC:.c=.o)
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU)
 
ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb  -specs=nosys.specs -Wa,-amhls=$(<:.s=.lst) $(ADEFS)
CPFLAGS = $(MCFLAGS) $(OPT) -specs=nosys.specs -gdwarf-2 -mthumb   -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
LDFLAGS = $(MCFLAGS) -mthumb -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--no-warn-mismatch $(LIBDIR)
 
# Generate dependency information
CPFLAGS += -MD -MP -MF .dep\$(@F).d
 
#
# makefile rules
#
 
all: $(OBJS) $(FULL_PRJ).elf  $(FULL_PRJ).hex $(FULL_PRJ).bin
ifeq ($(RUN_FROM_FLASH), 0)
	$(TRGT)size $(PROJECT)_ram.elf
else
	$(TRGT)size $(PROJECT)_rom.elf
endif
 
 
%.o: %.c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

 %.o: %.s
	$(AS) -c $(ASFLAGS) $< -o $@

 %elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

 %hex: %elf
	$(HEX) $< $@
	
 %bin: %elf
	$(BIN)  $< $@
	
clean:
	rm $(OBJS)
	rm $(FULL_PRJ).elf
	rm $(FULL_PRJ).map
	rm $(FULL_PRJ).hex
	rm $(FULL_PRJ).bin
#	rm $(SRC:.c=.c.bak)
	rm $(SRC:.c=.lst)
#   rm $(ASRC:.s=.s.bak)
	rm $(ASRC:.s=.lst)
	rm -r  .dep
    
    
# 
# Include the dependency files, should be the last of the makefile
#
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
#23-July-2012 /dev/null gives an error on Win 7 64-bit : Hussam
-include $(shell mkdir .dep) $(wildcard .dep/*)

# *** EOF ***
## Change these:
## HWREV: 
##   0 -> initial revision
MMCU = atxmega16a4
DEFINES = -DF_CPU=2000000 -DHWREV=0

# -b baudrate
AVRDUDE = avrdude -P usb -c jtag3pdi -p $(MMCU)

#-------------------------------------------------------------------------------

HEADERS = twi_master_driver.h twi_slave_driver.h eeprom_driver.h avr_compiler.h
SOURCES = twi_master_driver.c twi_slave_driver.c eeprom_driver.c edid_injector.c
OBJECTS = $(SOURCES:.c=.o)

INCLUDES = -I.

VPATH = .

#-------------------------------------------------------------------------------
CC = avr-gcc
CFLAGS = -W -Wall -Os -std=gnu99 -Werror-implicit-function-declaration

all: firmware.bin firmware.hex

run: all prg

firmware.elf: $(OBJECTS)
	$(CC) -s -mmcu=$(MMCU) $(OBJECTS) -o firmware.elf

%.o: %.c $(HEADERS)
	$(CC) $(INCLUDES) -mmcu=$(MMCU) $(CPPFLAGS) $(CFLAGS) $(DEFINES) -c $<

# FUSEBYTE0: JTAG user ID (arbitrary)
# FUSEBYTE1: 
#    bits 7..4: Watchdog Window Timeout Period
#    bits 3..0: Watchdog Timeout Period
# FUSEBYTE2:
#    bit 6: BOOTRST 
#              0: Reset vector = Boot loader reset
#              1: Reset vector = Application reset (address 0x0000)
#    bit 5: TOSCSEL  (32768 kHz osc pin position; usually 1)
#    bits 1..0:  BODPD[1:0]:  (in power-down mode)
#              10 = BOD enabled continuously
#              11 = BOD disabled
# FUSEBYTE4:
#    bit 4: RSTDISBL   0: disable reset
#    bits 3..2: STARTUPTIME[1:0]    11/01/00: wait 0/4/64 cycles
#    bit 1: WDLOCK     1: watchdog timer not locked
#    bit 0: JTAGEN     1: JTAG disabled
# FUSEBYTE5:
#    bits 5..4: BODACT[1:0]
#              10 = BOD enabled continuously
#              11 = BOD disabled
#    bit 3: EESAVE    0: EEPROM is preserved during chip erase
#    bits 2..0: BODLEVEL[2:0]
#              111: 1.6  011: 2.4
#              110: 1.8  010: 2.6
#              101: 2.0  001: 2.8
#              100: 2.2  000: 3.0
program_fuses:
	$(AVRDUDE) \
	-Ufuse0:w:0xff:m -Ufuse1:w:0x66:m -Ufuse2:w:0xfe:m
	-Ufuse5:w:0xeb:m -Ufuse4:w:0xff:m

prg: firmware.hex
	$(AVRDUDE) -Uflash:w:firmware.hex:i

dump: firmware.elf
	avr-objdump -D firmware.elf

clean:
	-rm -f firmware.bin firmware.elf firmware.hex
	-rm -f *.o

firmware.hex: firmware.elf
	avr-objcopy -j .text -j .data -O ihex firmware.elf firmware.hex

firmware.bin: firmware.elf
	avr-objcopy -j .text -j .data -O binary firmware.elf firmware.bin
	@ls -l firmware.bin


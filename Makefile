DEVICE     = attiny2313
CLOCK      = 1000000
PROGRAMMER = -c usbtiny -p t2313
OBJECTS    = progress-bar.o

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# symbolic targets:
all:	progress-bar.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:progress-bar.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

clean:
	rm -f progress-bar.hex progress-bar.elf $(OBJECTS)

# file targets:
progress-bar.elf: $(OBJECTS)
	$(COMPILE) -o progress-bar.elf $(OBJECTS)

progress-bar.hex: progress-bar.elf
	rm -f progress-bar.hex
	avr-objcopy -j .text -j .data -O ihex progress-bar.elf progress-bar.hex
	avr-size --format=avr --mcu=$(DEVICE) progress-bar.elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	progress-bar.elf
	avr-objdump -d progress-bar.elf

cpp:
	$(COMPILE) -E progress-bar.c

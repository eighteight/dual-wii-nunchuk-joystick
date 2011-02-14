# Name: Makefile
# Project: wii2usb
# Author: Ananda
# Creation Date: 2009-03-11
# Tabsize: 4

UISP = uisp -dprog=stk500 -dpart=atmega168 -dserial=/dev/avr
COMPILE = avr-gcc -Wall -Os -Iusbdrv -I. -mmcu=atmega168 -DF_CPU=16000000L #-DDEBUG_LEVEL=1
COMMON_OBJS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o twi/twi.o includes/buffer.o includes/uart.o  includes/rprintf.o  main.o 

OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o twi/twi.o includes/buffer.o includes/uart.o  includes/rprintf.o  main.o 


# symbolic targets:
all:	wii2usb.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@


clean:
	rm -f wii2usb.hex wii2usb.lst wii2usb.obj wii2usb.cof wii2usb.list wii2usb.map wii2usb.eep.hex wii2usb.bin *.o usbdrv/*.o twi/*.o includes/*.o wii2usb.s usbdrv/oddebug.s usbdrv/usbdrv.s

# file targets:
wii2usb.bin:	$(COMMON_OBJS)
	$(COMPILE) -lm -o wii2usb.bin $(OBJECTS) -Wl,-Map=wii2usb.map

wii2usb.hex:	wii2usb.bin
	rm -f wii2usb.hex wii2usb.eep.hex
	avr-objcopy -j .text -j .data -O ihex wii2usb.bin wii2usb.hex
	./checksize wii2usb.bin

flash:	all
	#$(UISP) --erase --upload --verify if=wii2usb.hex
	$(UISP) --erase --upload if=wii2usb.hex

flash_usb: all
	sudo avrdude -p m168 -P usb -c avrispmkII -Uflash:w:wii2usb.hex -B 1.0

fuse:
	$(UISP) --wr_fuse_h=0xdd --wr_fuse_l=0xff




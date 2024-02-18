#!/bin/bash

avr-gcc -std=gnu99 -DF_CPU=1000000 -mmcu=attiny45 -fdata-sections -ffunction-sections -Wl,-gc-sections -Wa,-a,-ad -Wall -Os spoof.c commands.h commands.c ipod.h ipod.c -o spoof > spoof.lst

avr-size spoof
avr-objcopy -O ihex spoof spoof.hex

echo 'avrdude -p t45 -c bsd -U flash:w:spoof.hex'

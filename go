rm -f micro.o micro.hex micro.elf

echo "C compiler" &&\
avr-gcc -ffunction-sections -fdata-sections -g -O4 -mmcu=at90s2313 -c micro.c &&\
echo "C linker" &&\
avr-gcc -Wl,--gc-sections -mmcu=attiny12 -o micro.elf micro.o &&\
echo "C dumps" &&\
avr-objcopy -j .text -O ihex micro.elf micro.hex &&\
avr-objdump -h -d micro.elf > micro.objdump &&\
echo "C done"

#sudo avrdude -v -p t12 -c usbasp -Uflash:w:micro.hex -Ereset,vcc

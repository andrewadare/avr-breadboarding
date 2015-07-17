# AVR example programs
Most of these programs are closely based on, or at least inspired by, examples from the book [Make: AVR Programming](http://shop.oreilly.com/product/0636920028161.do) by Elliot Williams [(github page)](https://github.com/hexagon5un/AVR-Programming.git). I tended to put comments and names into my own words for self-education. The book has been a fun read, and is needed for circuit diagrams and code explanations.

# Build tools and Makefile
I used `avrdude`, `avr-gcc`, and the rest of the toolchain from [Crosspack for AVR](https://www.obdev.at/products/crosspack/index.html) as my programming environment on OSX.

The Makefile is a modification from the reference above. I acquired several different ATMega328 chips and boards, and used various hardware setups to program them. The default Makefile settings are for my breadboard setup, which is an ATMega328P programmed over USB+ISP using a usbtiny FTDI programmer (I used a Sparkfun pocket AVR programmer). I also played with programming different arduino Uno-type boards over the USB line. For those, I did something like `make uno`, `make nano`, etc. (the serial port URIs are hard-coded in the Makefile). However, many examples are expecting a 1 MHz cpu clock frequency, and won't work correctly on a 16 MHz clock source without code adjustments.

# Serial communication 
I went ahead and reinvented the wheel by writing `simpleterm`, which is almost certainly crappier and less feature-rich than any other serial terminal program out there. (How's that for a raving endorsement?) It depends on ncurses and [libserialport](http://sigrok.org/wiki/Libserialport), which are both small, easy-to-install c libraries.

# My troubleshooting experience
 - When I receive garbage symbols in my terminal from the USART peripheral, the first thing I check for is a baud rate mismatch.
 - With a new hardware setup, I flash `blink.c` with a 1-second blink rate. This immediately identifies any obvious clock speed misconfigurations in the software. `make debug` is sometimes useful too.
 - Uploading code to arduino boards over USB requires a specific upload rate (the `-b` option to `avrdude`). Getting it wrong often results in `stk500_recv()` or `stk500_getsync()` messages. Note that this has nothing to do with the `BAUD` parameter (that's for USART interaction). It can be helpful to upload code from the arduino IDE with the verbosity switched on, in order to see what it uses.
 
# AVR example programs

Most of these programs are based on, or at least inspired by, examples from ["Make: AVR Programming"](http://shop.oreilly.com/product/0636920028161.do) by Elliot Williams [(github page)](https://github.com/hexagon5un/AVR-Programming.git). Many of the comments and names were put into my own words for self-education. The book has been a fun read, and is needed for circuit diagrams and code explanations. 

I used the toolchain from [Crosspack for AVR](https://www.obdev.at/products/crosspack/index.html) on OSX as the programming environment.

I played around with different hardware targets. Mostly, I used an ATMega328P-PU on a breadboard. The Makefile is a modification from the reference above. The default settings are for an ATMega328P programmed over USB+ISP using a usbtiny FTDI programmer (I used a Sparkfun pocket AVR programmer). I also played with different arduino Uno-type boards. For those, use something like `make uno`, `make nano`, etc. (after changing to your serial port URI under the `uno:` target). However, many examples are expecting a 1 MHz cpu clock frequency, and won't work correctly on a 16 MHz clock source without code adjustments.

For serial communication, I went ahead and reinvented the wheel by writing `simpleterm`, which is almost certainly crappier and less feature-rich than any other serial terminal program out there. (How's that for a raving endorsement?) It depends on ncurses and [libserialport](http://sigrok.org/wiki/Libserialport), which are both small, easy-to-install c libraries.


# AVR example programs

Most of these programs are based on, or at least inspired by, examples from "Make: AVR Programming" by Elliot Williams [(github page)](https://github.com/hexagon5un/AVR-Programming.git). The book is needed for circuit diagrams, parts, and code explanations. Many comments and names were put into my own words for self-education.

I used the toolchain from [Crosspack for AVR](https://www.obdev.at/products/crosspack/index.html) on OSX as the programming environment.

I played around with different hardware targets. Mostly, I used an ATMega328P-PU on a breadboard. The default Makefile settings are for an ATMega328P programmed over USB+ISP using a usbtiny FTDI programmer (I used a Sparkfun pocket AVR programmer). I also played with different arduino Uno-type boards. For those, use `make uno`, `make nano`, etc. (after changing to your serial port URI under the `uno:` target). However, many examples are expecting a 1 MHz cpu clock frequency, and won't work correctly on a 16 MHz clock source without code adjustments.

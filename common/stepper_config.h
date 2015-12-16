// Definitions for stepper motor and driver
// Timer 0 or 2 can be used, but not (yet) both. Any pins in the port will do. 

// #define USE_TIMER_0_FOR_STEPPER 1
// #define STEPPER_PORT PORTB
// #define STEPPER_DDR  DDRB
// #define STEPPER_VECT TIMER0_COMPA_vect
// #define STEP_PIN     PB0    // Arduino 8
// #define DIR_PIN      PB1    // Arduino 9

#define USE_TIMER_2_FOR_STEPPER 1
#define STEPPER_VECT TIMER2_COMPA_vect
#define STEPPER_PORT PORTD
#define STEPPER_DDR  DDRD
#define STEP_PIN     PD2        // Arduino 2
#define DIR_PIN      PD3        // Arduino 3
#define ONE_REV      400        // Steps/rev - 0.9 deg / step

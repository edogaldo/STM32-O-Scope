
#define TEST_WAVE_PIN       PB1       //PB1 PWM 500 Hz 
// Analog input
#define ANALOG_IN_PIN1      PB0
#define ANALOG_IN_PIN2      PA4
// refer to array boardADCPins[] in variants/<board>/board.cpp for available analog input pins

#define ANALOG_MAX_VALUE    4096

#define MAX_SAMPLES         1024*24   // MAX_SAMPLES depends on available RAM - each sample takes 2 bytes
// 1024*6  =  6144 samples = 12KB is about the limit on an STM32F103C8T6 (20KB ram available)
// 1024*24 = 24576 samples = 48KB is about the limit on an STM32F103ZET6 (64KB ram available)
// Bear in mind that the ILI9341 display is only able to display 240x320 pixels, at any time but we can output far more to the serial port, we effectively only show a window on our samples on the TFT.

//#define USE_ILI9341
//#define USE_TOUCH_SCREEN

#if defined(USE_ILI9341)

#include "ILI9341.h"

#endif



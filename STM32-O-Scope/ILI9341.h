#include "Adafruit_ILI9341_STM.h"
#include "Adafruit_GFX_AS.h"

// Be sure to use the latest version of the SPI libraries see stm32duino.com - http://stm32duino.com/viewtopic.php?f=13&t=127
#include <SPI.h>

#define PORTRAIT 0
#define LANDSCAPE 1


// Initialize touchscreen
// ----------------------
// Set the pins to the correct ones for your STM32F103 board
// -----------------------------------------------------------

//
// STM32F103C8XX Pin numbers - chosen for ease of use on the "Red Pill" and "Blue Pill" board

// Touch Panel Pins
// T_CLK T_CS T_DIN T_DOUT T_IRQ
// PB12 PB13 PB14 PB15 PA8
// Example wire colours Brown,Red,Orange,Yellow,Violet
// --------             Brown,Red,Orange,White,Grey


// Also define the orientation of the touch screen. Further
// information can be found in the UTouch library documentation.
// 


#if defined(USE_TOUCH_SCREEN)

#define TOUCH_SCREEN_AVAILABLE
#define TOUCH_ORIENTATION  LANDSCAPE
// URTouch Library
// http://www.rinkydinkelectronics.com/library.php?id=93
#include <URTouch.h>
URTouch  myTouch( PB12, PB13, PB14, PB15, PA8);

// #define NVRam register names for the touch calibration values.
#define  TOUCH_CALIB_X 0
#define  TOUCH_CALIB_Y 1
#define  TOUCH_CALIB_Z 2

#endif



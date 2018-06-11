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

//#if defined(USE_ILI9341)
// Additional  display specific signals (i.e. non SPI) for STM32F103C8T6 (Wire colour)
#define TFT_DC      PA0      //   (Green) 
#define TFT_CS      PA1      //   (Orange) 
#define TFT_RST     PA2      //   (Yellow)
#define TFT_LED     PA3      // Backlight 
// Display colours
#define BEAM1_COLOUR       ILI9341_GREEN
#define BEAM2_COLOUR       ILI9341_RED
#define GRATICULE_COLOUR   0x07FF
#define BEAM_OFF_COLOUR    ILI9341_BLACK
#define CURSOR_COLOUR      ILI9341_GREEN

// Create the lcd object
Adafruit_ILI9341_STM TFT = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST); // Using hardware SPI

// Variables for the beam position
uint16_t signalX ;
uint16_t signalY ;
uint16_t signalY1;

//#endif


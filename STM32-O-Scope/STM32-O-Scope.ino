/*.
(c) Andrew Hull - 2015

STM32-O-Scope - aka "The Pig Scope" or pigScope released under the GNU GENERAL PUBLIC LICENSE Version 2, June 1991

https://github.com/pingumacpenguin/STM32-O-Scope

Adafruit Libraries released under their specific licenses Copyright (c) 2013 Adafruit Industries.  All rights reserved.

*/

#include "STM32-O-Scope.h"


// RTC and NVRam initialisation

#if defined(USE_RTC)
#include "RTClock.h"
RTClock rt (RTCSEL_LSE); // initialise
#endif
uint32 tt;

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/dma.h>
#include <libmaple/pwr.h>
#include <libmaple/scb.h>
#include <libmaple/dac.h>


#if defined(USE_TIME_H)
// Time library - https://github.com/PaulStoffregen/Time
#include "Time.h" //If you have issues with the default Time library change the name of this library to Time1 for example.
#endif

#define TZ    UTC+1

// End RTC and NVRam initialization

// SeralCommand -> https://github.com/kroimon/Arduino-SerialCommand.git
#include <SerialCommand.h>

/* For reference on STM32F103CXXX

variants/generic_stm32f103c/board/board.h:#define BOARD_NR_SPI              2
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_NSS_PIN        PA4
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_MOSI_PIN       PA7
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_MISO_PIN       PA6
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_SCK_PIN        PA5

variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_NSS_PIN        PB12
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_MOSI_PIN       PB15
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_MISO_PIN       PB14
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_SCK_PIN        PB13


// Hardware SPI1 on the STM32F103C8T6 *ALSO* needs to be connected and pins are as follows.
//
// SPI1_NSS  (PA4) (LQFP48 pin 14)    (n.c.)
// SPI1_SCK  (PA5) (LQFP48 pin 15)    (Brown)
// SPI1_MOSO (PA6) (LQFP48 pin 16)    (White)
// SPI1_MOSI (PA7) (LQFP48 pin 17)    (Grey)
//

*/

// Screen dimensions
int16_t myWidth ;
int16_t myHeight ;


float samplingTime = 0;
float displayTime = 0;


// Variables for the beam position
int16_t xZoomFactor = 1;
// yZoomFactor (percentage)
int16_t yZoomFactor = 100; //Adjusted to get 3.3V wave to fit on screen
int16_t yPosition = 0 ;


unsigned long sweepDelayFactor = 1;
unsigned long timeBase = 200;  //Timebase in microseconds

//Trigger stuff
boolean triggered ;

// Sensitivity is the necessary change in AD value which will cause the scope to trigger.
// If VAD=3.3 volts, then 1 unit of sensitivity is around 0.8mV but this assumes no external attenuator. Calibration is needed to match this with the magnitude of the input signal.

// Trigger is setup in one of 32 positions
#define TRIGGER_POSITION_STEP ANALOG_MAX_VALUE/32
// Trigger default position (half of full scale)
int32_t triggerValue = 2048; 

int16_t retriggerDelay = 0;
int8_t triggerType = 2; //0-both 1-negative 2-positive

//Array for trigger points
uint16_t triggerPoints[2];


// Create Serial Command Object.
SerialCommand sCmd;

// Create USB serial port
#define serial_debug Serial

uint32_t startSample = 0; //10
uint32_t endSample = MAX_SAMPLES ;

// Array for the ADC data
uint32_t dataPoints32[MAX_SAMPLES / 2];
uint16_t *dataPoints = (uint16_t *)&dataPoints32;

//array for computed data (speedup)
uint16_t dataPlot[320]; //max(width,height) for this display


// End of DMA indication
volatile static bool dma1_ch1_Active;

// ADC DUAL MODE bits CR1[19-16]

/*
void manualSamples();
void trigger();
void toggleEdgeType();
void increaseTriggerPosition();
void decreaseTriggerPosition();
void decreaseTimebase();
void increaseTimebase();
void decreaseZoomFactor();
void increaseZoomFactor();
void scrollRight();
void scrollLeft();
void decreaseYposition();
void increaseYposition();
*/

void setup()
{

#if defined(LED_BUILTIN)
  // LED_BUILTIN blinks on triggering assuming you have an LED on your board. If not simply dont't define it at the start of the sketch.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
#endif
#if defined(USE_ILI9341)
  // Backlight, use with caution, depending on your display, you may exceed the max current per pin if you use this method.
  // A safer option would be to add a suitable transistor capable of sinking or sourcing 100mA (the ILI9341 backlight on my display is quoted as drawing 80mA at full brightness)
  // Alternatively, connect the backlight to 3v3 for an always on, bright display.
  pinMode(TFT_LED, OUTPUT);
  //analogWrite(TFT_LED, 127);
  digitalWrite(TFT_LED, HIGH);
#endif

  serial_debug.begin(115200);
  setADCs (); //Setup ADC peripherals for interleaved continuous mode.

  addSerialCommands();


  // Setup Touch Screen
  // http://www.rinkydinkelectronics.com/library.php?id=56
#if defined(TOUCH_SCREEN_AVAILABLE)
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_EXTREME);
#endif


  // The test pulse is a square wave of approx 3.3V (i.e. the STM32 supply voltage) at approx 1 kHz
  // "The Arduino has a fixed PWM frequency of 490Hz" - and it appears that this is also true of the STM32F103 using the current STM32F03 libraries as per
  // STM32, Maple and Maple mini port to IDE 1.5.x - http://forum.arduino.cc/index.php?topic=265904.2520
  // therefore if we want a precise test frequency we can't just use the default uncooked 50% duty cycle PWM output.
  //timer_set_period(Timer3, 1000);
  //toggleTestPulseOn();
  analogWrite(ANALOG_IN_PIN1, 200);
  //analogWriteDAC(uint8 DAC_Channel,int val,dac_wave wave,dac_mamp mamp)
  //analogWriteDAC(1, 0x0, WAVE_DISABLED, MAMP_1023);
  analogWriteDAC(1, 0x0, WAVE_TRIANGLE, MAMP_4095);
  //analogWriteDAC(2, 0x0, WAVE_TRIANGLE, MAMP_1023);
  //pinMode(ANALOG_IN_PIN2, OUTPUT);
  //digitalWrite(ANALOG_IN_PIN2,LOW);

  // Set up our sensor pin(s)
  //pinMode(ANALOG_IN_PIN1, INPUT_ANALOG);
  //pinMode(ANALOG_IN_PIN2, INPUT_ANALOG);
  
#if defined(USE_ILI9341)
  initTFT();
#endif
}

void loop()
{

#if defined(TOUCH_SCREEN_AVAILABLE)
  readTouch();
#endif

  sCmd.readSerial();     // Process serial commands
  
#if defined(USE_ILI9341)

    if ( triggered )
    {
      blinkLED();

      // Take our samples
      //takeSamples();
      
      //Blank  out previous plot
      TFTSamplesClear(BEAM_OFF_COLOUR);

      // Show the showGraticule
      showGraticule();

      //Display the samples
      TFTSamples(BEAM1_COLOUR);

      displayTime = (micros() - displayTime);
      
      // Display the Labels ( uS/Div, Volts/Div etc).
      showLabels();
	  
      displayTime = micros();

    }else {
      showGraticule();
    }
    // Display the RTC time.
#if defined(USE_RTC) && defined(USE_TIME_H)
    showTime();
#endif
#endif

  // Wait before allowing a re-trigger
  delay(retriggerDelay);
  // DEBUG: increment the sweepDelayFactor slowly to show the effect.
  // sweepDelayFactor ++;
}

void addSerialCommands() {
  //
  // Serial command setup
  // Setup callbacks for SerialCommand commands
#if defined(USE_TIME_H)
  sCmd.addCommand("timestamp",   setCurrentTime);          // Set the current time based on a unix timestamp
  sCmd.addCommand("date",        serialCurrentTime);       // Show the current time from the RTC
#endif
  sCmd.addCommand("sleep",       goToSleep);               // Experimental - puts system to sleep

#if defined(TOUCH_SCREEN_AVAILABLE)
  sCmd.addCommand("touchcalibrate", touchCalibrate);       // Calibrate Touch Panel
#endif

  sCmd.addCommand("m",   manualSamples);                   // Manual samples
  sCmd.addCommand("a",   trigger);                         // Trgger based samples
  sCmd.addCommand("e",   toggleEdgeType);                  // change trigger type: 0 - triggerBoth ; 1 - triggerNegative ; 2 - triggerPositive
  sCmd.addCommand("d",   dumpSamples);                     // Print samples to serial
  sCmd.addCommand("x",   dumpOptions);
  
  sCmd.addCommand("t",   decreaseTimebase);                // decrease Timebase by 10x
  sCmd.addCommand("T",   increaseTimebase);                // increase Timebase by 10x
  
  sCmd.addCommand("z",   decreaseZoomFactor);              // decrease Zoom
  sCmd.addCommand("Z",   increaseZoomFactor);              // increase Zoom
  
  sCmd.addCommand("r",   scrollRight);                     // start onscreen trace further right
  sCmd.addCommand("l",   scrollLeft);                      // start onscreen trae further left
  
  sCmd.addCommand("y",   decreaseYposition);               // move trace Down
  sCmd.addCommand("Y",   increaseYposition);               // move trace Up
  
  sCmd.addCommand("g",   decreaseTriggerPosition);         // move trigger position Down
  sCmd.addCommand("G",   increaseTriggerPosition);         // move trigger position Up
  
  sCmd.addCommand("P",   toggleTestPulseOn);               // Toggle the test pulse pin from high impedence input to square wave output.
  sCmd.addCommand("p",   toggleTestPulseOff);              // Toggle the Test pin from square wave test to high impedence input.

  sCmd.setDefaultHandler(unrecognized);                    // Handler for command that isn't matched  (says "Unknown")
  sCmd.clearBuffer();

}

void blinkLED() {
#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
#endif
}

void dumpSamples ()
{
  // Send *all* of the samples to the serial port.
  serial_debug.println("CH1\tCH2");
  for (int16_t j = 0; j < MAX_SAMPLES   ; j+=2 )
  {
    // Time from trigger in milliseconds
    serial_debug.print(dataPoints[j]);
    serial_debug.print("\t");
    serial_debug.println(dataPoints[j+1]);

  }
  serial_debug.print("\n");
}

void dumpOptions()
{
	serial_debug.print("triggerType: ");serial_debug.println(triggerType);
	serial_debug.println();
}

void unrecognized(const char *command) {
  serial_debug.print("# Unknown Command.[");
  serial_debug.print(command);
  serial_debug.println("]");
}

void toggleTestPulseOn () {
  pinMode(TEST_WAVE_PIN, OUTPUT);
  analogWrite(TEST_WAVE_PIN, 250);
  serial_debug.println("# Test Pulse On.");
}

void toggleTestPulseOff () {
  pinMode(TEST_WAVE_PIN, INPUT);
  serial_debug.println("# Test Pulse Off.");
}

uint16 timer_set_period(HardwareTimer timer, uint32 microseconds) {
  if (!microseconds) {
    timer.setPrescaleFactor(1);
    timer.setOverflow(1);
    return timer.getOverflow();
  }

  uint32 cycles = microseconds * (72000000 / 1000000); // 72 cycles per microsecond

  uint16 ps = (uint16)((cycles >> 16) + 1);
  timer.setPrescaleFactor(ps);
  timer.setOverflow((cycles / ps) - 1 );
  return timer.getOverflow();
}

/**
* @brief Enable DMA requests
* @param dev ADC device on which to enable DMA requests
*/
void adc_dma_enable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 1);
}

/**
* @brief Disable DMA requests
* @param dev ADC device on which to disable DMA requests
*/
void adc_dma_disable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 0);
}

static void DMA1_CH1_Event() {
  dma1_ch1_Active = 0;
}

#if defined(USE_TIME_H)
void setCurrentTime() {
  char *arg;
  arg = sCmd.next();
  String thisArg = arg;
  serial_debug.print("# Time command [");
  serial_debug.print(thisArg.toInt() );
  serial_debug.println("]");
  setTime(thisArg.toInt());
#if defined(USE_RTC)
  time_t tt = now();
  rt.setTime(tt);
#endif
  serialCurrentTime();
}

void serialCurrentTime() {
  serial_debug.print("# Current time - ");
  if (hour(tt) < 10) {
    serial_debug.print("0");
  }
  serial_debug.print(hour(tt));
  serial_debug.print(":");
  if (minute(tt) < 10) {
    serial_debug.print("0");
  }
  serial_debug.print(minute(tt));
  serial_debug.print(":");
  if (second(tt) < 10) {
    serial_debug.print("0");
  }
  serial_debug.print(second(tt));
  serial_debug.print(" ");
  serial_debug.print(day(tt));
  serial_debug.print("/");
  serial_debug.print(month(tt));
  serial_debug.print("/");
  serial_debug.print(year(tt));
  serial_debug.println("(TZ)");

}
#endif

void goToSleep()
{
  serial_debug.println("# Nighty night!");
  // Set PDDS and LPDS bits for standby mode, and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR |= PWR_CR_CWUF;
  PWR_BASE->CR |= PWR_CR_PDDS;

  // set sleepdeep in the system control register
  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

  // Now go into stop mode, wake up on interrupt
  // disableClocks();
  asm("wfi");
}



/* NOT USED

// Define the Base address of the RTC  registers (battery backed up CMOS Ram), so we can use them for config of touch screen and other calibration.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.

#define BKP_REG_BASE   (uint32_t *)(0x40006C00 +0x04)

static inline int readBKP(int registerNumber)
{
  if (registerNumber > 9)
  {
    registerNumber += 5; // skip over BKP_RTCCR,BKP_CR,BKP_CSR and 2 x Reserved registers
  }
  return *(BKP_REG_BASE + registerNumber) & 0xffff;
}

static inline void writeBKP(int registerNumber, int value)
{
  if (registerNumber > 9)
  {
    registerNumber += 5; // skip over BKP_RTCCR,BKP_CR,BKP_CSR and 2 x Reserved registers
  }

  *(BKP_REG_BASE + registerNumber) = value & 0xffff;
}
*/



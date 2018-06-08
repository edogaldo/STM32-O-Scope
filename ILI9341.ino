#if defined(USE_ILI9341)

void initTFT() {
  TFT.begin();
  // initialize the display
  clearTFT();
  TFT.setRotation(PORTRAIT);
  myHeight   = TFT.width() ;
  myWidth  = TFT.height();
  TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR) ;
#if defined(TOUCH_SCREEN_AVAILABLE)
  touchCalibrate();
#endif
  TFT.setRotation(LANDSCAPE);
  clearTFT();
//  showGraticule();
  showCredits(); // Honourable mentions ;¬)
  delay(1000) ; //5000
  clearTFT();
  triggered = false;
  showGraticule();
  showLabels();
}

void showGraticule()
{
  TFT.drawRect(0, 0, myHeight, myWidth, GRATICULE_COLOUR);
  // Dot grid - ten distinct divisions (9 dots) in both X and Y axis.
  for (uint16_t TicksX = 1; TicksX < 10; TicksX++)
  {
    for (uint16_t TicksY = 1; TicksY < 10; TicksY++)
    {
      TFT.drawPixel(  TicksX * (myHeight / 10), TicksY * (myWidth / 10), GRATICULE_COLOUR);
    }
  }
  // Horizontal and Vertical centre lines 5 ticks per grid square with a longer tick in line with our dots
  for (uint16_t TicksX = 0; TicksX < myWidth; TicksX += (myHeight / 50))
  {
    if (TicksX % (myWidth / 10) > 0 )
    {
      TFT.drawFastHLine(  (myHeight / 2) - 1 , TicksX, 3, GRATICULE_COLOUR);
    }
    else
    {
      TFT.drawFastHLine(  (myHeight / 2) - 3 , TicksX, 7, GRATICULE_COLOUR);
    }

  }
  for (uint16_t TicksY = 0; TicksY < myHeight; TicksY += (myHeight / 50) )
  {
    if (TicksY % (myHeight / 10) > 0 )
    {
      TFT.drawFastVLine( TicksY,  (myWidth / 2) - 1 , 3, GRATICULE_COLOUR);
    }
    else
    {
      TFT.drawFastVLine( TicksY,  (myWidth / 2) - 3 , 7, GRATICULE_COLOUR);
    }
  }
}

void clearTFT()
{
  TFT.fillScreen(BEAM_OFF_COLOUR);                // Blank the display
}

void TFTSamplesClear (uint16_t beamColour)
{
  for (signalX=1 ; signalX < myWidth - 2; signalX++)
  {
    //use saved data to improve speed
    TFT.drawLine (  dataPlot[signalX-1], signalX, dataPlot[signalX] , signalX + 1, beamColour) ;
  }
}

void TFTSamples (uint16_t beamColour)
{
  //calculate first sample
  signalY =  ((myHeight * dataPoints[0 * ((endSample - startSample) / (myWidth * timeBase / 100)) + 1]) / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition;
  dataPlot[0]=signalY * 99 / 100 + 1;
  
  for (signalX=1 ; signalX < myWidth - 2; signalX++)
  {
    // Scale our samples to fit our screen. Most scopes increase this in steps of 5,10,25,50,100 250,500,1000 etc
    // Pick the nearest suitable samples for each of our myWidth screen resolution points
    signalY1 = ((myHeight * dataPoints[(signalX + 1) * ((endSample - startSample) / (myWidth * timeBase / 100)) + 1]) / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition ;
    dataPlot[signalX] = signalY1 * 99 / 100 + 1;
    TFT.drawLine (  dataPlot[signalX-1], signalX, dataPlot[signalX] , signalX + 1, beamColour) ;
    signalY = signalY1;
  }
}

/*
// Run a bunch of NOOPs to trim the inter ADC conversion gap
void sweepDelay(unsigned long sweepDelayFactor) {
  volatile unsigned long i = 0;
  for (i = 0; i < sweepDelayFactor; i++) {
    __asm__ __volatile__ ("nop");
  }
}
*/

void showLabels()
{
  TFT.setRotation(LANDSCAPE);
  TFT.setTextSize(1);
  TFT.setCursor(10, 190);
  // TFT.print("Y=");
  //TFT.print((samplingTime * xZoomFactor) / MAX_SAMPLES);
  TFT.print(float (float(samplingTime) / float(MAX_SAMPLES)));

  TFT.setTextSize(1);
  TFT.print(" uS/Sample  ");
  TFT.print(MAX_SAMPLES);
  TFT.print(" samples ");
//  TFT.setCursor(10, 190);
//  TFT.print(displayTime);
  TFT.print(float (1000000 / float(displayTime)));
  TFT.print(" fps    ");
  TFT.setTextSize(2);
  TFT.setCursor(10, 210);
  TFT.print("0.3");
  TFT.setTextSize(1);
  TFT.print(" V/Div ");
  TFT.setTextSize(1);

  TFT.print("timeBase=");
  TFT.print(timeBase);
  TFT.print(" yzoom=");
  TFT.print(yZoomFactor);
  TFT.print(" ypos=");
  TFT.print(yPosition);
  //showTime();
  TFT.setRotation(PORTRAIT);
}

#if defined(USE_RTC) && defined(USE_TIME_H)
void showTime ()
{
  // Show RTC Time.
  TFT.setTextSize(1);
  TFT.setRotation(LANDSCAPE);
  if (rt.getTime() != tt)
  {
    tt = rt.getTime();
    TFT.setCursor(5, 10);
    if (hour(tt) < 10) {
      TFT.print("0");
    }
    TFT.print(hour(tt));
    TFT.print(":");
    if (minute(tt) < 10) {
      TFT.print("0");
    }
    TFT.print(minute(tt));
    TFT.print(":");
    if (second(tt) < 10) {
      TFT.print("0");
    }
    TFT.print(second(tt));
    TFT.print(" ");
    TFT.print(day(tt));
    TFT.print("-");
    TFT.print(month(tt));
    TFT.print("-");
    TFT.print(year(tt));
    TFT.print(" TZ ");
    // TFT.print(tt);

  }
  TFT.setRotation(PORTRAIT);
}
#endif

void clearTrace() {
  TFTSamples(BEAM_OFF_COLOUR);
  showGraticule();
}

void showTrace() {
  showLabels();
  TFTSamples(BEAM1_COLOUR);
}

void showCredits() {
  TFT.setTextSize(2);                           // Small 26 char / line
  //TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR) ;
  TFT.setCursor(0, 50);
  TFT.print(" STM-O-Scope by Andy Hull") ;
  TFT.setCursor(0, 70);
  TFT.print("      Inspired by");
  TFT.setCursor(0, 90);
  TFT.print("      Ray Burnette.");
  TFT.setCursor(0, 130);
  TFT.print("      Victor PV");
  TFT.setCursor(0, 150);
  TFT.print("      Roger Clark");
  TFT.setCursor(0, 170);
  TFT.print(" and all at stm32duino.com");
  TFT.setCursor(0, 190);
  TFT.print(" CH1 Probe STM32F Pin [");
  TFT.print(ANALOG_IN_PIN1);
  TFT.print("]");
  TFT.setCursor(0, 220);
  TFT.setTextSize(1);
  TFT.print("     GNU GENERAL PUBLIC LICENSE Version 2 ");
  TFT.setTextSize(2);
  TFT.setRotation(PORTRAIT);
}

#if defined(TOUCH_SCREEN_AVAILABLE)
void touchCalibrate() {
  // showGraticule();
  for (uint8_t screenLayout = 0 ; screenLayout < 4 ; screenLayout += 1)
  {
    TFT.setRotation(screenLayout);
    TFT.setCursor(0, 10);
    TFT.print("  Press and hold centre circle ");
    TFT.setCursor(0, 20);
    TFT.print("   to calibrate touch panel.");
  }
  TFT.setRotation(PORTRAIT);
  TFT.drawCircle(myHeight / 2, myWidth / 2, 20, GRATICULE_COLOUR);
  TFT.fillCircle(myHeight / 2, myWidth / 2, 10, BEAM1_COLOUR);
  //delay(5000);
  readTouchCalibrationCoordinates();
  clearTFT();
}

void readTouchCalibrationCoordinates()
{
  int calibrationTries = 6000;
  int failCount = 0;
  int thisCount = 0;
  uint32_t tx = 0;
  uint32_t ty = 0;
  boolean OK = false;

  while (OK == false)
  {
    while ((myTouch.dataAvailable() == false) && thisCount < calibrationTries) {
      thisCount += 1;
      delay(1);
    }
    if ((myTouch.dataAvailable() == false)) {
      return;
    }
    // myGLCD.print("*  HOLD!  *", CENTER, text_y_center);
    thisCount = 0;
    while ((myTouch.dataAvailable() == true) && (thisCount < calibrationTries) && (failCount < 10000))
    {
      myTouch.calibrateRead();
      if (!((myTouch.TP_X == 65535) || (myTouch.TP_Y == 65535)))
      {
        tx += myTouch.TP_X;
        ty += myTouch.TP_Y;
        thisCount++;
      }
      else
        failCount++;
    }
    if (thisCount >= calibrationTries)
    {
      for (thisCount = 10 ; thisCount < 100 ; thisCount += 10)
      {
        TFT.drawCircle(myHeight / 2, myWidth / 2, thisCount, GRATICULE_COLOUR);
      }
      delay(500);
      OK = true;
    }
    else
    {
      tx = 0;
      ty = 0;
      thisCount = 0;
    }
    if (failCount >= 10000)
      // Didn't calibrate so just leave calibration as is.
      return;
  }
  serial_debug.print("# Calib x: ");
  serial_debug.println(tx / thisCount, HEX);
  serial_debug.print("# Calib y: ");
  serial_debug.println(ty / thisCount, HEX);
  // Change calibration data from here..
  // cx = tx / iter;
  // cy = ty / iter;
}

void readTouch() {

  if (myTouch.dataAvailable())
  {
    myTouch.read();
    // Note: This is corrected to account for different orientation of screen origin (x=0,y=0) in Adafruit lib from UTouch lib
    uint32_t touchY = myWidth - myTouch.getX();
    uint32_t touchX = myTouch.getY();
    //

    serial_debug.print("# Touched ");
    serial_debug.print(touchX);
    serial_debug.print(",");
    serial_debug.println(touchY);

    TFT.drawPixel(touchX, touchY, BEAM2_COLOUR);
  }
}

#endif

#endif

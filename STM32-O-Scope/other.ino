void increaseZoomFactor() {
  if ( xZoomFactor < 21) 
  {
#if defined(USE_ILI9341)
    clearTrace();
#endif
    xZoomFactor += 1;
#if defined(USE_ILI9341)
    showTrace();
#endif
  }
  serial_debug.print("# Zoom=");
  serial_debug.println(xZoomFactor);

}

void decreaseZoomFactor() {
  if (xZoomFactor > 1) {
#if defined(USE_ILI9341)
    clearTrace();
#endif
    xZoomFactor -= 1;
#if defined(USE_ILI9341)
    showTrace();
#endif
  }
  serial_debug.print("# Zoom=");
  serial_debug.println(xZoomFactor);
  //clearTFT();
}

void scrollRight() {
  if (startSample < (endSample - 120)) {
#if defined(USE_ILI9341)
    clearTrace();
#endif
    startSample += 100;
#if defined(USE_ILI9341)
    showTrace();
#endif
  }
  serial_debug.print("# startSample=");
  serial_debug.println(startSample);


}

void scrollLeft() {
  if (startSample > (120)) {
#if defined(USE_ILI9341)
    clearTrace();
#endif
    startSample -= 100;
#if defined(USE_ILI9341)
    showTrace();
#endif
  }
  serial_debug.print("# startSample=");
  serial_debug.println(startSample);

}

void increaseYposition() {
  if (yPosition < myHeight ) {
#if defined(USE_ILI9341)
    clearTrace();
#endif
    yPosition ++;
#if defined(USE_ILI9341)
    showTrace();
#endif
  }
  serial_debug.print("# yPosition=");
  serial_debug.println(yPosition);
}

void decreaseYposition() {

  if (yPosition > -myHeight ) {
#if defined(USE_ILI9341)
    clearTrace();
#endif
    yPosition --;
#if defined(USE_ILI9341)
    showTrace();
#endif
  }
  serial_debug.print("# yPosition=");
  serial_debug.println(yPosition);
}

void decreaseTimebase() {
  if (timeBase > 100)
  {
#if defined(USE_ILI9341)
  clearTrace();
#endif
    timeBase -= 100;
#if defined(USE_ILI9341)
  showTrace();
#endif
  }
  serial_debug.print("# Timebase=");
  serial_debug.println(timeBase);

}

void increaseTimebase() {
  if (timeBase < 10000)
  {
#if defined(USE_ILI9341)
  clearTrace();
#endif
    timeBase += 100;
#if defined(USE_ILI9341)
  showTrace();
#endif
  }
  //sweepDelayFactor = 2 * sweepDelayFactor ;
  serial_debug.print("# Timebase=");
  serial_debug.println(timeBase);
}

void atAt() {
  serial_debug.println("# Hello");
}




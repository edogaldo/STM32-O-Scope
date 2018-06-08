extern HardwareTimer Timer6;

void analogWriteDAC(uint8 DAC_Channel,int val,dac_wave wave,dac_mamp mamp) {
  // configure Timer6.
  Timer6.pause();
  //Timer6.setPeriod(1);
  Timer6.setPrescaleFactor(1);
  Timer6.setOverflow(10);
  Timer6.setMasterModeTrGo(2<<4);
  //Timer6.attachInterrupt(TIMER_UPDATE_INTERRUPT,tim6_IRQ_handler);
  // configure DAC.
  DAC->regs->CR=0; // reset CR
  rcc_clk_enable(RCC_DAC);
  rcc_reset_dev(RCC_DAC);
  switch(DAC_Channel) {
    case 1:
      //gpio_set_mode(GPIOA, 4, GPIO_MODE_ANALOG);
      pinMode(PA4, OUTPUT);
      DAC->regs->CR&=0xFFFF<<16; // reset CR[15:0]
      DAC->regs->CR|=(mamp<<8)|(wave<<6)|(0b000<<3)|(1<<2);
      DAC->regs->DHR12R1=val;
      DAC->regs->CR|=1; //enable CH1
      break;
    case 2:
      //gpio_set_mode(GPIOA, 5, GPIO_MODE_ANALOG);
      pinMode(PA5, OUTPUT);
      DAC->regs->CR&=0xFFFF; // reset CR [31:16]
      DAC->regs->CR|=((mamp<<8)|(wave<<6)|(0b000<<3)|(1<<2))<<16;
      DAC->regs->DHR12R2=val;
      DAC->regs->CR|=1<<16; //enable CH2
  }
  dumpDevice((uint32*)DAC_BASE,sizeof(typeof *DAC_BASE)/sizeof(uint32),"DAC");
  dumpDevice((uint32*)TIMER6_BASE,sizeof(typeof *TIMER6_BASE)/sizeof(uint32),"Timer6");
  //dumpTimer(Timer6.c_dev());
  //dumpDAC(DAC);
  Timer6.resume();
}

void dumpDevice(uint32 *dev,int numRegs,const char *devName) {
  serial_debug.print(devName);serial_debug.println(": ");
  for(int i=0;i<numRegs;i++) {
    serial_debug.print((uint64)(dev+i),HEX,8);serial_debug.print(": ");serial_debug.print(dev[i],BIN,32);serial_debug.print(" - ");serial_debug.print(dev[i],HEX,8);serial_debug.println();
  }
}

void dumpTimer(timer_dev *timer) {
  serial_debug.print("timer->type=");serial_debug.println(timer->type);
  serial_debug.print("timer->regs.bas->CR1= ");serial_debug.println(timer->regs.bas->CR1,BIN,16);
  serial_debug.print("timer->regs.bas->CR2= ");serial_debug.println(timer->regs.bas->CR2,BIN,16);
  serial_debug.print("timer->regs.bas->DIER=");serial_debug.println(timer->regs.bas->DIER,BIN,16);
  serial_debug.print("timer->regs.bas->SR=  ");serial_debug.println(timer->regs.bas->SR,BIN,16);
  serial_debug.print("timer->regs.bas->EGR= ");serial_debug.println(timer->regs.bas->EGR,BIN,16);
  serial_debug.print("timer->regs.bas->CNT= ");serial_debug.println(timer->regs.bas->CNT,BIN,16);
  serial_debug.print("timer->regs.bas->PSC= ");serial_debug.println(timer->regs.bas->PSC,BIN,16);
  serial_debug.print("timer->regs.bas->ARR= ");serial_debug.println(timer->regs.bas->ARR,BIN,16);
}

void dumpDAC(const dac_dev *dac) {
  serial_debug.print("dac->regs->CR     =");serial_debug.println(dac->regs->CR,BIN,32);
  serial_debug.print("dac->regs->SWTRIGR=");serial_debug.println(dac->regs->SWTRIGR,BIN,32);
  serial_debug.print("dac->regs->DHR12R1=");serial_debug.println(dac->regs->DHR12R1,BIN,32);
  serial_debug.print("dac->regs->DHR12L1=");serial_debug.println(dac->regs->DHR12L1,BIN,32);
  serial_debug.print("dac->regs->DHR8R1 =");serial_debug.println(dac->regs->DHR8R1,BIN,32);
  serial_debug.print("dac->regs->DHR12R2=");serial_debug.println(dac->regs->DHR12R2,BIN,32);
  serial_debug.print("dac->regs->DHR12L2=");serial_debug.println(dac->regs->DHR12L2,BIN,32);
  serial_debug.print("dac->regs->DHR8R2 =");serial_debug.println(dac->regs->DHR8R2,BIN,32);
  serial_debug.print("dac->regs->DHR12RD=");serial_debug.println(dac->regs->DHR12RD,BIN,32);
  serial_debug.print("dac->regs->DHR12LD=");serial_debug.println(dac->regs->DHR12LD,BIN,32);
  serial_debug.print("dac->regs->DHR8RD =");serial_debug.println(dac->regs->DHR8RD,BIN,32);
  serial_debug.print("dac->regs->DOR1   =");serial_debug.println(dac->regs->DOR1,BIN,32);
  serial_debug.print("dac->regs->DOR2   =");serial_debug.println(dac->regs->DOR1,BIN,32);
}



#define ADC_DUAL_MODE 0x70000 // Fast interleave mode
//#define ADC_DUAL_MODE 0x60000 // Regular simultaneous mode

void setADCs ()
{
  adc_set_prescaler(ADC_PRE_PCLK2_DIV_6); // ADC_PRE_PCLK2_DIV_4: Overclock ADC to 18MHz if sysclock is 72MHz
  int pinMapADCin1 = PIN_MAP[ANALOG_IN_PIN1].adc_channel;
  int pinMapADCin2 = PIN_MAP[ANALOG_IN_PIN2].adc_channel;
  adc_set_sample_rate(ADC1, ADC_SMPR_1_5);    //=0,58uS/sample.  ADC_SMPR_13_5 = 1.08uS - use this one if Rin>10Kohm,
  adc_set_sample_rate(ADC2, ADC_SMPR_1_5);    // if not may get some sporadic noise. see datasheet.

  //  adc_reg_map *regs = dev->regs;
  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->CR1 |= ADC_DUAL_MODE;
  ADC1->regs->CR2 |= ADC_CR2_CONT; // | ADC_CR2_DMA; // Set continuous mode and DMA (set in takeSamples())
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;
  ADC1->regs->SQR3 = pinMapADCin1;

  ADC2->regs->CR2 |= ADC_CR2_CONT; // ADC 2 continuos
  ADC2->regs->CR2 |= ADC_CR2_SWSTART;
  ADC2->regs->SQR3 = pinMapADCin2;
  
  adc_calibrate(ADC1);
  adc_calibrate(ADC2);

}

// Grab the samples from the ADC
// Theoretically the ADC can not go any faster than this.
//
// According to specs, when using 72Mhz on the MCU main clock,the fastest ADC capture time is 1.17 uS. As we use 2 ADCs we get double the captures, so .58 uS, which is the times we get with ADC_SMPR_1_5.
// I think we have reached the speed limit of the chip, now all we can do is improve accuracy.
// See; http://stm32duino.com/viewtopic.php?f=19&t=107&p=1202#p1194

void takeSamples ()
{
  // This loop uses dual interleaved mode to get the best performance out of the ADCs
  //

  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);

  adc_dma_enable(ADC1);
  dma_setup_transfer(DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_32BITS,
                     dataPoints32, DMA_SIZE_32BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));// Receive buffer DMA
  dma_set_num_transfers(DMA1, DMA_CH1, MAX_SAMPLES / 2);
  dma1_ch1_Active = 1;
  //  regs->CR2 |= ADC_CR2_SWSTART; //moved to setADC
  dma_enable(DMA1, DMA_CH1); // Enable the channel and start the transfer.
  
  samplingTime = micros();
  while (dma1_ch1_Active);
  samplingTime = (micros() - samplingTime);

  dma_disable(DMA1, DMA_CH1); //End of trasfer, disable DMA and Continuous mode.
  // regs->CR2 &= ~ADC_CR2_CONT;
  
}



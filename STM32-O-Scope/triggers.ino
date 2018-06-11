void manualSamples()
{
  takeSamples();
  serial_debug.println("Samples taken!");
  serial_debug.println();
}

// Crude triggering on positive or negative or either change from previous to current sample.
void trigger()
{
  triggered = false;
  switch (triggerType) {
    case 1:
      triggerNegative() ;
      break;
    case 2:
      triggerPositive() ;
      break;
    default:
      triggerBoth() ;
      break;
  }
  blinkLED();
  takeSamples();
}

void triggerNegative() {
  triggerPoints[0] = analogRead(ANALOG_IN_PIN1);
  while(!triggered){
    triggerPoints[1] = analogRead(ANALOG_IN_PIN1);
    if ((triggerPoints[1] < triggerValue) && (triggerPoints[0] > triggerValue) ){
      triggered = true;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(ANALOG_IN_PIN1);
  }
}

void triggerPositive() {
  triggerPoints[0] = analogRead(ANALOG_IN_PIN1);
  while(!triggered){
    triggerPoints[1] = analogRead(ANALOG_IN_PIN1);
    if ((triggerPoints[1] > triggerValue) && (triggerPoints[0] < triggerValue) ){
      triggered = true;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(ANALOG_IN_PIN1);
  }
}

void triggerBoth()
{
  triggerPoints[0] = analogRead(ANALOG_IN_PIN1);
  while(!triggered){
    triggerPoints[1] = analogRead(ANALOG_IN_PIN1);
    if ( ((triggerPoints[1] < triggerValue) && (triggerPoints[0] > triggerValue)) ||
         ((triggerPoints[1] > triggerValue) && (triggerPoints[0] < triggerValue)) ){
      triggered = true;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(ANALOG_IN_PIN1);
  }
}

void toggleEdgeType() {
  triggerType += 1;
  if (triggerType > 2)
  {
    triggerType = 0;
  }
  serial_debug.print("triggerType: ");serial_debug.println(triggerType);
}

void increaseTriggerPosition() {

  if (triggerValue < ANALOG_MAX_VALUE ) {
    triggerValue += TRIGGER_POSITION_STEP;  //trigger position step
  }
  serial_debug.print("# TriggerPosition=");
  serial_debug.println(triggerValue);
}

void decreaseTriggerPosition() {

  if (triggerValue > 0 ) {
    triggerValue -= TRIGGER_POSITION_STEP;  //trigger position step
  }
  serial_debug.print("# TriggerPosition=");
  serial_debug.println(triggerValue);
}



/* 
Program for the CasuallyLoaded® Brushless DC-Motor Driver BLDC ESC kit, Created 2022.
Compiled for use on the Atmega88P with 16Mhz external clock and 4.3v BOD. Each kit has 
this code preprogrammed into the included IC so its not necessary to do the upload yourself. 
For those looking to modify the code it has been semi-annotated. 
*/
//create and define variables
#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n) //clock cycle based delay (1 = 62.5nS @16MHZ)
volatile long DutyMicros = 0;
volatile long triggerPoint = 0;
volatile long risePoint = 0;
volatile int PrevState = 0;
volatile int FilteredDuty = 0;
volatile int UpdateDuty = 0;
volatile int StepCount = 0;
volatile int spinDirection = 0;
volatile int StartRevving = 0;
volatile int ChngDir = 0;
volatile int debounceNum = 0;
long HoldUpnumber = 0;
int DutyPercent = 0;
int motorSpinning = 0;
int setDuty = 0;
int lastSetDuty = 0;
int BPCnt = 0;
int AsetPWM = B01100001;
int BsetPWM = B10000010;
int CsetPWM = B00100010;
int ALow = B10100000;
int BLow = B10000100;
int CLow = B00100100;
int currentstepDelay = 0;
int openLoopdt = 0;
int debounceSet = 0;

void setup() {
  
  cli(); //disables all interrupts
  
  DDRD |= B10101100; //set the data direction for outputs
  DDRB |= B00000110; 

  PCICR = B00000101; //enable pin change interrupts
  PCMSK2 = B00010000; //set the individual port pins to trigger said interrupts
  PCMSK0 = B00100000; //set the individual port pins to trigger said interrupts

  
  TCCR1A = 0; //16bit timer1 setup, used to control the highside bootstrap PWM (Motor Speed)
  TCCR1B = 0;
  TCCR1A = B10100010; //phase correct pwm 0 prescale
  TCCR1B = B00010001;
  ICR1 = 250; //set 32 kHz
  OCR1A = 0; //set % duty
  OCR1B = 0; //set % duty

  TCCR2A = 0; //8bit timer2 setup, used to control the highside bootstrap PWM (Motor Speed)
  TCCR2B = 0;
  TCCR2A = B01100001; //phase correct pwm 0 prescale
  TCCR2B = B00001001;
  OCR2A = 250; //set 32 kHz
  OCR2B = 0; //set % duty

  
  ACSR = B00011000; //enable the internal comparator
  ADCSRA &= ~(1<<ADEN); //disable the ADC module
  ADCSRB |= (1<<ACME); //enable the MUX selector for negative input of comparator
  ACSR |= B00000011; //sets rising edge interupt
  ADMUX = 001; //Select A1 as comparator negative input  

  StartUpSound(); //Plays a tone through the motor showing esc has started succesfully and is waiting in standby
  
  //sets all phases to floating to stand by
  TCCR2A = 0; //disables timers
  TCCR1A = 0;
  PORTD = B10100100; //turns off the lowsides
  OCR1A = 0; //sets PWM to 0% duty
  OCR1B = 0;
  OCR2B = 0;

  sei(); //enables all interrupts
  
}

void loop() {
    
  if (UpdateDuty == 1){ //check to see if the duty is in need of an update
    if ((DutyMicros >= 1000) && (DutyMicros <= 2000)) { FilteredDuty = map(DutyMicros, 1000, 2000, 99, 248); } 
    else if (DutyMicros < 1000) { FilteredDuty = 0; }
    else if (DutyMicros > 2000) { FilteredDuty = 248; }
    DutyPercent = FilteredDuty; //pulls duty from volatile into a stable variable for the loops use
    setDuty = 1; //tells loop to push update to the OCRxx registers
    UpdateDuty = 0; //ensures loop only runs once
  }

  if ((setDuty == 1) && (DutyPercent >= 100) && (motorSpinning == 0)) { //if the motor is off and needs to start spinning
    
    OCR1A = DutyPercent; //sets PWM duty
    OCR1B = DutyPercent;
    OCR2B = DutyPercent;
    ACSR = 0x10; //disable comparator
    debounceNum = 0; //disable the comparator debounce
    debounceSet = 0;
    openLoopdt = 2250; //delay time in uS for the motors open loop start sequence
    StepCount = 1;
    TakeStep(StepCount, spinDirection);
    delayMicroseconds(5000);
    StepCount = 2;
    TakeStep(StepCount, spinDirection);
    delayMicroseconds(5000);
    StepCount = 3;
    TakeStep(StepCount, spinDirection);
    while(openLoopdt > 500) {
      delayMicroseconds(openLoopdt);
      StepCount++; //increment step by 1
      if (StepCount > 6) { StepCount = 1; } //If step is larger than 6 set step back to 1
      TakeStep(StepCount, spinDirection);
      openLoopdt = openLoopdt - 20;
    }
    ACSR |= 0x08; //enable comparator to kick off into closed loop spin
    motorSpinning = 1;
    setDuty = 0;
  }
   
  if ((setDuty == 1) && (DutyPercent >= 100) && (motorSpinning == 1)) { //if the motor is already spinning and needs a speed update
    
    OCR1A = DutyPercent; //sets PWM duty
    OCR1B = DutyPercent;
    OCR2B = DutyPercent;
    lastSetDuty = DutyPercent;
    setDuty = 0;
    
  }
  else if ((setDuty == 1) && (DutyPercent < 100) && (motorSpinning == 1)) { //if the motor is spinning and needs to stop
    
    ACSR = 0x10; //disable comparator
    TCCR2A = 0; //sets all phases to floating 
    TCCR1A = 0; //turn off the highsides 
    PORTD = B10100100; //turn off the low sides
    OCR1A = 0; //sets PWM to 0% duty
    OCR1B = 0;
    OCR2B = 0;
    lastSetDuty = 0; //resets variables for next start
    motorSpinning = 0;
    setDuty = 0;
  }

  if ((DutyPercent >= 130) && (motorSpinning == 1) && (debounceSet == 0)) { //enables the comparators debounce once we know the motor started and is caught in closed loop
    debounceNum = 5; //filter strength for the comparator (higher the number the slower the motor will react)
    debounceSet = 1; //ensures this only runs once and when its told too 
  }

  if ((DutyPercent < 130) && (motorSpinning == 1) && (debounceSet == 1)) { //disables the debouce at low speeds to keep the motor stable
    debounceNum = 0; //disables the comparators debounce 
    debounceSet = 0; //ensures this only runs once and when its told too 
  }

  if ((StartRevving == 1) && (lastSetDuty < 100) && (ChngDir == 1)){ //sets the spin direction
    spinDirection = 1;
    ChngDir = 0;
  }
  else if ((StartRevving == 0) && (lastSetDuty < 100) && (ChngDir == 1)){ //sets the spin direction
    spinDirection = 0;
    ChngDir = 0;
  }
  
}

ISR (ANALOG_COMP_vect) { //analog comparator interrupt vector for step change

  for(int i = 0; i < debounceNum; i++) {   //Check the comparator output 10 times to be sure (debounce)
    if ((StepCount & 1) && (!(ACSR & B00100000))) { i = i - 1; }
    else if ((!(StepCount & 1)) && (ACSR & B00100000)) { i = i - 1; }
  }

  if ((StepCount & 1) && (ACSR & B00100000)){ //if step is odd and rising edge was just triggered
    StepCount++; //increment step by 1
    if (StepCount > 6) { StepCount = 1; } //if step is larger than 6 set step back to 1
    TakeStep(StepCount, spinDirection); //set half-bridges to reflect the step      
  }
  else if ((!(StepCount & 1)) && (!(ACSR & B00100000))) { //if not odd step must be even and falling edge was just triggered
    StepCount++; //increment step by 1
    if (StepCount > 6) { StepCount = 1; } //if step is larger than 6 set step back to 1
    TakeStep(StepCount, spinDirection); //set half-bridges to reflect the step 
  }
 
}

ISR(PCINT2_vect){ //pin change interrupt vector for the speed input signal detecting
  
  triggerPoint = micros(); //stores the time of interrupt start as micros
  if ((PIND & B00010000) && (PrevState == 0)){ //if input pin just went high and was just low
    PrevState = 1; //records that the input pin is currently high
    risePoint = triggerPoint; //saves the micros start time to storage variable
  }
  else if ((!(PIND & B00010000)) && (PrevState == 1)){ //if the input pin just went low and was just high
    PrevState = 0; //records that the input pin is currently low
    DutyMicros = triggerPoint - risePoint; //calculates the total duty cycle in micros of pulse
    UpdateDuty = 1; //tells main loop to update the duty
  }
}

ISR (PCINT0_vect) { //pin change interrupt vector for direction change

  if ((PINB & B00100000)){ //checks if pin PB5 went high and triggers reverse
    StartRevving = 1; //sets flag for the main loop
    ChngDir = 1;
  }
  else if (!(PINB & B00100000)){ //checks if pin PB5 went low and triggers forward
    StartRevving = 0; //sets flag for the main loop
    ChngDir = 1;
  }
   
}

void TakeStep(int stepNum, int RotationDirection){ //step setting function

  if (RotationDirection == 0) { //setting forward steps
    
    switch(stepNum) {

      case 1://step1: A, PWM. B, low. C, floating/bemfsense.
      TCCR1A = 0; //turn off unwanted highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = BLow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR2A = AsetPWM; //set the wanted highside pulsing
      ADMUX = 3; //set A3 as negative side comparator input (phase C)
      ACSR |= 0x03; //sets rising edge interrupt
      break;

      case 2: //step2: A, PWM. C, low. B, floating/bemfsense.
      TCCR1A = 0; //turn off unwanted highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = CLow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR2A = AsetPWM; //set the wanted highside pulsing
      ADMUX = 2; //set A2 as negative side comparator input (phase B)
      ACSR &= ~0x01; //sets falling edge interrupt
      break;

      case 3: //step3: B, PWM. C, low. A, floating/bemfsense.
      TCCR2A = 0; //turn off unwanted highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = CLow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR1A = BsetPWM; //set the wanted highside pulsing
      ADMUX = 1; //set A1 as negative side comparator input (phase A)
      ACSR |= 0x03; //sets rising edge interrupt
      break;

      case 4: //step4: B, PWM. A, low. C, floating/bemfsense.
      TCCR2A = 0; //turn off unwanted highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = ALow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR1A = BsetPWM; //set the wanted highside pulsing 
      ADMUX = 3; //set A3 as negative side comparator input (phase C)
      ACSR &= ~0x01; //sets falling edge interrupt
      break;

      case 5: //step5: C, PWM. A, low. B, floating/bemfsense.
      TCCR2A = 0; //turn off unwanted highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = ALow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR1A = CsetPWM; //set the wanted highside pulsing 
      ADMUX = 2; //set A2 as negative side comparator input (phase B)
      ACSR |= 0x03; //sets rising edge interrupt
      break;

      case 6: //step6: C, PWM. B, low. A, floating/bemfsense.
      TCCR2A = 0; //turn off unwanted highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = BLow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR1A = CsetPWM; //set the wanted highside pulsing  
      ADMUX = 1; //set A1 as negative side comparator input (phase A)
      ACSR &= ~0x01; //sets falling edge interrupt
      break;
    }
  }
  else if (RotationDirection == 1) { //setting reverse steps
    
    switch(stepNum) {

      case 1: //Step 1 C, PWM. B, low. A, floating/bemfsense.
      TCCR2A = 0; //turn off unwanted highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = BLow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR1A = CsetPWM; //set the wanted highside pulsing  
      ADMUX = 1; //set A1 as negative side comparator input (phase A)
      ACSR |= 0x03; //sets rising edge interrupt
      break; 

      case 2: //step5: C, PWM. A, low. B, floating/bemfsense.
      TCCR2A = 0; //turn off unwated highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = ALow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR1A = CsetPWM; //set the wanted highside pulsing 
      ADMUX = 2; //set A2 as negative side comparator input (phase B)
      ACSR &= ~0x01; //sets falling edge interrupt
      break;

      case 3: //step4: B, PWM. A, low. C, floating/bemfsense.
      TCCR2A = 0; //turn off unwated highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = ALow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR1A = BsetPWM; //set the wanted highside pulsing  
      ADMUX = 3; //set A3 as negative side comparator input (phase C)
      ACSR |= 0x03; //sets rising edge interrupt
      break;

      case 4: //step3: B, PWM. C, low. A, floating/bemfsense.
      TCCR2A = 0; //turn off unwated highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = CLow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through 
      TCCR1A = BsetPWM; //set the wanted highside pulsing 
      ADMUX = 1; //set A1 as negative side comparator input (phase A)
      ACSR &= ~0x01; //sets falling edge interrupt
      break;

      case 5: //step2: A, PWM. C, low. B, floating/bemfsense.
      TCCR1A = 0; //turn off highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = CLow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR2A = AsetPWM; //set the highside pulsing
      ADMUX = 2; //set A2 as negative side comparator input (phase B)
      ACSR |= 0x03; //sets rising edge interrupt
      break;

      case 6://step1: A, PWM. B, low. C, floating/bemfsense.
      TCCR1A = 0; //turn off highsides
      DELAY_CYCLES(3); //delay for deadtime to prevent shoot through
      PORTD = BLow; //set the lowsides
      DELAY_CYCLES(2); //delay for deadtime to prevent shoot through
      TCCR2A = AsetPWM; //set the highside pulsing
      ADMUX = 3; //set A3 as negative side comparator input (phase C)
      ACSR &= ~0x01; //sets falling edge interrupt
      break;
      
    }
  }
} 

void StartUpSound(){ //program to play 3 short beeps to indicate the ESC has completed its startup and is ready for use

  TCCR1A = 0; //turn off highsides
  TCCR2A = 0;
  OCR1A = 0; //sets PWM to no duty
  OCR1B = 5; //sets PWM duty, this controls volume, lower volume = lower current draw
  OCR2B = 3; //sets PWM duty
  PORTD = BLow; //sets B low with A & C floating.
  
  for (BPCnt = 0; BPCnt <= 10000; BPCnt++){ //loop to generate a beep
    TCCR2A = AsetPWM; //set phase A to be pulsing
    DELAY_CYCLES(90); //small delay
    TCCR1A = 0; //turn off highsides
    TCCR2A = 0;
    DELAY_CYCLES(100); //small delay
    TCCR1A = CsetPWM; //set phase C to be pulsing
    DELAY_CYCLES(90); //small delay
    TCCR1A = 0; //turn off highsides
    TCCR2A = 0;
    DELAY_CYCLES(100); //small delay
  }

  HoldUpnumber = 0;
  while (HoldUpnumber < 10000){ //while loop to introduce some rough (~ 7 millisecond) delay as interrupts are disabled in the setup and millis wont work
    HoldUpnumber++;
    DELAY_CYCLES(5); 
  }

  for (BPCnt = 0; BPCnt <= 10000; BPCnt++){ //loop to generate a beep
    TCCR2A = AsetPWM; //set phase A to be pulsing
    DELAY_CYCLES(90); //small delay
    TCCR1A = 0; //turn off highsides
    TCCR2A = 0;
    DELAY_CYCLES(100); //small delay
    TCCR1A = CsetPWM; //set phase C to be pulsing
    DELAY_CYCLES(90); //small delay
    TCCR1A = 0; //turn off highsides
    TCCR2A = 0;
    DELAY_CYCLES(100); //small delay
  }

  HoldUpnumber = 0; 
  while (HoldUpnumber < 125000){ //while loop to introduce some rough (~ 7 millisecond) delay as interrupts are disabled in the setup and millis wont work
    HoldUpnumber++;
    DELAY_CYCLES(5); 
  }

  for (BPCnt = 0; BPCnt <= 12500; BPCnt++){ //loop to generate a beep
    TCCR2A = AsetPWM; //set phase A to be pulsing
    DELAY_CYCLES(90); //small delay
    TCCR1A = 0; //turn off highsides
    TCCR2A = 0;
    DELAY_CYCLES(100); //small delay
    TCCR1A = CsetPWM; //set phase C to be pulsing
    DELAY_CYCLES(90); //small delay
    TCCR1A = 0; //turn off highsides
    TCCR2A = 0;
    DELAY_CYCLES(100); //small delay
  }
}

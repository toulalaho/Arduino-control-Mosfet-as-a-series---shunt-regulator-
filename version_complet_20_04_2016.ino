#include <LCD5110_Graph.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


/*******Interrupt-based Rotary Encoder Sketch*******
by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt, Steve Spence
http://www.instructables.com/files/orig/FSD/13QB/IM3QGLHR/FSD13QBIM3QGLHR.ino */

/*******Watchdog timer by http://donalmorrissey.blogspot.co.uk/2010/04/sleeping-arduino-part-5-wake-up-via.html *********/

/*******LCD5110_Library to control the lcd http://www.rinkydinkelectronics.com/library.php?id=48 *******/

static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
LCD5110 myGLCD(12,11,10,8,9);  //
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];
extern unsigned char MediumNumbers[];
volatile int f_wdt=1;   // Sleep arduino


// pins for the LEDs:


const int analogInPin = A5;  // Analog input pin that the potentiometer is attached to / Default = A5
const int buttonA = A0;      // User Buttons
const int buttonB = A1;
const int buttonC = 4;

char selection = 0; // Choose true for PWM, false for no PWM
const int overchargeIndicatorLed = 5; // LedOutput  Default = 2


// Global Declaration
const float VinMax = 12;                    // Maximum Input Voltage Default = 4
const float R1 = 10;                       // Bottom resistance Kohm Default = 10
const float R2 = 100;                      // top resistance Kohm     Default = 100
const float maxControllerVoltage = 22;      // the arduino Uno boardcard cant excess 5 volt   default = 5v
const float hysteresis = 2;                 // Value of the hysteresis
const float setpointHigh = (VinMax + hysteresis);  // Set setpointHigh Caution the battery may go beyond the overcharge
const float setpointLow = (VinMax - hysteresis) ;  // Set setpointLow





int limitOverchargeInAnalog;              // Limit of the overcharge in analog value
int limitSetpointHighInAnalog;            // Limit of the setpointhigh in analog value
int limitSetpointLowInAnalog;             // Limit of the setpointlow in analog value
float limitOverchargeInVolt;              // Limit of the overcharge in voltage
float limitSetpointHighInVolt;            // Limit of the setpointhigh in voltage
float limitSetpointLowInVolt;             // Limit of the setpointLow in voltage
bool verification;                         // False = Overcharged / True = not overcharged
float Vin;                                 // Actual Value of the voltage Input
unsigned char state;                      // get the state of the charge
unsigned char intensity;                   //Used to set the intensity of the led with PWM control
char pourcentageIntensity;
bool pwmState;
int intensityInPercent;
bool test;
bool oldValue = 0;
char i = 0;
float arrayVin[83];
char etat;
volatile byte dividedEncorderPos= 0;



int sensorValue = 0;        // value read from the pot

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  // initialize serial:
  Serial.begin(9600);
  // initialize digital pin 2 as an output.
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  pinMode(overchargeIndicatorLed, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  // initialize the user buttons
  pinMode(buttonA, INPUT_PULLUP);
  pinMode(buttonB, INPUT_PULLUP);
  pinMode(buttonC, INPUT_PULLUP);

  initLCD(500);
  /*** Setup the WDT ***/

  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);

  /* In order to change WDE or the prescaler, we need to
  * set WDCE (This will allow updates for 4 clock cycles).
  */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP1; /* 8.0 seconds */

  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);

  Serial.println("Initialisation complete.");
  delay(100); //Allow for serial print to complete.
}


/***************************************************
*  Name:        ISR(WDT_vect)
*
*  Returns:     Nothing.
*
*  Parameters:  None.
*
*  Description: Watchdog Interrupt Service. This
*               is executed when watchdog timed out.
*
***************************************************/
ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1; // default 1
  }
  else
  {
    //Serial.println("WDT Overrun!!!");
  }
}


void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  encoderPos --; //decrement the encoder's position count
  bFlag = 0; //reset flags for the next turn
  aFlag = 0; //reset flags for the next turn
}
else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  encoderPos ++; //increment the encoder's position count
  bFlag = 0; //reset flags for the next turn
  aFlag = 0; //reset flags for the next turn
}
else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
sei(); //restart interrupts
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////LOOP////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
  if(f_wdt == 1)
  {
    configureSelection();
    seriesRegulator();             //Write 0 in the parenthesis for no PWM or 1 for the PWM //default selection
    displayLCD(selection); //default selection
    displaySerialInformation(0);
    drawBatteryLimit();
    Serial.println(selection);
    if (i>83)
  { i = 0;
    drawCurve(i);
    i++;
  }
  else {
    drawCurve(i);
    i++;
  }




    test=testButtonC();
    enableSleepmode(test);
    if(oldEncPos != encoderPos) {
      Serial.println(encoderPos);
      oldEncPos = encoderPos;
    }
    f_wdt = 0;

    /* Re-enter sleep mode. */
   enterSleepArduino();
  }
  else
  {
    /* Do nothing. */
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////FUNCTION/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool CheckButtonA(void)
{

  if(digitalRead(buttonA) == LOW)
  {
    //something
  }
}

void displaySerialInformation(bool selection)                      // Read the global declaration in order to display information on the serial monitor
{                                                               // 0 for all value
  // 1 for just Vin value
  if(selection == 0)
  {
    Serial.print("Actual Voltage : ");
    Serial.println(Vin);

    Serial.print("limitOverchargeInVolt =");
    Serial.println(VinMax);

    Serial.print("sensor = " );
    Serial.println(sensorValue);

    Serial.print("limitOverchargeInAnalog =");
    Serial.println(limitOverchargeInAnalog);

    Serial.print("limitSetpointHighInVolt =");
    Serial.println(setpointHigh);

    Serial.print("limitSetpointLowInVolt =");
    Serial.println(setpointLow);

    Serial.print("limitSetpointHighInAnalog =");
    Serial.println(limitSetpointHighInAnalog);

    Serial.print("limitSetpointLowInAnalog =");
    Serial.println(limitSetpointLowInAnalog);


    Serial.println();
  }
  else
  {
    Serial.print("Actual Voltage : ");
    Serial.println(Vin);

  }

}

float getVin(int sensorValue)                                              // Get Vin value and store it in the global declaration Vin
{
  Vin = ((sensorValue) * maxControllerVoltage / 1024); // * ((R1 + R2) / R1);

  return (Vin);
}

bool verificationOvercharge(int sensorValue, int VinMax)                                    // Check for overcharge
{

  limitOverchargeInVolt = VinMax; // *  (R1 / (R1 + R2));                                        // PotentialDiviser R1 = bottom // R2 = top // VinMax= Maximum Input Voltage
  limitOverchargeInAnalog = 1024 * ( limitOverchargeInVolt / maxControllerVoltage) ;

  if (sensorValue >= limitOverchargeInAnalog)
  {
    return (true);
  }
  else
  {return (false);}

}

bool verificationSetpointHigh(int sensorValue, float setpointHigh)                                    // Check for overcharge
{

  limitSetpointHighInVolt = setpointHigh ;// *  (R1 / (R1 + R2));                                        // PotentialDiviser R1 = bottom // R2 = top // VinMax= Maximum Input Voltage
  limitSetpointHighInAnalog = 1024 * ( limitSetpointHighInVolt / maxControllerVoltage) ;

  if (sensorValue >= limitSetpointHighInAnalog)
  {
    return (true);
  }
  else
  {return (false);
  }
}

bool verificationSetpointLow(int sensorValue, float setpointLow)                                    // Check for overcharge
{

  limitSetpointLowInVolt = setpointLow; //* (R1 / (R1 + R2));                                        // PotentialDiviser R1 = bottom // R2 = top // VinMax= Maximum Input Voltage
  limitSetpointLowInAnalog = 1024 * ( limitSetpointLowInVolt / maxControllerVoltage) ;

  if (sensorValue <= limitSetpointLowInAnalog)
  {
    return (true);
  }
  else return (false);
}

void seriesRegulator()
{ char i;

  for (i=0; i<11; i++)
  {
    delay(50);
    sensorValue = analogRead(analogInPin);
    getVin(sensorValue);   // Get Vin value and store it in the global declaration Vin
    //controlShuntNoPWM();
    controlMosfet();
    CheckButtonA();
  }

  displaySerialInformation(1);                      // Read the global declaration in order to display information on the serial monitor

}

void PWMwrite(int intensity, int outputPin)
{ int mappedIntensity;
  mappedIntensity = map(intensity,limitSetpointLowInAnalog,limitSetpointHighInAnalog, 0, 255);
  analogWrite(outputPin, mappedIntensity);

}

void controlMosfetPWM(void)
{
  if(verificationSetpointHigh(sensorValue,setpointHigh)==1) //Voltage higher than setpointHigh
  {
    digitalWrite(overchargeIndicatorLed, HIGH);            // Light up the led
    // State set to overcharged
  }
  else if (verificationSetpointLow(sensorValue,setpointLow)==1)
  {
    digitalWrite(overchargeIndicatorLed, LOW);             // Light off the led
    // State set to not overcharged
  }
  else
  {
    PWMwrite(sensorValue, overchargeIndicatorLed);
    // State set to hysteresis
  }
}

void controlMosfetNoPWM(void)
{
  if(verificationSetpointHigh(sensorValue,setpointHigh)==1) //Voltage higher than setpointHigh
  {
    digitalWrite(overchargeIndicatorLed, HIGH);            // Light up the led
    // State set to overcharged
  }
  else if (verificationSetpointLow(sensorValue,setpointLow)==1)
  {
    digitalWrite(overchargeIndicatorLed, LOW);             // Light off the led
    // State set to not overcharged
  }
  else
  {
    // State set to hysteresis
  }
}

void controlMosfet()                     // selection of the control mode
{                                                       // 0 = No pwm
  if ( selection == 0)                                  // 1 = pwm
  {
    
    controlShuntNoPWM();
  }
  else if (selection == 1)

  {
    controlShuntPWM();
    
  }
  else if(selection == 2)
  {
    controlMosfetNoPWM();

  }
  else if (selection== 3)
  {
    controlMosfetPWM();
  }
}

char getIntensityInPercent()
{

  if(selection == 2)
  {
    if (verificationSetpointHigh(sensorValue,setpointHigh)==1)
    {
      intensityInPercent = 100;

    }
    else if (verificationSetpointLow(sensorValue,setpointLow)==1)
    {
      intensityInPercent = 0;
    }
  }
  else if(selection == 3)
  {
    intensityInPercent = map(sensorValue,limitSetpointLowInAnalog,limitSetpointHighInAnalog, 0, 100);
        if(intensityInPercent >= 100)
    {
      intensityInPercent = 100;
    }
    else if ( intensityInPercent <= 0)
    {
      intensityInPercent = 0;
    }

  }
  else if (selection == 0)
  {
    if (verificationSetpointHigh(sensorValue,setpointHigh)==1)
    {
      intensityInPercent = 0;

    }
    else if (verificationSetpointLow(sensorValue,setpointLow)==1)
    {
      intensityInPercent = 100;
    }
  }
  else if (selection==3)
  {
    intensityInPercent = map(sensorValue,limitSetpointHighInAnalog,limitSetpointLowInAnalog, 0, 100);
        if(intensityInPercent >= 100)
    {
      intensityInPercent = 100;
    }
    else if ( intensityInPercent <= 0)
    {
      intensityInPercent = 0;
    }
  }
  return (intensityInPercent);

}


void initLCD( int tempo )
{
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);

  myGLCD.clrScr();
  myGLCD.print("SerieRegulator", CENTER, 0);
  myGLCD.print("RE-INNOVATION", CENTER, 20);
  myGLCD.setFont(TinyFont);
  myGLCD.print("(C)2016 by", CENTER, 36);
  myGLCD.print("Matthew Little", CENTER, 42);
  myGLCD.update();

  delay(tempo);
  myGLCD.clrScr();

}

void displayLCD(char selection)
{ char i;
  myGLCD.setFont(SmallFont);
  myGLCD.print("Voltage", LEFT, 0);

  if(selection == 0)
  {
    myGLCD.print("SER: ", 45, 0);
  }
  else if (selection == 1)
  {
    myGLCD.print("SER: ", 45, 0);
    myGLCD.print("PWM",RIGHT,0);
  }
  else if (selection == 2)
{
  myGLCD.print("SHU: ", 45, 0);

}
  else if (selection == 3)
  {
    myGLCD.print("SHU: ", 45, 0);
    myGLCD.print("PWM",RIGHT,0);
  }
  myGLCD.setFont(MediumNumbers);

  if(Vin < 10){
    myGLCD.printNumF(Vin,1,LEFT,7);
  }
  else
  {
    myGLCD.printNumI(Vin,LEFT,7);
  }
  myGLCD.setFont(SmallFont);
  myGLCD.print("v",35,15);

  myGLCD.setFont(MediumNumbers);

  myGLCD.printNumI(getIntensityInPercent(),41,7,3,'.');
  myGLCD.setFont(SmallFont);
  myGLCD.print("%",77,15);
  for(i=83;i >=0 ; i--)      // line

  {
    myGLCD.setPixel(i,47);
  }
  for(i=47;i > 24 ; i--)      // colonne

  {
    myGLCD.setPixel(0,i);
  }
  myGLCD.update();
  myGLCD.clrScr();
}

bool testButtonC()
{ bool newValue;
  newValue = digitalRead(buttonC);
  if(newValue != 1)
  {
    return (0);
  }
  else
  {
    return (1);
  }


}

void enableSleepmode(bool enable)
{
  if(enable == 0)
  {
    if(oldValue == 0)
    { myGLCD.enableSleep();
      digitalWrite(13,LOW);
      delay(1000);
      oldValue = 1;
    }
    else
    {
      myGLCD.disableSleep();
      digitalWrite(13,HIGH);

      delay(1000);
      oldValue = 0;
    }
    if ( enable != oldValue )
    {delay(1000);

      Serial.println(oldValue);
      Serial.println("Je suis dans la fin");
    }
  }
}

bool configureSelection()
{

  if(( encoderPos >= 0) & (encoderPos < 5))
  {
    selection = 1;

  }
  else if ((encoderPos >= 5) &( encoderPos < 10))
  {
    selection = 0;
    if(intensityInPercent != 100)
    {
      digitalWrite(overchargeIndicatorLed, LOW);
      intensityInPercent =0;
    }


  }
  else if ((encoderPos >= 10) & (encoderPos < 15))
  {
    selection = 2;
  }
  else if ((encoderPos >= 15) & (encoderPos < 22))
  {
    selection = 3;
  }
  else
  {
    encoderPos = 0;
    selection = 1;
  }

}

/***************************************************
*  Name:        enterSleep
*
*  Returns:     Nothing.
*
*  Parameters:  None.
*
*  Description: Enters the arduino into sleep mode.
*
***************************************************/
void enterSleepArduino(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();

  /* Now enter sleep mode. */
  sleep_mode();

  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */

  /* Re-enable the peripherals. */
  power_all_enable();
}


void controlShuntNoPWM(void)
{
  if(verificationSetpointHigh(sensorValue,setpointHigh)==1) //Voltage higher than setpointHigh
  {
    digitalWrite(overchargeIndicatorLed, LOW);            // Light up the led
    state = 0x01;                                          // State set to overcharged
  }
  else if (verificationSetpointLow(sensorValue,setpointLow)==1)
  {
    digitalWrite(overchargeIndicatorLed, HIGH);             // Light off the led
    state = 0x02;                                          // State set to not overcharged
  }
  else
  {
    state = 0x03;                                      // State set to hysteresis
  }
}

void controlShuntPWM(void)
{
  if(verificationSetpointHigh(sensorValue,setpointHigh)==1) //Voltage higher than setpointHigh
  {
    digitalWrite(overchargeIndicatorLed, LOW);            // Light up the led
    state = 0x01;   
    Serial.print("I am in controlShuntPWM setpoint high") ;                                      // State set to overcharged
  }
  else if (verificationSetpointLow(sensorValue,setpointLow)==1)
  {
    digitalWrite(overchargeIndicatorLed, HIGH);             // Light off the led
    state = 0x02;   
    Serial.print("I am in controlShuntPWM setpoint Low");                                          // State set to not overcharged
  }
  else
  {
    PWMwriteShunt(sensorValue, overchargeIndicatorLed);
    state = 0x03;                                      // State set to hysteresis
  }
}


void drawCurve(char numberArray)
{ float mappedValueVin;
  char testNumberArray;
  mappedValueVin=map(Vin,0,maxControllerVoltage, 0, 22);
  arrayVin[numberArray]= mappedValueVin;
  for (testNumberArray=1; testNumberArray <= numberArray; testNumberArray++)
  {
    myGLCD.setPixel(testNumberArray,(46-arrayVin[testNumberArray]));
  }
}

void drawBatteryLimit(void)
{ float mappedVinMax;
  char positionX;
  bool test =0;

  mappedVinMax=map(VinMax,0,maxControllerVoltage, 0, 22);
  for (positionX=1; positionX <= 83; positionX++)
  {
    myGLCD.setPixel(positionX,(46-mappedVinMax));
    if(test==0)
    {
      test=1;
    }
    else
    {
      myGLCD.invPixel(positionX, (46-mappedVinMax));
      test=0;
    }
  }
}
void PWMwriteShunt(int intensity, int outputPin)
{ int mappedIntensity;
  mappedIntensity = map(intensity,limitSetpointHighInAnalog,limitSetpointLowInAnalog, 0, 255);
  analogWrite(outputPin, mappedIntensity);

}

/*void seriesRegulatorNoPWM(void)
{ char i;

for (i=0; i<11; i++)
{
delay(50);
sensorValue = analogRead(analogInPin);
getVin(sensorValue);   // Get Vin value and store it in the global declaration Vin
controlMosfet(0);
CheckButtonA();

}
displaySerialInformation(1);                           // Read the global declaration in order to display information on the serial monitor

}*/


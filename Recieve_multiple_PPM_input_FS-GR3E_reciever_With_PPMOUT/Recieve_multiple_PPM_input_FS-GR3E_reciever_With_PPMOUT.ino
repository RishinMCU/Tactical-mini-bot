#define PIN2_MAX 1950
#define PIN2_MIN 1050
#define PIN1_MAX 1950
#define PIN1_MIN 1050 
#include <MX1508.h>
uint16_t unChannel_2Min = PIN2_MIN;
uint16_t unChannel_2Max = PIN2_MAX;

uint16_t unChannel_1Min = PIN1_MIN;
uint16_t unChannel_1Max = PIN1_MAX;

// Assign your channel in pins
#define CHANNEL_1_PIN 1
#define CHANNEL_2_PIN 0

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define CHANNEL_1_FLAG 1
#define CHANNEL_2_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unChannel_1InShared;
volatile uint16_t unChannel_2InShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;



unsigned long pulse_time  ;
MX1508 motorA(5 ,6, FAST_DECAY, 2);
MX1508 motorB(10,9, FAST_DECAY, 2);
void setup()
{
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  analogWrite(5,0);
  analogWrite(6,0);
  analogWrite(7,0);
  analogWrite(10,0);
  //Serial.begin(9600);
  attachInterrupt(3 /* INT1 = CHANNEL_1_PIN */,calcChannel_1_Pin,CHANGE);
  attachInterrupt(2 /* INT0 = CHANNEL_2_PIN */,calcChannel_2_Pin,CHANGE);
  pulse_time =millis() ;
  
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unChannel_1In;
  static uint16_t unChannel_2In;
  // local copy of update flags
  static uint8_t bUpdateFlags;
  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    pulse_time =millis() ;
      // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if(bUpdateFlags & CHANNEL_1_FLAG)
    {
      unChannel_1In = unChannel_1InShared;
    }

    if(bUpdateFlags & CHANNEL_2_FLAG)
    {
      unChannel_2In = unChannel_2InShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
  
    if(bUpdateFlags & CHANNEL_1_FLAG)
    {
      unChannel_1In = constrain(unChannel_1In,unChannel_1Min,unChannel_1Max);
      if(unChannel_1In>1480 && unChannel_1In<1520)
        motorA.stopMotor();
      else
      {
        motorA.motorGo(map(unChannel_1In,1050,1950,-80,80));
      }
      /*if(unChannel_1In>1550)
      {
        unChannel_1In = map(unChannel_1In,1550,1950,200,200);
        analogWrite(6,0);
        analogWrite(5,unChannel_1In);   
      }
      else if(unChannel_1In<1450)
      {
        unChannel_1In = map(unChannel_1In,1050,1450,200,200);
        analogWrite(5,0);
        analogWrite(6,unChannel_1In);
      }
      else
      {
        analogWrite(5,0);
        analogWrite(6,0);
      }*/
    }
 
    if(bUpdateFlags & CHANNEL_2_FLAG)
    {
      unChannel_2In = constrain(unChannel_2In,unChannel_2Min,unChannel_2Max);
      if(unChannel_2In>1480 && unChannel_2In<1520)
        motorB.stopMotor();
      else
      {
        motorB.motorGo(map(unChannel_2In,1050,1950,-80,80));
      }
      /*if(unChannel_2In>1550)
      {
        unChannel_2In = map(unChannel_2In,1550,1950,200,200);
        analogWrite(9,0);
        analogWrite(10,unChannel_2In);
      }
      else if(unChannel_2In<1450)
      {
        unChannel_2In = map(unChannel_2In,1050,1450,200,200);
        analogWrite(10,0);
        analogWrite(9,unChannel_2In);
      }
      else
      {
        analogWrite(9,0);
        analogWrite(10,0);
      } */  
    }
  bUpdateFlags = 0;
}


// simple interrupt service routine
void calcChannel_1_Pin()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(CHANNEL_1_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unChannel_1InShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= CHANNEL_1_FLAG;
  }
}

void calcChannel_2_Pin()
{
  if(digitalRead(CHANNEL_2_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unChannel_2InShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= CHANNEL_2_FLAG;
  }
}

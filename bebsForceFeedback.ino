
#include <Joystick.h>
#include "DigitalWriteFast.h"

#define SHIFT_UP 1
#define SHIFT_DOWN 0

#define encoderPinA 2
#define encoderPinB 3

// #define motorPinA 7
// #define motorPinB 8
// #define motorPinPWM 9

#define motorL_PWM 9
#define motorR_PWM 10

#define ENCODER_MAX_VALUE 1200
#define ENCODER_MIN_VALUE -1200

#define MAX_PWM 200

bool isOutOfRange = false;
int32_t forces[2]={0};
Gains gains[2];
EffectParams effectparams[2];

int lastButton1State = 0;
int lastButton2State = 0;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  8, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  true, true, false,   //  Rx, Ry, no Rz
  false, false,          // No rudder or throttle
  false, false, false);    // No accelerator, brake, or steering

volatile long value = 0;
int32_t g_force = 0;

int32_t  currentPosition = 0;
volatile int8_t oldState = 0;
const int8_t KNOBDIR[] = {
  0, 1, -1, 0,
  -1, 0, 0, 1,
  1, 0, 0, -1,
  0, -1, 1, 0
};

void tick(void)
{
  int sig1 = digitalReadFast(encoderPinA);
  int sig2 = digitalReadFast(encoderPinB);
  int8_t thisState = sig1 | (sig2 << 1);

  if (oldState != thisState) {
    currentPosition += KNOBDIR[thisState | (oldState<<2)];
    oldState = thisState;
  } 
}

void setup() {
  Serial.begin(115200);                        
  attachInterrupt(digitalPinToInterrupt(encoderPinA),tick,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB),tick,CHANGE);
  Joystick.setRyAxisRange(0, 500);
  Joystick.setRxAxisRange(0, 500);
  Joystick.setYAxisRange(0, 500);
  Joystick.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  Joystick.setGains(gains);
  Joystick.begin(true);

  // pinMode(motorPinA, OUTPUT);
  // pinMode(motorPinB, OUTPUT);
  // pinMode(motorPinPWM, OUTPUT);
  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorR_PWM, OUTPUT);

  pinMode(A0, INPUT_PULLUP);

  // pins 1 and 0 to shifter and ground goes into shifter
  pinMode(SHIFT_UP, INPUT_PULLUP);
  pinMode(SHIFT_DOWN, INPUT_PULLUP);
  
  cli();
  TCCR3A = 0; //set TCCR1A 0
  TCCR3B = 0; //set TCCR1B 0
  TCNT3  = 0; //counter init
  OCR3A = 399;
  TCCR3B |= (1 << WGM32); //open CTC mode
  TCCR3B |= (1 << CS31); //set CS11 1(8-fold Prescaler)
  TIMSK3 |= (1 << OCIE3A);
  sei();
  
}

ISR(TIMER3_COMPA_vect){
  Joystick.getUSBPID();
}

unsigned int interval = 0;
void loop() {
  value = currentPosition;
  
  if(value > ENCODER_MAX_VALUE)
  {
    isOutOfRange = true;
    value = ENCODER_MAX_VALUE;
  }else if(value < ENCODER_MIN_VALUE)
  {
    isOutOfRange = true;
    value = ENCODER_MIN_VALUE;
  }else{
    isOutOfRange = false;
  }

  Joystick.setXAxis(value);
  Joystick.setRxAxis(analogRead(A1));
  Joystick.setRyAxis(analogRead(A2));
  Joystick.setYAxis(analogRead(A3));

  effectparams[0].springMaxPosition = ENCODER_MAX_VALUE;
  effectparams[0].springPosition = value;
  effectparams[1].springMaxPosition = 255;
  effectparams[1].springPosition = 0;
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

  
  if(!isOutOfRange){
    if(forces[0] > 0)
    {
      // digitalWrite(motorPinA, HIGH);
      // digitalWrite(motorPinB, LOW);
      // analogWrite(motorPinPWM, abs(forces[0]));
      analogWrite(motorL_PWM, abs(forces[0]));
      analogWrite(motorR_PWM, 0);
    }else{
      // digitalWrite(motorPinA, LOW);
      // digitalWrite(motorPinB, HIGH);
      // analogWrite(motorPinPWM, abs(forces[0]));
      analogWrite(motorL_PWM, 0);
      analogWrite(motorR_PWM, abs(forces[0]));
    }
  }else{
    if(value < 0){
      // digitalWrite(motorPinA, LOW);
      // digitalWrite(motorPinB, HIGH);
      analogWrite(motorR_PWM, MAX_PWM);
    }else{
      // digitalWrite(motorPinA, HIGH);
      // digitalWrite(motorPinB, LOW);
      analogWrite(motorL_PWM, MAX_PWM);
    }
    // analogWrite(motorPinPWM, MAX_PWM);
  }

  int currentButton1State = !digitalRead(SHIFT_UP);
  //If loop - Check that the button has actually changed.
  if (currentButton1State != lastButton1State){
    //If the button has changed, set the specified HID button to the Current Button State
    Joystick.setButton(0, currentButton1State);
    //Update the Stored Button State
    lastButton1State = currentButton1State;
  }

  int currentButton2State = !digitalRead(SHIFT_DOWN);
  if (currentButton2State != lastButton2State){
    Joystick.setButton(1, currentButton2State);
    lastButton2State = currentButton2State;
  }
}

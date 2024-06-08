/*
 * THE STEERING WHEEL OF ALL TIME
 */

// Include libraries
  #include <Joystick.h>
  #include <Arduino.h>
  #include "DigitalWriteFast.h"

// Force feedback(encoder & motor)
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

  int lastButton1State = 0;
  int lastButton2State = 0;

// Potentiometers
  #define acceleratorPin A0
  #define brakePin A1
  #define clutchPin A2
  #define dummyButton1 9

// Serial monitor config
  #define PRINT_PEDALS true
  #define PRINT_MOTOR false
  #define PRINT_ENCODER false

// Floor pedal filter config
  bool use_EMA = true; // Exponential Moving Average
  int LSB_TO_IGNORE = 1; // how many LSB to ignore from incoming potentiometer info

// configure EMA and LSB reduction filters for floor pedals
  int offset = 436; //offset pedal output
  float EMA_alpha = 0.9; //initialization of EMA alpha

  int dummyButtonState = 0;

// initialize starting pedal values for t = 1
  int acceleratorCommanded, brakeCommanded, clutchCommanded,
      accelPreviousState, brakePreviousState, clutchPreviousState
      = 0;

// create steering wheel "joystick"
Joystick_ WheelController(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  8, 0,                  // Button Count, Hat Switch Count
  true, true, true,     // X and Y, but no Z Axis
  true, true, true,   //  Rx, Ry, Rz
  false, false,          // No rudder or throttle
  true, true, false);    // No accelerator, brake, or steering

////////////////////////////////////////////////////////////////////////////////////////

// Start IO
void setup() {

  attachInterrupt(digitalPinToInterrupt(encoderPinA),tick,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB),tick,CHANGE);

  WheelController.setRyAxisRange(0, 500);
  WheelController.setRxAxisRange(0, 500);
  WheelController.setYAxisRange(0, 500);
  WheelController.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  WheelController.setGains(gains);
  WheelController.begin(true);

  // pinMode(motorPinA, OUTPUT);
  // pinMode(motorPinB, OUTPUT);
  // pinMode(motorPinPWM, OUTPUT);
  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorR_PWM, OUTPUT);

  //pinMode(A0, INPUT_PULLUP);

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

  // Floor pedals
  acceleratorCommanded = analogRead(acceleratorPin);  //set accelerator EMA for t=1
  brakeCommanded = analogRead(brakePin); // set brake EMA for t=1
  clutchCommanded = analogRead(clutchPin); // set clutch EMA for t=1
  
  // Dummy button
  pinMode(dummyButton1, INPUT_PULLUP);

  // Serial Monitor
  Serial.begin(115200);
  Serial.println("Started");
}

ISR(TIMER3_COMPA_vect){
  WheelController.getUSBPID();
}

unsigned int interval = 0;

void loop() {

  forceFeedbackLoop();
  pedalLoop();
  //SerialMonitorLoop();
}

void forceFeedbackLoop(){
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

  WheelController.setXAxis(value);
  WheelController.setRxAxis(analogRead(A1));
  WheelController.setRyAxis(analogRead(A2));
  WheelController.setYAxis(analogRead(A3));

  effectparams[0].springMaxPosition = ENCODER_MAX_VALUE;
  effectparams[0].springPosition = value;
  effectparams[1].springMaxPosition = 255;
  effectparams[1].springPosition = 0;
  WheelController.setEffectParams(effectparams);
  WheelController.getForce(forces);

  
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
    WheelController.setButton(0, currentButton1State);
    //Update the Stored Button State
    lastButton1State = currentButton1State;
  }

  int currentButton2State = !digitalRead(SHIFT_DOWN);
  if (currentButton2State != lastButton2State){
    WheelController.setButton(1, currentButton2State);
    lastButton2State = currentButton2State;
  }
}

void pedalLoop(){ // This section is kept as compact as possible in order to maximize floor pedal accuracy and efficency.

  acceleratorCommanded = (analogRead(acceleratorPin) >> LSB_TO_IGNORE) + offset;
  brakeCommanded = (analogRead(brakePin) >> LSB_TO_IGNORE) + offset;
  clutchCommanded = (analogRead(clutchPin) >> LSB_TO_IGNORE) + offset;

  // EMA filtering
  if(use_EMA){

    // run commanded values in the EMA
    acceleratorCommanded = EMA(accelPreviousState, acceleratorCommanded);
    brakeCommanded = EMA(brakePreviousState, brakeCommanded);
    clutchCommanded = EMA(clutchPreviousState, clutchCommanded);

    // set current potentiometer state as the new "previous" state
    accelPreviousState = acceleratorCommanded;
    brakePreviousState = brakeCommanded;
    clutchPreviousState = clutchCommanded;
  }

  // map controller to commanded potentiometer values
  WheelController.setAccelerator(acceleratorCommanded);
  WheelController.setRzAxis(brakeCommanded);
  //WheelController.setZAxis(clutchCommanded);

  int currentButton1State = !digitalRead(dummyButton1);
  //If loop - Check that the button has actually changed.
  if (currentButton1State != dummyButtonState){
    //If the button has changed, set the specified HID button to the Current Button State
    WheelController.setButton(0, currentButton1State);
    //Update the Stored Button State
    dummyButtonState = currentButton1State;
  }
}

void SerialMonitorLoop(){
  if(PRINT_PEDALS){
    Serial.print("Accelerator = ");
    Serial.print(acceleratorCommanded);
    Serial.print(" | Brake = ");
    Serial.print(brakeCommanded);
    Serial.print(" | Clutch = ");
    Serial.println(clutchCommanded);
  }
}

// Force feedback functions
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


// Floor pedal functions
int EMA(int previousState, int currentState){
  int y = currentState*EMA_alpha + (1-EMA_alpha)*previousState;
  return y;
}
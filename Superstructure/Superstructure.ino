/*
 * THE STEERING WHEEL OF ALL TIME
 */

// Include libraries
  #include <Joystick.h>
  #include <Encoder.h>
  #include <Arduino.h>

// Serial Monitor
  long baudRate = 9600;

// Encoder
  #define A 0
  #define B 1
  volatile byte temp = 0;
  volatile byte counter = 0;

// Motor
  #define SENSOR_PIN A4 // center pin of the potentiometer(motor)
  const int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
  const int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

// Potentiometers
  #define acceleratorPin A0
  #define brakePin A1
  #define clutchPin A2
  #define dummyButton1 9

// Joystick constructors
  #define INCLUDE_X_AXIS false
  #define INCLUDE_Y_AXIS false
  #define INCLUDE_Z_AXIS true
  #define INCLUDE_RX_AXIS false
  #define INCLUDE_RY_AXIS false
  #define INCLUDE_RZ_AXIS false
  #define INCLUDE_RUTTER false
  #define INCLUDE_THROTTLE false
  #define INCLUDE_ACCELERATOR true
  #define INCLUDE_BRAKE true
  #define INCLUDE_STEERING false

// Serial monitor config
  #define PRINT_PEDALS true
  #define PRINT_MOTOR false
  #define PRINT_ENCODER false

// Floor pedal filter config
  bool EMA = true; // Exponential Movement Average
  int LSB_TO_IGNORE = 1; // how many LSB to ignore from incoming potentiometer info

// configure EMA and LSB reduction filters for floor pedals
  int offset = 436; //offset pedal output
  float EMA_alpha = 0.6;      //initialization of EMA alpha

  int dummyButtonState = 0;

// initialize starting pedal values for t = 1
  int acceleratorCommanded, brakeCommanded, clutchCommanded,
      accelPreviousState, brakePreviousState, clutchPreviousState
      = 0;

// create floor pedal "joystick"
Joystick_ pedalController(0x12, JOYSTICK_TYPE_JOYSTICK, 1, 0, INCLUDE_X_AXIS, INCLUDE_Y_AXIS, INCLUDE_Z_AXIS, INCLUDE_RX_AXIS, INCLUDE_RY_AXIS, INCLUDE_RZ_AXIS, INCLUDE_RUTTER, INCLUDE_THROTTLE, INCLUDE_ACCELERATOR, INCLUDE_BRAKE, INCLUDE_STEERING); // zAxis, accel, brake = true

////////////////////////////////////////////////////////////////////////////////////////

// Start IO
void setup() {
  // Dummy button
  pinMode(dummyButton1, INPUT_PULLUP);

  // Encoder
  pinMode(A, INPUT_PULLUP);
  pinMode(B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A), A_INTERRUPT, RISING);
  attachInterrupt(digitalPinToInterrupt(B), B_INTERRUPT, RISING);

  // Motor
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  // Floor pedals
  acceleratorCommanded = analogRead(acceleratorPin);  //set accelerator EMA for t=1
  brakeCommanded = analogRead(brakePin); // set brake EMA for t=1
  clutchCommanded = analogRead(clutchPin); // set clutch EMA for t=1
  pedalController.begin();
  
  // Serial Monitor
  Serial.begin(baudRate);
  Serial.println("Started");
}

void loop() {

  encoderLoop();
  motorLoop();
  pedalLoop();
  SerialMonitorLoop();
}

// loop functions
void encoderLoop(){
  if (counter != temp) {
    Serial.println(counter);
    temp = counter;
  }
}

void motorLoop(){
  int sensorValue = analogRead(SENSOR_PIN);
  setMotorPower(sensorValue);
}

void pedalLoop(){ // This section is kept as compact as possible in order to maximize floor pedal accuracy and efficency.

  acceleratorCommanded = (analogRead(acceleratorPin) >> LSB_TO_IGNORE) + offset;
  brakeCommanded = (analogRead(brakePin) >> LSB_TO_IGNORE) + offset;
  clutchCommanded = (analogRead(clutchPin) >> LSB_TO_IGNORE) + offset;

  // EMA filtering
  if(EMA){

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
  pedalController.setAccelerator(acceleratorCommanded);
  pedalController.setBrake(brakeCommanded);
  pedalController.setZAxis(clutchCommanded);

  int currentButton1State = !digitalRead(dummyButton1);
  //If loop - Check that the button has actually changed.
  if (currentButton1State != dummyButtonState){
    //If the button has changed, set the specified HID button to the Current Button State
    pedalController.setButton(0, currentButton1State);
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

// Floor pedal functions
int EMA(int previousState, int currentState){
  
  int y = currentState*EMA_alpha
          + (1-EMA_alpha)*previousState;
  return y;
}

// Encoder functions
void A_INTERRUPT() {
  (LOW==digitalRead(A)) ? counter++ : counter--;
}

void B_INTERRUPT() {
  (LOW==digitalRead(B)) ? counter-- : counter++;
}

// Motor functions
void setMotorPower(int sensorValue){
    (512 > sensorValue) ? turn(true, sensorValue) : turn(false, sensorValue);
}

void turn(bool direction, int sensorValue){
  if (direction)
  {
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, -(sensorValue - 511) / 2);
  }
  else
  {
    analogWrite(LPWM_Output, (sensorValue - 512) / 2);
    analogWrite(RPWM_Output, 0);
  }
}
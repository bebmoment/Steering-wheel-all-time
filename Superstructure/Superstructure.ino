/*
 * THE STEERING WHEEL OF ALL TIME
 */

// Include libraries
  #include <Joystick.h>
  #include <Encoder.h>
  #include <Arduino.h>
  #include <Joystick.h>

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

//Potentiometers
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

  bool EMA = true; // Exponential Movement Average
  bool ignoreLSB = true; // ignore least significant bits

    // configure EMA and LSB reduction filters for floor pedals
  int LSB_TO_IGNORE = 1; // how many LSB to ignore from incoming potentiometer info
  float EMA_a = 0.6;      //initialization of EMA alpha

  int dummyButtonState = 0;

  // initialize starting pedal values for t = 1
  int acceleratorValue, brakeValue, clutchValue,
      EMA_ACCELERATOR, EMA_BRAKE, EMA_CLUTCH, 
      acceleratorCommanded, brakeCommanded, clutchCommanded 
      = 0;

////////////////////////////////////////////////////////////////////////////////////////

Joystick_ pedalController(0x12, JOYSTICK_TYPE_JOYSTICK, 1, 0, INCLUDE_X_AXIS, INCLUDE_Y_AXIS, INCLUDE_Z_AXIS, INCLUDE_RX_AXIS, INCLUDE_RY_AXIS, INCLUDE_RZ_AXIS, INCLUDE_RUTTER, INCLUDE_THROTTLE, INCLUDE_ACCELERATOR, INCLUDE_BRAKE, INCLUDE_STEERING); // zAxis, accel, brake = true

void setup() {
    startSerialMonitor();
    startEncoder();
    startMotor();
    startPedals();
}

void loop() {
  
}

// setup functions
void startSerialMonitor(){
  Serial.begin(baudRate);
  Serial.println("Started");
}

void startEncoder(){

    pinMode(A, INPUT_PULLUP);
    pinMode(B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(A), A_INTERRUPT, RISING);
    attachInterrupt(digitalPinToInterrupt(B), B_INTERRUPT, RISING);
}

void startMotor(){
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
}

void startPedals(){
  if(EMA){
    EMA_ACCELERATOR = analogRead(acceleratorPin);  //set accelerator EMA for t=1
    EMA_BRAKE = analogRead(brakePin); // set brake EMA for t=1
    EMA_CLUTCH = analogRead(clutchPin); // set clutch EMA for t=1
  }
}

void serialMonitorLoop(){
    printOutputs(ignoreLSB, EMA);
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

void pedalLoop(){
  // LSB reduction
  if(ignoreLSB){
    acceleratorValue = analogRead(acceleratorPin) >> LSB_TO_IGNORE;
    brakeValue = analogRead(brakePin) >> LSB_TO_IGNORE;
    clutchValue = analogRead(clutchPin) >> LSB_TO_IGNORE;
  }else{
    acceleratorValue = analogRead(acceleratorPin);
    brakeValue = analogRead(brakePin);
    clutchValue = analogRead(clutchPin);
  }

  // EMA filtering
  if(EMA){
    EMA_ACCELERATOR = (EMA_a*acceleratorValue) + ((1-EMA_a)*EMA_ACCELERATOR);    //run accelerator EMA
    EMA_BRAKE = (EMA_a*brakeValue) + ((1-EMA_a)*EMA_BRAKE);    //run brake EMA
    EMA_CLUTCH = (EMA_a*clutchValue) + ((1-EMA_a)*EMA_CLUTCH);    //run brake EMA

    acceleratorCommanded = EMA_ACCELERATOR + 512;
    brakeCommanded = EMA_BRAKE + 512;
    clutchCommanded = EMA_CLUTCH + 512;
  
    printOutputs(acceleratorCommanded); //print digital value to serial... add 512 as map input range is 512-1023
  }
  else{
    acceleratorCommanded = acceleratorValue + 512;
    brakeCommanded = brakeValue + 512;
    clutchCommanded = clutchValue + 512;
    
    printOutputs(acceleratorCommanded);
  }
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

// Encoder functions
void A_INTERRUPT() {
  (LOW==digitalRead(A)) ? counter++ : counter--;
}

void B_INTERRUPT() {
  (LOW==digitalRead(B)) ? counter-- : counter++;
}

// Motor functions
void setMotorPower(int sensorValue)
{
    (512 > sensorValue) ? turn(true, sensorValue) : turn(false, sensorValue);
}

void turn(bool direction, int sensorValue)
{
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

void printOutputs(bool ignoreLSB_, bool EMA_){
  // printing noise filter configuration
  if(ignoreLSB_){
    Serial.print("ignoreLSB | ");
  }if(EMA_){
    Serial.print("EMA | ");
  }
}
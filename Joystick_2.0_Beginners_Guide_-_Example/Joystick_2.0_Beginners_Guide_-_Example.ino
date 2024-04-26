//Arduino Joystick 2.0 Library, by MHeironimus (https://github.com/MHeironimus)
//Beginners Guide, by Daniel Cantore
//Example Code (Oct 2020), with in-depth commenting 

//Initial Definitions and Setup
//Libary Inclusion
#include <Joystick.h>
#include <Encoder.h>
//Define and Allocate Input Pins to memorable names
#define joyX A0
#define joyY A1
// #define joyRZ A3
// #define joyThrottle A2
#define joyButton1 9
#define joyButton2 8
#define joyButton3 7

#define ENCODER_INIT_VALUE -999

// bunch of booleans to know what the heck is the constructor
#define INCLUDE_X_AXIS false
#define INCLUDE_Y_AXIS false
#define INCLUDE_Z_AXIS false
#define INCLUDE_RX_AXIS false
#define INCLUDE_RY_AXIS false
#define INCLUDE_RZ_AXIS false
#define INCLUDE_RUTTER false
#define INCLUDE_THROTTLE false
#define INCLUDE_ACCELERATOR false
#define INCLUDE_BRAKE false
#define INCLUDE_STEERING true
//Initializing Axis as Integers, at a 0 default value
int xAxis_ = 0;
int yAxis_ = 0;
int rzAxis_ = 0;
int throttle_ = 0;
int steeringAxis = 0;

//Setting up Buttons
//Updating a static variable gives greater stability than reading directly from the digital pin.
//Giving Default Values to the Buttons for later use
  int lastButton1State = 0;
  int lastButton2State = 0;
  int lastButton3State = 0;

//Defining the Joystick
//The Joystick is defined in the following setup:
//Joystick(Joystick HID ID, Joystick Type, Button Count, Hat Switch Count, Include X, Include Y, Include Z, Include Rx, Include Ry, Include Rz, Include Rudder, Include Throttle, Include Accelerator, Include Brake, Include Steering
//Joystick HID ID: A Hex value identifier for HID Device Recognition (default: 0x03). DO NOT USE 0x01 or 0x02
//Joystick type: Define the type of joystick from the types supported. Types: DEFAULT Joystick (0x04 or JOYSTICK_TYPE_JOYSTICK), Gamepad (0x05 or JOYSTICK_TYPE_GAMEPAD), Multi-Axis Controller (0x08 or JOYSTICK_TYPE_MULTI_AXIS)
//Button Count: Number of Buttons shown to HID system (default: 32)
//Hat Switch Count: Number of Hat Switches, max 2. (default:2)
//Include X Axis: Determines whether the X axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Y Axis: Determines whether the Y axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Z Axis: Determines whether the Z axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Rx Axis: Determines whether the X Rotational axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Ry Axis: Determines whether the Y Rotational axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Rz Axis: Determines whether the Z Rotational axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Rudder: Determines whether a Rudder axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Throttle: Determines whether a Throttle axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Accelerator: Determines whether an Accelerator axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Brake: Determines whether a Brake axis is avalible for used by the HID system, defined as a bool value (default:true)
//Include Steering: Determines whether a Steering axis is avalible for used by the HID system, defined as a bool value (default:true)

Joystick_ Joystick(0x12, JOYSTICK_TYPE_JOYSTICK, 3, 0,INCLUDE_X_AXIS,INCLUDE_Y_AXIS,INCLUDE_Z_AXIS,INCLUDE_RX_AXIS,INCLUDE_RY_AXIS,INCLUDE_RZ_AXIS,INCLUDE_RUTTER,INCLUDE_THROTTLE,INCLUDE_ACCELERATOR,INCLUDE_BRAKE,INCLUDE_STEERING);
Encoder myEnc(0, 1);
//Set Auto Send State
//Enables Auto Sending, allowing the controller to send information to the HID system, rather than waiting to be asked.
const bool initAutoSendState = true;

void setup() {
  //Initialize Buttons
  //Buttons set up between Digital Pin and Ground, following pin allocations from earlier on
  pinMode(joyButton1, INPUT_PULLUP);
  pinMode(joyButton2, INPUT_PULLUP);
  pinMode(joyButton3, INPUT_PULLUP);
  Serial.begin(9600);
  //Start Joystick - Needed to start the Joystick function libary
  Joystick.begin();
}
long oldPosition = ENCODER_INIT_VALUE; 
void loop() {
  
  //Axis Reading during Runtime
  //Setting Read functions for each axis and parsing correctly. The X axis will be used as an example for explanation

  //Reading the X Axis analog pin to the xAxis_ variable for processing
  // xAxis_ = analogRead(joyX);
  //Mapping the X Axis data from a 0-1023 to 0-255 range for a smoother action
  // xAxis_ = map(xAxis_,4,1023,-255,255);
  //Set the Joystick X Axis value as the new, smoother, value
  // Joystick.setXAxis(xAxis_);
  // Serial.println(xAxis_);

  // yAxis_ = analogRead(joyY);
  // yAxis_ = map(yAxis_,4,1023,-255,255);
  // Joystick.setYAxis(yAxis_);
  // Serial.println(yAxis_);

  // rzAxis_ = analogRead(joyRZ);
  // rzAxis_ = map(rzAxis_,0,1023,0,255);
  // Joystick.setRzAxis(rzAxis_);

  // throttle_ = analogRead(joyThrottle);
  // throttle_ = map(throttle_,0,1023,0,255);
  // Joystick.setThrottle(throttle_);
  
  //Button Reading during Runtime
  //Setting Read functions for each button, using a state value for memory. Button 1 will be used as an example for explanation

  //Reading the current Button digital pin to the Current Button State for processing
  int currentButton1State = !digitalRead(joyButton1);
  //If loop - Check that the button has actually changed.
  if (currentButton1State != lastButton1State){
    //If the button has changed, set the specified HID button to the Current Button State
    Joystick.setButton(0, currentButton1State);
    //Update the Stored Button State
    lastButton1State = currentButton1State;
  }

    int currentButton2State = !digitalRead(joyButton2);
  if (currentButton2State != lastButton2State){
    Joystick.setButton(1, currentButton2State);
    lastButton2State = currentButton2State;
  }
    int currentButton3State = !digitalRead(joyButton3);
  if (currentButton3State != lastButton3State){
    Joystick.setButton(2, currentButton3State);
    lastButton3State = currentButton3State;
  }
  // read from the encoder and make send its reading to the arduino as joystick
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition); 
    steeringAxis = newPosition;
    Joystick.setSteering(steeringAxis);
  }

//Pole Delay/Debounce
//To reduce unessecary processing, the frequency of the reading loop is delayed. The value(in ms) can be changed to match requirement
// delay(10);
}

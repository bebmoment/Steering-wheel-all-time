#include <Arduino.h>
#include <Joystick.h>

//initialize joysticks
#define joyX A0
#define acceleratorPin A0
#define brakePin A1
#define clutchPin A2
#define joyButton1 9
// Noise filtering
bool EMA = true; // Exponential Movement Average
bool ignoreLSB = true; // ignore least significant bits

//Global Variables
int lastButton1State = 0;
int xAxis = 0;
// int acceleratorPin = 3;//potentiometer pin #
int acceleratorValue = 0;    //initialization of potentiometer value, equivalent to EMA Y
int brakeValue = 0;
int clutchValue = 0;
int LSB_TO_IGNORE = 1; // # of least significant bits to ignore
int EMA_ACCELERATOR = 0;  //set accelerator EMA for t=1
int EMA_BRAKE = 0; // set brake EMA for t=1
int EMA_CLUTCH = 0; // set clutch EMA for t=1
int acceleratorCommanded = 0;
int brakeCommanded = 0;
int clutchCommanded = 0;
float EMA_a = 0.6;      //initialization of EMA alpha

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

Joystick_ Joystick(0x12, JOYSTICK_TYPE_JOYSTICK, 1, 0,false,false,true,false,false,false,false,false,true,true,false); // zAxis, accel, brake = true

void setup(){
  Serial.begin(9600);
  Joystick.begin();

  if(EMA){
    EMA_ACCELERATOR = analogRead(acceleratorPin);  //set accelerator EMA for t=1
    EMA_BRAKE = analogRead(brakePin); // set brake EMA for t=1
    EMA_CLUTCH = analogRead(clutchPin); // set clutch EMA for t=1
  }
}
 
void loop(){
  // printing noise filter configuration
  if(ignoreLSB && EMA){
    Serial.print("EMA + ignoreLSB: ");
  }else if(ignoreLSB){
    Serial.print("ignoreLSB: ");
  }else if(EMA){
    Serial.print("EMA: ");
  }else{
    Serial.print("Unfiltered: ");
  }

  // LSB reduction
  if(ignoreLSB){
    acceleratorValue = analogRead(acceleratorPin) >> LSB_TO_IGNORE;
    brakeValue = analogRead(brakePin) >> LSB_TO_IGNORE;
    clutchValue = analogRead(clutchValue) >> LSB_TO_IGNORE;
  }else{
    acceleratorValue = analogRead(acceleratorPin);
    brakeValue = analogRead(brakePin);
    clutchValue = analogRead(clutchValue);
  }

  // EMA filtering
  if(EMA){
    EMA_ACCELERATOR = (EMA_a*acceleratorValue) + ((1-EMA_a)*EMA_ACCELERATOR);    //run accelerator EMA
    EMA_BRAKE = (EMA_a*brakeValue) + ((1-EMA_a)*EMA_BRAKE);    //run brake EMA
    EMA_CLUTCH = (EMA_a*clutchValue) + ((1-EMA_a)*EMA_CLUTCH);    //run brake EMA

    acceleratorCommanded = EMA_ACCELERATOR + 512;
    brakeCommanded = EMA_BRAKE + 512;
    clutchCommanded = EMA_CLUTCH + 512;
  
    //Serial.println(acceleratorCommanded); //print digital value to serial... add 512 as map input range is 512-1023
  }
  else{
    acceleratorCommanded = acceleratorValue + 512;
    brakeCommanded = brakeValue + 512;
    clutchCommanded = clutchValue + 512;
    
    //Serial.println(acceleratorCommanded);
  }
  Joystick.setAccelerator(acceleratorCommanded);
  Joystick.setBrake(brakeCommanded);
  Joystick.setZAxis(clutchCommanded);

  int currentButton1State = !digitalRead(joyButton1);
  //If loop - Check that the button has actually changed.
  if (currentButton1State != lastButton1State){
    //If the button has changed, set the specified HID button to the Current Button State
    Joystick.setButton(0, currentButton1State);
    //Update the Stored Button State
    lastButton1State = currentButton1State;
  }
}
#include <Arduino.h>
#include <Joystick.h>

//initialize joysticks
#define joyX A0

// Noise filtering
bool EMA = true; // Exponential Movement Average
bool ignoreLSB = true; // ignore least significant bits

//Global Variables

int xAxis = 0;
int potentiometerPin = 3;//potentiometer pin #
int potValue = 0;    //initialization of potentiometer value, equivalent to EMA Y
int LSB_TO_IGNORE = 1; // # of least significant bits to ignore
int EMA_S = 0;  //set EMA S for t=1
int xCommanded = 0;
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

Joystick_ Joystick(0x12, JOYSTICK_TYPE_JOYSTICK, 0, 0,false,false,false,false,false,false,false,false,true,false,false);

void setup(){
  Serial.begin(9600);
  Joystick.begin();

  if(EMA){
    int EMA_S = analogRead(potentiometerPin);  //set EMA S for t=1
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
    potValue = analogRead(potentiometerPin) >> LSB_TO_IGNORE;
  }else{
    potValue = analogRead(potentiometerPin);
  }

  // EMA filtering
  if(EMA){
    EMA_S = (EMA_a*potValue) + ((1-EMA_a)*EMA_S);    //run the EMA
    xCommanded = EMA_S + 512;
    Joystick.setAccelerator(xCommanded);
    Serial.println(xCommanded); //print digital value to serial... add 512 as map input range is 512-1023
  }
  else{
    xCommanded = potValue + 512;
    Joystick.setAccelerator(xCommanded);
    Serial.println(xCommanded);
  }
}
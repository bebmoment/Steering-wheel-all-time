#include <Joystick.h>

#include <Encoder.h>

#include <Arduino.h>

#define SENSOR_PIN A4      // center pin of the potentiometer
const int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
void setup()
{
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
}
void loop()
{
    int sensorValue = analogRead(SENSOR_PIN);
    setMotorPower(sensorValue);
}

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

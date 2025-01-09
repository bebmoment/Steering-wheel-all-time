# Steering Wheel of All Time

The Steering Wheel of All Time is a project that implements a force-feedback steering wheel, shifter, and floor pedals using an Arduino Leonardo. This setup aims to provide a realistic and responsive driving experience in virtual simulations. The system integrates sensor feedback from encoders and potentiometers, along with a motor to simulate vibrations and steering resistance.

## Getting Started

1. **Clone the repository**:
    ```sh
    git clone [repository url]
    ```

2. **Open the project in Arduino IDE**:
    - Open the `.ino` files in the respective directories.

3. **Upload the code to your Arduino board**:
    - Connect your Arduino board to your computer.
    - Select the appropriate board and port in the Arduino IDE.
    - Upload the code.

## Dependencies

- [Joystick Library](https://github.com/MHeironimus/ArduinoJoystickLibrary)
- [Encoder Library](https://www.pjrc.com/teensy/td_libs_Encoder.html)

## Usage

- Connect the pedals, shifter and steering wheel motor as per the pin definitions in <code>Superstructure.ino</code>.
- Deploy the code onto the Arduino board.
- The steering wheel should now be functional with force feedback and pedal inputs.

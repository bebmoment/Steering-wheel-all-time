# Steering Wheel of All Time

The Steering Wheel of All Time is a project that implements a force-feedback steering wheel, shifter, and floor pedals using an Arduino Leonardo. This setup aims to provide a realistic and responsive driving experience in virtual simulations. The system integrates sensor feedback from encoders and potentiometers, along with a motor to simulate vibrations and steering resistance.

## Getting Started

1. **Clone the repository**:
    ```sh
    git clone [repository url]
    ```

2. **Dependencides**:
    - Install the dependencies for this project:
        - [Joystick Library](https://github.com/MHeironimus/ArduinoJoystickLibrary)
        - [Encoder Library](https://www.pjrc.com/teensy/td_libs_Encoder.html)

3. **Upload the code to the Arduino board**:
    - Connect the Leonardo to the device on which the project has been set-up.
    - Deploy code.

## Usage

- Connect the pedals, shifter and steering wheel motor as per the pin definitions in <code>Superstructure.ino</code>.
- Deploy the code onto the Arduino board.
- The steering wheel should now be functional with force feedback and pedal inputs.

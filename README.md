# Steering Wheel of All Time

The Steering Wheel of All Time is a project that implements a force-feedback steering wheel, shifter, and floor pedals using an Arduino Leonardo. This setup aims to provide a realistic and responsive driving experience in virtual simulations. The system integrates sensor feedback from encoders and potentiometers, along with a motor to simulate vibrations and steering resistance.

## Usage

1. **Clone the repository**:
    ```sh
    git clone [repository url]
    ```

2. **Dependencides**:
    - Install the dependencies for this project:
        - [Joystick Library](https://github.com/MHeironimus/ArduinoJoystickLibrary)
        - [Encoder Library](https://www.pjrc.com/teensy/td_libs_Encoder.html)


3. **Upload the code to the Arduino board**:
    - Connect the Arduino to the device on which the project has been set-up.
    - Deploy code onto the board.


4. **Initializing the controller**
    - Connect the pedals, shifter and steering wheel motor as per the pin definitions in <code>Superstructure.ino</code>.
    - In-simulation controller set-up. (Various applications may have differing controls)
    - The steering wheel should now be functional with force feedback, shifter and pedal inputs.
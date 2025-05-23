# Arduino Ride-On Vehicle Controller (Accessible)
<img src="https://github.com/user-attachments/assets/f1b44f85-2b2b-48dc-9eb3-cc79c6233b46" alt="pod car" width="50%">

## Description

This Arduino project provides firmware for controlling a modified ride-on vehicle using an Arduino Uno R4 Minima (or compatible board). It's designed with accessibility in mind for children with disabilities, offering direct assistive switch input and sip-and-puff control. It features various operational modes to cater to different needs and skill levels. The controller saves key configuration settings to EEPROM and includes safety mechanisms like switch debouncing.

**Important Note on Motor Control (Current Sink Method):**
This firmware is designed for ride-on vehicles where the existing motor controller activates specific functions (e.g., Right Motor Forward, Left Motor Reverse) when its corresponding control line is connected to Ground (GND), effectively "sinking current" through the Arduino pin. The Arduino digital pins, when set to `OUTPUT LOW`, act as this connection to Ground. When set to `INPUT` (high-impedance), they effectively disconnect the control line, deactivating the function. **This method requires careful identification of the correct control lines on your vehicle's existing motor controller. Incorrect wiring can damage the vehicle's controller or the Arduino.**

## Features

* **Multiple Input Methods:**
    * **Direct Switch Input:** Standard control via up to four normally-open assistive switches for Forward, Reverse, Clockwise, and Counter-Clockwise movements.
    * **Sip-and-Puff Input:** Control using sequences of sips and puffs (e.g., Sip-Sip for Forward). Sip/Puff detection thresholds are configurable via serial commands.
* **Multiple Activation Modes (for the selected command):**
    * **Direct Mode:** Motors are active only while the command's condition is met (e.g., switch held, or immediately after a sip-puff command).
    * **Latching Mode:** Activate a command once (e.g., by a switch press or a completed sip-puff sequence) to keep it active; activate again or select another to change/deactivate.
    * **Timed Latch Mode:** Activate a command for a predefined duration (e.g., 5 seconds for linear movement, or a shorter calibrated duration for turns).
* **Operation Modes:**
    * **Normal Operation:** Standard vehicle behavior.
    * **One-Touch Mode:** Commands run for a short, fixed duration upon activation and then stop automatically.
    * **Gentle Mode:** Introduces a configurable delay between command executions for smoother control.
    * **Practice Mode:** Processes inputs and gives feedback (LEDs, Serial) but does not activate motors, allowing for safe practice.
* **Mode Cycling:** A dedicated input pin (Pin 9 by default) allows cycling through the main `ActivationMode` (Direct, Latching, Timed Latch).
* **EEPROM Configuration Saving:** Key settings (Activation Mode, Operation Mode, Input Method) are saved to EEPROM and loaded on startup.
* **Assistive Switch Interface:** Uses digital input pins with pull-up resistors, designed for normally open switches that connect to ground when closed.
* **Robust Switch Debouncing:** Prevents multiple activations from a single switch press.
* **LED Indicators:**
    * Mode LED (Pin 11): Indicates current Activation Mode.
    * Command LED (Pin 12): Indicates if a command is currently active.
    * Status LED (Pin 13): General system status/ready indicator.
* **Safety Features:**
    * Prevents activation of conflicting motor commands (motors stop if multiple command switches are pressed in Direct Mode with switch input).
* **Modular & Well-Commented Code.**
* **Serial Debugging & Configuration Output:** Provides detailed information and allows configuration changes via the Serial Monitor (e.g., setting input method, sip/puff thresholds).

## ⚠️ Important Cautions & Disclaimer ⚠️

* **Electrical Safety:** Modifying electronic devices, especially children's ride-on toys, involves inherent risks. If you are not competent and comfortable with electronics, wiring, and soldering, **do not attempt this project.** Incorrect wiring can lead to short circuits, damage to the Arduino, damage to the vehicle's electronics, or even create a fire hazard. Always double-check your connections before applying power.
* **Common Ground Connection (Crucial!):** For the Arduino to reliably control the vehicle's motor controller, they **must share a common reference voltage, known as a "common ground."**
    * **What this means (for laypeople):** Think of electricity like water flowing. For different electrical parts to "understand" each other's signals (like "on" or "off"), they need to agree on what "zero volts" or the "ground level" is. If they don't share this common ground, the signals from the Arduino might not be correctly interpreted by the vehicle's controller, leading to erratic behavior or no control at all.
    * **How to achieve it:** The simplest way is often to power the Arduino from the vehicle's own battery system (making sure the voltage is appropriate for the Arduino, possibly using a voltage regulator). If the Arduino and the vehicle's controller are powered by the same battery, they will inherently share a common ground. If they are powered separately, you **must** connect a wire from one of the Arduino's GND pins to the Ground (negative terminal) of the vehicle's battery or motor controller.
    * **Failure to do this can result in unpredictable operation or complete failure of the control system.**
* **Vehicle Compatibility:** This project assumes a specific type of motor control interface on the ride-on vehicle (control lines activated by connecting to Ground). Not all ride-on toys use this system. You **must** verify how your vehicle's controller works before attempting to interface with it.
* **Use At Your Own Risk:** This software and accompanying documentation are provided "as is" without warranty of any kind, express or implied. The authors and contributors are not responsible for any damage to property, injury, or other negative outcomes that may result from building, using, or modifying this project. You assume full responsibility for testing the safety and functionality of your modified vehicle.
* **Supervision Required:** Any ride-on vehicle modified with this controller should only be used under close adult supervision, especially if used by individuals with disabilities. Ensure the environment is safe and free of hazards.

## Hardware Requirements

* **Arduino Board:** Developed for Arduino Uno R4 Minima. Adaptable to other boards with sufficient digital/analog pins and EEPROM.
* **Ride-On Vehicle:** A ride-on vehicle with an accessible motor controller that uses LOW signals (connection to Ground) to activate motor directions.
* **Assistive Input Devices:**
    * Up to four normally open command switches (for switch input).
    * One mode cycle switch.
    * A sip-and-puff sensor with analog output (for sip-and-puff input).
* **Wiring Components:** Wires, connectors, (optional) 3.5mm jacks, soldering equipment.
* **Power Supply:** Appropriate power supply for the Arduino and the ride-on vehicle. **Ensure a common ground connection as described in the Cautions section.**

### Example of Potentially Compatible Vehicle
The following is an example of a ride-on vehicle that *may* be compatible with the control methods described in this project. However, **it is crucial to independently verify the control mechanism of any specific vehicle before attempting modification, as manufacturers can change components and wiring without notice.**
* Kidzone 2-Speeds Electric Ride On Bumper Car: [https://www.amazon.com/Kidzone-2-Speeds-Electric-Bluetooth-Certified/dp/B0CM5MMXRX](https://www.amazon.com/Kidzone-2-Speeds-Electric-Bluetooth-Certified/dp/B0CM5MMXRX)
    * *Disclaimer: This link is provided as a potential starting point for investigation and does not constitute an endorsement or guarantee of compatibility.*
    * *For reference, the following four pins are used on the unit that I have worked with most recently:*
    * <img src="https://github.com/user-attachments/assets/1607a5da-9572-4cf9-bca0-f78e6250d1d8" alt="1000046798" width="50%">
    * * Yellow - left reverse
    * * Green - left forward
    * * Orange - right reverse
    * * Blue - right forward

## Software Requirements

* **Arduino IDE:** Version 1.8.x or 2.x.x.
* **Libraries:**
    * `<Arduino.h>` (Standard)
    * `<EEPROM.h>` (Standard)
* The main sketch file (e.g., `AccessibleRideOnController.ino`).

## Pinout

### Motor Control Output Pins (Arduino -> Vehicle Controller)
These Arduino pins are set to `INPUT` (high-impedance) to be OFF, and `OUTPUT LOW` to activate the corresponding motor line on the vehicle's controller by connecting it to Ground.

* **Digital Pin 1 (MOTOR_CTRL_RF_PIN):** Right Motor Forward (RF)
* **Digital Pin 2 (MOTOR_CTRL_RR_PIN):** Right Motor Reverse (RR)
* **Digital Pin 3 (MOTOR_CTRL_LF_PIN):** Left Motor Forward (LF)
* **Digital Pin 4 (MOTOR_CTRL_LR_PIN):** Left Motor Reverse (LR)

### Input Pins (Assistive Switches/Sensors -> Arduino)
Command switch pins (5-8) use `INPUT_PULLUP`. Connect one terminal of your switch to the Arduino pin and the other terminal to Ground (GND). **The Arduino's GND and the vehicle's electrical system GND must be connected.**

* **Digital Pin 5 (SWITCH_FORWARD_PIN):** Command: Forward
* **Digital Pin 6 (SWITCH_REVERSE_PIN):** Command: Reverse
* **Digital Pin 7 (SWITCH_CLOCKWISE_PIN):** Command: Rotate Clockwise
* **Digital Pin 8 (SWITCH_COUNTERCLOCKWISE_PIN):** Command: Rotate Counter-Clockwise
* **Digital Pin 9 (MODE_SWITCH_PIN):** Cycle Activation Mode
* **Analog Pin A0 (SIP_PUFF_PIN):** Analog input for sip-and-puff sensor.

### LED Indicator Pins (Arduino -> LEDs)
* **Digital Pin 11 (LED_MODE_PIN):** Mode indicator LED
* **Digital Pin 12 (LED_COMMAND_PIN):** Command indicator LED
* **Digital Pin 13 (LED_STATUS_PIN):** Status/ready indicator LED

## Configuration (via Code Constants & Serial Commands)

* **Default Modes (loaded from EEPROM if previously saved, otherwise defaults set in code):**
    * `currentActivationMode`: `DIRECT_MODE`
    * `currentOperationMode`: `NORMAL_OPERATION`
    * `currentInputMethod`: `SWITCH_INPUT`
* **Switch Debounce Delay:** `const unsigned long DEBOUNCE_DELAY = 50;` (ms)
* **Timed Latch Durations:**
    * Non-Rotational: `const unsigned long TIMED_LATCH_DURATION_NON_ROTATIONAL = 5000;` (ms)
    * Rotational: Calculated based on `DEGREES_PER_TIMED_ROTATION_PRESS` and `FULL_ROTATION_DURATION_MS`.
* **Sip-and-Puff Parameters (configurable via Serial):**
    * `config.sipThreshold` (Default: `SIP_THRESHOLD = 400` in code)
    * `config.puffThreshold` (Default: `PUFF_THRESHOLD = 600` in code)
    * `NEUTRAL_THRESHOLD = 50` (Deadband around neutral for sip-puff sensor)
    * `SIP_PUFF_SEQUENCE_TIMEOUT = 2000` (ms, max time between sip/puff actions in a sequence)
* **One-Touch Mode Duration:** `ONE_TOUCH_DURATION = 2000;` (ms)
* **Gentle Mode Delay:** `config.gentleModeDelay = 500;` (ms, configurable via serial command `gentledelay <ms>` - *Note: serial command not explicitly shown in provided help, but struct member exists*)
* **Serial Commands for Configuration (see `printHelp()` output in Serial Monitor for a complete list):**
    * `operation <normal|onetouch|gentle|practice>`
    * `input <switch|sippuff>`
    * `sipthreshold <value>`
    * `puffthreshold <value>`
    * `save` (to save current main modes to EEPROM)
    *(Note: While `config.gentleModeDelay`, `config.sipThreshold`, `config.puffThreshold` can be changed via serial, explicit EEPROM saving for these specific values beyond the main modes is not detailed in the `saveConfigToEEPROM` function.)*

## How to Use

1.  **Understand the Risks:** Read the "Important Cautions & Disclaimer" section thoroughly.
2.  **Hardware Setup:**
    * **Carefully identify** motor control signal lines on your vehicle's controller.
    * **Ensure a common ground connection** between Arduino and vehicle.
    * Connect Arduino motor output pins (1-4) to vehicle control lines.
    * Connect assistive switches/sip-puff sensor to Arduino input pins (5-9, A0).
    * Optionally, connect LEDs to pins 11-13 for visual feedback.
    * Power the Arduino and the vehicle.
3.  **Software Setup:**
    * Open the sketch in Arduino IDE.
    * Ensure Arduino R4 board support is installed.
    * Select "Arduino Uno R4 Minima" (or your board) and the correct COM port.
    * Upload the sketch.
4.  **Operation:**
    * Open Serial Monitor (115200 baud) for status messages and configuration.
    * Type `help` in the Serial Monitor to see available commands.
    * Use the Mode Switch (Pin 9) to cycle through Direct, Latching, and Timed Latch activation modes.
    * Use serial commands to change Input Method (switch/sippuff), Operation Mode, and sip/puff thresholds.
    * Use `save` to store the primary mode settings (Activation, Operation, Input Method) to EEPROM.
    * Operate the vehicle using the configured input method.
    * **Always test thoroughly in a safe, controlled environment before allowing regular use.**

## Contributing

Contributions are welcome! If you have improvements or bug fixes, please feel free to fork this repository and submit a pull request.

## License

This project is licensed under the GNU General Public License v3.0.
A copy of the license can typically be found in a file named `LICENSE` or `COPYING` in the root of the source code repository. If not, you can obtain it from [https://www.gnu.org/licenses/gpl-3.0.html](https://www.gnu.org/licenses/gpl-3.0.html).

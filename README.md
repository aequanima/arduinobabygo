# Arduino Ride-On Vehicle Controller

## Description

This Arduino project provides firmware for controlling a modified ride-on vehicle using an Arduino Uno R4 Minima (or compatible board). It's designed to interface with assistive technology switches (e.g., 3.5mm jack switches) to allow users with limited mobility to control the vehicle's movement. The controller supports multiple activation modes, saves the selected mode to EEPROM, includes power-saving sleep mode, and features safety mechanisms like switch debouncing and an inactivity timeout.

**Important Note on Motor Control (Current Sink Method):**
This firmware is designed for ride-on vehicles where the existing motor controller activates specific functions (e.g., Right Motor Forward, Left Motor Reverse) when its corresponding control line is connected to Ground (GND), effectively "sinking current" through the Arduino pin. The Arduino digital pins, when set to `OUTPUT LOW`, act as this connection to Ground. When set to `INPUT` (high-impedance), they effectively disconnect the control line, deactivating the function. **This method requires careful identification of the correct control lines on your vehicle's existing motor controller. Incorrect wiring can damage the vehicle's controller or the Arduino.**

## Features

* **Multiple Activation Modes:**
    * **Direct Mode:** Motors are active only while the corresponding switch is pressed.
    * **Latching Mode:** Press a switch once to activate its command; press again to deactivate. Only one command can be latched at a time.
    * **Timed Latch Mode:** Press a switch to activate its command for a predefined duration (e.g., 5 seconds).
* **Mode Cycling:** A dedicated input pin (Pin 9) allows cycling through the activation modes.
* **EEPROM Mode Saving:** The currently selected activation mode is saved to EEPROM (with wear-reduction logic) and loaded on startup, persisting across power cycles.
* **Assistive Switch Interface:** Uses digital input pins with pull-up resistors, designed for normally open switches that connect to ground when closed.
* **Robust Switch Debouncing:** Prevents multiple activations from a single switch press.
* **Power Saving (Sleep Mode):** After a period of inactivity, the Arduino enters a low-power sleep mode and wakes up on any switch press to conserve energy.
* **Safety Features:**
    * Prevents activation of conflicting motor commands.
    * If multiple command switches are pressed simultaneously in Direct Mode, motors will stop.
    * 5-Minute Inactivity Timeout: If no switch activity is detected for 5 minutes, all motors stop, active latches are cleared, and the system enters sleep mode.
* **Modular Code:** Well-commented and organized.
* **Serial Debugging Output:** Provides detailed information via the Serial Monitor.

## ⚠️ Important Cautions & Disclaimer ⚠️

* **Electrical Safety:** Modifying electronic devices, especially children's ride-on toys, involves inherent risks. If you are not competent and comfortable with electronics, wiring, and soldering, **do not attempt this project.** Incorrect wiring can lead to short circuits, damage to the Arduino, damage to the vehicle's electronics, or even create a fire hazard. Always double-check your connections before applying power.
* **Common Ground Connection (Crucial!):** For the Arduino to reliably control the vehicle's motor controller, they **must share a common reference voltage, known as a "common ground."**
    * **What this means (for laypeople):** For different electrical parts to "understand" each other's signals (like "on" or "off"), they need to agree on what "zero volts" or the "ground level" is. If they don't share this common ground, the signals from the Arduino might not be correctly interpreted by the vehicle's controller, leading to erratic behavior or no control at all.
    * **How to achieve it:** The simplest way is often to power the Arduino from the vehicle's own battery system (making sure the voltage is appropriate for the Arduino, possibly using a voltage regulator). If the Arduino and the vehicle's controller are powered by the same battery, they will inherently share a common ground. If they are powered separately, you **must** connect a wire from one of the Arduino's GND pins to the Ground (negative terminal) of the vehicle's battery or motor controller.
    * **Failure to do this can result in unpredictable operation or complete failure of the control system.**
* **Vehicle Compatibility:** This project assumes a specific type of motor control interface on the ride-on vehicle (control lines activated by connecting to Ground). Not all ride-on toys use this system. You **must** verify how your vehicle's controller works before attempting to interface with it.
* **Use At Your Own Risk:** This software and accompanying documentation are provided "as is" without warranty of any kind, express or implied. The authors and contributors are not responsible for any damage to property, injury, or other negative outcomes that may result from building, using, or modifying this project. You assume full responsibility for testing the safety and functionality of your modified vehicle.
* **Supervision Required:** Any ride-on vehicle modified with this controller should only be used under close adult supervision, especially if used by individuals with disabilities. Ensure the environment is safe and free of hazards.

## Hardware Requirements

* **Arduino Board:** Developed for Arduino Uno R4 Minima. Adaptable to other boards with sufficient digital pins (at least 4 output, 5 input) and EEPROM/LowPower capabilities.
* **Ride-On Vehicle:** A ride-on vehicle with an accessible motor controller that uses LOW signals (connection to Ground) to activate motor directions.
* **Assistive Switches:** Up to four normally open command switches and one mode cycle switch (e.g., button switches, sip-and-puff switches).
* **Wiring Components:** Wires, connectors, (optional) 3.5mm jacks, soldering equipment.
* **Power Supply:** Appropriate power supply for the Arduino and the ride-on vehicle. **Ensure a common ground connection as described in the Cautions section.**

## Software Requirements

* **Arduino IDE:** Version 1.8.x or 2.x.x.
* **Libraries:**
    * `<EEPROM.h>` (Standard)
    * `<LowPower.h>` (For R4 series, installable via Arduino Library Manager if not pre-installed with the R4 core)
* The `RideOnVehicleController.ino` sketch.

## Pinout

### Motor Control Output Pins (Arduino -> Vehicle Controller)
These Arduino pins are set to `INPUT` (high-impedance) to be OFF, and `OUTPUT LOW` to activate the corresponding motor line on the vehicle's controller by connecting it to Ground.

* **Digital Pin 1:** Right Motor Forward (RF)
* **Digital Pin 2:** Right Motor Reverse (RR)
* **Digital Pin 3:** Left Motor Forward (LF)
* **Digital Pin 4:** Left Motor Reverse (LR)

### Switch Input Pins (Assistive Switches -> Arduino)
These Arduino pins use `INPUT_PULLUP`. Connect one terminal of your switch to the Arduino pin and the other terminal to Ground (GND). **The Arduino's GND and the vehicle's electrical system GND must be connected.**

* **Digital Pin 5:** Command: Forward
* **Digital Pin 6:** Command: Reverse
* **Digital Pin 7:** Command: Rotate Clockwise
* **Digital Pin 8:** Command: Rotate Counter-Clockwise
* **Digital Pin 9:** Cycle Activation Mode

## Configuration

* **Activation Mode:** Loaded from EEPROM. Can be changed using the Mode Switch (Pin 9). Default on first run is Direct Mode.
* **Debounce Delay:** `const unsigned long DEBOUNCE_DELAY = 50;` (ms)
* **Timed Latch Duration:** `const unsigned long TIMED_LATCH_DURATION = 5000;` (ms)
* **Inactivity Timeout:** `const unsigned long INACTIVITY_TIMEOUT_DURATION = 5 * 60 * 1000;` (ms) - before sleep.
* **EEPROM Address:** `const int EEPROM_MODE_ADDRESS = 0;`
* **Min. Mode Duration for EEPROM Save:** `const unsigned long MIN_MODE_DURATION_FOR_EEPROM_SAVE = 3000;` (ms)

## How to Use

1.  **Understand the Risks:** Read the "Important Cautions & Disclaimer" section thoroughly, especially regarding common ground.
2.  **Hardware Setup:**
    * **Carefully identify** the motor control signal lines on your ride-on vehicle's existing controller.
    * **Ensure a common ground connection** between the Arduino and the's electrical system.
    * Connect the Arduino motor control output pins (1-4) to these identified signal lines.
    * Connect your assistive switches to the Arduino input pins (5-9) and Ground.
    * Power the Arduino and the vehicle. **Start with low-current or current-limited power supplies for initial testing if possible.**
3.  **Software Setup:**
    * Open `RideOnVehicleController.ino` in Arduino IDE.
    * Ensure the Arduino R4 board support package and the `LowPower` library are installed.
    * Select "Arduino Uno R4 Minima" (or your board) and the correct COM port.
    * Upload the sketch.
4.  **Operation:**
    * Open Serial Monitor (115200 baud) for debugging.
    * Use Pin 9 switch to cycle to the desired activation mode. The mode is saved automatically after being active for a few seconds.
    * Operate the vehicle using command switches (Pins 5-8).
    * The device will enter sleep mode after 5 minutes of inactivity and wake on any switch press.
    * **Always test thoroughly in a safe, controlled environment before allowing regular use.**

## Contributing

Contributions are welcome! If you have improvements or bug fixes, please feel free to fork this repository and submit a pull request.

## License

This project is licensed under the GNU General Public License v3.0.
A copy of the license can typically be found in a file named `LICENSE` or `COPYING` in the root of the source code repository. If not, you can obtain it from [https://www.gnu.org/licenses/gpl-3.0.html](https://www.gnu.org/licenses/gpl-3.0.html).


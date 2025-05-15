# Arduino Ride-On Vehicle Controller

## Description

This Arduino project provides firmware for controlling a modified ride-on vehicle using an Arduino Uno R4 Minima (or compatible board). It's designed to interface with assistive technology switches (e.g., 3.5mm jack switches) to allow users with limited mobility to control the vehicle's movement. The controller supports multiple activation modes for flexibility and includes safety features like switch debouncing and an inactivity timeout.

The vehicle's original motor controller is assumed to have four control lines exposed:
* Right Motor Forward
* Right Motor Reverse
* Left Motor Forward
* Left Motor Reverse

These lines are activated when brought LOW. This firmware uses Arduino digital pins to sink current and activate these motor control lines.

## Features

* **Multiple Activation Modes:**
    * **Direct Mode:** Motors are active only while the corresponding switch is pressed.
    * **Latching Mode:** Press a switch once to activate its command; press again to deactivate. Only one command can be latched at a time.
    * **Timed Latch Mode:** Press a switch to activate its command for a predefined duration (e.g., 5 seconds).
* **Assistive Switch Interface:** Uses digital input pins with pull-up resistors, designed for normally open switches that connect to ground when closed.
* **Robust Switch Debouncing:** Prevents multiple activations from a single switch press due to electrical noise or mechanical bounce.
* **Safety Features:**
    * Prevents activation of conflicting motor commands (e.g., forward and reverse simultaneously on the same motor).
    * If multiple command switches are pressed at the same time in Direct Mode, motors will stop.
    * 5-Minute Inactivity Timeout: If no switch activity is detected for 5 minutes, all motors will stop, and any active latches will be cleared.
* **Modular Code:** Well-commented and organized for readability and future modifications.
* **Serial Debugging Output:** Provides detailed information via the Serial Monitor about the current mode, active commands, and state changes.

## Hardware Requirements

* **Arduino Board:** Developed for Arduino Uno R4 Minima. Should be adaptable to other Arduino boards with sufficient digital pins (at least 4 output, 4 input).
* **Ride-On Vehicle:** A ride-on vehicle with an accessible motor controller that uses LOW signals to activate motor directions.
* **Assistive Switches:** Up to four normally open switches (e.g., button switches, sip-and-puff switches) that can be connected via 3.5mm jacks or directly wired.
* **Wiring Components:** Wires, connectors, (optional) 3.5mm jacks.
* **Power Supply:** Appropriate power supply for the Arduino and the ride-on vehicle.

## Software Requirements

* **Arduino IDE:** Version 1.8.x or 2.x.x.
* The `RideOnVehicleController.ino` sketch (this firmware).

## Pinout

### Motor Control Output Pins (Arduino -> Vehicle Controller)
These Arduino pins are set to `INPUT` (high-impedance) to be OFF, and `OUTPUT LOW` to activate the corresponding motor line on the vehicle's controller.

* **Digital Pin 1:** Right Motor Forward (RF)
* **Digital Pin 2:** Right Motor Reverse (RR)
* **Digital Pin 3:** Left Motor Forward (LF)
* **Digital Pin 4:** Left Motor Reverse (LR)

### Switch Input Pins (Assistive Switches -> Arduino)
These Arduino pins use `INPUT_PULLUP`. Connect one terminal of your switch to the Arduino pin and the other terminal to Ground (GND).

* **Digital Pin 5:** Command: Forward (Both motors forward)
* **Digital Pin 6:** Command: Reverse (Both motors reverse)
* **Digital Pin 7:** Command: Rotate Clockwise (Right motor reverse, Left motor forward)
* **Digital Pin 8:** Command: Rotate Counter-Clockwise (Right motor forward, Left motor reverse)

## Configuration

* **Activation Mode:** The default activation mode can be set by changing the `currentActivationMode` variable near the top of the `.ino` file:
    ```cpp
    ActivationMode currentActivationMode = DIRECT_MODE;
    // Or LATCHING_MODE, or TIMED_LATCH_MODE
    ```
* **Debounce Delay:** `const unsigned long DEBOUNCE_DELAY = 50;` (milliseconds)
* **Timed Latch Duration:** `const unsigned long TIMED_LATCH_DURATION = 5000;` (milliseconds)
* **Inactivity Timeout:** `const unsigned long INACTIVITY_TIMEOUT_DURATION = 5 * 60 * 1000;` (milliseconds)

## How to Use

1.  **Hardware Setup:**
    * Connect the Arduino motor control output pins (1-4) to the corresponding signal lines on your ride-on vehicle's motor controller.
    * Connect your assistive switches to the Arduino input pins (5-8) and Ground.
    * Power the Arduino and the vehicle.
2.  **Software Setup:**
    * Open the `RideOnVehicleController.ino` sketch in the Arduino IDE.
    * Select your Arduino board (e.g., Arduino Uno R4 Minima) and COM port from the Tools menu.
    * Configure the desired `currentActivationMode` and other timing parameters if needed.
    * Upload the sketch to your Arduino.
3.  **Operation:**
    * Open the Serial Monitor (baud rate 115200) to view debugging information.
    * Operate the vehicle using the connected assistive switches according to the selected activation mode.


## Contributing

Contributions are welcome! If you have improvements or bug fixes, please feel free to fork this repository and submit a pull request.

## License

(Consider adding an open-source license here, e.g., MIT, GPL, Apache 2.0. For example:
This project is licensed under the MIT License - see the LICENSE.md file for details.)


// Arduino Ride-On Vehicle Controller Firmware
#include <Arduino.h>


// Includes enhanced debugging features, safety improvements, clarifications,
// and a 5-minute inactivity timeout.

// --- 1. Definitions and Constants ---

// Motor Control Output Pins (Arduino Digital Pins 1-4)
const int MOTOR_CTRL_RF_PIN = 1; // Right Motor Forward control
const int MOTOR_CTRL_RR_PIN = 2; // Right Motor Reverse control
const int MOTOR_CTRL_LF_PIN = 3; // Left Motor Forward control
const int MOTOR_CTRL_LR_PIN = 4; // Left Motor Reverse control

// Switch Input Pins (Arduino Digital Pins 5-8)
const int SWITCH_FORWARD_PIN = 5;
const int SWITCH_REVERSE_PIN = 6;
const int SWITCH_CLOCKWISE_PIN = 7;
const int SWITCH_COUNTERCLOCKWISE_PIN = 8;

const int NUM_SWITCHES = 4;
const int switchPins[NUM_SWITCHES] = {
  SWITCH_FORWARD_PIN,
  SWITCH_REVERSE_PIN,
  SWITCH_CLOCKWISE_PIN,
  SWITCH_COUNTERCLOCKWISE_PIN
};

// Debounce Parameters
const unsigned long DEBOUNCE_DELAY = 50; // Milliseconds

// Timed Latch Parameters
const unsigned long TIMED_LATCH_DURATION = 5000; // Milliseconds (e.g., 5 seconds)

// Inactivity Timeout Parameters
const unsigned long INACTIVITY_TIMEOUT_DURATION = 5 * 60 * 1000; // 5 minutes in milliseconds

// Activation Modes (Enum)
enum ActivationMode {
  DIRECT_MODE,    // Motor active only while switch is pressed
  LATCHING_MODE,  // Press once to turn on, press again to turn off
  TIMED_LATCH_MODE // Press once for a timed duration of motor activity
};

// --- Select Activation Mode Here ---
ActivationMode currentActivationMode = DIRECT_MODE; // Default mode. Change as needed.
// ActivationMode currentActivationMode = LATCHING_MODE;
// ActivationMode currentActivationMode = TIMED_LATCH_MODE;


// Commands (Enum)
enum Command {
  CMD_NONE,
  CMD_FORWARD,
  CMD_REVERSE,
  CMD_CLOCKWISE,
  CMD_COUNTERCLOCKWISE
};

// --- 2. Global State Variables ---

// Switch States
int lastRawSwitchState[NUM_SWITCHES];
bool currentDebouncedSwitchState[NUM_SWITCHES];
bool switchJustPressed[NUM_SWITCHES];
bool switchJustReleased[NUM_SWITCHES];
unsigned long lastDebounceTime[NUM_SWITCHES];

// Latching Mode State
bool commandLatched[NUM_SWITCHES];

// Timed Latch State
bool timedLatchActive = false;
unsigned long timedLatchStartTime = 0;
Command activeTimedCommand = CMD_NONE;

// Inactivity Timeout State
unsigned long lastSwitchActivityTime;

// Current Processed Command
Command currentProcessedCommand = CMD_NONE;

// --- Forward declaration for debugging functions ---
void printCommand(Command cmd);

// --- 3. setup() Function ---
void setup() {
  Serial.begin(115200);
  Serial.println("Ride-On Vehicle Controller Initializing...");
  Serial.println("----------------------------------");

  pinMode(MOTOR_CTRL_RF_PIN, INPUT);
  pinMode(MOTOR_CTRL_RR_PIN, INPUT);
  pinMode(MOTOR_CTRL_LF_PIN, INPUT);
  pinMode(MOTOR_CTRL_LR_PIN, INPUT);
  Serial.println("Motor pins initialized as INPUT (OFF).");

  for (int i = 0; i < NUM_SWITCHES; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
  }
  Serial.println("Switch pins initialized as INPUT_PULLUP.");

  for (int i = 0; i < NUM_SWITCHES; i++) {
    lastRawSwitchState[i] = HIGH;
    currentDebouncedSwitchState[i] = false;
    switchJustPressed[i] = false;
    switchJustReleased[i] = false;
    lastDebounceTime[i] = 0;
    commandLatched[i] = false;
  }

  lastSwitchActivityTime = millis(); // Initialize last activity time

  printActivationMode();
  Serial.println("Setup complete.");
}

// --- 4. loop() Function ---
void loop() {
  // Check for inactivity first
  handleInactivityTimeout();

  // Read and process switch inputs
  readAllSwitches();

  // Process inputs based on current activation mode
  // (only if not recently timed out to CMD_NONE by inactivity)
  if (currentProcessedCommand != CMD_NONE || (millis() - lastSwitchActivityTime < INACTIVITY_TIMEOUT_DURATION) ) {
      // If an activity just happened, or we are not in an inactivity-induced CMD_NONE state,
      // then process modes. This avoids modes re-activating motors immediately after inactivity timeout
      // if a switch was stuck ON.
      // A switch press will reset lastSwitchActivityTime and allow modes to process again.
    switch (currentActivationMode) {
        case DIRECT_MODE:
        handleDirectMode();
        break;
        case LATCHING_MODE:
        handleLatchingMode();
        break;
        case TIMED_LATCH_MODE:
        handleTimedLatchMode();
        break;
    }
  }


  // Execute the determined command
  executeMotorCommand(currentProcessedCommand);

  // Print current state for debugging
  printCurrentState();

  // Reset one-shot flags for switch presses/releases
  for (int i = 0; i < NUM_SWITCHES; i++) {
    switchJustPressed[i] = false;
    switchJustReleased[i] = false;
  }
}

// --- 5. Helper Functions/Modules ---

// --- 5.1. Input Handling Module ---
void readAllSwitches() {
  unsigned long currentTime = millis();
  bool activityDetectedThisCycle = false;
  for (int i = 0; i < NUM_SWITCHES; i++) {
    int rawReading = digitalRead(switchPins[i]);

    if (rawReading != lastRawSwitchState[i]) {
      lastDebounceTime[i] = currentTime;
    }

    if ((currentTime - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
      bool debouncedReadingIsPressed = (rawReading == LOW);
      if (debouncedReadingIsPressed != currentDebouncedSwitchState[i]) {
        currentDebouncedSwitchState[i] = debouncedReadingIsPressed;
        if (currentDebouncedSwitchState[i]) {
          switchJustPressed[i] = true;
          activityDetectedThisCycle = true; // Mark activity
        } else {
          switchJustReleased[i] = true;
          // activityDetectedThisCycle = true; // Optionally, count release as activity too
        }
      }
    }
    lastRawSwitchState[i] = rawReading;
  }
  if (activityDetectedThisCycle) {
    lastSwitchActivityTime = currentTime; // Update on any confirmed press
  }
}

// --- 5.X Inactivity Timeout Handler ---
/**
 * @brief Checks for prolonged inactivity and stops motors if timeout is reached.
 * Resets latches and timed mode.
 */
void handleInactivityTimeout() {
  // Check only if motors might be active or a latch is set.
  // No need to check if already idle and no latches are set.
  bool anyLatchOrTimedActive = timedLatchActive;
  for(int i=0; i<NUM_SWITCHES; ++i) {
    if(commandLatched[i]) {
      anyLatchOrTimedActive = true;
      break;
    }
  }

  if (currentProcessedCommand != CMD_NONE || anyLatchOrTimedActive) {
    if ((millis() - lastSwitchActivityTime) > INACTIVITY_TIMEOUT_DURATION) {
      Serial.println("--- Inactivity Timeout: Stopping all operations. ---");
      currentProcessedCommand = CMD_NONE; // This will lead to stopping motors

      // Clear all latches
      for (int i = 0; i < NUM_SWITCHES; i++) {
        commandLatched[i] = false;
      }
      // Deactivate timed latch
      if (timedLatchActive) {
        timedLatchActive = false;
        activeTimedCommand = CMD_NONE;
      }
      // IMPORTANT: Update lastSwitchActivityTime to current time AFTER timeout action
      // to prevent continuous re-triggering until a new switch press.
      lastSwitchActivityTime = millis();
    }
  }
}


// --- 5.2. Activation Mode Logic Module ---
Command mapSwitchIndexToCommand(int index) {
  switch (index) {
    case 0: return CMD_FORWARD;
    case 1: return CMD_REVERSE;
    case 2: return CMD_CLOCKWISE;
    case 3: return CMD_COUNTERCLOCKWISE;
    default: return CMD_NONE;
  }
}

Command getCommandFromSwitches() {
  int pressedSwitchIndex = -1;
  int pressedCount = 0;

  for (int i = 0; i < NUM_SWITCHES; i++) {
    if (currentDebouncedSwitchState[i]) {
      pressedCount++;
      if (pressedSwitchIndex == -1) {
        pressedSwitchIndex = i;
      }
    }
  }

  if (pressedCount > 1) {
    Serial.println("Warning: Multiple switches active. Commanding CMD_NONE.");
    return CMD_NONE;
  } else if (pressedCount == 1) {
    return mapSwitchIndexToCommand(pressedSwitchIndex);
  }
  return CMD_NONE;
}

void handleDirectMode() {
  currentProcessedCommand = getCommandFromSwitches();
}

void handleLatchingMode() {
  Command newCommandToProcess = CMD_NONE;
  bool anyLatchActionThisCycle = false;

  for (int i = 0; i < NUM_SWITCHES; i++) {
    if (switchJustPressed[i]) {
      anyLatchActionThisCycle = true;
      commandLatched[i] = !commandLatched[i];

      if (commandLatched[i]) {
        newCommandToProcess = mapSwitchIndexToCommand(i);
        for (int j = 0; j < NUM_SWITCHES; j++) {
          if (i != j) {
            commandLatched[j] = false;
          }
        }
      } // If unlatched, newCommandToProcess remains CMD_NONE unless another latch is active
      break; 
    }
  }

  if (!anyLatchActionThisCycle) {
    for (int i = 0; i < NUM_SWITCHES; i++) {
      if (commandLatched[i]) {
        newCommandToProcess = mapSwitchIndexToCommand(i);
        break;
      }
    }
  }
  currentProcessedCommand = newCommandToProcess;
}

void handleTimedLatchMode() {
  unsigned long currentTime = millis();

  if (timedLatchActive && (currentTime - timedLatchStartTime >= TIMED_LATCH_DURATION)) {
    timedLatchActive = false;
    activeTimedCommand = CMD_NONE;
    currentProcessedCommand = CMD_NONE; // Ensure it's CMD_NONE if latch expires
    Serial.println("Timed latch expired.");
  }

  for (int i = 0; i < NUM_SWITCHES; i++) {
    if (switchJustPressed[i]) {
      Command newTimedCommand = mapSwitchIndexToCommand(i);
      if (newTimedCommand != CMD_NONE) {
          activeTimedCommand = newTimedCommand;
          currentProcessedCommand = activeTimedCommand; // Set command immediately
          timedLatchActive = true;
          timedLatchStartTime = currentTime;
          Serial.print("Started timed latch for command: ");
          printCommand(activeTimedCommand);
          Serial.println();
      }
      return; 
    }
  }

  // If a timed latch is active (and hasn't just expired), ensure its command is processed.
  // If no timed latch is active and no new press, command should be CMD_NONE (or determined by other modes if this logic is nested).
  // The inactivity timeout will override this if no switch presses occur for its duration.
  if (timedLatchActive) {
    currentProcessedCommand = activeTimedCommand;
  } else if (!anyOtherLatchActive()) { // If no regular latches are active either
     // Check if currentProcessedCommand was set by another mode before calling this
     // If not, and timed latch is not active, it might need to be CMD_NONE.
     // This part is tricky if modes can override each other.
     // For now, if timedLatch is not active, it doesn't force CMD_NONE by itself,
     // relying on other modes or the inactivity timeout.
     // Let's ensure that if it's not active, and no other latches are on, it's CMD_NONE
     if (currentProcessedCommand == activeTimedCommand && activeTimedCommand != CMD_NONE) {
        // This means the timed latch was the one running, and it's now not active.
        // currentProcessedCommand = CMD_NONE; // This was handled by expiry logic.
     }
  }
   // If timed latch is not active, currentProcessedCommand is determined by other modes or inactivity.
}

// Helper to check if any regular latch is active
bool anyOtherLatchActive() {
    for(int i=0; i < NUM_SWITCHES; ++i) {
        if(commandLatched[i]) return true;
    }
    return false;
}


// --- 5.3. Motor Control Module ---
void setMotorPinActive(int motorPin, bool active) {
  if (active) {
    pinMode(motorPin, OUTPUT);
    digitalWrite(motorPin, LOW);
  } else {
    pinMode(motorPin, INPUT);
  }
}

void stopAllMotors() {
  setMotorPinActive(MOTOR_CTRL_RF_PIN, false);
  setMotorPinActive(MOTOR_CTRL_RR_PIN, false);
  setMotorPinActive(MOTOR_CTRL_LF_PIN, false);
  setMotorPinActive(MOTOR_CTRL_LR_PIN, false);
}

void executeMotorCommand(Command cmd) {
  static Command lastExecutedCommand = CMD_NONE; 
  
  if (cmd != lastExecutedCommand || (cmd == CMD_NONE && lastExecutedCommand != CMD_NONE) ) {
    stopAllMotors(); 

    switch (cmd) {
      case CMD_FORWARD:
        setMotorPinActive(MOTOR_CTRL_RF_PIN, true);
        setMotorPinActive(MOTOR_CTRL_LF_PIN, true);
        break;
      case CMD_REVERSE:
        setMotorPinActive(MOTOR_CTRL_RR_PIN, true);
        setMotorPinActive(MOTOR_CTRL_LR_PIN, true);
        break;
      case CMD_CLOCKWISE:
        setMotorPinActive(MOTOR_CTRL_RR_PIN, true);
        setMotorPinActive(MOTOR_CTRL_LF_PIN, true);
        break;
      case CMD_COUNTERCLOCKWISE:
        setMotorPinActive(MOTOR_CTRL_RF_PIN, true);
        setMotorPinActive(MOTOR_CTRL_LR_PIN, true);
        break;
      case CMD_NONE:
        break;
      default:
        break;
    }
    lastExecutedCommand = cmd;
  }
}

// --- 5.4. Debugging Print Functions ---
void printActivationMode() {
  Serial.print("Current Mode: ");
  switch (currentActivationMode) {
    case DIRECT_MODE: Serial.println("Direct"); break;
    case LATCHING_MODE: Serial.println("Latching"); break;
    case TIMED_LATCH_MODE: Serial.println("Timed Latch"); break;
    default: Serial.println("Unknown"); break;
  }
}

void printCommand(Command cmd) {
  switch (cmd) {
    case CMD_NONE: Serial.print("None"); break;
    case CMD_FORWARD: Serial.print("Forward"); break;
    case CMD_REVERSE: Serial.print("Reverse"); break;
    case CMD_CLOCKWISE: Serial.print("Clockwise"); break;
    case CMD_COUNTERCLOCKWISE: Serial.print("Counterclockwise"); break;
    default: Serial.print("UnknownCmd"); break;
  }
}

void printCurrentState() {
  static Command lastPrintedCommand = CMD_NONE;
  static unsigned long lastPrintTimeForTimedMode = 0; // Renamed for clarity

  // Determine if we should print: command changed OR periodic update for active timed latch
  bool shouldPrint = (currentProcessedCommand != lastPrintedCommand);
  if (!shouldPrint && currentActivationMode == TIMED_LATCH_MODE && timedLatchActive) {
    if (millis() - lastPrintTimeForTimedMode > 1000) { // Print every 1 sec for active timed latch
      shouldPrint = true;
    }
  }

  if (shouldPrint) {
    Serial.print("Active Command: ");
    printCommand(currentProcessedCommand);

    if (currentActivationMode == TIMED_LATCH_MODE && timedLatchActive) {
      unsigned long elapsed = millis() - timedLatchStartTime;
      unsigned long remaining = (TIMED_LATCH_DURATION > elapsed) ? (TIMED_LATCH_DURATION - elapsed) : 0;
      Serial.print(" (Time Remaining: ");
      Serial.print(remaining / 1000);
      Serial.print(".");
      Serial.print((remaining % 1000) / 100);
      Serial.print("s)");
    }
    Serial.println();

    lastPrintedCommand = currentProcessedCommand;
    // Update print time only if we actually printed for timed mode, or if command changed
    if (currentActivationMode == TIMED_LATCH_MODE && timedLatchActive) {
        lastPrintTimeForTimedMode = millis();
    } else if (currentProcessedCommand != lastPrintedCommand) { // Reset if command changed
        lastPrintTimeForTimedMode = millis();
    }
  }
}

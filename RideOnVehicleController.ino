//this version uses the arduino uno r4 minima

#include <Arduino.h>     // Standard Arduino header
#include <EEPROM.h>      // For saving mode across power cycles
// #include <LowPower.h> // Removed due to issues

// --- 1. Definitions and Constants ---

// Motor Control Output Pins
const int MOTOR_CTRL_RF_PIN = 1; //Blue Wire from vehicle control board on developer's setup
const int MOTOR_CTRL_RR_PIN = 2; //Orange Wire from vehicle control board on developer's setup
const int MOTOR_CTRL_LF_PIN = 3; //Green Wire from vehicle control board on developer's setup
const int MOTOR_CTRL_LR_PIN = 4; //Yellow Wire from vehicle control board on developer's setup

// Switch Input Pins
const int SWITCH_FORWARD_PIN = 5;
const int SWITCH_REVERSE_PIN = 6;
const int SWITCH_CLOCKWISE_PIN = 7;
const int SWITCH_COUNTERCLOCKWISE_PIN = 8;
const int MODE_SWITCH_PIN = 9; // Pin to cycle through activation modes

const int NUM_COMMAND_SWITCHES = 4;
const int commandSwitchPins[NUM_COMMAND_SWITCHES] = {
  SWITCH_FORWARD_PIN,
  SWITCH_REVERSE_PIN,
  SWITCH_CLOCKWISE_PIN,
  SWITCH_COUNTERCLOCKWISE_PIN
};

// Debounce Parameters
const unsigned long DEBOUNCE_DELAY = 50; // Milliseconds

// Timed Latch Parameters
const unsigned long TIMED_LATCH_DURATION_NON_ROTATIONAL = 5000; // Milliseconds (for Forward/Reverse)

// NEW: Timed Rotation Parameters
const float DEGREES_PER_TIMED_ROTATION_PRESS = 30.0; // Target degrees of rotation per press
const unsigned long FULL_ROTATION_DURATION_MS = 3000; // Time (ms) for vehicle to complete a 360-degree turn
// Calculated duration for a timed rotation press:
const unsigned long TIMED_ROTATION_PRESS_DURATION_MS = (unsigned long)((FULL_ROTATION_DURATION_MS / 360.0) * DEGREES_PER_TIMED_ROTATION_PRESS);


// Inactivity Timeout Parameters
const unsigned long INACTIVITY_TIMEOUT_DURATION = 5 * 60 * 1000; // 5 minutes

// EEPROM Address for storing activation mode
const int EEPROM_MODE_ADDRESS = 0;
// Minimum time a mode must be active before being saved to EEPROM to prevent wear
const unsigned long MIN_MODE_DURATION_FOR_EEPROM_SAVE = 3000; // 3 seconds

// Activation Modes (Enum) - Based on user's provided code
enum ActivationMode {
  DIRECT_MODE = 0,
  LATCHING_MODE = 1,
  TIMED_LATCH_MODE = 2
};
const int NUM_ACTIVATION_MODES = 3;

ActivationMode currentActivationMode = DIRECT_MODE;

// Commands (Enum)
enum Command {
  CMD_NONE,
  CMD_FORWARD,
  CMD_REVERSE,
  CMD_CLOCKWISE,
  CMD_COUNTERCLOCKWISE
};

// --- 2. Global State Variables ---

// Switch States (for command switches)
int lastRawCmdSwitchState[NUM_COMMAND_SWITCHES];
bool currentDebouncedCmdSwitchState[NUM_COMMAND_SWITCHES];
bool cmdSwitchJustPressed[NUM_COMMAND_SWITCHES];
bool cmdSwitchJustReleased[NUM_COMMAND_SWITCHES];
unsigned long lastCmdSwitchDebounceTime[NUM_COMMAND_SWITCHES];

// Mode Switch State
int lastRawModeSwitchState = HIGH;
bool currentDebouncedModeSwitchState = false;
bool modeSwitchJustPressed = false;
unsigned long lastModeSwitchDebounceTime = 0;

// Latching Mode State
bool commandLatched[NUM_COMMAND_SWITCHES];

// Timed Latch State
bool timedLatchActive = false;
unsigned long timedLatchStartTime = 0;
unsigned long currentActiveTimedLatchDuration = 0; // NEW: Stores the duration for the current timed latch
Command activeTimedCommand = CMD_NONE;


// Inactivity Timeout State
unsigned long lastSwitchActivityTime;

// EEPROM Save Logic for Mode
bool pendingEEPROMSave = false;
unsigned long modeChangeInitiatedTime = 0;
ActivationMode modeToSave;


// Current Processed Command
Command currentProcessedCommand = CMD_NONE;

// --- Forward declaration for debugging functions ---
void printCommandName(Command cmd);
void printActivationModeSerial();

// --- 3. setup() Function ---
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\nRide-On Vehicle Controller Initializing...");
  Serial.println("----------------------------------");
  Serial.print("Timed Rotation Degrees: "); Serial.println(DEGREES_PER_TIMED_ROTATION_PRESS);
  Serial.print("Full 360 Turn Time (ms): "); Serial.println(FULL_ROTATION_DURATION_MS);
  Serial.print("Calculated Timed Rotation Press Duration (ms): "); Serial.println(TIMED_ROTATION_PRESS_DURATION_MS);
  Serial.println("----------------------------------");


  pinMode(MOTOR_CTRL_RF_PIN, INPUT);
  pinMode(MOTOR_CTRL_RR_PIN, INPUT);
  pinMode(MOTOR_CTRL_LF_PIN, INPUT);
  pinMode(MOTOR_CTRL_LR_PIN, INPUT);
  Serial.println("Motor pins initialized as INPUT (OFF).");

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    pinMode(commandSwitchPins[i], INPUT_PULLUP);
  }
  Serial.println("Command switch pins initialized for input.");

  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  Serial.println("Mode switch pin initialized for input.");

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    lastRawCmdSwitchState[i] = HIGH;
    currentDebouncedCmdSwitchState[i] = false;
    cmdSwitchJustPressed[i] = false;
    cmdSwitchJustReleased[i] = false;
    lastCmdSwitchDebounceTime[i] = 0;
    commandLatched[i] = false;
  }

  byte savedModeByte = EEPROM.read(EEPROM_MODE_ADDRESS);
  if (savedModeByte >= 0 && savedModeByte < NUM_ACTIVATION_MODES) {
    currentActivationMode = (ActivationMode)savedModeByte;
    Serial.print("Loaded mode from EEPROM: ");
  } else {
    Serial.print("Invalid mode in EEPROM or first run. Defaulting to DIRECT_MODE and saving: ");
    currentActivationMode = DIRECT_MODE;
    byte currentEEPROMValue = EEPROM.read(EEPROM_MODE_ADDRESS);
    if (currentEEPROMValue != (byte)currentActivationMode) {
        EEPROM.write(EEPROM_MODE_ADDRESS, (byte)currentActivationMode);
    }
  }
  printActivationModeSerial();
  Serial.println();

  lastSwitchActivityTime = millis();
  pendingEEPROMSave = false;
  Serial.println("Setup complete.");
}

// --- 4. loop() Function ---
void loop() {
  readAndProcessModeSwitch();
  checkAndSaveModeToEEPROM();
  handleInactivityTimeout();

  readAllCommandSwitches();

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

  executeMotorCommand(currentProcessedCommand);
  printCurrentState();

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    cmdSwitchJustPressed[i] = false;
    cmdSwitchJustReleased[i] = false;
  }
  modeSwitchJustPressed = false;
}

// --- 5. Helper Functions/Modules ---

// --- 5.1. Input Handling Modules ---
void readAndProcessModeSwitch() {
  unsigned long currentTime = millis();
  int rawReading = digitalRead(MODE_SWITCH_PIN);

  if (rawReading != lastRawModeSwitchState) {
    lastModeSwitchDebounceTime = currentTime;
  }

  if ((currentTime - lastModeSwitchDebounceTime) > DEBOUNCE_DELAY) {
    bool debouncedReadingIsPressed = (rawReading == LOW);
    if (debouncedReadingIsPressed != currentDebouncedModeSwitchState) {
      currentDebouncedModeSwitchState = debouncedReadingIsPressed;
      if (currentDebouncedModeSwitchState) {
        modeSwitchJustPressed = true;
      }
    }
  }
  lastRawModeSwitchState = rawReading;

  if (modeSwitchJustPressed) {
    timedLatchActive = false;
    activeTimedCommand = CMD_NONE;
    currentActiveTimedLatchDuration = 0; // Reset this on mode change

    currentActivationMode = (ActivationMode)((currentActivationMode + 1) % NUM_ACTIVATION_MODES);

    Serial.print("Mode changed to: ");
    printActivationModeSerial();
    Serial.println();

    pendingEEPROMSave = true;
    modeToSave = currentActivationMode;
    modeChangeInitiatedTime = millis();

    lastSwitchActivityTime = millis();
    currentProcessedCommand = CMD_NONE;
    stopAllMotors();
    for(int i=0; i < NUM_COMMAND_SWITCHES; ++i) commandLatched[i] = false;
  }
}

void checkAndSaveModeToEEPROM() {
  if (pendingEEPROMSave) {
    if ((millis() - modeChangeInitiatedTime) > MIN_MODE_DURATION_FOR_EEPROM_SAVE) {
      if (currentActivationMode == modeToSave) {
        byte currentEEPROMValue = EEPROM.read(EEPROM_MODE_ADDRESS);
        if (currentEEPROMValue != (byte)modeToSave) {
            EEPROM.write(EEPROM_MODE_ADDRESS, (byte)modeToSave);
            Serial.print("Mode '");
            printActivationModeSerial();
            Serial.println("' saved to EEPROM after delay.");
        }
      }
      pendingEEPROMSave = false;
    }
  }
}


void readAllCommandSwitches() {
  unsigned long currentTime = millis();
  bool activityDetectedThisCycle = false;
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    int rawReading = digitalRead(commandSwitchPins[i]);

    if (rawReading != lastRawCmdSwitchState[i]) {
      lastCmdSwitchDebounceTime[i] = currentTime;
    }

    if ((currentTime - lastCmdSwitchDebounceTime[i]) > DEBOUNCE_DELAY) {
      bool debouncedReadingIsPressed = (rawReading == LOW);
      if (debouncedReadingIsPressed != currentDebouncedCmdSwitchState[i]) {
        currentDebouncedCmdSwitchState[i] = debouncedReadingIsPressed;
        if (currentDebouncedCmdSwitchState[i]) {
          cmdSwitchJustPressed[i] = true;
          activityDetectedThisCycle = true;
        } else {
          cmdSwitchJustReleased[i] = true;
        }
      }
    }
    lastRawCmdSwitchState[i] = rawReading;
  }
  if (activityDetectedThisCycle) {
    lastSwitchActivityTime = currentTime;
    if(pendingEEPROMSave && currentActivationMode != modeToSave) {
        modeToSave = currentActivationMode;
        modeChangeInitiatedTime = millis();
    }
  }
}

// --- 5.X Inactivity Timeout Handler ---
void handleInactivityTimeout() {
  bool anyLatchOrTimedActive = timedLatchActive;
  for(int i=0; i<NUM_COMMAND_SWITCHES; ++i) {
    if(commandLatched[i]) {
      anyLatchOrTimedActive = true;
      break;
    }
  }

  // Check if timed out based on switch activity, but only if motors are running or a latch/timed action is set
  if (currentProcessedCommand != CMD_NONE || anyLatchOrTimedActive) {
      if ((millis() - lastSwitchActivityTime) > INACTIVITY_TIMEOUT_DURATION) {
        Serial.println("--- Inactivity Timeout: Stopping operations. ---");

        if(pendingEEPROMSave && (millis() - modeChangeInitiatedTime) > MIN_MODE_DURATION_FOR_EEPROM_SAVE && currentActivationMode == modeToSave) {
          byte currentEEPROMValue = EEPROM.read(EEPROM_MODE_ADDRESS);
          if (currentEEPROMValue != (byte)modeToSave) {
              EEPROM.write(EEPROM_MODE_ADDRESS, (byte)modeToSave);
              Serial.println("Pending mode saved to EEPROM before timeout action.");
          }
          pendingEEPROMSave = false;
        }

        currentProcessedCommand = CMD_NONE;
        stopAllMotors();

        for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
          commandLatched[i] = false;
        }
        if (timedLatchActive) {
          timedLatchActive = false;
          activeTimedCommand = CMD_NONE;
          currentActiveTimedLatchDuration = 0; // Reset
        }
        lastSwitchActivityTime = millis(); // Reset activity timer
      }
  // If no command is active AND no latches/timed events are set,
  // we still want to reset the lastSwitchActivityTime if a switch is pressed,
  // to prevent timeout when user is just cycling modes without activating motors.
  // This is handled by readAllCommandSwitches and readAndProcessModeSwitch updating lastSwitchActivityTime.
  } else if ((millis() - lastSwitchActivityTime) > INACTIVITY_TIMEOUT_DURATION) {
      // This block handles the case where the system has been idle (no commands, no latches)
      // for the timeout duration. We might not need to print "Stopping operations"
      // if nothing was running, but we should ensure the activity timer is kept fresh
      // if the user does eventually press a switch (handled by the input functions).
      // For now, just ensure the timer is reset if it truly expires without any interaction.
      // A Serial print here could indicate "System idle timeout, awake." if needed for debugging.
      lastSwitchActivityTime = millis(); // Keep this updated to prevent spamming timeout messages if system is truly idle
  }
}


// --- 5.2. Activation Mode Logic Module ---
Command mapSwitchIndexToCommand(int index) {
  if (index < 0 || index >= NUM_COMMAND_SWITCHES) return CMD_NONE;
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

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (currentDebouncedCmdSwitchState[i]) {
      pressedCount++;
      if (pressedSwitchIndex == -1) {
        pressedSwitchIndex = i;
      }
    }
  }

  if (pressedCount > 1) {
    return CMD_NONE;
  } else if (pressedCount == 1) {
    return mapSwitchIndexToCommand(pressedSwitchIndex);
  }
  return CMD_NONE;
}

void handleDirectMode() {
  if (timedLatchActive) return; // Should not happen if mode switch logic is correct
  currentProcessedCommand = getCommandFromSwitches();
}

void handleLatchingMode() {
  if (timedLatchActive) return; // Should not happen

  Command newCommandToProcess = CMD_NONE;
  bool anyLatchActionThisCycle = false;

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (cmdSwitchJustPressed[i]) {
      anyLatchActionThisCycle = true;
      commandLatched[i] = !commandLatched[i];

      if (commandLatched[i]) {
        newCommandToProcess = mapSwitchIndexToCommand(i);
        for (int j = 0; j < NUM_COMMAND_SWITCHES; j++) {
          if (i != j) {
            commandLatched[j] = false;
          }
        }
      }
      break;
    }
  }

  if (!anyLatchActionThisCycle) {
    for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
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

  // Check if current timed latch has expired
  if (timedLatchActive && (currentTime - timedLatchStartTime >= currentActiveTimedLatchDuration)) {
    Serial.print("Timed latch expired for "); printCommandName(activeTimedCommand); Serial.println();
    timedLatchActive = false;
    activeTimedCommand = CMD_NONE;
    currentProcessedCommand = CMD_NONE;
    currentActiveTimedLatchDuration = 0;
  }

  // Check for new switch presses to start/override a timed latch
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (cmdSwitchJustPressed[i]) {
      Command newTimedCommand = mapSwitchIndexToCommand(i);
      if (newTimedCommand != CMD_NONE) {
        // If a timed latch was active for a *different* command, this new press overrides it.
        // If it was for the SAME command, this new press restarts its timer.
        if (timedLatchActive && activeTimedCommand != newTimedCommand) {
          Serial.print("Timed latch for "); printCommandName(activeTimedCommand); Serial.print(" overridden by "); printCommandName(newTimedCommand); Serial.println();
          // No need to explicitly stop, new command will take over
        } else if (timedLatchActive && activeTimedCommand == newTimedCommand) {
          Serial.print("Timed latch for "); printCommandName(activeTimedCommand); Serial.println(" re-triggered.");
        }


        activeTimedCommand = newTimedCommand;
        currentProcessedCommand = activeTimedCommand; // Start command immediately

        // Set the duration based on command type
        if (activeTimedCommand == CMD_CLOCKWISE || activeTimedCommand == CMD_COUNTERCLOCKWISE) {
          currentActiveTimedLatchDuration = TIMED_ROTATION_PRESS_DURATION_MS;
          Serial.print("Started timed ROTATION ("); Serial.print(currentActiveTimedLatchDuration); Serial.print("ms) for: ");
        } else {
          currentActiveTimedLatchDuration = TIMED_LATCH_DURATION_NON_ROTATIONAL;
          Serial.print("Started timed NON-ROTATIONAL ("); Serial.print(currentActiveTimedLatchDuration); Serial.print("ms) for: ");
        }
        printCommandName(activeTimedCommand); Serial.println();

        timedLatchActive = true;
        timedLatchStartTime = currentTime;
        lastSwitchActivityTime = currentTime; // Update activity time on new timed latch
      }
      return; // Process only one press at a time for timed latch
    }
  }

  // If a timed latch is currently active (and not expired by the check at the start of this function),
  // ensure its command is the current processed command.
  if (timedLatchActive) {
    currentProcessedCommand = activeTimedCommand;
  } else {
    // If no timed latch is active (either expired or never started), set command to NONE.
    // This prevents the vehicle from continuing a previous non-timed command if a timed one just expired.
    currentProcessedCommand = CMD_NONE;
  }
}


bool anyOtherLatchActive() {
    for(int i=0; i < NUM_COMMAND_SWITCHES; ++i) {
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
        // Motors already stopped by stopAllMotors() if command changed to CMD_NONE
        break;
      default:
        // Unknown command, ensure motors are stopped
        break;
    }
    lastExecutedCommand = cmd;
  }
}

// --- 5.4. Debugging Print Functions ---
void printActivationModeSerial() {
  switch (currentActivationMode) {
    case DIRECT_MODE: Serial.print("Direct"); break;
    case LATCHING_MODE: Serial.print("Latching"); break;
    case TIMED_LATCH_MODE: Serial.print("Timed Latch"); break;
    default: Serial.print("Unknown Mode"); break;
  }
}

void printCommandName(Command cmd) {
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
  static unsigned long lastPrintTime = 0; // General last print time
  static ActivationMode lastPrintedActivationMode = (ActivationMode)-1;
  const unsigned long PRINT_INTERVAL = 250; // Print status roughly 4 times a second if state is stable but timed

  bool commandChanged = (currentProcessedCommand != lastPrintedCommand);
  bool modeChanged = (currentActivationMode != lastPrintedActivationMode);
  bool shouldPrint = commandChanged || modeChanged;

  // For TIMED_LATCH_MODE, print periodically if the command itself hasn't changed but time is elapsing.
  if (!shouldPrint && currentActivationMode == TIMED_LATCH_MODE && timedLatchActive) {
    if (millis() - lastPrintTime > PRINT_INTERVAL) {
      shouldPrint = true;
    }
  }
  // Also print if command just became NONE from something else, even if mode is not timed
  if (!shouldPrint && lastPrintedCommand != CMD_NONE && currentProcessedCommand == CMD_NONE) {
      shouldPrint = true;
  }


  if (shouldPrint) {
    Serial.print("State: Cmd=");
    printCommandName(currentProcessedCommand);
    Serial.print(" | Mode=");
    printActivationModeSerial();

    if (currentActivationMode == TIMED_LATCH_MODE && timedLatchActive) {
      unsigned long elapsed = millis() - timedLatchStartTime;
      // Ensure remaining is not negative if somehow elapsed > currentActiveTimedLatchDuration slightly due to timing
      unsigned long remaining = (currentActiveTimedLatchDuration > elapsed) ? (currentActiveTimedLatchDuration - elapsed) : 0;
      Serial.print(" (TL Rem: "); Serial.print(remaining / 1000.0, 1); Serial.print("s / ");
      Serial.print(currentActiveTimedLatchDuration / 1000.0, 1); Serial.print("s)");
    }
    Serial.println();

    lastPrintedCommand = currentProcessedCommand;
    lastPrintedActivationMode = currentActivationMode;
    lastPrintTime = millis();
  }
}


//This version uses the arduino uno r4 minima

#include <Arduino.h>     // Standard Arduino header
#include <EEPROM.h>      // For saving mode across power cycles

// --- 1. Definitions and Constants ---

// Motor Control Output Pins
const int MOTOR_CTRL_RF_PIN = 1; //Blue Wire from vehicle control board
const int MOTOR_CTRL_RR_PIN = 2; //Orange Wire from vehicle control board
const int MOTOR_CTRL_LF_PIN = 3; //Green Wire from vehicle control board
const int MOTOR_CTRL_LR_PIN = 4; //Yellow Wire from vehicle control board

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

// Timing Constants
const unsigned long DEBOUNCE_DELAY = 50; // Milliseconds
const unsigned long TIMED_LATCH_DURATION_NON_ROTATIONAL = 5000; // Milliseconds (for Forward/Reverse)
const unsigned long INACTIVITY_TIMEOUT_DURATION = 5UL * 60UL * 1000UL; // 5 minutes (using UL to prevent overflow)
const unsigned long MIN_MODE_DURATION_FOR_EEPROM_SAVE = 3000; // 3 seconds
const unsigned long PRINT_INTERVAL = 250; // Print status roughly 4 times a second

// Rotation Parameters
const float DEGREES_PER_TIMED_ROTATION_PRESS = 30.0; // Target degrees of rotation per press
const unsigned long FULL_ROTATION_DURATION_MS = 3000; // Time (ms) for vehicle to complete a 360-degree turn
const unsigned long TIMED_ROTATION_PRESS_DURATION_MS = (unsigned long)((FULL_ROTATION_DURATION_MS / 360.0) * DEGREES_PER_TIMED_ROTATION_PRESS);

// EEPROM Configuration
const int EEPROM_MODE_ADDRESS = 0;

// Activation Modes (Enum)
enum ActivationMode : uint8_t {
  DIRECT_MODE = 0,
  LATCHING_MODE = 1,
  TIMED_LATCH_MODE = 2,
  NUM_ACTIVATION_MODES = 3
};

// Commands (Enum)
enum Command : uint8_t {
  CMD_NONE,
  CMD_FORWARD,
  CMD_REVERSE,
  CMD_CLOCKWISE,
  CMD_COUNTERCLOCKWISE
};

// --- 2. Structs for Better Organization ---

struct SwitchState {
  int lastRawState = HIGH;
  bool currentDebouncedState = false;
  bool justPressed = false;
  bool justReleased = false;
  unsigned long lastDebounceTime = 0;
  bool latched = false; // For latching mode
};

struct TimedLatchState {
  bool active = false;
  unsigned long startTime = 0;
  unsigned long duration = 0;
  Command activeCommand = CMD_NONE;
};

struct EEPROMState {
  bool pendingSave = false;
  unsigned long modeChangeTime = 0;
  ActivationMode modeToSave = DIRECT_MODE;
};

// --- 3. Global State Variables ---

ActivationMode currentActivationMode = DIRECT_MODE;
Command currentProcessedCommand = CMD_NONE;

SwitchState commandSwitches[NUM_COMMAND_SWITCHES];
SwitchState modeSwitch;
TimedLatchState timedLatch;
EEPROMState eepromState;

unsigned long lastSwitchActivityTime = 0;

// Debug printing state
struct DebugState {
  Command lastPrintedCommand = CMD_NONE;
  ActivationMode lastPrintedMode = static_cast<ActivationMode>(-1);
  unsigned long lastPrintTime = 0;
} debugState;

// --- 4. Function Declarations ---
void initializePins();
void loadModeFromEEPROM();
void updateSwitchState(SwitchState& sw, int pin);
void updateAllSwitches();
void handleModeSwitch();
void handleActivationMode();
void handleInactivityTimeout();
void executeMotorCommand(Command cmd);
void updateEEPROMState();
Command mapSwitchIndexToCommand(int index);
Command getActiveCommand();
void resetAllStates();
void printStatus();
void printCommandName(Command cmd);
void printModeName(ActivationMode mode);

// --- 5. setup() Function ---
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println(F("\nRide-On Vehicle Controller Initializing..."));
  Serial.println(F("----------------------------------"));
  Serial.print(F("Timed Rotation Degrees: ")); Serial.println(DEGREES_PER_TIMED_ROTATION_PRESS);
  Serial.print(F("Full 360 Turn Time (ms): ")); Serial.println(FULL_ROTATION_DURATION_MS);
  Serial.print(F("Calculated Timed Rotation Press Duration (ms): ")); Serial.println(TIMED_ROTATION_PRESS_DURATION_MS);
  Serial.println(F("----------------------------------"));

  initializePins();
  loadModeFromEEPROM();
  
  lastSwitchActivityTime = millis();
  Serial.println(F("Setup complete."));
}

// --- 6. loop() Function ---
void loop() {
  updateAllSwitches();
  handleModeSwitch();
  updateEEPROMState();
  handleInactivityTimeout();
  handleActivationMode();
  executeMotorCommand(currentProcessedCommand);
  printStatus();
  
  // Clear just-pressed/released flags after processing
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    commandSwitches[i].justPressed = false;
    commandSwitches[i].justReleased = false;
  }
  modeSwitch.justPressed = false;
}

// --- 7. Implementation Functions ---

void initializePins() {
  // Initialize motor pins as INPUT (OFF state)
  pinMode(MOTOR_CTRL_RF_PIN, INPUT);
  pinMode(MOTOR_CTRL_RR_PIN, INPUT);
  pinMode(MOTOR_CTRL_LF_PIN, INPUT);
  pinMode(MOTOR_CTRL_LR_PIN, INPUT);
  Serial.println(F("Motor pins initialized as INPUT (OFF)."));

  // Initialize switch pins
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    pinMode(commandSwitchPins[i], INPUT_PULLUP);
  }
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  Serial.println(F("Switch pins initialized with pullups."));
}

void loadModeFromEEPROM() {
  uint8_t savedMode = EEPROM.read(EEPROM_MODE_ADDRESS);
  
  if (savedMode < NUM_ACTIVATION_MODES) {
    currentActivationMode = static_cast<ActivationMode>(savedMode);
    Serial.print(F("Loaded mode from EEPROM: "));
  } else {
    Serial.print(F("Invalid EEPROM data. Defaulting to DIRECT_MODE: "));
    currentActivationMode = DIRECT_MODE;
    EEPROM.write(EEPROM_MODE_ADDRESS, static_cast<uint8_t>(currentActivationMode));
  }
  
  printModeName(currentActivationMode);
  Serial.println();
}

void updateSwitchState(SwitchState& sw, int pin) {
  unsigned long currentTime = millis();
  int rawReading = digitalRead(pin);

  if (rawReading != sw.lastRawState) {
    sw.lastDebounceTime = currentTime;
  }

  if ((currentTime - sw.lastDebounceTime) > DEBOUNCE_DELAY) {
    bool isPressed = (rawReading == LOW);
    
    if (isPressed != sw.currentDebouncedState) {
      sw.currentDebouncedState = isPressed;
      
      if (isPressed) {
        sw.justPressed = true;
        lastSwitchActivityTime = currentTime;
      } else {
        sw.justReleased = true;
      }
    }
  }
  
  sw.lastRawState = rawReading;
}

void updateAllSwitches() {
  // Update command switches
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    updateSwitchState(commandSwitches[i], commandSwitchPins[i]);
  }
  
  // Update mode switch
  updateSwitchState(modeSwitch, MODE_SWITCH_PIN);
}

void handleModeSwitch() {
  if (!modeSwitch.justPressed) return;
  
  // Reset all states when changing modes
  resetAllStates();
  
  // Cycle to next mode
  currentActivationMode = static_cast<ActivationMode>((currentActivationMode + 1) % NUM_ACTIVATION_MODES);
  
  Serial.print(F("Mode changed to: "));
  printModeName(currentActivationMode);
  Serial.println();
  
  // Setup EEPROM save
  eepromState.pendingSave = true;
  eepromState.modeToSave = currentActivationMode;
  eepromState.modeChangeTime = millis();
}

void handleActivationMode() {
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

void handleDirectMode() {
  currentProcessedCommand = getActiveCommand();
}

void handleLatchingMode() {
  Command newCommand = CMD_NONE;
  
  // Check for new presses
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (commandSwitches[i].justPressed) {
      // Toggle latch state
      commandSwitches[i].latched = !commandSwitches[i].latched;
      
      if (commandSwitches[i].latched) {
        // Turn off all other latches and activate this command
        for (int j = 0; j < NUM_COMMAND_SWITCHES; j++) {
          if (i != j) commandSwitches[j].latched = false;
        }
        newCommand = mapSwitchIndexToCommand(i);
      }
      break; // Process only one press per cycle
    }
  }
  
  // If no new press, check for existing latched command
  if (newCommand == CMD_NONE) {
    for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
      if (commandSwitches[i].latched) {
        newCommand = mapSwitchIndexToCommand(i);
        break;
      }
    }
  }
  
  currentProcessedCommand = newCommand;
}

void handleTimedLatchMode() {
  unsigned long currentTime = millis();
  
  // Check if current timed latch has expired
  if (timedLatch.active && (currentTime - timedLatch.startTime >= timedLatch.duration)) {
    Serial.print(F("Timed latch expired for "));
    printCommandName(timedLatch.activeCommand);
    Serial.println();
    
    timedLatch.active = false;
    timedLatch.activeCommand = CMD_NONE;
    currentProcessedCommand = CMD_NONE;
  }
  
  // Check for new switch presses
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (commandSwitches[i].justPressed) {
      Command newCommand = mapSwitchIndexToCommand(i);
      
      if (timedLatch.active && timedLatch.activeCommand == newCommand) {
        Serial.print(F("Timed latch for "));
        printCommandName(newCommand);
        Serial.println(F(" re-triggered."));
      } else if (timedLatch.active) {
        Serial.print(F("Timed latch for "));
        printCommandName(timedLatch.activeCommand);
        Serial.print(F(" overridden by "));
        printCommandName(newCommand);
        Serial.println();
      }
      
      // Set up new timed latch
      timedLatch.activeCommand = newCommand;
      timedLatch.startTime = currentTime;
      
      // Set duration based on command type
      if (newCommand == CMD_CLOCKWISE || newCommand == CMD_COUNTERCLOCKWISE) {
        timedLatch.duration = TIMED_ROTATION_PRESS_DURATION_MS;
        Serial.print(F("Started timed ROTATION ("));
      } else {
        timedLatch.duration = TIMED_LATCH_DURATION_NON_ROTATIONAL;
        Serial.print(F("Started timed NON-ROTATIONAL ("));
      }
      
      Serial.print(timedLatch.duration);
      Serial.print(F("ms) for: "));
      printCommandName(newCommand);
      Serial.println();
      
      timedLatch.active = true;
      currentProcessedCommand = newCommand;
      
      break; // Process only one press per cycle
    }
  }
  
  // Ensure active timed command is processed
  if (timedLatch.active) {
    currentProcessedCommand = timedLatch.activeCommand;
  } else {
    currentProcessedCommand = CMD_NONE;
  }
}

void handleInactivityTimeout() {
  bool anyActiveState = (currentProcessedCommand != CMD_NONE) || timedLatch.active;
  
  // Check for latched commands
  for (int i = 0; i < NUM_COMMAND_SWITCHES && !anyActiveState; i++) {
    if (commandSwitches[i].latched) {
      anyActiveState = true;
    }
  }
  
  if (anyActiveState && (millis() - lastSwitchActivityTime) > INACTIVITY_TIMEOUT_DURATION) {
    Serial.println(F("--- Inactivity Timeout: Stopping operations. ---"));
    
    // Save pending EEPROM data if needed
    if (eepromState.pendingSave && 
        (millis() - eepromState.modeChangeTime) > MIN_MODE_DURATION_FOR_EEPROM_SAVE &&
        currentActivationMode == eepromState.modeToSave) {
      
      if (EEPROM.read(EEPROM_MODE_ADDRESS) != static_cast<uint8_t>(eepromState.modeToSave)) {
        EEPROM.write(EEPROM_MODE_ADDRESS, static_cast<uint8_t>(eepromState.modeToSave));
        Serial.println(F("Pending mode saved to EEPROM before timeout."));
      }
      eepromState.pendingSave = false;
    }
    
    resetAllStates();
    lastSwitchActivityTime = millis();
  }
}

void updateEEPROMState() {
  if (!eepromState.pendingSave) return;
  
  if ((millis() - eepromState.modeChangeTime) > MIN_MODE_DURATION_FOR_EEPROM_SAVE) {
    if (currentActivationMode == eepromState.modeToSave) {
      if (EEPROM.read(EEPROM_MODE_ADDRESS) != static_cast<uint8_t>(eepromState.modeToSave)) {
        EEPROM.write(EEPROM_MODE_ADDRESS, static_cast<uint8_t>(eepromState.modeToSave));
        Serial.print(F("Mode '"));
        printModeName(eepromState.modeToSave);
        Serial.println(F("' saved to EEPROM."));
      }
    }
    eepromState.pendingSave = false;
  }
}

Command mapSwitchIndexToCommand(int index) {
  switch (index) {
    case 0: return CMD_FORWARD;
    case 1: return CMD_REVERSE;
    case 2: return CMD_CLOCKWISE;
    case 3: return CMD_COUNTERCLOCKWISE;
    default: return CMD_NONE;
  }
}

Command getActiveCommand() {
  int activeCount = 0;
  int activeIndex = -1;
  
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (commandSwitches[i].currentDebouncedState) {
      activeCount++;
      activeIndex = i;
    }
  }
  
  // Return command only if exactly one switch is pressed
  return (activeCount == 1) ? mapSwitchIndexToCommand(activeIndex) : CMD_NONE;
}

void resetAllStates() {
  currentProcessedCommand = CMD_NONE;
  stopAllMotors();
  
  // Reset latch states
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    commandSwitches[i].latched = false;
  }
  
  // Reset timed latch
  timedLatch.active = false;
  timedLatch.activeCommand = CMD_NONE;
}

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

  if (cmd != lastExecutedCommand) {
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
        // Motors already stopped
        break;
    }
    lastExecutedCommand = cmd;
  }
}

void printStatus() {
  bool commandChanged = (currentProcessedCommand != debugState.lastPrintedCommand);
  bool modeChanged = (currentActivationMode != debugState.lastPrintedMode);
  bool shouldPrint = commandChanged || modeChanged;

  // Periodic printing for timed latch mode
  if (!shouldPrint && currentActivationMode == TIMED_LATCH_MODE && timedLatch.active) {
    if (millis() - debugState.lastPrintTime > PRINT_INTERVAL) {
      shouldPrint = true;
    }
  }

  if (shouldPrint) {
    Serial.print(F("State: Cmd="));
    printCommandName(currentProcessedCommand);
    Serial.print(F(" | Mode="));
    printModeName(currentActivationMode);

    if (currentActivationMode == TIMED_LATCH_MODE && timedLatch.active) {
      unsigned long elapsed = millis() - timedLatch.startTime;
      unsigned long remaining = (timedLatch.duration > elapsed) ? (timedLatch.duration - elapsed) : 0;
      Serial.print(F(" (TL Rem: "));
      Serial.print(remaining / 1000.0, 1);
      Serial.print(F("s / "));
      Serial.print(timedLatch.duration / 1000.0, 1);
      Serial.print(F("s)"));
    }
    Serial.println();

    debugState.lastPrintedCommand = currentProcessedCommand;
    debugState.lastPrintedMode = currentActivationMode;
    debugState.lastPrintTime = millis();
  }
}

void printCommandName(Command cmd) {
  switch (cmd) {
    case CMD_NONE: Serial.print(F("None")); break;
    case CMD_FORWARD: Serial.print(F("Forward")); break;
    case CMD_REVERSE: Serial.print(F("Reverse")); break;
    case CMD_CLOCKWISE: Serial.print(F("Clockwise")); break;
    case CMD_COUNTERCLOCKWISE: Serial.print(F("Counterclockwise")); break;
    default: Serial.print(F("Unknown")); break;
  }
}

void printModeName(ActivationMode mode) {
  switch (mode) {
    case DIRECT_MODE: Serial.print(F("Direct")); break;
    case LATCHING_MODE: Serial.print(F("Latching")); break;
    case TIMED_LATCH_MODE: Serial.print(F("Timed Latch")); break;
    default: Serial.print(F("Unknown")); break;
  }
}

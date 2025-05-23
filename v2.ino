//Enhanced Accessible Ride-On Vehicle Controller for Arduino Uno R4 Minima
//Designed for children with disabilities - cognitive and accessibility focused

//Don't use! it's a work in progress

#include <Arduino.h>
#include <EEPROM.h>

// --- PIN DEFINITIONS ---

// Motor Control Output Pins
const int MOTOR_CTRL_RF_PIN = 1; //Blue Wire - Right Forward
const int MOTOR_CTRL_RR_PIN = 2; //Orange Wire - Right Reverse  
const int MOTOR_CTRL_LF_PIN = 3; //Green Wire - Left Forward
const int MOTOR_CTRL_LR_PIN = 4; //Yellow Wire - Left Reverse

// Input Pins
const int SWITCH_FORWARD_PIN = 5;
const int SWITCH_REVERSE_PIN = 6;
const int SWITCH_CLOCKWISE_PIN = 7;
const int SWITCH_COUNTERCLOCKWISE_PIN = 8;
const int MODE_SWITCH_PIN = 9;
const int SIP_PUFF_PIN = A0; // Analog input for sip-and-puff
const int SCAN_SELECT_PIN = 10; // Single switch for scanning mode

// LED Indicator Pins
const int LED_MODE_PIN = 11;     // Mode indicator LED
const int LED_COMMAND_PIN = 12;  // Command indicator LED  
const int LED_STATUS_PIN = 13;   // Status/ready indicator LED

// --- CONSTANTS ---

const int NUM_COMMAND_SWITCHES = 4;
const int commandSwitchPins[NUM_COMMAND_SWITCHES] = {
  SWITCH_FORWARD_PIN, SWITCH_REVERSE_PIN, SWITCH_CLOCKWISE_PIN, SWITCH_COUNTERCLOCKWISE_PIN
};

// Timing Constants
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long TIMED_LATCH_DURATION_NON_ROTATIONAL = 5000;
const unsigned long INACTIVITY_TIMEOUT_DURATION = 5UL * 60UL * 1000UL; // 5 minutes
const unsigned long MIN_MODE_DURATION_FOR_EEPROM_SAVE = 3000;
const unsigned long PRINT_INTERVAL = 250;

// Rotation Parameters
const float DEGREES_PER_TIMED_ROTATION_PRESS = 30.0;
const unsigned long FULL_ROTATION_DURATION_MS = 3000;
const unsigned long TIMED_ROTATION_PRESS_DURATION_MS = (unsigned long)((FULL_ROTATION_DURATION_MS / 360.0) * DEGREES_PER_TIMED_ROTATION_PRESS);

// Sip-and-Puff Parameters
const int SIP_THRESHOLD = 400;    // Analog reading for sip detection
const int PUFF_THRESHOLD = 600;   // Analog reading for puff detection
const int NEUTRAL_THRESHOLD = 50; // Deadband around neutral
const unsigned long SIP_PUFF_SEQUENCE_TIMEOUT = 2000; // Max time between sequence elements
const int MAX_SEQUENCE_LENGTH = 4;

// Scanning Mode Parameters
const unsigned long SCAN_CYCLE_TIME = 1500; // Time to highlight each option
const unsigned long SCAN_DWELL_TIME = 3000; // Time to hold selection

// One-Touch Mode Parameters
const unsigned long ONE_TOUCH_DURATION = 2000; // Duration for one-touch movements

// Audio Feedback Parameters
const unsigned long BEEP_DURATION = 100;
const unsigned long CONFIRMATION_DELAY = 200;

// EEPROM Addresses
const int EEPROM_MODE_ADDRESS = 0;
const int EEPROM_COMPLEXITY_ADDRESS = 1;
const int EEPROM_OPERATION_MODE_ADDRESS = 2;
const int EEPROM_INPUT_METHOD_ADDRESS = 3;
const int EEPROM_CONFIG_BASE = 10; // Base address for configuration parameters

// --- ENUMS ---

enum ActivationMode : uint8_t {
  DIRECT_MODE = 0,
  LATCHING_MODE = 1,
  TIMED_LATCH_MODE = 2,
  NUM_ACTIVATION_MODES = 3
};

enum Command : uint8_t {
  CMD_NONE,
  CMD_FORWARD,
  CMD_REVERSE,
  CMD_CLOCKWISE,
  CMD_COUNTERCLOCKWISE
};

enum ComplexityLevel : uint8_t {
  BEGINNER = 0,    // Forward/Reverse only
  INTERMEDIATE = 1, // Forward/Reverse/Turn
  ADVANCED = 2     // Full directional control
};

enum OperationMode : uint8_t {
  NORMAL_OPERATION = 0,
  ONE_TOUCH_MODE = 1,
  GENTLE_MODE = 2,
  PRACTICE_MODE = 3
};

enum InputMethod : uint8_t {
  SWITCH_INPUT = 0,
  SIP_PUFF_INPUT = 1,
  SCANNING_INPUT = 2
};

enum SipPuffAction : uint8_t {
  SP_NONE = 0,
  SP_SIP = 1,
  SP_PUFF = 2
};

enum SessionState : uint8_t {
  SESSION_READY = 0,
  SESSION_ACTIVE = 1,
  SESSION_PAUSED = 2
};

// --- STRUCTS ---

struct SwitchState {
  int lastRawState = HIGH;
  bool currentDebouncedState = false;
  bool justPressed = false;
  bool justReleased = false;
  unsigned long lastDebounceTime = 0;
  bool latched = false;
};

struct TimedLatchState {
  bool active = false;
  unsigned long startTime = 0;
  unsigned long duration = 0;
  Command activeCommand = CMD_NONE;
};

struct SipPuffState {
  int sequence[MAX_SEQUENCE_LENGTH];
  int sequenceLength = 0;
  unsigned long lastActionTime = 0;
  bool waitingForNeutral = false;
  int lastReading = 512; // Neutral position
};

struct ScanningState {
  bool active = false;
  int currentOption = 0;
  unsigned long lastScanTime = 0;
  bool selectionMade = false;
  unsigned long selectionTime = 0;
  Command availableCommands[NUM_COMMAND_SWITCHES];
  int numAvailableCommands = 0;
};

struct SessionStats {
  unsigned long sessionStartTime = 0;
  unsigned long totalForwardTime = 0;
  unsigned long totalReverseTime = 0;
  unsigned long totalTurnTime = 0;
  int commandCount = 0;
  int errorCount = 0;
};

struct ConfigState {
  bool configMode = false;
  unsigned long gentleModeDelay = 500;
  float gentleModeSensitivity = 0.7;
  bool audioEnabled = true;
  int scanSpeed = 1500; // milliseconds
  int sipThreshold = SIP_THRESHOLD;
  int puffThreshold = PUFF_THRESHOLD;
};

// --- GLOBAL VARIABLES ---

ActivationMode currentActivationMode = DIRECT_MODE;
ComplexityLevel currentComplexity = ADVANCED;
OperationMode currentOperationMode = NORMAL_OPERATION;
InputMethod currentInputMethod = SWITCH_INPUT;
SessionState currentSessionState = SESSION_READY;
Command currentProcessedCommand = CMD_NONE;

SwitchState commandSwitches[NUM_COMMAND_SWITCHES];
SwitchState modeSwitch;
SwitchState scanSelectSwitch;
TimedLatchState timedLatch;
SipPuffState sipPuff;
ScanningState scanning;
SessionStats stats;
ConfigState config;

unsigned long lastSwitchActivityTime = 0;
unsigned long lastStatusUpdate = 0;

// LED state variables
bool ledModeState = false;
bool ledCommandState = false;
bool ledStatusState = false;
unsigned long lastLedUpdate = 0;
const unsigned long LED_BLINK_INTERVAL = 500;

// --- FUNCTION DECLARATIONS ---
void initializePins();
void loadConfigFromEEPROM();
void saveConfigToEEPROM();
void updateAllInputs();
void handleModeSwitch();
void handleInputMethod();
void handleSipPuffInput();
void handleScanningInput();
void handleComplexityLevel();
void handleActivationMode();
void handleOperationMode();
void updateLEDs();
void executeMotorCommand(Command cmd);
void resetAllStates();
void startSession();
void pauseSession();
void endSession();
void updateSessionStats();
void processSerialCommands();
void printStatus();
void printHelp();
void audioFeedback(const char* message);
void playTone(int frequency, int duration);
bool isCommandAvailable(Command cmd);
Command getNextAvailableCommand(int currentIndex);

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println(F("\n=== Enhanced Accessible Ride-On Controller ==="));
  Serial.println(F("Designed for children with disabilities"));
  Serial.println(F("Type 'help' for configuration commands"));
  Serial.println(F("==============================================="));

  initializePins();
  loadConfigFromEEPROM();
  
  // Initialize scanning available commands
  updateAvailableCommands();
  
  lastSwitchActivityTime = millis();
  stats.sessionStartTime = millis();
  
  Serial.println(F("System ready. Press any switch to begin session."));
  audioFeedback("System ready");
}

// --- MAIN LOOP ---
void loop() {
  updateAllInputs();
  processSerialCommands();
  handleModeSwitch();
  handleInputMethod();
  handleComplexityLevel();
  handleActivationMode();
  handleOperationMode();
  updateSessionStats();
  executeMotorCommand(currentProcessedCommand);
  updateLEDs();
  printStatus();
  
  // Clear just-pressed/released flags
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    commandSwitches[i].justPressed = false;
    commandSwitches[i].justReleased = false;
  }
  modeSwitch.justPressed = false;
  scanSelectSwitch.justPressed = false;
}

// --- INITIALIZATION ---
void initializePins() {
  // Motor pins as INPUT (OFF state)
  pinMode(MOTOR_CTRL_RF_PIN, INPUT);
  pinMode(MOTOR_CTRL_RR_PIN, INPUT);
  pinMode(MOTOR_CTRL_LF_PIN, INPUT);
  pinMode(MOTOR_CTRL_LR_PIN, INPUT);

  // Switch pins with pullups
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    pinMode(commandSwitchPins[i], INPUT_PULLUP);
  }
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SCAN_SELECT_PIN, INPUT_PULLUP);
  
  // LED pins as outputs
  pinMode(LED_MODE_PIN, OUTPUT);
  pinMode(LED_COMMAND_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  
  // Set initial LED states
  digitalWrite(LED_STATUS_PIN, HIGH); // Ready state
  
  Serial.println(F("Pins initialized"));
}

void loadConfigFromEEPROM() {
  uint8_t mode = EEPROM.read(EEPROM_MODE_ADDRESS);
  uint8_t complexity = EEPROM.read(EEPROM_COMPLEXITY_ADDRESS);
  uint8_t operation = EEPROM.read(EEPROM_OPERATION_MODE_ADDRESS);
  uint8_t input = EEPROM.read(EEPROM_INPUT_METHOD_ADDRESS);
  
  if (mode < NUM_ACTIVATION_MODES) currentActivationMode = (ActivationMode)mode;
  if (complexity <= ADVANCED) currentComplexity = (ComplexityLevel)complexity;
  if (operation <= PRACTICE_MODE) currentOperationMode = (OperationMode)operation;
  if (input <= SCANNING_INPUT) currentInputMethod = (InputMethod)input;
  
  Serial.println(F("Configuration loaded from EEPROM"));
}

void saveConfigToEEPROM() {
  EEPROM.write(EEPROM_MODE_ADDRESS, currentActivationMode);
  EEPROM.write(EEPROM_COMPLEXITY_ADDRESS, currentComplexity);
  EEPROM.write(EEPROM_OPERATION_MODE_ADDRESS, currentOperationMode);
  EEPROM.write(EEPROM_INPUT_METHOD_ADDRESS, currentInputMethod);
  Serial.println(F("Configuration saved to EEPROM"));
}

// --- INPUT HANDLING ---
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
        if (currentSessionState == SESSION_READY) {
          startSession();
        }
      } else {
        sw.justReleased = true;
      }
    }
  }
  
  sw.lastRawState = rawReading;
}

void updateAllInputs() {
  // Update command switches
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    updateSwitchState(commandSwitches[i], commandSwitchPins[i]);
  }
  
  // Update mode and scan switches
  updateSwitchState(modeSwitch, MODE_SWITCH_PIN);
  updateSwitchState(scanSelectSwitch, SCAN_SELECT_PIN);
  
  // Handle different input methods
  switch (currentInputMethod) {
    case SIP_PUFF_INPUT:
      handleSipPuffInput();
      break;
    case SCANNING_INPUT:
      handleScanningInput();
      break;
    case SWITCH_INPUT:
    default:
      // Standard switch handling in activation mode functions
      break;
  }
}

void handleSipPuffInput() {
  int reading = analogRead(SIP_PUFF_PIN);
  unsigned long currentTime = millis();
  
  // Detect sip or puff action
  SipPuffAction action = SP_NONE;
  if (reading < (512 - config.sipThreshold)) {
    action = SP_SIP;
  } else if (reading > (512 + config.puffThreshold)) {
    action = SP_PUFF;
  }
  
  // Handle sequence building
  if (action != SP_NONE && !sipPuff.waitingForNeutral) {
    if (sipPuff.sequenceLength < MAX_SEQUENCE_LENGTH) {
      sipPuff.sequence[sipPuff.sequenceLength++] = action;
      sipPuff.lastActionTime = currentTime;
      sipPuff.waitingForNeutral = true;
      
      audioFeedback(action == SP_SIP ? "Sip" : "Puff");
      
      // Check for complete command sequence
      Command cmd = interpretSipPuffSequence();
      if (cmd != CMD_NONE) {
        currentProcessedCommand = cmd;
        sipPuff.sequenceLength = 0; // Reset sequence
        audioFeedback("Command recognized");
      }
    }
  }
  
  // Return to neutral detection
  if (sipPuff.waitingForNeutral && abs(reading - 512) < NEUTRAL_THRESHOLD) {
    sipPuff.waitingForNeutral = false;
  }
  
  // Sequence timeout
  if (sipPuff.sequenceLength > 0 && (currentTime - sipPuff.lastActionTime) > SIP_PUFF_SEQUENCE_TIMEOUT) {
    sipPuff.sequenceLength = 0;
    audioFeedback("Sequence timeout");
  }
}

Command interpretSipPuffSequence() {
  if (sipPuff.sequenceLength == 2) {
    // Two-element sequences
    if (sipPuff.sequence[0] == SP_SIP && sipPuff.sequence[1] == SP_SIP) {
      return isCommandAvailable(CMD_FORWARD) ? CMD_FORWARD : CMD_NONE;
    }
    if (sipPuff.sequence[0] == SP_PUFF && sipPuff.sequence[1] == SP_PUFF) {
      return isCommandAvailable(CMD_REVERSE) ? CMD_REVERSE : CMD_NONE;
    }
    if (sipPuff.sequence[0] == SP_SIP && sipPuff.sequence[1] == SP_PUFF) {
      return isCommandAvailable(CMD_CLOCKWISE) ? CMD_CLOCKWISE : CMD_NONE;
    }
    if (sipPuff.sequence[0] == SP_PUFF && sipPuff.sequence[1] == SP_SIP) {
      return isCommandAvailable(CMD_COUNTERCLOCKWISE) ? CMD_COUNTERCLOCKWISE : CMD_NONE;
    }
  }
  return CMD_NONE;
}

void handleScanningInput() {
  unsigned long currentTime = millis();
  
  if (!scanning.active) {
    // Start scanning if scan select is pressed
    if (scanSelectSwitch.justPressed) {
      scanning.active = true;
      scanning.currentOption = 0;
      scanning.lastScanTime = currentTime;
      audioFeedback("Scanning started");
    }
    return;
  }
  
  // Handle scanning cycle
  if (currentTime - scanning.lastScanTime > config.scanSpeed) {
    scanning.currentOption = (scanning.currentOption + 1) % scanning.numAvailableCommands;
    scanning.lastScanTime = currentTime;
    
    // Announce current option
    Command cmd = scanning.availableCommands[scanning.currentOption];
    switch (cmd) {
      case CMD_FORWARD: audioFeedback("Forward"); break;
      case CMD_REVERSE: audioFeedback("Reverse"); break;
      case CMD_CLOCKWISE: audioFeedback("Turn right"); break;
      case CMD_COUNTERCLOCKWISE: audioFeedback("Turn left"); break;
    }
  }
  
  // Handle selection
  if (scanSelectSwitch.justPressed && !scanning.selectionMade) {
    scanning.selectionMade = true;
    scanning.selectionTime = currentTime;
    currentProcessedCommand = scanning.availableCommands[scanning.currentOption];
    audioFeedback("Selected");
  }
  
  // Handle selection timeout or completion
  if (scanning.selectionMade && (currentTime - scanning.selectionTime > SCAN_DWELL_TIME)) {
    scanning.active = false;
    scanning.selectionMade = false;
    currentProcessedCommand = CMD_NONE;
  }
}

void updateAvailableCommands() {
  scanning.numAvailableCommands = 0;
  
  if (isCommandAvailable(CMD_FORWARD)) {
    scanning.availableCommands[scanning.numAvailableCommands++] = CMD_FORWARD;
  }
  if (isCommandAvailable(CMD_REVERSE)) {
    scanning.availableCommands[scanning.numAvailableCommands++] = CMD_REVERSE;
  }
  if (isCommandAvailable(CMD_CLOCKWISE)) {
    scanning.availableCommands[scanning.numAvailableCommands++] = CMD_CLOCKWISE;
  }
  if (isCommandAvailable(CMD_COUNTERCLOCKWISE)) {
    scanning.availableCommands[scanning.numAvailableCommands++] = CMD_COUNTERCLOCKWISE;
  }
}

bool isCommandAvailable(Command cmd) {
  switch (currentComplexity) {
    case BEGINNER:
      return (cmd == CMD_FORWARD || cmd == CMD_REVERSE);
    case INTERMEDIATE:
      return true; // All commands available
    case ADVANCED:
      return true; // All commands available
    default:
      return false;
  }
}

// --- MODE HANDLING ---
void handleModeSwitch() {
  if (modeSwitch.justPressed) {
    resetAllStates();
    currentActivationMode = (ActivationMode)((currentActivationMode + 1) % NUM_ACTIVATION_MODES);
    
    Serial.print(F("Mode changed to: "));
    printModeName(currentActivationMode);
    Serial.println();
    
    audioFeedback("Mode changed");
    saveConfigToEEPROM();
  }
}

void handleActivationMode() {
  if (currentSessionState != SESSION_ACTIVE) return;
  
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
  if (currentInputMethod == SWITCH_INPUT) {
    currentProcessedCommand = getActiveCommand();
  }
  // Other input methods handle their own command setting
}

void handleLatchingMode() {
  if (currentInputMethod != SWITCH_INPUT) return;
  
  Command newCommand = CMD_NONE;
  
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (commandSwitches[i].justPressed) {
      commandSwitches[i].latched = !commandSwitches[i].latched;
      
      if (commandSwitches[i].latched) {
        for (int j = 0; j < NUM_COMMAND_SWITCHES; j++) {
          if (i != j) commandSwitches[j].latched = false;
        }
        newCommand = mapSwitchIndexToCommand(i);
      }
      break;
    }
  }
  
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
  
  // Check for expired timed latch
  if (timedLatch.active && (currentTime - timedLatch.startTime >= timedLatch.duration)) {
    timedLatch.active = false;
    timedLatch.activeCommand = CMD_NONE;
    currentProcessedCommand = CMD_NONE;
    audioFeedback("Timer expired");
  }
  
  // Handle new activations based on input method
  if (currentInputMethod == SWITCH_INPUT) {
    for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
      if (commandSwitches[i].justPressed) {
        Command newCommand = mapSwitchIndexToCommand(i);
        startTimedLatch(newCommand);
        break;
      }
    }
  }
  
  if (timedLatch.active) {
    currentProcessedCommand = timedLatch.activeCommand;
  }
}

void startTimedLatch(Command cmd) {
  timedLatch.activeCommand = cmd;
  timedLatch.startTime = millis();
  
  if (cmd == CMD_CLOCKWISE || cmd == CMD_COUNTERCLOCKWISE) {
    timedLatch.duration = TIMED_ROTATION_PRESS_DURATION_MS;
  } else {
    timedLatch.duration = TIMED_LATCH_DURATION_NON_ROTATIONAL;
  }
  
  timedLatch.active = true;
  audioFeedback("Timer started");
}

// --- OPERATION MODE HANDLING ---
void handleOperationMode() {
  switch (currentOperationMode) {
    case ONE_TOUCH_MODE:
      handleOneTouchMode();
      break;
    case GENTLE_MODE:
      handleGentleMode();
      break;
    case PRACTICE_MODE:
      handlePracticeMode();
      break;
    case NORMAL_OPERATION:
    default:
      // Normal operation - no special handling needed
      break;
  }
}

void handleOneTouchMode() {
  static unsigned long oneTouchStartTime = 0;
  static bool oneTouchActive = false;
  
  if (currentProcessedCommand != CMD_NONE && !oneTouchActive) {
    oneTouchActive = true;
    oneTouchStartTime = millis();
  }
  
  if (oneTouchActive && (millis() - oneTouchStartTime >= ONE_TOUCH_DURATION)) {
    currentProcessedCommand = CMD_NONE;
    oneTouchActive = false;
  }
}

void handleGentleMode() {
  static unsigned long lastCommandTime = 0;
  
  if (currentProcessedCommand != CMD_NONE) {
    if (millis() - lastCommandTime < config.gentleModeDelay) {
      currentProcessedCommand = CMD_NONE; // Delay command execution
    } else {
      lastCommandTime = millis();
    }
  }
}

void handlePracticeMode() {
  // In practice mode, we process commands but don't actually move motors
  // This is handled in executeMotorCommand()
}

// --- MOTOR CONTROL ---
void setMotorPinActive(int motorPin, bool active) {
  if (active) {
    pinMode(motorPin, OUTPUT);
    digitalWrite(motorPin, LOW); // Sink current to activate
  } else {
    pinMode(motorPin, INPUT); // High impedance to deactivate
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

    // In practice mode, don't actually move motors
    if (currentOperationMode == PRACTICE_MODE) {
      lastExecutedCommand = cmd;
      return;
    }

    switch (cmd) {
      case CMD_FORWARD:
        if (isCommandAvailable(cmd)) {
          setMotorPinActive(MOTOR_CTRL_RF_PIN, true);
          setMotorPinActive(MOTOR_CTRL_LF_PIN, true);
        }
        break;
      case CMD_REVERSE:
        if (isCommandAvailable(cmd)) {
          setMotorPinActive(MOTOR_CTRL_RR_PIN, true);
          setMotorPinActive(MOTOR_CTRL_LR_PIN, true);
        }
        break;
      case CMD_CLOCKWISE:
        if (isCommandAvailable(cmd)) {
          setMotorPinActive(MOTOR_CTRL_RR_PIN, true);
          setMotorPinActive(MOTOR_CTRL_LF_PIN, true);
        }
        break;
      case CMD_COUNTERCLOCKWISE:
        if (isCommandAvailable(cmd)) {
          setMotorPinActive(MOTOR_CTRL_RF_PIN, true);
          setMotorPinActive(MOTOR_CTRL_LR_PIN, true);
        }
        break;
      case CMD_NONE:
        // Motors already stopped
        break;
    }
    lastExecutedCommand = cmd;
  }
}

// --- LED CONTROL ---
void updateLEDs() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastLedUpdate > LED_BLINK_INTERVAL) {
    // Mode LED - blinks according to current activation mode
    switch (currentActivationMode) {
      case DIRECT_MODE:
        digitalWrite(LED_MODE_PIN, HIGH); // Solid on
        break;
      case LATCHING_MODE:
        ledModeState = !ledModeState; // Slow blink
        digitalWrite(LED_MODE_PIN, ledModeState);
        break;
      case TIMED_LATCH_MODE:
        // Fast blink when timer active, slow when not
        if (timedLatch.active && (currentTime % 200) < 100) {
          digitalWrite(LED_MODE_PIN, HIGH);
        } else if (!timedLatch.active) {
          ledModeState = !ledModeState;
          digitalWrite(LED_MODE_PIN, ledModeState);
        } else {
          digitalWrite(LED_MODE_PIN, LOW);
        }
        break;
    }
    
    // Command LED - indicates active command
    digitalWrite(LED_COMMAND_PIN, currentProcessedCommand != CMD_NONE);
    
    // Status LED - indicates session state
    switch (currentSessionState) {
      case SESSION_READY:
        digitalWrite(LED_STATUS_PIN, HIGH); // Solid on
        break;
      case SESSION_ACTIVE:
        ledStatusState = !ledStatusState; // Blink
        digitalWrite(LED_STATUS_PIN, ledStatusState);
        break;
      case SESSION_PAUSED:
        digitalWrite(LED_STATUS_PIN, LOW); // Off
        break;
    }
    
    lastLedUpdate = currentTime;
  }
}

// --- SESSION MANAGEMENT ---
void startSession() {
  if (currentSessionState != SESSION_READY) return;
  
  currentSessionState = SESSION_ACTIVE;
  stats.sessionStartTime = millis();
  audioFeedback("Session started");
  Serial.println(F("Session started"));
}

void pauseSession() {
  if (currentSessionState != SESSION_ACTIVE) return;
  
  currentSessionState = SESSION_PAUSED;
  currentProcessedCommand = CMD_NONE;
  resetAllStates();
  audioFeedback("Session paused");
  Serial.println(F("Session paused"));
}

void endSession() {
  currentSessionState = SESSION_READY;
  currentProcessedCommand = CMD_NONE;
  resetAllStates();
  printSessionStats();
  audioFeedback("Session ended");
  Serial.println(F("Session ended"));
}

void updateSessionStats() {
  static unsigned long lastStatsUpdate = 0;
  static Command lastTrackedCommand = CMD_NONE;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastStatsUpdate > 1000) { // Update every second
    if (currentProcessedCommand != CMD_NONE && currentSessionState == SESSION_ACTIVE) {
      switch (currentProcessedCommand) {
        case CMD_FORWARD:
        case CMD_REVERSE:
          if (currentProcessedCommand == CMD_FORWARD) stats.totalForwardTime += 1000;
          else stats.totalReverseTime += 1000;
          break;
        case CMD_CLOCKWISE:
        case CMD_COUNTERCLOCKWISE:
          stats.totalTurnTime += 1000;
          break;
      }
      
      // Count command changes
      if (currentProcessedCommand != lastTrackedCommand) {
        stats.commandCount++;
        lastTrackedCommand = currentProcessedCommand;
      }
    }
    lastStatsUpdate = currentTime;
  }
}

void printSessionStats() {
  unsigned long sessionDuration = millis() - stats.sessionStartTime;
  
  Serial.println(F("\n=== SESSION STATISTICS ==="));
  Serial.print(F("Total session time: ")); Serial.print(sessionDuration / 1000); Serial.println(F(" seconds"));
  Serial.print(F("Forward time: ")); Serial.print(stats.totalForwardTime / 1000); Serial.println(F(" seconds"));
  Serial.print(F("Reverse time: ")); Serial.print(stats.totalReverseTime / 1000); Serial.println(F(" seconds"));
  Serial.print(F("Turn time: ")); Serial.print(stats.totalTurnTime / 1000); Serial.println(F(" seconds"));
  Serial.print(F("Total commands: ")); Serial.println(stats.commandCount);
  Serial.print(F("Errors: ")); Serial.println(stats.errorCount);
  Serial.println(F("========================\n"));
  
  // Reset stats for next session
  memset(&stats, 0, sizeof(SessionStats));
  stats.sessionStartTime = millis();
}

// --- SERIAL COMMAND PROCESSING ---
void processSerialCommands() {
  if (!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();
  
  if (command == "help") {
    printHelp();
  }
  else if (command == "status") {
    printDetailedStatus();
  }
  else if (command == "stats") {
    printSessionStats();
  }
  else if (command == "pause") {
    pauseSession();
  }
  else if (command == "resume") {
    if (currentSessionState == SESSION_PAUSED) {
      currentSessionState = SESSION_ACTIVE;
      audioFeedback("Session resumed");
    }
  }
  else if (command == "end") {
    endSession();
  }
  else if (command == "reset") {
    resetAllStates();
    Serial.println(F("All states reset"));
  }
  else if (command.startsWith("complexity ")) {
    String level = command.substring(11);
    if (level == "beginner") {
      currentComplexity = BEGINNER;
      Serial.println(F("Complexity set to BEGINNER (Forward/Reverse only)"));
    }
    else if (level == "intermediate") {
      currentComplexity = INTERMEDIATE;
      Serial.println(F("Complexity set to INTERMEDIATE (All commands)"));
    }
    else if (level == "advanced") {
      currentComplexity = ADVANCED;
      Serial.println(F("Complexity set to ADVANCED (All commands)"));
    }
    updateAvailableCommands();
    saveConfigToEEPROM();
  }
  else if (command.startsWith("operation ")) {
    String mode = command.substring(10);
    if (mode == "normal") {
      currentOperationMode = NORMAL_OPERATION;
      Serial.println(F("Operation mode: NORMAL"));
    }
    else if (mode == "onetouch") {
      currentOperationMode = ONE_TOUCH_MODE;
      Serial.println(F("Operation mode: ONE-TOUCH"));
    }
    else if (mode == "gentle") {
      currentOperationMode = GENTLE_MODE;
      Serial.println(F("Operation mode: GENTLE"));
    }
    else if (mode == "practice") {
      currentOperationMode = PRACTICE_MODE;
      Serial.println(F("Operation mode: PRACTICE (motors disabled)"));
    }
    saveConfigToEEPROM();
  }
  else if (command.startsWith("input ")) {
    String input = command.substring(6);
    if (input == "switch") {
      currentInputMethod = SWITCH_INPUT;
      Serial.println(F("Input method: SWITCH"));
    }
    else if (input == "sippuff") {
      currentInputMethod = SIP_PUFF_INPUT;
      Serial.println(F("Input method: SIP-AND-PUFF"));
      Serial.println(F("Sequences: Sip-Sip=Forward, Puff-Puff=Reverse"));
      Serial.println(F("           Sip-Puff=Right, Puff-Sip=Left"));
    }
    else if (input == "scan") {
      currentInputMethod = SCANNING_INPUT;
      updateAvailableCommands();
      Serial.println(F("Input method: SCANNING"));
    }
    saveConfigToEEPROM();
  }
  else if (command.startsWith("scanspeed ")) {
    int speed = command.substring(10).toInt();
    if (speed >= 500 && speed <= 5000) {
      config.scanSpeed = speed;
      Serial.print(F("Scan speed set to ")); Serial.print(speed); Serial.println(F("ms"));
    }
  }
  else if (command.startsWith("sipthreshold ")) {
    int threshold = command.substring(13).toInt();
    if (threshold >= 50 && threshold <= 500) {
      config.sipThreshold = threshold;
      Serial.print(F("Sip threshold set to ")); Serial.println(threshold);
    }
  }
  else if (command.startsWith("puffthreshold ")) {
    int threshold = command.substring(14).toInt();
    if (threshold >= 50 && threshold <= 500) {
      config.puffThreshold = threshold;
      Serial.print(F("Puff threshold set to ")); Serial.println(threshold);
    }
  }
  else if (command == "audio on") {
    config.audioEnabled = true;
    Serial.println(F("Audio feedback enabled"));
  }
  else if (command == "audio off") {
    config.audioEnabled = false;
    Serial.println(F("Audio feedback disabled"));
  }
  else if (command == "save") {
    saveConfigToEEPROM();
  }
  else {
    Serial.println(F("Unknown command. Type 'help' for available commands."));
  }
}

void printHelp() {
  Serial.println(F("\n=== COMMAND REFERENCE ==="));
  Serial.println(F("System Control:"));
  Serial.println(F("  help          - Show this help"));
  Serial.println(F("  status        - Show detailed status"));
  Serial.println(F("  stats         - Show session statistics"));
  Serial.println(F("  pause         - Pause current session"));
  Serial.println(F("  resume        - Resume paused session"));
  Serial.println(F("  end           - End session and show stats"));
  Serial.println(F("  reset         - Reset all states"));
  Serial.println(F("  save          - Save current config to EEPROM"));
  Serial.println();
  Serial.println(F("Configuration:"));
  Serial.println(F("  complexity <level>    - beginner/intermediate/advanced"));
  Serial.println(F("  operation <mode>      - normal/onetouch/gentle/practice"));
  Serial.println(F("  input <method>        - switch/sippuff/scan"));
  Serial.println(F("  scanspeed <ms>        - Set scanning speed (500-5000)"));
  Serial.println(F("  sipthreshold <val>    - Set sip sensitivity (50-500)"));
  Serial.println(F("  puffthreshold <val>   - Set puff sensitivity (50-500)"));
  Serial.println(F("  audio on/off          - Enable/disable audio feedback"));
  Serial.println(F("========================\n"));
  
  Serial.println(F("Sip-Puff Sequences:"));
  Serial.println(F("  Sip-Sip: Forward"));
  Serial.println(F("  Puff-Puff: Reverse"));
  Serial.println(F("  Sip-Puff: Turn Right"));
  Serial.println(F("  Puff-Sip: Turn Left"));
  Serial.println();
}

void printDetailedStatus() {
  Serial.println(F("\n=== DETAILED STATUS ==="));
  Serial.print(F("Session State: "));
  switch (currentSessionState) {
    case SESSION_READY: Serial.println(F("Ready")); break;
    case SESSION_ACTIVE: Serial.println(F("Active")); break;
    case SESSION_PAUSED: Serial.println(F("Paused")); break;
  }
  
  Serial.print(F("Activation Mode: "));
  printModeName(currentActivationMode);
  Serial.println();
  
  Serial.print(F("Complexity Level: "));
  switch (currentComplexity) {
    case BEGINNER: Serial.println(F("Beginner (F/R only)")); break;
    case INTERMEDIATE: Serial.println(F("Intermediate (All)")); break;
    case ADVANCED: Serial.println(F("Advanced (All)")); break;
  }
  
  Serial.print(F("Operation Mode: "));
  switch (currentOperationMode) {
    case NORMAL_OPERATION: Serial.println(F("Normal")); break;
    case ONE_TOUCH_MODE: Serial.println(F("One-Touch")); break;
    case GENTLE_MODE: Serial.println(F("Gentle")); break;
    case PRACTICE_MODE: Serial.println(F("Practice")); break;
  }
  
  Serial.print(F("Input Method: "));
  switch (currentInputMethod) {
    case SWITCH_INPUT: Serial.println(F("Switch")); break;
    case SIP_PUFF_INPUT: Serial.println(F("Sip-Puff")); break;
    case SCANNING_INPUT: Serial.println(F("Scanning")); break;
  }
  
  Serial.print(F("Current Command: "));
  printCommandName(currentProcessedCommand);
  Serial.println();
  
  Serial.print(F("Audio Enabled: ")); Serial.println(config.audioEnabled ? F("Yes") : F("No"));
  Serial.print(F("Scan Speed: ")); Serial.print(config.scanSpeed); Serial.println(F("ms"));
  Serial.println(F("=====================\n"));
}

// --- UTILITY FUNCTIONS ---
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
    if (commandSwitches[i].currentDebouncedState && isCommandAvailable(mapSwitchIndexToCommand(i))) {
      activeCount++;
      activeIndex = i;
    }
  }
  
  return (activeCount == 1) ? mapSwitchIndexToCommand(activeIndex) : CMD_NONE;
}

void resetAllStates() {
  currentProcessedCommand = CMD_NONE;
  stopAllMotors();
  
  // Reset switch states
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    commandSwitches[i].latched = false;
  }
  
  // Reset timed latch
  timedLatch.active = false;
  timedLatch.activeCommand = CMD_NONE;
  
  // Reset sip-puff state
  sipPuff.sequenceLength = 0;
  sipPuff.waitingForNeutral = false;
  
  // Reset scanning state
  scanning.active = false;
  scanning.selectionMade = false;
}

void audioFeedback(const char* message) {
  if (!config.audioEnabled) return;
  
  // Send audio feedback via serial (can be picked up by text-to-speech system)
  Serial.print(F("AUDIO: "));
  Serial.println(message);
  
  // Simple tone feedback using built-in LED as indicator
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(50);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(50);
  }
}

void printStatus() {
  static Command lastPrintedCommand = CMD_NONE;
  static ActivationMode lastPrintedMode = static_cast<ActivationMode>(-1);
  static unsigned long lastPrintTime = 0;
  
  bool commandChanged = (currentProcessedCommand != lastPrintedCommand);
  bool modeChanged = (currentActivationMode != lastPrintedMode);
  bool shouldPrint = commandChanged || modeChanged;
  
  // Periodic printing for active states
  if (!shouldPrint && currentSessionState == SESSION_ACTIVE) {
    if (millis() - lastPrintTime > PRINT_INTERVAL * 4) { // Less frequent updates
      shouldPrint = true;
    }
  }
  
  if (shouldPrint) {
    Serial.print(F("Status: "));
    
    // Session state
    switch (currentSessionState) {
      case SESSION_READY: Serial.print(F("READY")); break;
      case SESSION_ACTIVE: Serial.print(F("ACTIVE")); break;
      case SESSION_PAUSED: Serial.print(F("PAUSED")); break;
    }
    
    Serial.print(F(" | Cmd="));
    printCommandName(currentProcessedCommand);
    
    Serial.print(F(" | Mode="));
    printModeName(currentActivationMode);
    
    Serial.print(F(" | Input="));
    switch (currentInputMethod) {
      case SWITCH_INPUT: Serial.print(F("SW")); break;
      case SIP_PUFF_INPUT: Serial.print(F("SP")); break;
      case SCANNING_INPUT: Serial.print(F("SC")); break;
    }
    
    Serial.print(F(" | Level="));
    switch (currentComplexity) {
      case BEGINNER: Serial.print(F("BEG")); break;
      case INTERMEDIATE: Serial.print(F("INT")); break;
      case ADVANCED: Serial.print(F("ADV")); break;
    }
    
    // Special state indicators
    if (currentActivationMode == TIMED_LATCH_MODE && timedLatch.active) {
      unsigned long elapsed = millis() - timedLatch.startTime;
      unsigned long remaining = (timedLatch.duration > elapsed) ? (timedLatch.duration - elapsed) : 0;
      Serial.print(F(" | Timer: "));
      Serial.print(remaining / 1000.0, 1);
      Serial.print(F("s"));
    }
    
    if (currentInputMethod == SIP_PUFF_INPUT && sipPuff.sequenceLength > 0) {
      Serial.print(F(" | Seq: "));
      Serial.print(sipPuff.sequenceLength);
    }
    
    if (currentInputMethod == SCANNING_INPUT && scanning.active) {
      Serial.print(F(" | Scanning: "));
      printCommandName(scanning.availableCommands[scanning.currentOption]);
    }
    
    Serial.println();
    
    lastPrintedCommand = currentProcessedCommand;
    lastPrintedMode = currentActivationMode;
    lastPrintTime = millis();
  }
}

void printCommandName(Command cmd) {
  switch (cmd) {
    case CMD_NONE: Serial.print(F("None")); break;
    case CMD_FORWARD: Serial.print(F("Forward")); break;
    case CMD_REVERSE: Serial.print(F("Reverse")); break;
    case CMD_CLOCKWISE: Serial.print(F("Right")); break;
    case CMD_COUNTERCLOCKWISE: Serial.print(F("Left")); break;
    default: Serial.print(F("Unknown")); break;
  }
}

void printModeName(ActivationMode mode) {
  switch (mode) {
    case DIRECT_MODE: Serial.print(F("Direct")); break;
    case LATCHING_MODE: Serial.print(F("Latching")); break;
    case TIMED_LATCH_MODE: Serial.print(F("Timed")); break;
    default: Serial.print(F("Unknown")); break;
  }
}

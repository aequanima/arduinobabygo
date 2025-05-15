#include <Arduino.h>     // Standard Arduino header
#include <EEPROM.h>      // For saving mode across power cycles
#include <LowPower.h>    // For power saving on R4 Minima

// --- 1. Definitions and Constants ---

// Motor Control Output Pins
const int MOTOR_CTRL_RF_PIN = 1;
const int MOTOR_CTRL_RR_PIN = 2;
const int MOTOR_CTRL_LF_PIN = 3;
const int MOTOR_CTRL_LR_PIN = 4;

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
const unsigned long TIMED_LATCH_DURATION = 5000; // Milliseconds

// Inactivity Timeout Parameters
const unsigned long INACTIVITY_TIMEOUT_DURATION = 5 * 60 * 1000; // 5 minutes

// EEPROM Address for storing activation mode
const int EEPROM_MODE_ADDRESS = 0;
// Minimum time a mode must be active before being saved to EEPROM to prevent wear
const unsigned long MIN_MODE_DURATION_FOR_EEPROM_SAVE = 3000; // 3 seconds

// Activation Modes (Enum)
enum ActivationMode {
  DIRECT_MODE = 0,
  LATCHING_MODE = 1,
  TIMED_LATCH_MODE = 2
};
const int NUM_ACTIVATION_MODES = 3; // Total number of activation modes

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
Command activeTimedCommand = CMD_NONE;

// Inactivity Timeout State
unsigned long lastSwitchActivityTime;
bool isSleeping = false;

// EEPROM Save Logic for Mode
bool pendingEEPROMSave = false;
unsigned long modeChangeInitiatedTime = 0;
ActivationMode modeToSave;


// Current Processed Command
Command currentProcessedCommand = CMD_NONE;

// --- Forward declaration for debugging functions ---
void printCommand(Command cmd);
void printActivationModeSerial();

// --- Empty ISR for waking from sleep ---
void wakeUpISR() {
  isSleeping = false;
}

// --- 3. setup() Function ---
void setup() {
  Serial.begin(115200);
  delay(100); 
  Serial.println("\nRide-On Vehicle Controller Initializing...");
  Serial.println("----------------------------------");

  pinMode(MOTOR_CTRL_RF_PIN, INPUT);
  pinMode(MOTOR_CTRL_RR_PIN, INPUT);
  pinMode(MOTOR_CTRL_LF_PIN, INPUT);
  pinMode(MOTOR_CTRL_LR_PIN, INPUT);
  Serial.println("Motor pins initialized as INPUT (OFF).");

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    pinMode(commandSwitchPins[i], INPUT_PULLUP);
    LowPower.attachInterruptWakeup(digitalPinToInterrupt(commandSwitchPins[i]), wakeUpISR, FALLING);
  }
  Serial.println("Command switch pins initialized for input and wake-up.");

  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(MODE_SWITCH_PIN), wakeUpISR, FALLING);
  Serial.println("Mode switch pin initialized for input and wake-up.");

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
    EEPROM.update(EEPROM_MODE_ADDRESS, (byte)currentActivationMode); // Use update to write only if different
  }
  printActivationModeSerial();

  lastSwitchActivityTime = millis();
  isSleeping = false;
  pendingEEPROMSave = false; // No pending save on startup
  Serial.println("Setup complete.");
}

// --- 4. loop() Function ---
void loop() {
  if (isSleeping) {
    delay(10); 
    isSleeping = false;
  }

  readAndProcessModeSwitch(); // Handles mode cycling and initiates EEPROM save logic
  checkAndSaveModeToEEPROM(); // Handles delayed EEPROM write for mode

  handleInactivityTimeout();    // Check for inactivity and potential sleep

  if (!isSleeping) {
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
    modeSwitchJustPressed = false; // Reset after processing in readAndProcessModeSwitch
  }
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
      if (currentDebouncedModeSwitchState) { // Pressed
        modeSwitchJustPressed = true;
      }
    }
  }
  lastRawModeSwitchState = rawReading;

  if (modeSwitchJustPressed) {
    currentActivationMode = (ActivationMode)((currentActivationMode + 1) % NUM_ACTIVATION_MODES);
    
    Serial.print("Mode changed to: ");
    printActivationModeSerial();
    
    // Initiate delayed EEPROM save
    pendingEEPROMSave = true;
    modeToSave = currentActivationMode;
    modeChangeInitiatedTime = millis();

    lastSwitchActivityTime = millis(); 
    currentProcessedCommand = CMD_NONE; 
    stopAllMotors(); 
    for(int i=0; i < NUM_COMMAND_SWITCHES; ++i) commandLatched[i] = false;
    if(timedLatchActive) {
        timedLatchActive = false;
        activeTimedCommand = CMD_NONE;
    }
  }
}

void checkAndSaveModeToEEPROM() {
  if (pendingEEPROMSave) {
    if ((millis() - modeChangeInitiatedTime) > MIN_MODE_DURATION_FOR_EEPROM_SAVE) {
      // Check if the current mode is still the one we intended to save
      // (e.g., user didn't rapidly cycle again within the save delay)
      if (currentActivationMode == modeToSave) {
        byte currentEEPROMValue = EEPROM.read(EEPROM_MODE_ADDRESS);
        if (currentEEPROMValue != (byte)modeToSave) { // Only write if different
            EEPROM.write(EEPROM_MODE_ADDRESS, (byte)modeToSave);
            Serial.print("Mode '");
            printActivationModeSerial(); // Prints the mode name
            Serial.println("' saved to EEPROM after delay.");
        }
      }
      pendingEEPROMSave = false; // Reset flag whether saved or not (if mode changed again)
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
        // If a command switch is pressed while a mode save is pending for a *different* mode
        // (meaning user cycled modes then immediately used a command switch for the *new* mode),
        // cancel the old pending save and re-initiate for the current actual mode.
        modeToSave = currentActivationMode;
        modeChangeInitiatedTime = millis(); // Reset timer for the current mode
        Serial.println("Mode save timer reset due to activity in new mode.");
    }
  }
}

// --- 5.X Inactivity Timeout Handler & Sleep ---
void handleInactivityTimeout() {
  bool anyLatchOrTimedActive = timedLatchActive;
  for(int i=0; i<NUM_COMMAND_SWITCHES; ++i) {
    if(commandLatched[i]) {
      anyLatchOrTimedActive = true;
      break;
    }
  }

  if (currentProcessedCommand != CMD_NONE || anyLatchOrTimedActive) {
    if ((millis() - lastSwitchActivityTime) > INACTIVITY_TIMEOUT_DURATION) {
      Serial.println("--- Inactivity Timeout: Stopping operations. ---");
      
      // Before sleeping, ensure any pending EEPROM save is processed if conditions met
      // This is a quick check, normally checkAndSaveModeToEEPROM in loop handles it.
      if(pendingEEPROMSave && (millis() - modeChangeInitiatedTime) > MIN_MODE_DURATION_FOR_EEPROM_SAVE && currentActivationMode == modeToSave) {
        byte currentEEPROMValue = EEPROM.read(EEPROM_MODE_ADDRESS);
        if (currentEEPROMValue != (byte)modeToSave) {
            EEPROM.write(EEPROM_MODE_ADDRESS, (byte)modeToSave);
            Serial.println("Pending mode saved to EEPROM before sleep.");
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
      }
      
      Serial.println("Entering sleep mode due to inactivity...");
      Serial.flush(); 
      
      isSleeping = true; 
      LowPower.sleep(); 
      
      Serial.println("--- Woke up from sleep. ---");
      lastSwitchActivityTime = millis(); 
    }
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
    Serial.println("Warning: Multiple command switches active. Commanding CMD_NONE.");
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

  if (timedLatchActive && (currentTime - timedLatchStartTime >= TIMED_LATCH_DURATION)) {
    timedLatchActive = false;
    activeTimedCommand = CMD_NONE;
    if(currentProcessedCommand != CMD_NONE && !anyOtherLatchActive()){
        currentProcessedCommand = CMD_NONE;
    }
    Serial.println("Timed latch expired.");
  }

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (cmdSwitchJustPressed[i]) {
      Command newTimedCommand = mapSwitchIndexToCommand(i);
      if (newTimedCommand != CMD_NONE) {
          activeTimedCommand = newTimedCommand;
          currentProcessedCommand = activeTimedCommand;
          timedLatchActive = true;
          timedLatchStartTime = currentTime;
          Serial.print("Started timed latch for command: ");
          printCommand(activeTimedCommand);
          Serial.println();
      }
      return; 
    }
  }

  if (timedLatchActive) {
    currentProcessedCommand = activeTimedCommand;
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
        break;
      default:
        break;
    }
    lastExecutedCommand = cmd;
  }
}

// --- 5.4. Debugging Print Functions ---
void printActivationModeSerial() { 
  // This function now only prints the name, actual saving is handled elsewhere.
  switch (currentActivationMode) {
    case DIRECT_MODE: Serial.print("Direct"); break;
    case LATCHING_MODE: Serial.print("Latching"); break;
    case TIMED_LATCH_MODE: Serial.print("Timed Latch"); break;
    default: Serial.print("Unknown Mode"); break;
  }
  // Serial.println(); // Removed println to make it usable inline
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
  static unsigned long lastPrintTimeForTimedMode = 0;
  static ActivationMode lastPrintedActivationMode = (ActivationMode)-1; // Ensure it prints on first run

  bool shouldPrint = (currentProcessedCommand != lastPrintedCommand) || (currentActivationMode != lastPrintedActivationMode) ;
  
  if (!shouldPrint && currentActivationMode == TIMED_LATCH_MODE && timedLatchActive) {
    if (millis() - lastPrintTimeForTimedMode > 1000) {
      shouldPrint = true;
    }
  }

  if (shouldPrint) {
    Serial.print("State: Cmd=");
    printCommand(currentProcessedCommand);
    Serial.print(" | Mode=");
    printActivationModeSerial(); // Use the modified inline version

    if (currentActivationMode == TIMED_LATCH_MODE && timedLatchActive) {
      unsigned long elapsed = millis() - timedLatchStartTime;
      unsigned long remaining = (TIMED_LATCH_DURATION > elapsed) ? (TIMED_LATCH_DURATION - elapsed) : 0;
      Serial.print(" (Time Rem: ");
      Serial.print(remaining / 1000.0, 1); 
      Serial.print("s)");
    }
    Serial.println();

    lastPrintedCommand = currentProcessedCommand;
    lastPrintedActivationMode = currentActivationMode;

    if (currentActivationMode == TIMED_LATCH_MODE && timedLatchActive) {
        lastPrintTimeForTimedMode = millis();
    } else if (currentProcessedCommand != lastPrintedCommand || currentActivationMode != lastPrintedActivationMode) {
        lastPrintTimeForTimedMode = millis();
    }
  }
}

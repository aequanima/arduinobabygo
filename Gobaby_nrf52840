//nrf52840 seeed studio based microcontroller variation
//WIP don't use yet


#include <Arduino.h>

// --- Adafruit Bluefruit nRF52 BLE Libraries ---
#include <bluefruit.h>

// --- nRF52840 Flash Storage (LittleFS via InternalFileSystem) ---
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

// Configuration file on the LittleFS
#define CONFIG_FILENAME "/rideon_config.dat"

// --- 1. Definitions and Configuration Structure ---

// Activation Modes (Enum)
enum ActivationMode {
  DIRECT_MODE = 0,
  LATCHING_MODE = 1,
  TIMED_LATCH_MODE = 2
};
const int NUM_ACTIVATION_MODES = 3; // Should match enum size

// Configuration Data Structure
struct ConfigData {
  unsigned long debounceDelay;
  unsigned long timedLatchDurationNonRotational;
  float degreesPerTimedRotationPress;
  unsigned long fullRotationDurationMs;
  unsigned long inactivityTimeoutDuration;
  unsigned long minModeDurationForFlashSave;
  ActivationMode activationMode;
  // Add a version number for future struct changes
  uint16_t version;
};

// Global instance of our configuration, initialized with defaults
ConfigData currentConfig = {
  50,    // debounceDelay
  5000,  // timedLatchDurationNonRotational
  30.0f, // degreesPerTimedRotationPress
  3000,  // fullRotationDurationMs
  5 * 60 * 1000, // inactivityTimeoutDuration (5 minutes)
  3000,  // minModeDurationForFlashSave
  DIRECT_MODE, // activationMode
  1      // version
};

// THESE PINS MUST BE RE-ASSIGNED FOR YOUR XIAO NRF52840 BOARD!
// Example pins based on the XIAO nRF52840 diagram (P<port>_<pin>)
// Use the integer defines provided by the board package if available (e.g., PIN_P0_13)
// For XIAO nRF52840, you can use D0-D10 as digital pins.
// The schematic maps D0-D5 to P0.02, P0.03, P0.28, P0.29, P0.04, P0.05
// D6-D10 to P0.31(AREF), P0.00(NFC1), P0.01(NFC2), P1.11, P1.10
// Let's choose some plausible digital pins:
const int MOTOR_CTRL_RF_PIN = D0; // P0.02
const int MOTOR_CTRL_RR_PIN = D1; // P0.03
const int MOTOR_CTRL_LF_PIN = D2; // P0.28
const int MOTOR_CTRL_LR_PIN = D3; // P0.29

const int SWITCH_FORWARD_PIN        = D4; // P0.04
const int SWITCH_REVERSE_PIN        = D5; // P0.05
const int SWITCH_CLOCKWISE_PIN      = D6; // P0.31 (Be careful if also using as AREF)
const int SWITCH_COUNTERCLOCKWISE_PIN = D7; // P0.00 (NFC1 - ensure NFC is not actively used if re-purposing)
const int MODE_SWITCH_PIN           = D8; // P0.01 (NFC2 - ensure NFC is not actively used if re-purposing)

// Re-verify D6, D7, D8 for your use case if NFC or specific analog functions are needed.
// Using higher numbered digital pins like D9 (P1.11), D10 (P1.10) is safer if unsure.

const int NUM_COMMAND_SWITCHES = 4;
const int commandSwitchPins[NUM_COMMAND_SWITCHES] = {
  SWITCH_FORWARD_PIN,
  SWITCH_REVERSE_PIN,
  SWITCH_CLOCKWISE_PIN,
  SWITCH_COUNTERCLOCKWISE_PIN
};

// Calculated duration for a timed rotation press - updated by updateCalculatedDurations()
unsigned long TIMED_ROTATION_PRESS_DURATION_MS;


// Commands (Enum)
enum Command {
  CMD_NONE,
  CMD_FORWARD,
  CMD_REVERSE,
  CMD_CLOCKWISE,
  CMD_COUNTERCLOCKWISE
};

// --- 2. Global State Variables ---
// (Switch states, latching states, timed latch states, etc. remain the same)
int lastRawCmdSwitchState[NUM_COMMAND_SWITCHES];
bool currentDebouncedCmdSwitchState[NUM_COMMAND_SWITCHES];
bool cmdSwitchJustPressed[NUM_COMMAND_SWITCHES];
bool cmdSwitchJustReleased[NUM_COMMAND_SWITCHES];
unsigned long lastCmdSwitchDebounceTime[NUM_COMMAND_SWITCHES];

int lastRawModeSwitchState = HIGH;
bool currentDebouncedModeSwitchState = false; // false means not pressed
bool modeSwitchJustPressed = false;
unsigned long lastModeSwitchDebounceTime = 0;

bool commandLatched[NUM_COMMAND_SWITCHES];

bool timedLatchActive = false;
unsigned long timedLatchStartTime = 0;
unsigned long currentActiveTimedLatchDuration = 0;
Command activeTimedCommand = CMD_NONE;

unsigned long lastSwitchActivityTime;

// Flash Save Logic for Mode (now part of general config save)
bool pendingConfigSave = false; // Flag to indicate currentConfig in RAM differs from flash
unsigned long modeChangeInitiatedTime = 0; // Used for MIN_MODE_DURATION_FOR_FLASH_SAVE logic

Command currentProcessedCommand = CMD_NONE;

// BLE UART Service
BLEUart bleuart;

// --- Forward declarations ---
void printCommandName(Command cmd);
void printActivationModeSerial(ActivationMode mode); // Takes mode as param
void updateCalculatedDurations();
void handleBleCommands(String cmd);
void sendBleReply(const String& reply);
void saveConfiguration();
void loadConfiguration();
void applyDefaultsAndSave();


// --- Helper: Update calculated values if parameters change ---
void updateCalculatedDurations() {
    TIMED_ROTATION_PRESS_DURATION_MS = (unsigned long)((currentConfig.fullRotationDurationMs / 360.0f) * currentConfig.degreesPerTimedRotationPress);
    Serial.println("Recalculated TIMED_ROTATION_PRESS_DURATION_MS: " + String(TIMED_ROTATION_PRESS_DURATION_MS));
}


// --- 3. setup() Function ---
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nRide-On Vehicle Controller Initializing (XIAO nRF52840)...");
  Serial.println("------------------------------------------------------");
  Serial.println("IMPORTANT: Verify MOTOR and SWITCH pin assignments!");
  Serial.println("Pins D6, D7, D8 might conflict with NFC/AREF if used.");
  Serial.println("------------------------------------------------------");

  Serial.print("Initializing Filesystem...");
  if (InternalFS.begin()) {
    Serial.println(" Success!");
    loadConfiguration(); // Load config from flash, or apply defaults if not found/invalid
  } else {
    Serial.println(" Failed! Using hardcoded default configuration.");
    // If FS fails, we can't save, so we just use currentConfig defaults
    updateCalculatedDurations(); // Ensure calculations are based on defaults
  }

  Serial.println("Current Configuration in use:");
  Serial.print("  Debounce Delay: "); Serial.println(currentConfig.debounceDelay);
  Serial.print("  Timed Fwd/Rev Duration: "); Serial.println(currentConfig.timedLatchDurationNonRotational);
  Serial.print("  Degrees per Timed Rot Press: "); Serial.println(currentConfig.degreesPerTimedRotationPress);
  Serial.print("  Full Rotation Duration: "); Serial.println(currentConfig.fullRotationDurationMs);
  Serial.print("  Calculated Timed Rot Press Duration: "); Serial.println(TIMED_ROTATION_PRESS_DURATION_MS);
  Serial.print("  Inactivity Timeout: "); Serial.println(currentConfig.inactivityTimeoutDuration);
  Serial.print("  Min Mode Duration for Save: "); Serial.println(currentConfig.minModeDurationForFlashSave);
  Serial.print("  Activation Mode: "); printActivationModeSerial(currentConfig.activationMode); Serial.println();
  Serial.print("  Config Version: "); Serial.println(currentConfig.version);
  Serial.println("----------------------------------");

  // --- Initialize BLE ---
  Serial.println("Initializing Bluetooth...");
  Bluefruit.begin();
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  bleuart.begin(); // Configure and Start BLE Uart Service
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName(); // Device name configured by Bluefruit.setName() or default
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
  Serial.println("Bluetooth Initialized. Advertising...");

  // --- Initialize Pins ---
  // (Motor and Switch pin initialization logic remains the same)
  pinMode(MOTOR_CTRL_RF_PIN, INPUT);
  pinMode(MOTOR_CTRL_RR_PIN, INPUT);
  pinMode(MOTOR_CTRL_LF_PIN, INPUT);
  pinMode(MOTOR_CTRL_LR_PIN, INPUT);
  Serial.println("Motor pins initialized as INPUT (OFF).");

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    pinMode(commandSwitchPins[i], INPUT_PULLUP);
  }
  Serial.println("Command switch pins initialized for input with PULLUP.");
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  Serial.println("Mode switch pin initialized for input with PULLUP.");

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    lastRawCmdSwitchState[i] = HIGH;
    currentDebouncedCmdSwitchState[i] = false;
    cmdSwitchJustPressed[i] = false;
    cmdSwitchJustReleased[i] = false;
    lastCmdSwitchDebounceTime[i] = 0;
    commandLatched[i] = false;
  }
  lastRawModeSwitchState = HIGH;
  currentDebouncedModeSwitchState = false;
  modeSwitchJustPressed = false;
  lastModeSwitchDebounceTime = 0;

  lastSwitchActivityTime = millis();
  // pendingConfigSave is managed by BLE commands or explicit save triggers
  Serial.println("Setup complete.");
}

// --- BLE Connection Callbacks ---
void connect_callback(uint16_t conn_handle) {
  (void) conn_handle;
  Serial.println("BLE Client Connected");
  sendBleReply("RideOnController Connected. Type 'HELP'.");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;
  Serial.println("BLE Client Disconnected");
}

// --- Configuration Save/Load Functions ---
void loadConfiguration() {
  if (InternalFS.exists(CONFIG_FILENAME)) {
    File configFile = InternalFS.open(CONFIG_FILENAME, FILE_READ);
    if (configFile) {
      ConfigData tempConfig;
      if (configFile.read((uint8_t*)&tempConfig, sizeof(ConfigData)) == sizeof(ConfigData)) {
        // Basic version check (can be more sophisticated)
        if (tempConfig.version == currentConfig.version) {
          currentConfig = tempConfig;
          Serial.println("Configuration loaded from Flash.");
        } else {
          Serial.print("Config file version mismatch (file: ");
          Serial.print(tempConfig.version);
          Serial.print(", expected: ");
          Serial.print(currentConfig.version);
          Serial.println("). Applying defaults and resaving.");
          applyDefaultsAndSave(); // Load defaults and save them with current version
        }
      } else {
        Serial.println("Failed to read correct size from config file. Applying defaults.");
        applyDefaultsAndSave();
      }
      configFile.close();
    } else {
      Serial.println("Could not open config file for reading. Applying defaults.");
      applyDefaultsAndSave();
    }
  } else {
    Serial.println("Config file not found. Applying defaults and creating file.");
    applyDefaultsAndSave();
  }
  updateCalculatedDurations(); // Ensure calculations are based on loaded/default values
}

void saveConfiguration() {
  File configFile = InternalFS.open(CONFIG_FILENAME, FILE_WRITE);
  if (configFile) {
    if (configFile.write((uint8_t*)&currentConfig, sizeof(ConfigData)) == sizeof(ConfigData)) {
      Serial.println("Configuration saved to Flash.");
      sendBleReply("Configuration saved to Flash.");
      pendingConfigSave = false; // Reset flag
    } else {
      Serial.println("Failed to write complete configuration to Flash!");
      sendBleReply("Error: Failed to write full config.");
    }
    configFile.close();
  } else {
    Serial.println("Failed to open config file for writing!");
    sendBleReply("Error: Cannot open config file to save.");
  }
}

void applyDefaultsAndSave() {
  // Initialize currentConfig with hardcoded defaults (already done at global declaration)
  // For clarity, can re-assign here if desired or if defaults might change based on some logic
  currentConfig = {
    50, 5000, 30.0f, 3000, 5 * 60 * 1000, 3000, DIRECT_MODE, 1
  };
  Serial.println("Applied hardcoded default configuration values.");
  saveConfiguration(); // Save these defaults to flash
}


// --- 4. loop() Function ---
void loop() {
  if (bleuart.available()) {
    String command = bleuart.readStringUntil('\n');
    command.trim();
    Serial.print("[BLE CMD]: "); Serial.println(command);
    handleBleCommands(command);
  }

  readAndProcessModeSwitch(); // This will modify currentConfig.activationMode
  checkAndSaveModeConfigLogic(); // Checks if mode changed by switch needs saving

  handleInactivityTimeout();
  readAllCommandSwitches();

  switch (currentConfig.activationMode) {
    case DIRECT_MODE: handleDirectMode(); break;
    case LATCHING_MODE: handleLatchingMode(); break;
    case TIMED_LATCH_MODE: handleTimedLatchMode(); break;
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

void readAndProcessModeSwitch() {
  unsigned long currentTime = millis();
  int rawReading = digitalRead(MODE_SWITCH_PIN);

  if (rawReading != lastRawModeSwitchState) {
    lastModeSwitchDebounceTime = currentTime;
  }

  if ((currentTime - lastModeSwitchDebounceTime) > currentConfig.debounceDelay) {
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
    timedLatchActive = false; activeTimedCommand = CMD_NONE; currentActiveTimedLatchDuration = 0;

    currentConfig.activationMode = (ActivationMode)((currentConfig.activationMode + 1) % NUM_ACTIVATION_MODES);
    pendingConfigSave = true; // Mark that config (specifically mode) has changed
    modeChangeInitiatedTime = millis(); // For the min duration check before auto-save

    Serial.print("Mode changed by switch to: "); printActivationModeSerial(currentConfig.activationMode); Serial.println();
    sendBleReply("Mode changed by switch to: " + String(currentConfig.activationMode));

    lastSwitchActivityTime = millis(); currentProcessedCommand = CMD_NONE; stopAllMotors();
    for(int i=0; i < NUM_COMMAND_SWITCHES; ++i) commandLatched[i] = false;
  }
}

void checkAndSaveModeConfigLogic() {
  // This function handles the specific case where the physical mode switch
  // changes the mode, and we want to save it after a small delay to prevent excessive writes.
  // General config changes via BLE are saved explicitly with SAVE_CONFIG.
  if (pendingConfigSave && modeChangeInitiatedTime != 0) { // modeChangeInitiatedTime is set by physical switch
    if ((millis() - modeChangeInitiatedTime) > currentConfig.minModeDurationForFlashSave) {
        Serial.println("Auto-saving mode change from physical switch.");
        saveConfiguration(); // This saves the whole currentConfig, which includes the new mode
        modeChangeInitiatedTime = 0; // Reset timer, pendingConfigSave is reset in saveConfiguration
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
    if ((currentTime - lastCmdSwitchDebounceTime[i]) > currentConfig.debounceDelay) {
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
    // If mode changed again by switch before auto-save timer elapsed for previous change
    if(pendingConfigSave && modeChangeInitiatedTime != 0) {
        // The modeToSave concept is implicitly handled as currentConfig.activationMode is already updated.
        // We just need to ensure the timer for auto-save is for the *latest* switch-initiated change.
        // modeChangeInitiatedTime = millis(); // This might be too aggressive, let current timer run.
                                            // The checkAndSaveModeConfigLogic saves currentConfig.activationMode.
    }
  }
}

void handleInactivityTimeout() {
  bool anyLatchOrTimedActive = timedLatchActive;
  for(int i=0; i<NUM_COMMAND_SWITCHES; ++i) { if(commandLatched[i]) anyLatchOrTimedActive = true; }

  if (currentProcessedCommand != CMD_NONE || anyLatchOrTimedActive) {
      if ((millis() - lastSwitchActivityTime) > currentConfig.inactivityTimeoutDuration) {
        Serial.println("--- Inactivity Timeout: Stopping operations. ---");
        sendBleReply("Inactivity Timeout: Operations stopped.");

        // Auto-save mode if it was changed by switch and timer elapsed during inactivity
        if(pendingConfigSave && modeChangeInitiatedTime != 0 && (millis() - modeChangeInitiatedTime) > currentConfig.minModeDurationForFlashSave) {
            Serial.println("Auto-saving mode before inactivity stop.");
            saveConfiguration();
            modeChangeInitiatedTime = 0;
        }

        currentProcessedCommand = CMD_NONE; stopAllMotors();
        for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) commandLatched[i] = false;
        if (timedLatchActive) { timedLatchActive = false; activeTimedCommand = CMD_NONE; currentActiveTimedLatchDuration = 0;}
        lastSwitchActivityTime = millis();
      }
  } else if ((millis() - lastSwitchActivityTime) > currentConfig.inactivityTimeoutDuration) {
      lastSwitchActivityTime = millis();
  }
}

// mapSwitchIndexToCommand, getCommandFromSwitches, handleDirectMode, handleLatchingMode remain the same

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
      if (pressedSwitchIndex == -1) pressedSwitchIndex = i;
    }
  }
  if (pressedCount > 1) return CMD_NONE;
  else if (pressedCount == 1) return mapSwitchIndexToCommand(pressedSwitchIndex);
  return CMD_NONE;
}

void handleDirectMode() {
  if (timedLatchActive) return;
  currentProcessedCommand = getCommandFromSwitches();
}

void handleLatchingMode() {
  if (timedLatchActive) return;
  Command newCommandToProcess = CMD_NONE;
  bool anyLatchActionThisCycle = false;
  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (cmdSwitchJustPressed[i]) {
      anyLatchActionThisCycle = true;
      commandLatched[i] = !commandLatched[i];
      if (commandLatched[i]) {
        newCommandToProcess = mapSwitchIndexToCommand(i);
        for (int j = 0; j < NUM_COMMAND_SWITCHES; j++) if (i != j) commandLatched[j] = false;
      }
      break;
    }
  }
  if (!anyLatchActionThisCycle) {
    for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) if (commandLatched[i]) newCommandToProcess = mapSwitchIndexToCommand(i);
  }
  currentProcessedCommand = newCommandToProcess;
}


void handleTimedLatchMode() {
  unsigned long currentTime = millis();
  if (timedLatchActive && (currentTime - timedLatchStartTime >= currentActiveTimedLatchDuration)) {
    Serial.print("Timed latch expired for "); printCommandName(activeTimedCommand); Serial.println();
    timedLatchActive = false; activeTimedCommand = CMD_NONE; currentProcessedCommand = CMD_NONE; currentActiveTimedLatchDuration = 0;
  }

  for (int i = 0; i < NUM_COMMAND_SWITCHES; i++) {
    if (cmdSwitchJustPressed[i]) {
      Command newTimedCommand = mapSwitchIndexToCommand(i);
      if (newTimedCommand != CMD_NONE) {
        if (timedLatchActive && activeTimedCommand != newTimedCommand) { Serial.print("Timed latch overridden: "); printCommandName(activeTimedCommand); Serial.print(" by "); printCommandName(newTimedCommand); Serial.println();}
        else if (timedLatchActive && activeTimedCommand == newTimedCommand) { Serial.print("Timed latch re-triggered: "); printCommandName(activeTimedCommand); Serial.println();}
        activeTimedCommand = newTimedCommand; currentProcessedCommand = activeTimedCommand;
        if (activeTimedCommand == CMD_CLOCKWISE || activeTimedCommand == CMD_COUNTERCLOCKWISE) {
          currentActiveTimedLatchDuration = TIMED_ROTATION_PRESS_DURATION_MS; // Uses calculated global
          Serial.print("Started timed ROTATION (");
        } else {
          currentActiveTimedLatchDuration = currentConfig.timedLatchDurationNonRotational;
          Serial.print("Started timed NON-ROTATIONAL (");
        }
        Serial.print(currentActiveTimedLatchDuration); Serial.print("ms) for: "); printCommandName(activeTimedCommand); Serial.println();
        timedLatchActive = true; timedLatchStartTime = currentTime; lastSwitchActivityTime = currentTime;
      }
      return;
    }
  }
  if (timedLatchActive) currentProcessedCommand = activeTimedCommand;
  else currentProcessedCommand = CMD_NONE;
}

// setMotorPinActive, stopAllMotors, executeMotorCommand remain the same

void setMotorPinActive(int motorPin, bool active) {
  if (active) { pinMode(motorPin, OUTPUT); digitalWrite(motorPin, LOW); }
  else { pinMode(motorPin, INPUT); }
}

void stopAllMotors() {
  setMotorPinActive(MOTOR_CTRL_RF_PIN, false); setMotorPinActive(MOTOR_CTRL_RR_PIN, false);
  setMotorPinActive(MOTOR_CTRL_LF_PIN, false); setMotorPinActive(MOTOR_CTRL_LR_PIN, false);
}

void executeMotorCommand(Command cmd) {
  static Command lastExecutedCommand = CMD_NONE;
  if (cmd != lastExecutedCommand || (cmd == CMD_NONE && lastExecutedCommand != CMD_NONE) ) {
    stopAllMotors();
    switch (cmd) {
      case CMD_FORWARD: setMotorPinActive(MOTOR_CTRL_RF_PIN, true); setMotorPinActive(MOTOR_CTRL_LF_PIN, true); break;
      case CMD_REVERSE: setMotorPinActive(MOTOR_CTRL_RR_PIN, true); setMotorPinActive(MOTOR_CTRL_LR_PIN, true); break;
      case CMD_CLOCKWISE: setMotorPinActive(MOTOR_CTRL_RR_PIN, true); setMotorPinActive(MOTOR_CTRL_LF_PIN, true); break;
      case CMD_COUNTERCLOCKWISE: setMotorPinActive(MOTOR_CTRL_RF_PIN, true); setMotorPinActive(MOTOR_CTRL_LR_PIN, true); break;
      case CMD_NONE: break;
      default: break;
    }
    lastExecutedCommand = cmd;
  }
}

// --- 5.4. Debugging Print Functions & BLE Command Handling ---
void printActivationModeSerial(ActivationMode mode) { // Now takes mode as a parameter
  switch (mode) {
    case DIRECT_MODE: Serial.print("Direct"); break;
    case LATCHING_MODE: Serial.print("Latching"); break;
    case TIMED_LATCH_MODE: Serial.print("Timed Latch"); break;
    default: Serial.print("Unknown Mode"); break;
  }
}

void printCommandName(Command cmd) { /* ... same ... */ } // Unchanged, keep for brevity
// void printCommandName(Command cmd) { ... } // Placeholder for unchanged function

void printCurrentState() {
  static Command lastPrintedCommand = CMD_NONE;
  static unsigned long lastPrintTime = 0;
  static ActivationMode lastPrintedActivationMode = (ActivationMode)-1; // Cast to avoid warning
  const unsigned long PRINT_INTERVAL = 250;

  bool commandChanged = (currentProcessedCommand != lastPrintedCommand);
  bool modeChanged = (currentConfig.activationMode != lastPrintedActivationMode); // Use currentConfig
  bool shouldPrint = commandChanged || modeChanged;

  if (!shouldPrint && currentConfig.activationMode == TIMED_LATCH_MODE && timedLatchActive) {
    if (millis() - lastPrintTime > PRINT_INTERVAL) shouldPrint = true;
  }
  if (!shouldPrint && lastPrintedCommand != CMD_NONE && currentProcessedCommand == CMD_NONE) shouldPrint = true;

  if (shouldPrint) {
    Serial.print("State: Cmd="); printCommandName(currentProcessedCommand);
    Serial.print(" | Mode="); printActivationModeSerial(currentConfig.activationMode); // Use currentConfig
    if (currentConfig.activationMode == TIMED_LATCH_MODE && timedLatchActive) {
      unsigned long elapsed = millis() - timedLatchStartTime;
      unsigned long remaining = (currentActiveTimedLatchDuration > elapsed) ? (currentActiveTimedLatchDuration - elapsed) : 0;
      Serial.print(" (TL Rem: "); Serial.print(remaining / 1000.0, 1); Serial.print("s / ");
      Serial.print(currentActiveTimedLatchDuration / 1000.0, 1); Serial.print("s)");
    }
    Serial.println();
    lastPrintedCommand = currentProcessedCommand;
    lastPrintedActivationMode = currentConfig.activationMode; // Use currentConfig
    lastPrintTime = millis();
  }
}


void sendBleReply(const String& reply) {
    if (Bluefruit.connected()) bleuart.println(reply);
    Serial.print("[BLE REPLY]: "); Serial.println(reply);
}

void handleBleCommands(String cmd) {
  cmd.toUpperCase();
  String reply = "Unknown command: " + cmd;
  bool configChangedByBle = false;

  if (cmd.startsWith("HELP")) {
    reply = "RideOn Controller Commands:\n";
    reply += "SAVE_CONFIG\nLOAD_CONFIG\nRESET_CONFIG\n";
    reply += "GET_CONFIG\n";
    reply += "SET MODE [0-Direct,1-Latch,2-Timed]\n";
    reply += "SET DEBOUNCE_MS [val]\n";
    reply += "SET TIMED_FWDREV_MS [val]\n";
    reply += "SET ROT_DEG [val]\n";
    reply += "SET ROT_FULL_MS [val]\n";
    reply += "SET INACTIVITY_MS [val]\n";
    reply += "SET SAVE_DELAY_MS [val]\n"; // MinModeDurationForFlashSave
  } else if (cmd.startsWith("SAVE_CONFIG")) {
    saveConfiguration();
    reply = "Configuration save attempt finished."; // saveConfiguration sends its own success/fail
    // Avoid double reply by not setting 'reply' directly here unless specific
    return; // saveConfiguration calls sendBleReply
  } else if (cmd.startsWith("LOAD_CONFIG")) {
    loadConfiguration(); // Loads from flash and updates currentConfig
    reply = "Configuration loaded from flash into RAM.";
  } else if (cmd.startsWith("RESET_CONFIG")) {
    applyDefaultsAndSave(); // Applies defaults to currentConfig and saves to flash
    reply = "Configuration reset to defaults and saved.";
    return; // applyDefaultsAndSave calls saveConfiguration which calls sendBleReply
  } else if (cmd.startsWith("GET_CONFIG")) {
    reply = "Current RAM Config:\n";
    reply += " MODE=" + String(currentConfig.activationMode) + "\n";
    reply += " DEBOUNCE_MS=" + String(currentConfig.debounceDelay) + "\n";
    reply += " TIMED_FWDREV_MS=" + String(currentConfig.timedLatchDurationNonRotational) + "\n";
    reply += " ROT_DEG=" + String(currentConfig.degreesPerTimedRotationPress, 1) + "\n";
    reply += " ROT_FULL_MS=" + String(currentConfig.fullRotationDurationMs) + "\n";
    reply += " INACTIVITY_MS=" + String(currentConfig.inactivityTimeoutDuration) + "\n";
    reply += " SAVE_DELAY_MS=" + String(currentConfig.minModeDurationForFlashSave) + "\n";
    reply += " CALC_ROT_PRESS_MS=" + String(TIMED_ROTATION_PRESS_DURATION_MS);
  } else if (cmd.startsWith("SET MODE ")) {
    int val = cmd.substring(9).toInt();
    if (val >= 0 && val < NUM_ACTIVATION_MODES) {
      if (currentConfig.activationMode != (ActivationMode)val) {
        currentConfig.activationMode = (ActivationMode)val;
        // Reset relevant states when mode changes programmatically
        timedLatchActive = false; activeTimedCommand = CMD_NONE; currentActiveTimedLatchDuration = 0;
        currentProcessedCommand = CMD_NONE; stopAllMotors();
        for(int i=0; i < NUM_COMMAND_SWITCHES; ++i) commandLatched[i] = false;
        reply = "RAM: Mode set to " + String(currentConfig.activationMode) + ". Use SAVE_CONFIG.";
        configChangedByBle = true;
      } else { reply = "RAM: Mode already " + String(val) + ". No change.";}
    } else { reply = "Invalid mode value."; }
  } else if (cmd.startsWith("SET DEBOUNCE_MS ")) {
    unsigned long val = cmd.substring(16).toInt();
    if (val > 0 && val < 1000) { currentConfig.debounceDelay = val; reply = "RAM: DEBOUNCE_MS=" + String(val); configChangedByBle = true;}
    else { reply = "Invalid DEBOUNCE_MS value.";}
  } else if (cmd.startsWith("SET TIMED_FWDREV_MS ")) {
    unsigned long val = cmd.substring(20).toInt();
    if (val >= 100 && val <= 60000) { currentConfig.timedLatchDurationNonRotational = val; reply = "RAM: TIMED_FWDREV_MS=" + String(val); configChangedByBle = true;}
    else { reply = "Invalid TIMED_FWDREV_MS value.";}
  } else if (cmd.startsWith("SET ROT_DEG ")) {
    float val = cmd.substring(12).toFloat();
    if (val > 0 && val <= 360) { currentConfig.degreesPerTimedRotationPress = val; updateCalculatedDurations(); reply = "RAM: ROT_DEG=" + String(val,1); configChangedByBle = true;}
    else { reply = "Invalid ROT_DEG value.";}
  } else if (cmd.startsWith("SET ROT_FULL_MS ")) {
    unsigned long val = cmd.substring(16).toInt();
    if (val >= 100 && val <= 60000) { currentConfig.fullRotationDurationMs = val; updateCalculatedDurations(); reply = "RAM: ROT_FULL_MS=" + String(val); configChangedByBle = true;}
    else { reply = "Invalid ROT_FULL_MS value.";}
  } else if (cmd.startsWith("SET INACTIVITY_MS ")) {
    unsigned long val = cmd.substring(18).toInt();
    if (val >= 10000) { currentConfig.inactivityTimeoutDuration = val; reply = "RAM: INACTIVITY_MS=" + String(val); configChangedByBle = true;} // Min 10 sec
    else { reply = "Invalid INACTIVITY_MS value (min 10000).";}
  } else if (cmd.startsWith("SET SAVE_DELAY_MS ")) {
    unsigned long val = cmd.substring(18).toInt();
    if (val >= 500 && val <= 30000) { currentConfig.minModeDurationForFlashSave = val; reply = "RAM: SAVE_DELAY_MS=" + String(val); configChangedByBle = true;}
    else { reply = "Invalid SAVE_DELAY_MS value (500-30000).";}
  }

  if (configChangedByBle) {
    pendingConfigSave = true; // Remind user to save
    reply += " (Unsaved)";
  }
  sendBleReply(reply);
}

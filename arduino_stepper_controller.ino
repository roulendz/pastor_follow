/**
 * @file arduino_stepper_controller.ino
 * @brief Arduino Stepper Motor Controller for Pastor Follow System.
 *
 * This sketch controls a stepper motor using the AccelStepper library,
 * designed for precise angular positioning in a pastor follow system.
 * It communicates via serial interface, accepting commands for movement,
 * settings adjustment, and status queries.
 *
 * Hardware Connections:
 * D2 -> DIR+ (DM542 Driver Direction Pin)
 * D3 -> PUL+ (DM542 Driver Pulse Pin - Must be PWM capable)
 * D4 -> ENA+ (DM542 Driver Enable Pin)
 *
 * Common Cathode Wiring:
 * Arduino GND -> DM542 DIR-, PUL-, ENA-
 *
 * @author Rolands Zeltins | Logingrupa
 * @date 2025-11-18
 * @version 1.0.0
 * @license MIT License (or appropriate license)
 */

#include <AccelStepper.h>
#include <EEPROM.h>

// Pin definitions for stepper motor control
#define DIR_PIN 2      // Digital pin connected to the DIR+ (Direction) pin of the stepper driver
#define STEP_PIN 3     // Digital pin connected to the PUL+ (Pulse/Step) pin of the stepper driver (must be PWM capable)
#define ENABLE_PIN 4   // Digital pin connected to the ENA+ (Enable) pin of the stepper driver

// Enable polarity configuration:
// Set to 1 if HIGH voltage enables the driver (typical for common cathode wiring)
// Set to 0 if LOW voltage enables the driver (for common anode or inverted logic)
#define ENABLE_ACTIVE_HIGH 0

// Limit Switch Pin Definitions
#define LIMIT_SWITCH_MIN_PIN 5 // Digital pin for the minimum limit switch
#define LIMIT_SWITCH_MAX_PIN 6 // Digital pin for the maximum limit switch

// EEPROM Address Definitions
#define EEPROM_MAX_SPEED_ADDR 0
#define EEPROM_MAX_ACCEL_ADDR sizeof(float) // Offset by size of float
#define EEPROM_PID_P_ADDR (2 * sizeof(float)) // Offset by 2 * size of float
#define EEPROM_PID_I_ADDR (3 * sizeof(float))
#define EEPROM_PID_D_ADDR (4 * sizeof(float))

// Stepper motor and driver configuration parameters
#define STEPS_PER_REV 200  // Number of full steps per revolution of the motor (e.g., 1.8 degree/step motor has 200 steps/rev)
#define GEAR_RATIO 180     // Gear ratio of the gearbox attached to the motor (e.g., 180:1)
#define MICROSTEPS 8       // Microstepping setting on the DM542 driver (e.g., 8 for 1/8 microstepping)
#define TOTAL_STEPS_PER_REV ( (long)STEPS_PER_REV * GEAR_RATIO * MICROSTEPS) // Total microsteps for one full revolution of the output shaft

// Create an instance of the AccelStepper library for controlling the stepper motor.
// AccelStepper::DRIVER specifies that a stepper driver (like DM542) is used,
// where direction and step signals are separate.
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Helper function to enable the stepper motor driver.
// The actual HIGH/LOW state depends on the ENABLE_ACTIVE_HIGH definition.
void enableDriver() {
  digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? HIGH : LOW);
}

// Helper function to disable the stepper motor driver.
// This typically cuts power to the motor, allowing it to be moved manually or to save power.
void disableDriver() {
  digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? LOW : HIGH);
}

// Offset for command data in the serial input string.
// Commands are expected in the format "C,data", where 'C' is the command type
// and 'data' is the argument. The offset skips the command type and comma.
#define COMMAND_DATA_OFFSET 2

// Structure to hold parsed command information.
struct Command {
  char type;  // Type of command: 'M' (move), 'S' (settings), 'R' (reset), 'Q' (query), etc.
  char* args; // Pointer to the arguments string following the command type (e.g., "10.5,20.0")
};

// Structure to hold feedback data sent back to the host system.
struct Feedback {
  long currentPosition;   // Current position of the stepper motor in steps.
  float currentSpeed;     // Current speed of the stepper motor in steps/second.
  bool isMoving;          // True if the motor is currently moving, false otherwise.
  long targetPosition;    // The target position the motor is moving towards in steps.
  unsigned long timestamp; // Timestamp when the feedback was generated (in milliseconds since Arduino startup).
};

// PID parameters (currently placeholders, as AccelStepper handles motion profiles internally).
// These variables are kept for potential future custom PID loop implementation or
// for passing to a higher-level control system.
float maxSpeed = 25000.0; // Maximum speed setting for the stepper motor in steps/second.
                          // Corresponds to 25 deg/s stage speed with 10 microsteps.
float maxAccel = 12500.0; // Maximum acceleration setting for the stepper motor in steps/second^2.
                          // Typically set to half of maxSpeed for smooth acceleration/deceleration.
float pidP = 1.0;         // Proportional gain for a PID controller (if implemented).
float pidI = 0.0;         // Integral gain for a PID controller (if implemented).
float pidD = 0.1;         // Derivative gain for a PID controller (if implemented).

// Global variables for tracking motor state and feedback timing.
long targetPosition = 0; // The desired target position in steps.
long lastPosition = 0;   // Stores the last known position for speed calculation or other purposes.
unsigned long lastFeedbackTime = 0; // Timestamp of the last feedback sent.
const unsigned long FEEDBACK_INTERVAL = 20;  // Interval in milliseconds for sending periodic feedback (e.g., 20ms for 50Hz).

// Buffer for incoming serial data.
char inputBuffer[32]; // Character array to store incoming serial commands.
int bufferIndex = 0;  // Current index in the inputBuffer.

// Function to parse a raw command string received via serial.
// It extracts the command type and its arguments.
Command parseCommand(char* commandString) {
  Command cmd;
  cmd.type = '\0'; // Initialize command type to null character.
  cmd.args = nullptr; // Initialize arguments pointer to null.

  // Validate the input command string.
  if (commandString == nullptr || strlen(commandString) == 0) {
    reportError(EMPTY_COMMAND_STRING_ERROR, "Empty command string");
    return cmd; // Return an empty command struct on error.
  }

  cmd.type = commandString[0]; // The first character is the command type.
  // If there are characters beyond the command type and the comma separator,
  // set args to point to the beginning of the arguments string.
  if (strlen(commandString) > COMMAND_DATA_OFFSET) {
    cmd.args = commandString + COMMAND_DATA_OFFSET;
  }
  return cmd; // Return the parsed command.
}

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate.
  Serial.setTimeout(10); // Set serial read timeout to 10 milliseconds.
  
  // Configure the AccelStepper library with initial motor parameters.
  stepper.setMaxSpeed(maxSpeed);       // Set the maximum speed for the stepper motor.
  stepper.setAcceleration(maxAccel);   // Set the acceleration rate for the stepper motor.
  stepper.setCurrentPosition(0);       // Initialize the current position of the stepper to 0.
  
  // Configure the enable pin as an output and enable the driver.
  pinMode(ENABLE_PIN, OUTPUT); // Set the ENABLE_PIN as an output.
  enableDriver();              // Call helper function to enable the stepper driver.

  // Configure limit switch pins as inputs with pull-up resistors
  pinMode(LIMIT_SWITCH_MIN_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_MAX_PIN, INPUT_PULLUP);

  currentMotorState = IDLE; // Initialize motor state to IDLE

  loadSettings(); // Load settings from EEPROM on startup

  // Print header for feedback data and a ready message to the serial monitor.
  Serial.println("FB_HEADER:currentAngle,targetAngle,speed,isRunning,timestamp");
  Serial.println("READY");
}

void loop() {
  // State machine for motor control
  switch (currentMotorState) {
    case IDLE:
      // Motor is idle, waiting for commands
      stepper.run(); // Keep AccelStepper running to handle potential position updates even when idle
      break;
    case MOVING:
      stepper.run(); // Continue moving
      if (!stepper.isRunning()) {
        currentMotorState = IDLE; // Transition to IDLE when movement completes
      }
      break;
    case STOPPED:
      // Motor is stopped, possibly due to emergency stop or error
      // No action needed here, waiting for a new command to clear the stop
      break;
    case HOMING:
      // Homing sequence (to be implemented)
      stepper.run();
      // For now, assume homing completes quickly and transitions to IDLE
      // In a real scenario, this would involve limit switches and more complex logic
      if (!stepper.isRunning()) {
        currentMotorState = IDLE;
      }
      break;
    case ERROR:
      // Handle error state (e.g., log error, wait for reset command)
      disableDriver(); // Ensure driver is disabled on error
      break;
  }

  // stepper.run(); // This function must be called repeatedly to make the stepper motor move.
                 // It handles acceleration, deceleration, and step generation.

  // Handle any incoming serial commands.
  handleSerialInput();
  
  // Send periodic feedback data to the host system.
  sendFeedback();
}

// Reads incoming serial data and processes complete command strings.
void handleSerialInput() {
  while (Serial.available()) { // Check if there is any data available in the serial buffer.
    char c = Serial.read();    // Read a character from the serial buffer.
    
    // If a newline or carriage return character is received, it signifies the end of a command.
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) { // If there's data in the buffer, process it.
        inputBuffer[bufferIndex] = '\0'; // Null-terminate the received command string.
        processCommand(inputBuffer);      // Call processCommand to handle the received command.
        bufferIndex = 0;                  // Reset buffer index for the next command.
      }
    } else {
      // Store the character in the input buffer if there's space.
      if (bufferIndex < sizeof(inputBuffer) - 1) {
        inputBuffer[bufferIndex++] = c;
      }
    }
  }
}

// Dispatches parsed commands to their respective handler functions based on command type.
void processCommand(char* commandString) {
  Command parsedCommand = parseCommand(commandString); // Parse the raw command string.
  char commandChar = parsedCommand.type;               // Get the command type character.

  // Use a switch statement to call the appropriate handler function for each command type.
  switch (commandChar) {
    case 'M': {  // Move command: M,angle (e.g., M,90.5)
        handleMoveCommand(parsedCommand);
        break;
    }
    
    case 'D': {  // Diagnostic move: D,steps (e.g., D,1000) - moves a specific number of steps.
        handleDiagnosticMoveCommand(parsedCommand);
        break;
    }
    
    case 'S': {  // Settings command: S,maxSpeed,maxAccel,pidP,pidI,pidD (e.g., S,20000,10000,1.0,0.0,0.1)
        handleSettingsCommand(parsedCommand);
        break;
    }
    
    case 'R': {  // Reset position: R - sets the current position to 0.
        handleResetCommand(parsedCommand);
        break;
    }
    
    case 'Q': {  // Query status: Q - requests immediate feedback.
        handleQueryCommand(parsedCommand);
        break;
    }
    
    case 'E': {  // Emergency stop: E - immediately stops the motor and disables the driver.
        handleEmergencyStopCommand(parsedCommand);
        break;
    }
    
    case 'H': {  // Home command: H - moves the motor to the 0 position.
        handleHomeCommand(parsedCommand);
        break;
    }

    case 'X': {  // Driver control: X,1 (enable) or X,0 (disable) - controls the stepper driver enable pin.
        handleDriverCommand(parsedCommand);
        break;
    }
    default: // Handle unknown command types.
      reportError(UNKNOWN_COMMAND_TYPE_ERROR, "Unknown command type");
      break;
  }
}

// Moves the stepper motor to a specified absolute angle.
// The angle is converted to steps, and the stepper is commanded to move to that position.
void moveToAngle(float angle) {
  // Convert the desired angle (in degrees) into corresponding microsteps.
  // 360 degrees corresponds to TOTAL_STEPS_PER_REV microsteps.
  long steps = (long)(angle * TOTAL_STEPS_PER_REV / 360.0);
  
  // Set the target position for the stepper motor.
  // The AccelStepper library will then manage the movement to this target.
  targetPosition = steps;
  stepper.moveTo(targetPosition);
}

// Handles the 'M' (Move) command.
// Expects an angle as an argument (e.g., "M,90.0").
void handleMoveCommand(Command cmd) {
  // Validate if arguments are provided.
  if (cmd.args == nullptr || strlen(cmd.args) == 0) {
    reportError(MOVE_COMMAND_MISSING_ARG_ERROR, "Move command missing angle argument");
    return;
  }
  float angle = atof(cmd.args); // Convert the argument string to a float (angle).
  // Additional validation for angle range could be added here if needed.
  moveToAngle(angle); // Command the motor to move to the specified angle.
  currentMotorState = MOVING; // Set state to MOVING when a move command is issued
}

// Handles the 'D' (Diagnostic Move) command.
// Expects a number of steps as an argument (e.g., "D,1000").
// This command moves the motor by a relative number of steps.
void handleDiagnosticMoveCommand(Command cmd) {
  // Validate if arguments are provided.
  if (cmd.args == nullptr || strlen(cmd.args) == 0) {
    reportError(DIAGNOSTIC_MOVE_MISSING_ARG_ERROR, "Diagnostic move command missing steps argument");
    return;
  }
  long diagnosticSteps = atol(cmd.args); // Convert the argument string to a long (steps).
  stepper.move(diagnosticSteps);         // Command the motor to move by the specified relative steps.
  Serial.print("Diagnostic: Moving ");  // Print diagnostic information to serial.
  Serial.print(diagnosticSteps);
  Serial.println(" steps");
}

// Handles the 'S' (Settings) command.
// Expects a comma-separated list of settings: maxSpeed,maxAccel,pidP,pidI,pidD
// (e.g., "S,20000,10000,1.0,0.0,0.1").
void handleSettingsCommand(Command cmd) {
  // Validate if arguments are provided.
  if (cmd.args == nullptr || strlen(cmd.args) == 0) {
    reportError(SETTINGS_COMMAND_MISSING_ARG_ERROR, "Settings command missing arguments");
    return;
  }
  char* argsCopy = strdup(cmd.args); // Create a mutable copy of the arguments string for strtok.
  char* token = strtok(argsCopy, ","); // Tokenize the string by commas.

  // Parse each setting from the tokenized string.
  if (token) maxSpeed = atof(token);
  token = strtok(NULL, ",");
  if (token) maxAccel = atof(token);
  token = strtok(NULL, ",");
  if (token) pidP = atof(token);
  token = strtok(NULL, ",");
  if (token) pidI = atof(token);
  token = strtok(NULL, ",");
  if (token) pidD = atof(token);
  
  // Apply the new speed and acceleration settings to the stepper motor.
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxAccel);
  
  // Print the updated settings to the serial monitor for confirmation.
  Serial.print("SETTINGS:");
  Serial.print(maxSpeed);
  Serial.print(",");
  Serial.print(maxAccel);
  Serial.print(",");
  Serial.print(pidP);
  Serial.print(",");
  Serial.print(pidI);
  Serial.print(",");
  Serial.println(pidD);
  free(argsCopy); // Free the dynamically allocated memory for the string copy.
  saveSettings(); // Save settings to EEPROM after modification
 }
 
 // Handles the 'R' (Reset) command.
// Resets the stepper motor's current position to 0 and clears the target position.
void handleResetCommand(Command cmd) {
  stepper.setCurrentPosition(0); // Set the stepper's current position to 0.
  targetPosition = 0;            // Reset the target position.
  Serial.println("RESET:OK");    // Confirm reset.
  currentMotorState = IDLE; // Set state to IDLE on reset
}

// Handles the 'Q' (Query) command.
// Triggers an immediate feedback message to be sent.
void handleQueryCommand(Command cmd) {
  sendImmediateFeedback(); // Send current status feedback.
}

// Handles the 'E' (Emergency Stop) command.
// Immediately stops the motor, sets its current position, and disables the driver.
void handleEmergencyStopCommand(Command cmd) {
  stepper.stop(); // Stop any ongoing motor movement.
  // Set current position to its current value to clear any pending moves.
  stepper.setCurrentPosition(stepper.currentPosition());
  disableDriver();  // Disable the motor driver to cut power to the motor.
  Serial.println("STOP:OK"); // Confirm emergency stop.
  enableDriver(); // Re-enable the driver after the stop.
  currentMotorState = STOPPED; // Set state to STOPPED on emergency stop
}

// Handles the 'H' (Home) command.
// Commands the motor to move to the absolute 0 angle.
void handleHomeCommand(Command cmd) {
  moveToAngle(0); // Move to the home position (0 degrees).
  currentMotorState = HOMING; // Set state to HOMING when a home command is issued
}

// Handles the 'X' (Driver Control) command.
// Expects an argument: '1' to enable the driver, '0' to disable it.
// (e.g., "X,1" or "X,0").
void handleDriverCommand(Command cmd) {
  // Validate if arguments are provided.
  if (cmd.args == nullptr || strlen(cmd.args) == 0) {
    reportError(DRIVER_COMMAND_MISSING_ARG_ERROR, "Driver command missing argument");
    return;
  }
  int enable = atoi(cmd.args); // Convert the argument string to an integer.

  // Control the driver based on the argument.
  if (enable == 1) {
    digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? HIGH : LOW); // Enable driver.
    Serial.println("Driver Enabled");
  } else if (enable == 0) {
    digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? LOW : HIGH); // Disable driver.
    Serial.println("Driver Disabled");
  } else {
    reportError(INVALID_DRIVER_COMMAND_ARG_ERROR, "Invalid driver command argument. Use 0 to disable, 1 to enable.");
  }
}

// Sends periodic feedback data to the serial monitor.
// This function uses millis() for non-blocking timing.
void sendFeedback() {
  unsigned long currentTime = millis(); // Get the current time in milliseconds.
  
  // Check if enough time has passed since the last feedback was sent.
  if (currentTime - lastFeedbackTime >= FEEDBACK_INTERVAL) {
    sendImmediateFeedback(); // Send feedback.
    lastFeedbackTime = currentTime; // Update the timestamp of the last feedback.
  }
}

// Sends an immediate feedback message to the serial monitor.
// This includes current angle, target angle, speed, movement status, and timestamp.
void sendImmediateFeedback() {
  long currentPos = stepper.currentPosition(); // Get the current position in steps.
  // Convert current position from steps to degrees.
  float currentAngle = (float)currentPos * 360.0 / TOTAL_STEPS_PER_REV;
  // Convert target position from steps to degrees.
  float targetAngle = (float)targetPosition * 360.0 / TOTAL_STEPS_PER_REV;
  
  // Print formatted feedback data to the serial monitor.
  Serial.print("FB:");
  Serial.print(currentAngle, 2); // Current angle, 2 decimal places.
  Serial.print(",");
  Serial.print(targetAngle, 2); // Target angle, 2 decimal places.
  Serial.print(",");
  Serial.print(stepper.speed()); // Current speed.
  Serial.print(",");
  Serial.print(stepper.isRunning() ? 1 : 0); // Is motor running (1) or stopped (0).
  Serial.print(",");
  Serial.println(millis()); // Current timestamp.
}

// Utility function to convert an angle in degrees to microsteps.
float angleToSteps(float angle) {
  return angle * TOTAL_STEPS_PER_REV / 360.0;
}

// Utility function to convert microsteps to an angle in degrees.
float stepsToAngle(long steps) {
  return (float)steps * 360.0 / TOTAL_STEPS_PER_REV;
}

// Function to perform the homing sequence
void performHomingSequence() {
  Serial.println("HOMING: Starting homing sequence...");
  currentMotorState = HOMING;

  // Move towards the minimum limit switch until it's pressed
  stepper.setMaxSpeed(maxSpeed / 2); // Use a slower speed for homing
  stepper.setAcceleration(maxAccel / 2);
  stepper.moveTo(-TOTAL_STEPS_PER_REV * 2); // Move a large distance in the negative direction

  while (digitalRead(LIMIT_SWITCH_MIN_PIN) == HIGH) { // Assuming HIGH when not pressed
    stepper.run();
    if (!stepper.isRunning()) { // Should not happen if moving a large distance, but as a safeguard
      reportError(HOMING_FAILED_LIMIT_SWITCH_ERROR, "Homing failed to reach limit switch.");
      return;
    }
  }

  // Stop the motor once the limit switch is pressed
  stepper.stop();
  stepper.setCurrentPosition(stepper.currentPosition()); // Set current position to stop any pending moves

  // Move slightly off the limit switch
  stepper.moveTo(stepper.currentPosition() + (TOTAL_STEPS_PER_REV / 100)); // Move 1/100th of a revolution positively
  while (stepper.isRunning()) {
    stepper.run();
  }

  // Set the current position as the new zero
  stepper.setCurrentPosition(0);
  targetPosition = 0;
  Serial.println("HOMING: Sequence complete. Position set to 0.");
  currentMotorState = IDLE;
  stepper.setMaxSpeed(maxSpeed); // Restore original speed
  stepper.setAcceleration(maxAccel); // Restore original acceleration
}

// Enum for motor states
enum MotorState {
  IDLE,
  MOVING,
  STOPPED,
  HOMING,
  ERROR
};

// Enum for error types
enum ErrorType {
  NO_ERROR = 0,
  EMPTY_COMMAND_STRING_ERROR,
  MOVE_COMMAND_MISSING_ARG_ERROR,
  DIAGNOSTIC_MOVE_MISSING_ARG_ERROR,
  SETTINGS_COMMAND_MISSING_ARG_ERROR,
  DRIVER_COMMAND_MISSING_ARG_ERROR,
  INVALID_DRIVER_COMMAND_ARG_ERROR,
  UNKNOWN_COMMAND_TYPE_ERROR,
  HOMING_FAILED_LIMIT_SWITCH_ERROR
};

// Current state of the motor
MotorState currentMotorState = IDLE;
// Current error state
ErrorType currentError = NO_ERROR;

// Function to report errors, set the error state, and print to serial
void reportError(ErrorType error, const char* message = "") {
  currentError = error;
  currentMotorState = ERROR;
  Serial.print("ERROR:");
  Serial.print(error);
  Serial.print(" - ");
  Serial.println(message);
}

// Function to load settings from EEPROM
void loadSettings() {
  EEPROM.get(EEPROM_MAX_SPEED_ADDR, maxSpeed);
  EEPROM.get(EEPROM_MAX_ACCEL_ADDR, maxAccel);
  EEPROM.get(EEPROM_PID_P_ADDR, pidP);
  EEPROM.get(EEPROM_PID_I_ADDR, pidI);
  EEPROM.get(EEPROM_PID_D_ADDR, pidD);

  // Apply loaded settings to stepper motor
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxAccel);

  Serial.println("SETTINGS: Loaded from EEPROM");
  Serial.print("Max Speed: "); Serial.println(maxSpeed);
  Serial.print("Max Accel: "); Serial.println(maxAccel);
  Serial.print("PID P: "); Serial.println(pidP);
  Serial.print("PID I: "); Serial.println(pidI);
  Serial.print("PID D: "); Serial.println(pidD);
}

// Function to save settings to EEPROM
void saveSettings() {
  EEPROM.put(EEPROM_MAX_SPEED_ADDR, maxSpeed);
  EEPROM.put(EEPROM_MAX_ACCEL_ADDR, maxAccel);
  EEPROM.put(EEPROM_PID_P_ADDR, pidP);
  EEPROM.put(EEPROM_PID_I_ADDR, pidI);
  EEPROM.put(EEPROM_PID_D_ADDR, pidD);

  Serial.println("SETTINGS: Saved to EEPROM");
}

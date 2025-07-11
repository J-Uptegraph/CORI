/*
 * CORI Hardware Head Controller with Menu System
 * Arduino code for controlling 9G servo motors with menu selection
 * 
 * Hardware:
 * - Arduino Nano/Uno
 * - 2x 9G Servo Motors (Pan/Tilt)
 * - External 5V power supply for servos
 * 
 * Wiring:
 * - Pan Servo: Pin D11 (PWM) - Left/Right movement
 * - Tilt Servo: Pin D12 (PWM) - Up/Down movement (for nodding)
 * - Servo Power: External 5V, Arduino GND
 * - Serial: USB connection to ROS bridge
 */

#include <Servo.h>

// Servo objects
Servo panServo;
Servo tiltServo;

// Pin definitions
const int PAN_SERVO_PIN = 11;  // D11 - Left/Right movement
const int TILT_SERVO_PIN = 12; // D12 - Up/Down movement (for nodding)
const int LED_PIN = 13;

// Servo calibration
const int SERVO_MIN = 0;      // Minimum servo angle (degrees)
const int SERVO_MAX = 180;    // Maximum servo angle (degrees)
const int SERVO_CENTER = 90;  // Center position (degrees)

// CORI angle mappings
const float ANGLE_BLACK = 0.76;     // Far right
const float ANGLE_GREY = 0.62;      // Right
const float ANGLE_PURPLE = 0.45;    // Mid-right
const float ANGLE_BLUE = 0.23;      // Slight right
const float ANGLE_GREEN = 0.0;      // Center
const float ANGLE_YELLOW = -0.23;   // Slight left
const float ANGLE_ORANGE = -0.45;   // Mid-left
const float ANGLE_RED = -0.62;      // Left

// CORI simulation limits
const float CORI_MIN_ANGLE = -1.8;  // -103 degrees
const float CORI_MAX_ANGLE = 1.8;   // +103 degrees

// Current positions
int currentPanAngle = SERVO_CENTER;
int currentTiltAngle = SERVO_CENTER;

// Serial communication
String inputString = "";
boolean stringComplete = false;

// Status tracking
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 5000; // 5 seconds

// Mode control
enum Mode { MENU, GAZEBO, MANUAL };
Mode currentMode = MENU;
const int ANGLE_STEP = 30; // Degrees to move in manual mode

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Attach servos to pins
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  
  // Move to center position
  panServo.write(SERVO_CENTER);
  tiltServo.write(SERVO_CENTER);
  
  // Startup sequence
  blinkLED(3);
  
  // Show menu
  showMenu();
  
  // Reserve string buffer
  inputString.reserve(200);
}

void loop() {
  // Handle serial input
  if (stringComplete) {
    processInput(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Check for command timeout in Gazebo mode
  if (currentMode == GAZEBO && millis() - lastCommandTime > COMMAND_TIMEOUT) {
    digitalWrite(LED_PIN, (millis() / 1000) % 2);
  }
  
  delay(10);
}

void showMenu() {
  Serial.println("\n=== CORI Head Controller Menu ===");
  Serial.println("1. Gazebo Mode (ROS-compatible commands)");
  Serial.println("2. Manual Control Mode");
  Serial.println("Enter choice (1 or 2):");
}

void processInput(String command) {
  command.trim();
  command.toUpperCase();
  
  lastCommandTime = millis();
  digitalWrite(LED_PIN, HIGH);
  
  if (currentMode == MENU) {
    if (command == "1") {
      currentMode = GAZEBO;
      Serial.println("Entering Gazebo Mode");
      Serial.println("Commands: ANGLE:<radians>, COLOR:<color_name>, CENTER, STATUS, NOD, TEST");
      Serial.println("Special: RED color triggers automatic nodding");
    } else if (command == "2") {
      currentMode = MANUAL;
      Serial.println("Entering Manual Control Mode");
      Serial.println("Commands: FRONT, BACK, LEFT, RIGHT, UP, DOWN, NOD, X:<degrees>, Y:<degrees>, CENTER, STATUS, <color_name>");
    } else {
      Serial.println("ERROR: Please enter 1 or 2");
      showMenu();
    }
  } else if (currentMode == GAZEBO) {
    processGazeboCommand(command);
  } else if (currentMode == MANUAL) {
    processManualCommand(command);
  }
  
  digitalWrite(LED_PIN, LOW);
}

void processGazeboCommand(String command) {
  if (command.startsWith("ANGLE:")) {
    float angleRad = command.substring(6).toFloat();
    moveToAngle(angleRad);
    Serial.print("OK:ANGLE:");
    Serial.println(angleRad, 3);
  } else if (command.startsWith("COLOR:")) {
    String colorName = command.substring(6);
    colorName.toLowerCase();
    float angleRad = getColorAngle(colorName);
    moveToAngle(angleRad);
    
    if (colorName.equals("red")) {
      delay(500);
      performNodding();
    }
    
    Serial.print("OK:COLOR:");
    Serial.print(colorName);
    Serial.print(":");
    Serial.println(angleRad, 3);
  } else if (command == "CENTER") {
    moveToAngle(0.0);
    smoothTiltMove(currentTiltAngle, SERVO_CENTER);
    currentTiltAngle = SERVO_CENTER;
    Serial.println("OK:CENTER");
  } else if (command == "STATUS") {
    sendStatus();
  } else if (command == "TEST") {
    runTestSequence();
    Serial.println("OK:TEST");
  } else if (command == "NOD") {
    performNodding();
    Serial.println("OK:NOD");
  } else {
    Serial.print("ERROR:UNKNOWN_COMMAND:");
    Serial.println(command);
  }
}

void processManualCommand(String command) {
  command.toLowerCase(); // Convert to lowercase for color commands and NOD
  float angleRad = getColorAngle(command); // Check if command is a color
  if (angleRad != ANGLE_GREEN || command == "green") { // If it's a valid color (including green)
    moveToAngle(angleRad);
    if (command == "red") {
      delay(500);
      performNodding();
    }
    Serial.print("OK:COLOR:");
    Serial.print(command);
    Serial.print(":");
    Serial.println(angleRad, 3);
  } else if (command == "nod") { // Handle NOD command in Manual mode
    performNodding();
    Serial.println("OK:NOD");
  } else { // Handle non-color, non-NOD commands (convert back to uppercase for consistency)
    command.toUpperCase();
    if (command == "FRONT" || command == "FORWARD") {
      smoothTiltMove(currentTiltAngle, currentTiltAngle - ANGLE_STEP); // Decreases angle (down)
      currentTiltAngle = constrain(currentTiltAngle - ANGLE_STEP, SERVO_MIN, SERVO_MAX);
      Serial.println("OK:FRONT");
    } else if (command == "BACK" || command == "BACKWARD") {
      smoothTiltMove(currentTiltAngle, currentTiltAngle + ANGLE_STEP); // Increases angle (up)
      currentTiltAngle = constrain(currentTiltAngle + ANGLE_STEP, SERVO_MIN, SERVO_MAX);
      Serial.println("OK:BACK");
    } else if (command == "LEFT") {
      smoothMove(currentPanAngle, currentPanAngle + ANGLE_STEP);
      currentPanAngle = constrain(currentPanAngle + ANGLE_STEP, SERVO_MIN, SERVO_MAX);
      Serial.println("OK:LEFT");
    } else if (command == "RIGHT") {
      smoothMove(currentPanAngle, currentPanAngle - ANGLE_STEP);
      currentPanAngle = constrain(currentPanAngle - ANGLE_STEP, SERVO_MIN, SERVO_MAX);
      Serial.println("OK:RIGHT");
    } else if (command == "UP") {
      smoothTiltMove(currentTiltAngle, currentTiltAngle + ANGLE_STEP); // Increases angle (up)
      currentTiltAngle = constrain(currentTiltAngle + ANGLE_STEP, SERVO_MIN, SERVO_MAX);
      Serial.println("OK:UP");
    } else if (command == "DOWN") {
      smoothTiltMove(currentTiltAngle, currentTiltAngle - ANGLE_STEP); // Decreases angle (down)
      currentTiltAngle = constrain(currentTiltAngle - ANGLE_STEP, SERVO_MIN, SERVO_MAX);
      Serial.println("OK:DOWN");
    } else if (command.startsWith("X:")) {
      int angle = command.substring(2).toInt();
      smoothMove(currentPanAngle, constrain(angle, SERVO_MIN, SERVO_MAX));
      currentPanAngle = constrain(angle, SERVO_MIN, SERVO_MAX);
      Serial.print("OK:X:");
      Serial.println(angle);
    } else if (command.startsWith("Y:")) {
      int angle = command.substring(2).toInt();
      smoothTiltMove(currentTiltAngle, constrain(angle, SERVO_MIN, SERVO_MAX));
      currentTiltAngle = constrain(angle, SERVO_MIN, SERVO_MAX);
      Serial.print("OK:Y:");
      Serial.println(angle);
    } else if (command == "CENTER") {
      smoothMove(currentPanAngle, SERVO_CENTER);
      smoothTiltMove(currentTiltAngle, SERVO_CENTER);
      currentPanAngle = SERVO_CENTER;
      currentTiltAngle = SERVO_CENTER;
      Serial.println("OK:CENTER");
    } else if (command == "STATUS") {
      sendStatus();
    } else {
      Serial.print("ERROR:UNKNOWN_COMMAND:");
      Serial.println(command);
    }
  }
}

float getColorAngle(String color) {
  if (color == "black") return ANGLE_BLACK;
  if (color == "grey" || color == "gray") return ANGLE_GREY;
  if (color == "purple") return ANGLE_PURPLE;
  if (color == "blue") return ANGLE_BLUE;
  if (color == "green") return ANGLE_GREEN;
  if (color == "yellow") return ANGLE_YELLOW;
  if (color == "orange") return ANGLE_ORANGE;
  if (color == "red") return ANGLE_RED;
  if (color == "white") return ANGLE_GREEN;
  if (color == "light_gray") return ANGLE_GREY;
  if (color == "navy") return ANGLE_BLUE;
  if (color == "cyan") return ANGLE_BLUE;
  if (color == "pink") return ANGLE_RED;
  return ANGLE_GREEN; // Default for unknown colors
}

void performNodding() {
  Serial.println("NOD:Starting natural nodding sequence");
  int originalPanAngle = currentPanAngle;
  int originalTiltAngle = currentTiltAngle;
  
  // Step 1: Tilt head back (look up)
  Serial.println("NOD:Tilting head back");
  smoothTiltMoveNatural(currentTiltAngle, SERVO_CENTER - 20, 10); // Tilt back 20 degrees
  currentTiltAngle = SERVO_CENTER - 20;
  delay(300);
  
  // Step 2: Look off to the left while tilted back
  Serial.println("NOD:Looking left");
  smoothMove(currentPanAngle, SERVO_CENTER + 35); // Look left 35 degrees
  currentPanAngle = SERVO_CENTER + 35;
  delay(400); // Hold the left gaze
  
  // Step 3: Return pan to center while bringing head back down
  Serial.println("NOD:Returning to center focus");
  smoothMove(currentPanAngle, SERVO_CENTER); // Pan back to center
  currentPanAngle = SERVO_CENTER;
  delay(100);
  smoothTiltMoveNatural(currentTiltAngle, SERVO_CENTER, 8); // Bring head to neutral
  currentTiltAngle = SERVO_CENTER;
  delay(200);
  
  // Step 4: Two quick but small jerky movements (down then up)
  Serial.println("NOD:Quick jerky movements");
  // First jerk - quick down
  smoothTiltMoveNatural(currentTiltAngle, SERVO_CENTER + 12, 3); // Quick 12 degrees down
  currentTiltAngle = SERVO_CENTER + 12;
  delay(120);
  
  // Second jerk - quick up
  smoothTiltMoveNatural(currentTiltAngle, SERVO_CENTER - 8, 3); // Quick 8 degrees up
  currentTiltAngle = SERVO_CENTER - 8;
  delay(120);
  
  // Step 5: Return to home position
  Serial.println("NOD:Returning to home position");
  smoothTiltMoveNatural(currentTiltAngle, originalTiltAngle, 12);
  currentTiltAngle = originalTiltAngle;
  delay(100);
  smoothMove(currentPanAngle, originalPanAngle);
  currentPanAngle = originalPanAngle;
  
  Serial.println("NOD:Natural nodding sequence complete");
}

void moveToAngle(float angleRad) {
  angleRad = constrain(angleRad, CORI_MIN_ANGLE, CORI_MAX_ANGLE);
  int servoAngle = map(angleRad * 1000, CORI_MIN_ANGLE * 1000, CORI_MAX_ANGLE * 1000, 
                      SERVO_MIN, SERVO_MAX);
  
  smoothMove(currentPanAngle, servoAngle);
  currentPanAngle = servoAngle;
  
  Serial.print("DEBUG:MOVE:");
  Serial.print(angleRad, 3);
  Serial.print("rad:");
  Serial.print(servoAngle);
  Serial.println("deg");
}

void smoothMove(int fromAngle, int toAngle) {
  int steps = abs(toAngle - fromAngle);
  int stepDelay = max(5, 20 - steps);
  
  if (steps > 0) {
    int direction = (toAngle > fromAngle) ? 1 : -1;
    
    for (int i = 0; i <= steps; i++) {
      int currentAngle = fromAngle + (i * direction);
      panServo.write(currentAngle);
      delay(stepDelay);
    }
  }
}

void smoothTiltMove(int fromAngle, int toAngle) {
  int steps = abs(toAngle - fromAngle);
  int stepDelay = max(3, 15 - steps);
  
  if (steps > 0) {
    int direction = (toAngle > fromAngle) ? 1 : -1;
    
    for (int i = 0; i <= steps; i++) {
      int currentAngle = fromAngle + (i * direction);
      tiltServo.write(currentAngle);
      delay(stepDelay);
    }
  }
}

// New function for natural nodding movements
void smoothTiltMoveNatural(int fromAngle, int toAngle, int baseDelay) {
  int steps = abs(toAngle - fromAngle);
  
  if (steps > 0) {
    int direction = (toAngle > fromAngle) ? 1 : -1;
    
    for (int i = 0; i <= steps; i++) {
      int currentAngle = fromAngle + (i * direction);
      tiltServo.write(currentAngle);
      
      // Variable delay for more natural movement
      // Faster at the beginning and end, slower in the middle
      float progress = (float)i / steps;
      int dynamicDelay = baseDelay + (int)(sin(progress * PI) * (baseDelay * 0.3));
      delay(dynamicDelay);
    }
  }
}

void runTestSequence() {
  Serial.println("TEST:Starting movement test sequence...");
  
  String testColors[] = {"red", "orange", "yellow", "green", "blue", "purple", "grey", "black"};
  
  for (int i = 0; i < 8; i++) {
    Serial.print("TEST:Moving to ");
    Serial.println(testColors[i]);
    float angle = getColorAngle(testColors[i]);
    moveToAngle(angle);
    
    if (testColors[i] == "red") {
      delay(500);
      performNodding();
    }
    
    delay(500);
  }
  
  Serial.println("TEST:Returning to center");
  moveToAngle(0.0);
  
  Serial.println("TEST:Sequence complete");
}

void sendStatus() {
  Serial.println("STATUS:READY");
  Serial.print("STATUS:PAN_ANGLE:");
  Serial.println(currentPanAngle);
  Serial.print("STATUS:TILT_ANGLE:");
  Serial.println(currentTiltAngle);
  Serial.print("STATUS:MODE:");
  Serial.println(currentMode == GAZEBO ? "GAZEBO" : currentMode == MANUAL ? "MANUAL" : "MENU");
  Serial.print("STATUS:LAST_COMMAND:");
  Serial.print(millis() - lastCommandTime);
  Serial.println("ms_ago");
  Serial.print("STATUS:UPTIME:");
  Serial.print(millis() / 1000);
  Serial.println("s");
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') {
      inputString += inChar;
    }
  }
}

void debugPrint(String message) {
  Serial.print("DEBUG:");
  Serial.println(message);
}
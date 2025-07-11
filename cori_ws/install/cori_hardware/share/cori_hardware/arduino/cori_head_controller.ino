/*
 * CORI Hardware Head Controller
 * Arduino code for controlling 9G servo motors to mirror CORI's head movement
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

// Servo calibration (adjust these values for your specific servos)
const int SERVO_MIN = 0;      // Minimum servo angle (degrees)
const int SERVO_MAX = 180;    // Maximum servo angle (degrees)
const int SERVO_CENTER = 90;  // Center position (degrees)

// CORI angle mappings (from laundry_color_detector.py)
// These match the exact values from the simulation
const float ANGLE_BLACK = 0.76;     // Far right
const float ANGLE_GREY = 0.62;      // Right
const float ANGLE_PURPLE = 0.45;    // Mid-right
const float ANGLE_BLUE = 0.23;      // Slight right
const float ANGLE_GREEN = 0.0;      // Center
const float ANGLE_YELLOW = -0.23;   // Slight left
const float ANGLE_ORANGE = -0.45;   // Mid-left
const float ANGLE_RED = -0.62;      // Left

// CORI simulation limits (from URDF)
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
  
  // Startup sequence - indicate system is ready
  blinkLED(3);
  
  // Send ready signal
  Serial.println("CORI_HARDWARE_READY");
  Serial.println("Commands: ANGLE:<radians>, COLOR:<color_name>, CENTER, STATUS, NOD, TEST");
  Serial.println("Special: RED color triggers automatic nodding with D12 tilt servo");
  
  // Reserve string buffer
  inputString.reserve(200);
}

void loop() {
  // Check for serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Check for command timeout
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    // Flash LED slowly when no commands received
    digitalWrite(LED_PIN, (millis() / 1000) % 2);
  }
  
  // Small delay to prevent overwhelming
  delay(10);
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  lastCommandTime = millis();
  digitalWrite(LED_PIN, HIGH);
  
  if (command.startsWith("ANGLE:")) {
    // Direct angle command: ANGLE:<radians>
    float angleRad = command.substring(6).toFloat();
    moveToAngle(angleRad);
    Serial.println("OK:ANGLE:" + String(angleRad, 3));
    
  } else if (command.startsWith("COLOR:")) {
    // Color command: COLOR:<color_name>
    String colorName = command.substring(6);
    colorName.toLowerCase();
    float angleRad = getColorAngle(colorName);
    moveToAngle(angleRad);
    
    // Special behavior for RED - perform nodding
    if (colorName == "red") {
      delay(500);  // Wait for pan movement to complete
      performNodding();
    }
    
    Serial.println("OK:COLOR:" + colorName + ":" + String(angleRad, 3));
    
  } else if (command == "CENTER") {
    // Center command
    moveToAngle(0.0);
    Serial.println("OK:CENTER");
    
  } else if (command == "STATUS") {
    // Status request
    sendStatus();
    
  } else if (command == "TEST") {
    // Test sequence
    runTestSequence();
    Serial.println("OK:TEST");
    
  } else if (command == "NOD") {
    // Manual nodding command
    performNodding();
    Serial.println("OK:NOD");
    
  } else {
    // Unknown command
    Serial.println("ERROR:UNKNOWN_COMMAND:" + command);
  }
  
  digitalWrite(LED_PIN, LOW);
}

float getColorAngle(String color) {
  // Convert color name to angle (matching CORI simulation)
  if (color == "black") return ANGLE_BLACK;
  if (color == "grey" || color == "gray") return ANGLE_GREY;
  if (color == "purple") return ANGLE_PURPLE;
  if (color == "blue") return ANGLE_BLUE;
  if (color == "green") return ANGLE_GREEN;
  if (color == "yellow") return ANGLE_YELLOW;
  if (color == "orange") return ANGLE_ORANGE;
  if (color == "red") return ANGLE_RED;
  
  // Additional colors (from laundry_color_detector.py)
  if (color == "white") return ANGLE_GREEN;        // Center
  if (color == "light_gray") return ANGLE_GREY;    // Same as gray
  if (color == "navy") return ANGLE_BLUE;          // Same as blue
  if (color == "cyan") return ANGLE_BLUE;          // Same as blue
  if (color == "pink") return ANGLE_RED;           // Same as red
  
  // Default to center for unknown colors
  return ANGLE_GREEN;
}

void performNodding() {
  // Nodding sequence for red color detection
  Serial.println("NOD:Starting nodding sequence for RED");
  
  // Save current tilt position
  int originalTiltAngle = currentTiltAngle;
  
  // Perform nodding motion (3 nods)
  for (int i = 0; i < 3; i++) {
    // Nod down
    smoothTiltMove(currentTiltAngle, SERVO_CENTER + 30);  // 30 degrees down
    currentTiltAngle = SERVO_CENTER + 30;
    delay(300);
    
    // Nod up
    smoothTiltMove(currentTiltAngle, SERVO_CENTER - 20);  // 20 degrees up
    currentTiltAngle = SERVO_CENTER - 20;
    delay(300);
    
    // Return to center
    smoothTiltMove(currentTiltAngle, SERVO_CENTER);
    currentTiltAngle = SERVO_CENTER;
    delay(200);
  }
  
  // Return to original tilt position
  smoothTiltMove(currentTiltAngle, originalTiltAngle);
  currentTiltAngle = originalTiltAngle;
  
  Serial.println("NOD:Nodding sequence complete");
}

void moveToAngle(float angleRad) {
  // Constrain angle to CORI's limits
  angleRad = constrain(angleRad, CORI_MIN_ANGLE, CORI_MAX_ANGLE);
  
  // Convert radians to servo degrees
  // Map CORI range (-1.8 to 1.8 rad) to servo range (0 to 180 degrees)
  int servoAngle = map(angleRad * 1000, CORI_MIN_ANGLE * 1000, CORI_MAX_ANGLE * 1000, 
                      SERVO_MIN, SERVO_MAX);
  
  // Smooth movement to target angle
  smoothMove(currentPanAngle, servoAngle);
  currentPanAngle = servoAngle;
  
  // Debug output
  Serial.println("DEBUG:MOVE:" + String(angleRad, 3) + "rad:" + String(servoAngle) + "deg");
}

void smoothMove(int fromAngle, int toAngle) {
  // Smooth pan servo movement to reduce jitter
  int steps = abs(toAngle - fromAngle);
  int stepDelay = max(5, 20 - steps); // Faster for small moves
  
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
  // Smooth tilt servo movement for nodding
  int steps = abs(toAngle - fromAngle);
  int stepDelay = max(3, 15 - steps); // Faster movement for nodding
  
  if (steps > 0) {
    int direction = (toAngle > fromAngle) ? 1 : -1;
    
    for (int i = 0; i <= steps; i++) {
      int currentAngle = fromAngle + (i * direction);
      tiltServo.write(currentAngle);
      delay(stepDelay);
    }
  }
}

void runTestSequence() {
  // Test sequence to verify servo operation
  Serial.println("TEST:Starting movement test sequence...");
  
  // Test each color position
  String testColors[] = {"red", "orange", "yellow", "green", "blue", "purple", "grey", "black"};
  
  for (int i = 0; i < 8; i++) {
    Serial.println("TEST:Moving to " + testColors[i]);
    float angle = getColorAngle(testColors[i]);
    moveToAngle(angle);
    
    // Special test for red - demonstrate nodding
    if (testColors[i] == "red") {
      delay(500);  // Wait for pan movement
      performNodding();
    }
    
    delay(500);
  }
  
  // Return to center
  Serial.println("TEST:Returning to center");
  moveToAngle(0.0);
  
  Serial.println("TEST:Sequence complete");
}

void sendStatus() {
  // Send current status
  Serial.println("STATUS:READY");
  Serial.println("STATUS:PAN_ANGLE:" + String(currentPanAngle));
  Serial.println("STATUS:TILT_ANGLE:" + String(currentTiltAngle));
  Serial.println("STATUS:LAST_COMMAND:" + String(millis() - lastCommandTime) + "ms_ago");
  Serial.println("STATUS:UPTIME:" + String(millis() / 1000) + "s");
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// Serial event handler
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

// Helper function for debugging
void debugPrint(String message) {
  Serial.println("DEBUG:" + message);
}
  /*
   * CORI Hardware Head Controller with Menu System - ESP32 Version
   * ESP32 code for controlling 9G servo motors with menu selection and WiFi capabilities
   * 
   * Hardware:
   * - ESP32-WROOM-32 Development Board
   * - 2x 9G Servo Motors (Pan/Tilt)
   * - External 5V power supply for servos
   * 
   * Wiring for ESP32:
   * - Pan Servo: GPIO12 (PWM) - Left/Right movement
   * - Tilt Servo: GPIO14 (PWM) - Up/Down movement (for nodding)
   * - Status LED: GPIO2 (Built-in LED)
   * - Servo Power: External 5V, ESP32 GND
   * - Serial: USB connection or WiFi
   */

  #include <ESP32Servo.h>
  #include <WiFi.h>
  #include <WebServer.h>

  // Servo objects
  Servo panServo;
  Servo tiltServo;

  // Pin definitions for ESP32
  const int PAN_SERVO_PIN = 12;   // GPIO12 - Left/Right movement
  const int TILT_SERVO_PIN = 14;  // GPIO14 - Up/Down movement (for nodding)
  const int LED_PIN = 2;          // Built-in LED on GPIO2

  // Servo calibration (for 5V operation)
  const int SERVO_MIN = 0;      // Minimum servo angle (degrees)
  const int SERVO_MAX = 180;    // Maximum servo angle (degrees)
  const int SERVO_CENTER = 90;  // Center position (degrees)

  // PWM Configuration for ESP32 (for 5V servos)
  const int SERVO_FREQ = 50;    // 50Hz for servos
  const int SERVO_RESOLUTION = 16; // 16-bit resolution
  const int PULSE_MIN_5V = 1000;   // Minimum pulse width for 5V operation (microseconds)
  const int PULSE_MAX_5V = 2000;   // Maximum pulse width for 5V operation (microseconds)

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
  enum Mode { MENU, GAZEBO, MANUAL, WIFI };
  Mode currentMode = MENU;
  const int ANGLE_STEP = 30; // Degrees to move in manual mode

  // WiFi Configuration
  const char* ssid = "CORI_HEAD_CONTROLLER";
  const char* password = "cori123456";
  WebServer server(80);
  bool wifiEnabled = false;

  void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

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

    // ESP32 specific initialization
    Serial.println("ESP32 CORI Head Controller Initializing...");
    Serial.print("Chip Model: ");
    Serial.println(ESP.getChipModel());
    Serial.print("Chip Revision: ");
    Serial.println(ESP.getChipRevision());
    Serial.print("Flash Size: ");
    Serial.println(ESP.getFlashChipSize());

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

    // Handle WiFi server if enabled
    if (wifiEnabled) {
      server.handleClient();
    }

    // Check for command timeout in Gazebo mode
    if (currentMode == GAZEBO && millis() - lastCommandTime > COMMAND_TIMEOUT) {
      digitalWrite(LED_PIN, (millis() / 1000) % 2);
    }

    delay(10);
  }

  void showMenu() {
    Serial.println("\n=== ESP32 CORI Head Controller Menu ===");
    Serial.println("1. Gazebo Mode (ROS-compatible commands)");
    Serial.println("2. Manual Control Mode");
    Serial.println("3. WiFi Control Mode");
    Serial.println("Enter choice (1, 2, or 3):");
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
        Serial.println("Commands: ANGLE:<radians>, COLOR:<color_name>, <color_name>, CENTER, STATUS, NOD, TEST, MENU, MANUAL, WIFI");
        Serial.println("Colors: RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE, GREY, BLACK");
      } else if (command == "2") {
        currentMode = MANUAL;
        Serial.println("Entering Manual Control Mode");
        Serial.println("Commands: FRONT, BACK, LEFT, RIGHT, UP, DOWN, NOD, X:<degrees>, Y:<degrees>, CENTER, STATUS, <color_name>, MENU, GAZEBO, WIFI");
      } else if (command == "3") {
        currentMode = WIFI;
        initializeWiFi();
        Serial.println("ERROR: Please enter 1, 2, or 3");
        showMenu();
      }
    } else if (currentMode == GAZEBO) {
      processGazeboCommand(command);
    } else if (currentMode == MANUAL) {
      processManualCommand(command);
    } else if (currentMode == WIFI) {
      processWiFiCommand(command);
    }

    digitalWrite(LED_PIN, LOW);
  }

  void initializeWiFi() {
    Serial.println("Initializing WiFi Access Point...");

    // Create WiFi Access Point
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Setup web server routes
    server.on("/", handleRoot);
    server.on("/control", HTTP_POST, handleControl);
    server.on("/status", handleStatus);
    server.onNotFound(handleNotFound);

    server.begin();
    wifiEnabled = true;

    Serial.println("WiFi Mode Active");
    Serial.println("Connect to: " + String(ssid));
    Serial.println("Password: " + String(password));
    Serial.println("Open browser to: http://" + IP.toString());
    Serial.println("Commands: MENU (return to menu), STATUS");
  }

  void processWiFiCommand(String command) {
    if (command == "MENU") {
      WiFi.softAPdisconnect(true);
      server.stop();
      wifiEnabled = false;
      currentMode = MENU;
      Serial.println("WiFi disabled, returning to menu");
      showMenu();
    } else if (command == "STATUS") {
      sendStatus();
      Serial.print("WiFi clients connected: ");
      Serial.println(WiFi.softAPgetStationNum());
    } else {
      // Process as regular command in WiFi mode
      processGazeboCommand(command);
    }
  }

  // Web server handlers
  void handleRoot() {
    String html = "<!DOCTYPE html>"
      "<html>"
      "<head>"
      "<title>CORI Head Controller</title>"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
      "<style>"
      "body { font-family: Arial; text-align: center; margin: 20px; }"
      "button { padding: 15px 25px; margin: 10px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; }"
      ".color-btn { background-color: #4CAF50; color: white; }"
      ".control-btn { background-color: #008CBA; color: white; }"
      ".action-btn { background-color: #f44336; color: white; }"
      ".grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 300px; margin: 0 auto; }"
      "</style>"
      "</head>"
      "<body>"
      "<h1>CORI Head Controller</h1>"
      "<h2>Color Commands</h2>"
      "<div class=\"grid\">"
      "<button class=\"color-btn\" onclick=\"sendCommand('red')\">Red</button>"
      "<button class=\"color-btn\" onclick=\"sendCommand('orange')\">Orange</button>"
      "<button class=\"color-btn\" onclick=\"sendCommand('yellow')\">Yellow</button>"
      "<button class=\"color-btn\" onclick=\"sendCommand('green')\">Green</button>"
      "<button class=\"color-btn\" onclick=\"sendCommand('blue')\">Blue</button>"
      "<button class=\"color-btn\" onclick=\"sendCommand('purple')\">Purple</button>"
      "<button class=\"color-btn\" onclick=\"sendCommand('grey')\">Grey</button>"
      "<button class=\"color-btn\" onclick=\"sendCommand('black')\">Black</button>"
      "<button class=\"color-btn\" onclick=\"sendCommand('white')\">White</button>"
      "</div>"
      "<h2>Manual Control</h2>"
      "<div class=\"grid\">"
      "<button class=\"control-btn\" onclick=\"sendCommand('left')\">Left</button>"
      "<button class=\"control-btn\" onclick=\"sendCommand('up')\">Up</button>"
      "<button class=\"control-btn\" onclick=\"sendCommand('right')\">Right</button>"
      "<button class=\"control-btn\" onclick=\"sendCommand('back')\">Back</button>"
      "<button class=\"control-btn\" onclick=\"sendCommand('center')\">Center</button>"
      "<button class=\"control-btn\" onclick=\"sendCommand('down')\">Down</button>"
      "</div>"
      "<h2>Actions</h2>"
      "<button class=\"action-btn\" onclick=\"sendCommand('nod')\">Nod</button>"
      "<button class=\"action-btn\" onclick=\"sendCommand('test')\">Test</button>"
      "<button class=\"action-btn\" onclick=\"getStatus()\">Status</button>"
      "<div id=\"response\" style=\"margin-top: 20px; padding: 10px; background-color: #f0f0f0;\"></div>"
      "<script>"
      "function sendCommand(cmd) {"
      "fetch('/control', {"
      "method: 'POST',"
      "headers: {'Content-Type': 'application/x-www-form-urlencoded'},"
      "body: 'command=' + cmd"
      "})"
      ".then(response => response.text())"
      ".then(data => {"
      "document.getElementById('response').innerHTML = '<strong>Response:</strong> ' + data;"
      "});"
      "}"
      "function getStatus() {"
      "fetch('/status')"
      ".then(response => response.text())"
      ".then(data => {"
      "document.getElementById('response').innerHTML = '<strong>Status:</strong><pre>' + data + '</pre>';"
      "});"
      "}"
      "</script>"
      "</body>"
      "</html>";
    server.send(200, "text/html", html);
  }

  void handleControl() {
    if (server.hasArg("command")) {
      String command = server.arg("command");
      command.toUpperCase();

      // Process command through existing handlers
      if (isColorCommand(command)) {
        command.toLowerCase();
        float angleRad = getColorAngle(command);
        moveToAngle(angleRad);
        server.send(200, "text/plain", "OK:COLOR:" + command);
      } else {
        processManualCommand(command);
        server.send(200, "text/plain", "OK:" + command);
      }
    } else {
      server.send(400, "text/plain", "No command provided");
    }
  }

  void handleStatus() {
    String status = "ESP32 CORI Head Controller Status:\n";
    status += "Mode: WIFI\n";
    status += "Pan Angle: " + String(currentPanAngle) + "°\n";
    status += "Tilt Angle: " + String(currentTiltAngle) + "°\n";
    status += "Uptime: " + String(millis() / 1000) + "s\n";
    status += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
    status += "WiFi Clients: " + String(WiFi.softAPgetStationNum()) + "\n";
    server.send(200, "text/plain", status);
  }

  void handleNotFound() {
    server.send(404, "text/plain", "Not found");
  }

  bool isColorCommand(String command) {
    command.toLowerCase();
    return (command == "red" || command == "orange" || command == "yellow" ||
            command == "green" || command == "blue" || command == "purple" ||
            command == "grey" || command == "gray" || command == "black" || command == "white");
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
    } else if (command == "MENU") {
      currentMode = MENU;
      Serial.println("Returning to main menu");
      showMenu();
    } else if (command == "MANUAL") {
      currentMode = MANUAL;
      Serial.println("Switching to Manual Control Mode");
      Serial.println("Commands: FRONT, BACK, LEFT, RIGHT, UP, DOWN, NOD, X:<degrees>, Y:<degrees>, CENTER, STATUS, <color_name>, MENU");
    } else if (command == "WIFI") {
      currentMode = WIFI;
      initializeWiFi();
    } else if (isColorCommand(command)) {
      command.toLowerCase();
      float angleRad = getColorAngle(command);
      moveToAngle(angleRad);
      Serial.print("OK:COLOR:");
      Serial.print(command);
      Serial.print(":");
      Serial.println(angleRad, 3);
    } else {
      Serial.print("ERROR:UNKNOWN_COMMAND:");
      Serial.println(command);
    }
  }

  void processManualCommand(String command) {
    command.toLowerCase();
    float angleRad = getColorAngle(command);
    if (angleRad != ANGLE_GREEN || command == "green") {
      moveToAngle(angleRad);
      Serial.print("OK:COLOR:");
      Serial.print(command);
      Serial.print(":");
      Serial.println(angleRad, 3);
    } else if (command == "nod") {
      performNodding();
      Serial.println("OK:NOD");
    } else {
      command.toUpperCase();
      if (command == "FRONT" || command == "FORWARD") {
        smoothTiltMove(currentTiltAngle, currentTiltAngle - ANGLE_STEP);
        currentTiltAngle = constrain(currentTiltAngle - ANGLE_STEP, SERVO_MIN, SERVO_MAX);
        Serial.println("OK:FRONT");
      } else if (command == "BACK" || command == "BACKWARD") {
        smoothTiltMove(currentTiltAngle, currentTiltAngle + ANGLE_STEP);
        currentTiltAngle = constrain(currentTiltAngle + ANGLE_STEP, SERVO_MIN, SERVO_MAX);
        Serial.println("OK:BACK");
      } else if (command == "LEFT") {
        smoothMove(currentPanAngle, currentPanAngle - ANGLE_STEP);
        currentPanAngle = constrain(currentPanAngle - ANGLE_STEP, SERVO_MIN, SERVO_MAX);
        Serial.println("OK:LEFT");
      } else if (command == "RIGHT") {
        smoothMove(currentPanAngle, currentPanAngle + ANGLE_STEP);
        currentPanAngle = constrain(currentPanAngle + ANGLE_STEP, SERVO_MIN, SERVO_MAX);
        Serial.println("OK:RIGHT");
      } else if (command == "UP") {
        smoothTiltMove(currentTiltAngle, currentTiltAngle - ANGLE_STEP);
        currentTiltAngle = constrain(currentTiltAngle - ANGLE_STEP, SERVO_MIN, SERVO_MAX);
        Serial.println("OK:UP");
      } else if (command == "DOWN") {
        smoothTiltMove(currentTiltAngle, currentTiltAngle + ANGLE_STEP);
        currentTiltAngle = constrain(currentTiltAngle + ANGLE_STEP, SERVO_MIN, SERVO_MAX);
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
      } else if (command == "MENU") {
        currentMode = MENU;
        Serial.println("Returning to main menu");
        showMenu();
      } else if (command == "GAZEBO") {
        currentMode = GAZEBO;
        Serial.println("Switching to Gazebo Mode");
        Serial.println("Commands: ANGLE:<radians>, COLOR:<color_name>, <color_name>, CENTER, STATUS, NOD, TEST, MENU, MANUAL, WIFI");
        Serial.println("Colors: RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE, GREY, BLACK");
      } else if (command == "WIFI") {
        currentMode = WIFI;
        initializeWiFi();
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
    return ANGLE_GREEN;
  }

  void performNodding() {
    Serial.println("NOD:Starting authentic nod sequence");
    int originalPanAngle = currentPanAngle;
    int originalTiltAngle = currentTiltAngle;

    // Step 1: Tilt head back slightly
    Serial.println("NOD:Tilting head back");
    smoothTiltMove(currentTiltAngle, SERVO_CENTER - 25);
    currentTiltAngle = SERVO_CENTER - 25;
    delay(400);

    // Step 2: Turn to look off to the right
    Serial.println("NOD:Looking off to the right");
    smoothMove(currentPanAngle, SERVO_CENTER + 40);
    currentPanAngle = SERVO_CENTER + 40;
    delay(500);

    // Step 3: Return pan to center and tilt to straight ahead
    Serial.println("NOD:Straightening to look straight ahead");
    smoothMove(currentPanAngle, SERVO_CENTER);
    currentPanAngle = SERVO_CENTER;
    delay(200);
    smoothTiltMove(currentTiltAngle, SERVO_CENTER);
    currentTiltAngle = SERVO_CENTER;
    delay(300);

    // Step 4: Two quick mini dips (authentic nod)
    Serial.println("NOD:First quick dip");
    smoothTiltMove(currentTiltAngle, SERVO_CENTER + 15);
    currentTiltAngle = SERVO_CENTER + 15;
    delay(150);
    smoothTiltMove(currentTiltAngle, SERVO_CENTER);
    currentTiltAngle = SERVO_CENTER;
    delay(100);

    Serial.println("NOD:Second quick dip");
    smoothTiltMove(currentTiltAngle, SERVO_CENTER + 15);
    currentTiltAngle = SERVO_CENTER + 15;
    delay(150);
    smoothTiltMove(currentTiltAngle, SERVO_CENTER);
    currentTiltAngle = SERVO_CENTER;
    delay(200);

    // Step 5: Return to original home position
    Serial.println("NOD:Returning to home position");
    smoothMove(currentPanAngle, originalPanAngle);
    currentPanAngle = originalPanAngle;
    delay(200);
    smoothTiltMove(currentTiltAngle, originalTiltAngle);
    currentTiltAngle = originalTiltAngle;
    delay(300);

    Serial.println("NOD:Authentic nod sequence complete");
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
    int stepDelay = max(8, 20 - steps); // Faster movement

    if (steps > 0) {
      int direction = (toAngle > fromAngle) ? 1 : -1;

      for (int i = 0; i <= steps; i++) {
        int currentAngle = fromAngle + (i * direction);
        panServo.write(currentAngle);
        delay(stepDelay);
      }
      delay(100); // Short settling time
    }
  }

  void smoothTiltMove(int fromAngle, int toAngle) {
    int steps = abs(toAngle - fromAngle);
    int stepDelay = max(10, 25 - steps); // Faster tilt movement

    if (steps > 0) {
      int direction = (toAngle > fromAngle) ? 1 : -1;

      for (int i = 0; i <= steps; i++) {
        int currentAngle = fromAngle + (i * direction);
        tiltServo.write(currentAngle);
        delay(stepDelay);
      }
      delay(150); // Short settling time for tilt
    }
  }

  void smoothTiltMoveNatural(int fromAngle, int toAngle, int baseDelay) {
    int steps = abs(toAngle - fromAngle);

    if (steps > 0) {
      int direction = (toAngle > fromAngle) ? 1 : -1;

      for (int i = 0; i <= steps; i++) {
        int currentAngle = fromAngle + (i * direction);
        tiltServo.write(currentAngle);

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

      delay(500);
    }

    Serial.println("TEST:Returning to center");
    moveToAngle(0.0);
    Serial.println("TEST:Sequence complete");
  }

  void sendStatus() {
    Serial.println("STATUS:READY");
    Serial.print("STATUS:BOARD:ESP32-WROOM-32");
    Serial.print("STATUS:PAN_ANGLE:");
    Serial.println(currentPanAngle);
    Serial.print("STATUS:TILT_ANGLE:");
    Serial.println(currentTiltAngle);
    Serial.print("STATUS:MODE:");
    if (currentMode == GAZEBO) Serial.println("GAZEBO");
    else if (currentMode == MANUAL) Serial.println("MANUAL");
    else if (currentMode == WIFI) Serial.println("WIFI");
    else Serial.println("MENU");
    Serial.print("STATUS:WIFI_ENABLED:");
    Serial.println(wifiEnabled ? "TRUE" : "FALSE");
    if (wifiEnabled) {
      Serial.print("STATUS:WIFI_CLIENTS:");
      Serial.println(WiFi.softAPgetStationNum());
    }
    Serial.print("STATUS:FREE_HEAP:");
    Serial.print(ESP.getFreeHeap());
    Serial.println("_bytes");
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

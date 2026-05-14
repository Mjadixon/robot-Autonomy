/*
 * Rover Motor Control Firmware (L298N)
 * 
 * Receives commands from Python via serial:
 *   F:PWM  → Forward (PWM: 0-255)
 *   S:0    → Stop
 *   L:PWM  → Steer Left (slow left motor)
 *   R:PWM  → Steer Right (slow right motor)
 * 
 * HARDWARE SETUP:
 * - Arduino Uno / Nano / Mega
 * - Motor Driver: L298N, TB6612FNG, or similar
 * - Left motor PWM:  Pin 9  (or adjust below)
 * - Right motor PWM: Pin 10 (or adjust below)
 * - Left motor DIR:  Pin 8
 * - Right motor DIR: Pin 11
 * - GND: Connect Arduino GND to motor driver GND
 * - USB: Connect to laptop/RPi for serial communication
 */

// ──────────────────────────────────────────────────────────────────────────────
// PIN CONFIGURATION (L298N Motor Driver)
// ──────────────────────────────────────────────────────────────────────────────

// PWM pins (speed control)
#define LEFT_PWM_PIN   10   // ENA pin for left motor speed (0-255)
#define RIGHT_PWM_PIN  9    // ENB pin for right motor speed (0-255)

// Direction pins
#define LEFT_DIR_PIN_1   7  // IN1 - left motor direction bit 1
#define LEFT_DIR_PIN_2   6  // IN2 - left motor direction bit 2
#define RIGHT_DIR_PIN_1  5  // IN3 - right motor direction bit 1
#define RIGHT_DIR_PIN_2  4  // IN4 - right motor direction bit 2

// ──────────────────────────────────────────────────────────────────────────────
// SETUP
// ──────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);  // Faster serial (was 9600)
  
  // Configure motor pins
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN_1, OUTPUT);
  pinMode(LEFT_DIR_PIN_2, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN_1, OUTPUT);
  pinMode(RIGHT_DIR_PIN_2, OUTPUT);
  
  // Start stopped
  stopMotors();
  delay(100);
  
  Serial.println("[MOTOR] Ready @ 115200");
}

// ──────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ──────────────────────────────────────────────────────────────────────────────

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() < 2) return;  // Need at least "F:0"
    
    char action = command.charAt(0);
    int pwm = 0;
    
    // Parse PWM value if present (format: "F:200")
    int colonIndex = command.indexOf(':');
    
    if (colonIndex > 0 && colonIndex < command.length() - 1) {
      pwm = command.substring(colonIndex + 1).toInt();
      pwm = constrain(pwm, 0, 255);  // Clamp to 0-255
    }
    
    // Execute command
    switch (action) {
      case 'F':
      case 'f':
        moveForward(pwm);
        break;
      case 'S':
      case 's':
        stopMotors();
        break;
      case 'L':
      case 'l':
        steerLeft(pwm);
        break;
      case 'R':
      case 'r':
        steerRight(pwm);
        break;
    }
    
    // Fast ACK (single byte instead of string)
    Serial.write(254);
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// MOTOR FUNCTIONS
// ──────────────────────────────────────────────────────────────────────────────

void moveForward(int pwm) {
  // Left motor forward: IN1 HIGH, IN2 LOW
  digitalWrite(LEFT_DIR_PIN_1, HIGH);
  digitalWrite(LEFT_DIR_PIN_2, LOW);
  
  // Right motor forward: IN3 HIGH, IN4 LOW
  digitalWrite(RIGHT_DIR_PIN_1, HIGH);
  digitalWrite(RIGHT_DIR_PIN_2, LOW);
  
  analogWrite(LEFT_PWM_PIN, pwm);
  analogWrite(RIGHT_PWM_PIN, pwm);
}

void stopMotors() {
  // Set both direction pins LOW (stops motor)
  digitalWrite(LEFT_DIR_PIN_1, LOW);
  digitalWrite(LEFT_DIR_PIN_2, LOW);
  digitalWrite(RIGHT_DIR_PIN_1, LOW);
  digitalWrite(RIGHT_DIR_PIN_2, LOW);
  
  // Set PWM to 0
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

void steerLeft(int pwm) {
  // Left motor slower
  digitalWrite(LEFT_DIR_PIN_1, HIGH);
  digitalWrite(LEFT_DIR_PIN_2, LOW);
  analogWrite(LEFT_PWM_PIN, pwm / 2);
  
  // Right motor full speed
  digitalWrite(RIGHT_DIR_PIN_1, HIGH);
  digitalWrite(RIGHT_DIR_PIN_2, LOW);
  analogWrite(RIGHT_PWM_PIN, pwm);
}

void steerRight(int pwm) {
  // Left motor full speed
  digitalWrite(LEFT_DIR_PIN_1, HIGH);
  digitalWrite(LEFT_DIR_PIN_2, LOW);
  analogWrite(LEFT_PWM_PIN, pwm);
  
  // Right motor slower
  digitalWrite(RIGHT_DIR_PIN_1, HIGH);
  digitalWrite(RIGHT_DIR_PIN_2, LOW);
  analogWrite(RIGHT_PWM_PIN, pwm / 2);
}

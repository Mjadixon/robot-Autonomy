/*
  Rover Arduino sketch
  - Reads MPU6050 (gyro/accel) via I2C
  - Reads two single-channel encoders on pins 2 and 3 (interrupts)
  - Sends telemetry lines: T,left_ticks,right_ticks,gyro_z,ax,ay,az,t_ms\n
  - Receives motor commands via serial: M,<left_pwm>,<right_pwm>\n
  - Implements watchdog: stop motors if no cmd received within 500ms
  - Implements simple ramping for PWM

  Configure pins and encoder wiring to match your robot.
*/

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Encoder pins (attach interrupts)
const uint8_t ENC_L_PIN = 2; // INT0
const uint8_t ENC_R_PIN = 3; // INT1
volatile long enc_l = 0;
volatile long enc_r = 0;

// Motor pins (L298N example) - adapt to your wiring
const uint8_t L_IN1 = 5; // PWM left
const uint8_t L_IN2 = 4; // dir
const uint8_t R_IN1 = 6; // PWM right
const uint8_t R_IN2 = 7; // dir

int target_l = 0;
int target_r = 0;
int current_l = 0;
int current_r = 0;

unsigned long last_cmd_time = 0;
const unsigned long WATCHDOG_MS = 500;
const int RAMP_STEP = 8; // PWM units per loop

void IRAM_ATTR onEncL() {
  enc_l++;
}
void IRAM_ATTR onEncR() {
  enc_r++;
}

void setMotorPWM(int left_pwm, int right_pwm) {
  // left
  if (left_pwm >= 0) {
    digitalWrite(L_IN2, LOW);
    analogWrite(L_IN1, constrain(left_pwm, 0, 255));
  } else {
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_IN1, constrain(-left_pwm, 0, 255));
  }
  // right
  if (right_pwm >= 0) {
    digitalWrite(R_IN2, LOW);
    analogWrite(R_IN1, constrain(right_pwm, 0, 255));
  } else {
    digitalWrite(R_IN2, HIGH);
    analogWrite(R_IN1, constrain(-right_pwm, 0, 255));
  }
}

void stopMotors() {
  current_l = 0; current_r = 0; target_l = 0; target_r = 0;
  analogWrite(L_IN1, 0); analogWrite(R_IN1, 0);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(ENC_L_PIN, INPUT_PULLUP);
  pinMode(ENC_R_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), onEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), onEncR, RISING);

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

  // init MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[MPU] connection failed");
  }
  delay(100);
  last_cmd_time = millis();
}

String input_buffer = "";

void processSerialLine(String &line) {
  // Commands: M,left_pwm,right_pwm
  if (line.length() == 0) return;
  if (line.charAt(0) == 'M') {
    // parse
    int parts[2];
    int idx = 0;
    char *cstr = (char *)line.c_str();
    char *tok = strtok(cstr, ",");
    tok = strtok(NULL, ",");
    while (tok != NULL && idx < 2) {
      parts[idx++] = atoi(tok);
      tok = strtok(NULL, ",");
    }
    if (idx == 2) {
      target_l = constrain(parts[0], -255, 255);
      target_r = constrain(parts[1], -255, 255);
      last_cmd_time = millis();
    }
  }
}

void loop() {
  // serial input
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input_buffer.length() > 0) {
        processSerialLine(input_buffer);
        input_buffer = "";
      }
    } else {
      input_buffer += c;
      if (input_buffer.length() > 200) input_buffer = "";
    }
  }

  // watchdog
  if (millis() - last_cmd_time > WATCHDOG_MS) {
    target_l = 0; target_r = 0;
  }

  // ramp current towards target
  if (current_l < target_l) current_l = min(current_l + RAMP_STEP, target_l);
  else if (current_l > target_l) current_l = max(current_l - RAMP_STEP, target_l);
  if (current_r < target_r) current_r = min(current_r + RAMP_STEP, target_r);
  else if (current_r > target_r) current_r = max(current_r - RAMP_STEP, target_r);

  setMotorPWM(current_l, current_r);

  // read sensors and send telemetry periodically
  static unsigned long last_send = 0;
  if (millis() - last_send >= 50) {
    last_send = millis();
    // read MPU6050 gyro and accel
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float gyro_z = gz / 131.0; // deg/s (MPU default)
    float afx = ax / 16384.0;
    float afy = ay / 16384.0;
    float afz = az / 16384.0;
    // send telemetry
    noInterrupts();
    long l = enc_l;
    long r = enc_r;
    interrupts();
    Serial.print("T,");
    Serial.print(l); Serial.print(",");
    Serial.print(r); Serial.print(",");
    Serial.print(gyro_z); Serial.print(",");
    Serial.print(afx); Serial.print(",");
    Serial.print(afy); Serial.print(",");
    Serial.print(afz); Serial.print(",");
    Serial.print(millis());
    Serial.print('\n');
  }
}

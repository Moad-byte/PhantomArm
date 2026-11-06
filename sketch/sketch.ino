
#include <Wire.h>
#include <math.h>

// PhantomArm Phase 2 - MPU6050 + 4 buttons
// Raspberry Pi Pico 2 W

const uint8_t MPU = 0x68;

// MPU6050 I2C connections
const int SDA_PIN = 4;
const int SCL_PIN = 5;

// Physical pushbuttons
const int Z_UP_PIN   = 16;
const int Z_DOWN_PIN = 17;
const int GRIP_PIN   = 18;
const int OPEN_PIN   = 19;

// Button debounce
const unsigned long DEBOUNCE_MS = 25;
const unsigned long SEND_MS = 50;

struct Button {
  uint8_t pin;
  bool stablePressed;
  bool lastRawPressed;
  unsigned long changedAt;
};

Button zUp   = {Z_UP_PIN,   false, false, 0};
Button zDown = {Z_DOWN_PIN, false, false, 0};
Button grip  = {GRIP_PIN,   false, false, 0};
Button openGrip = {OPEN_PIN, false, false, 0};

uint32_t sequenceNumber = 0;
unsigned long lastSend = 0;

void setupButton(Button &b) {
  pinMode(b.pin, INPUT_PULLUP);
  bool pressed = digitalRead(b.pin) == LOW;
  b.stablePressed = pressed;
  b.lastRawPressed = pressed;
  b.changedAt = millis();
}

void updateButton(Button &b) {
  bool rawPressed = digitalRead(b.pin) == LOW;
  unsigned long now = millis();

  if (rawPressed != b.lastRawPressed) {
    b.lastRawPressed = rawPressed;
    b.changedAt = now;
  }

  if (now - b.changedAt >= DEBOUNCE_MS) {
    b.stablePressed = rawPressed;
  }
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int16_t read16() {
  uint16_t highByte = Wire.read();
  uint16_t lowByte = Wire.read();

  return (int16_t)((highByte << 8) | lowByte);
}

bool readMPU(float &roll, float &pitch) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU, (uint8_t)6) != 6) {
    return false;
  }

  int16_t ax = read16();
  int16_t ay = read16();
  int16_t az = read16();

  float x = ax / 16384.0f;
  float y = ay / 16384.0f;
  float z = az / 16384.0f;

  roll = atan2(y, z) * 180.0f / PI;
  pitch = atan2(
    -x, sqrt(x * x + z * z)
  ) * 180.0f / PI;

  return true;
}

void setup() {
  Serial.begin(115200);

  setupButton(zUp);
  setupButton(zDown);
  setupButton(grip);
  setupButton(openGrip);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);

  delay(1000);

  if (!writeRegister(0x6B, 0x00) ||
      !writeRegister(0x1C, 0x00)) {
    Serial.println("MPU_INIT_ERROR");

    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  updateButton(zUp);
  updateButton(zDown);
  updateButton(grip);
  updateButton(openGrip);

  unsigned long now = millis();

  if (now - lastSend < SEND_MS) {
    delay(2);
    return;
  }

  lastSend = now;

  float roll = 0.0f;
  float pitch = 0.0f;

  bool valid = readMPU(roll, pitch);

  // Prevent conflicting button commands.
  bool up = zUp.stablePressed && !zDown.stablePressed;
  bool down = zDown.stablePressed && !zUp.stablePressed;

  bool closing = grip.stablePressed &&
                 !openGrip.stablePressed;
  bool opening = openGrip.stablePressed &&
                 !grip.stablePressed;

  // Extended PhantomArm telemetry:
  // PA2,time,seq,roll,pitch,up,down,grip,open,valid

  Serial.print("PA2,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(++sequenceNumber);
  Serial.print(",");
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(up ? 1 : 0);
  Serial.print(",");
  Serial.print(down ? 1 : 0);
  Serial.print(",");
  Serial.print(closing ? 1 : 0);
  Serial.print(",");
  Serial.print(opening ? 1 : 0);
  Serial.print(",");
  Serial.println(valid ? 1 : 0);
}

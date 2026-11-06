/*
 * PhantomArm v1.0 / Pico 2 W motion teleoperation transmitter
 * Stack: Arduino-Pico core + Adafruit MPU6050 + USB CDC Serial.
 * Wiring: MPU SDA GP4, SCL GP5, VCC 3V3, GND GND.
 * Button GP15 to GND (internal pull-up); optional green GP16 / red GP17
 * LEDs each need a series resistor (e.g. 220-330 ohms).
 * Emits PA1,millis,seq,roll_deg,pitch_deg,button,valid at 50 Hz.
 * For PC teleoperation only, NOT a certified motion-control device.
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

static constexpr uint8_t PIN_SDA=4;
static constexpr uint8_t PIN_SCL=5;
static constexpr uint8_t PIN_BUTTON=15;
static constexpr uint8_t PIN_LED_OK=16;
static constexpr uint8_t PIN_LED_FAULT=17;
static constexpr unsigned long PERIOD_MS=20;
static constexpr float EMA_MIX=0.98f; // gyro/accelerometer complementary fusion
static constexpr float RAD_TO_DEG_F=57.295779513f;

Adafruit_MPU6050 imu;
bool sensorPresent=false;
float roll=0.0f, pitch=0.0f;
bool firstReading=true;
unsigned long lastSampleMs=0, lastRetryMs=0;
uint32_t seq=0;

// Debounce and expose stable button state rather than raw contact bounce.
bool stableButton=false;
bool prevRawButton=false;
unsigned long lastButtonChange=0;

void refreshButton(unsigned long now){
  bool raw=(digitalRead(PIN_BUTTON)==LOW);
  if(raw!=prevRawButton)lastButtonChange=now;
  if(now-lastButtonChange>=25)stableButton=raw;
  prevRawButton=raw;
}

bool beginSensor(){
  if(!imu.begin(0x68,&Wire))return false;
  imu.setAccelerometerRange(MPU6050_RANGE_4_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  firstReading=true;
  return true;
}

void setup(){
  pinMode(PIN_BUTTON,INPUT_PULLUP);
  pinMode(PIN_LED_OK,OUTPUT);
  pinMode(PIN_LED_FAULT,OUTPUT);
  digitalWrite(PIN_LED_OK,LOW);
  digitalWrite(PIN_LED_FAULT,HIGH);
  Serial.begin(115200);
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  Wire.setClock(400000);
  sensorPresent=beginSensor();
  lastSampleMs=millis();
  lastRetryMs=lastSampleMs;
}

void loop(){
  const unsigned long now=millis();
  refreshButton(now);
  if(!sensorPresent && (unsigned long)(now-lastRetryMs)>1000){
    sensorPresent=beginSensor();
    lastRetryMs=now;
  }
  if((unsigned long)(now-lastSampleMs)<PERIOD_MS)return;
  const float dt=constrain((float)(now-lastSampleMs)/1000.0f,0.001f,0.08f);
  lastSampleMs=now;

  bool ok=false;
  if(sensorPresent){
    sensors_event_t a,g,temp;
    imu.getEvent(&a,&g,&temp);
    const float ax=a.acceleration.x,ay=a.acceleration.y,az=a.acceleration.z;
    const float gx=g.gyro.x,gy=g.gyro.y;
    const float amag=sqrtf(ax*ax+ay*ay+az*az);
    if(isfinite(ax)&&isfinite(ay)&&isfinite(az)&&isfinite(gx)&&isfinite(gy)
         &&amag>1.0f&&amag<35.0f){
      const float accRoll=atan2f(ay,az)*RAD_TO_DEG_F;
      const float accPitch=atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG_F;
      if(firstReading){roll=accRoll;pitch=accPitch;firstReading=false;}
      else{
        roll=EMA_MIX*(roll+gx*RAD_TO_DEG_F*dt)+(1.0f-EMA_MIX)*accRoll;
        pitch=EMA_MIX*(pitch+gy*RAD_TO_DEG_F*dt)+(1.0f-EMA_MIX)*accPitch;
      }
      ok=isfinite(roll)&&isfinite(pitch)&&fabsf(roll)<=180&&fabsf(pitch)<=180;
    }
  }
  digitalWrite(PIN_LED_OK,ok?HIGH:LOW);
  digitalWrite(PIN_LED_FAULT,ok?LOW:HIGH);
  // Invalid samples retain their explicit status; PC controller must HOLD.
  Serial.print(F("PA1,"));Serial.print(now);Serial.print(',');Serial.print(seq++);
  Serial.print(',');Serial.print(ok?roll:0.0f,3);
  Serial.print(',');Serial.print(ok?pitch:0.0f,3);
  Serial.print(',');Serial.print(stableButton?1:0);
  Serial.print(',');Serial.println(ok?1:0);
}

#include <Servo.h>
#include <math.h>

#define PIN_SERVO 10
#define PIN_TRIG  12
#define PIN_ECHO  13

const int ANG_CLOSED = 30;
const int ANG_OPENED = 90;
unsigned long MOVING_TIME = 3000;

const float APPROACH_TH = 22.0;
const float CLEAR_TH = 32.0;

Servo myServo;
bool moving = false;
bool openTarget = false;
unsigned long moveStartMs = 0;
int startAngle = ANG_CLOSED;
int stopAngle  = ANG_CLOSED;
int currentAngle = ANG_CLOSED;

float spring01(float t, float w = 7.0f) {
  if (t <= 0) return 0.0f;
  if (t >= 1) return 1.0f;
  float s  = 1.0f - (1.0f + w * t) * expf(-w * t);
  float s1 = 1.0f - (1.0f + w * 1.0f) * expf(-w * 1.0f);
  return s / s1;
}

float getDistance() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (dur == 0) return 999.0f;
  return dur * 0.0343f / 2.0f;
}

void setup() {
  myServo.attach(PIN_SERVO);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  myServo.write(ANG_CLOSED);
  currentAngle = ANG_CLOSED;
  delay(300);
}

void loop() {
  if (!moving) {
    float d = getDistance();
    if (!openTarget && d < APPROACH_TH) {
      openTarget = true;
      startAngle = currentAngle;
      stopAngle  = ANG_OPENED;
      moveStartMs = millis();
      moving = true;
    } else if (openTarget && d > CLEAR_TH) {
      openTarget = false;
      startAngle = currentAngle;
      stopAngle  = ANG_CLOSED;
      moveStartMs = millis();
      moving = true;
    }
  }

  if (moving) {
    unsigned long el = millis() - moveStartMs;
    if (el <= MOVING_TIME) {
      float t = (float)el / (float)MOVING_TIME;
      float e = spring01(t, 7.0f);
      int ang = (int)lround(startAngle + (stopAngle - startAngle) * e);
      myServo.write(ang);
      currentAngle = ang;
    } else {
      myServo.write(stopAngle);
      currentAngle = stopAngle;
      moving = false;
    }
  }

  delay(10);
}

#include <Servo.h>
#include <math.h>

#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

Servo myServo;

unsigned long MOVING_TIME = 3000;
unsigned long moveStartTime;

int startAngle = 30;
int stopAngle = 90;
int currentAngle = 30;

const float APPROACH_TH = 22.0;
const float CLEAR_TH = 32.0;

bool moving = false;
bool gateOpenTarget = false;

float sigmoid01(float t) {
  const float k = 10.0f;
  if(t <= 0) return 0;
  if(t >= 1) return 1;
  float s = 1.0f/(1.0f + expf(-k*(t-0.5f)));
  float s0 = 1.0f/(1.0f + expf(-k*(0.0f-0.5f)));
  float s1 = 1.0f/(1.0f + expf(-k*(1.0f-0.5f)));
  return (s - s0) / (s1 - s0);
}

float getDistance(){
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (dur == 0)return 999.0f;
  return dur * 0.0343f / 2.0f;
}

void setup() {
  myServo.attach(PIN_SERVO);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  
  moveStartTime = millis();

  myServo.write(startAngle);
  currentAngle = startAngle;
  delay(500);
}

void loop() {
  if (!moving){
    float d = getDistance();
    if(!gateOpenTarget && d < APPROACH_TH) {
      gateOpenTarget = true;
      startAngle = currentAngle;
      stopAngle = 90;
      moveStartTime = millis();
      moving = true;
    }else if (gateOpenTarget && d > CLEAR_TH) {
      gateOpenTarget = false;
      startAngle = currentAngle;
      stopAngle = 30;
      moveStartTime = millis();
      moving = true;
    }
  }

  if (moving) {
    unsigned long progress = millis() - moveStartTime;
    if (progress <= MOVING_TIME){
      float t = (float)progress / (float)MOVING_TIME;
      float e = sigmoid01(t);
      long angle = (long)lround(startAngle + (stopAngle - startAngle) * e);
      myServo.write((int)angle);
      currentAngle = (int)angle;
    }else {
      myServo.write(stopAngle);
      currentAngle = stopAngle;
      moving = false;
    }
  }
  delay(10);
}

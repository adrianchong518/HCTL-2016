#include <Arduino.h>

#include <PID.h>

const uint8_t pin_CLK = 10;
const uint8_t pin_SEL = 39;
const uint8_t pin_OE = 40;
const uint8_t pin_RST = 41;

const uint8_t pin_PWM = 5;
const uint8_t pin_inA = 7;
const uint8_t pin_inB = 8;

const double Kp = 2.9;
const double Ki = 0;
const double Kd = 6.8;
const long min = -255;
const long max = 255;

PID pid(Kp, Ki, Kd, min, max);

void setMotorSpeed(int pwmSpeed) {
  if (pwmSpeed > 0) {
    digitalWrite(pin_inA, HIGH);
    digitalWrite(pin_inB, LOW);
    analogWrite(pin_PWM, pwmSpeed);
  } else if (pwmSpeed < 0) {
    digitalWrite(pin_inA, LOW);
    digitalWrite(pin_inB, HIGH);
    analogWrite(pin_PWM, -pwmSpeed);
  } else {
    digitalWrite(pin_inA, LOW);
    digitalWrite(pin_inB, LOW);
    analogWrite(pin_PWM, 0);
  }
}

void setup() {
  Serial.begin(115200);

  // Set up HCTL-2016 interfacing pins
  pinMode(pin_SEL, OUTPUT);
  digitalWrite(pin_SEL, LOW);

  pinMode(pin_OE, OUTPUT);
  digitalWrite(pin_OE, HIGH);

  pinMode(pin_RST, OUTPUT);
  digitalWrite(pin_RST, HIGH);

  // Generate clock ouput
  pinMode(pin_CLK, OUTPUT);
  TCCR2A = bit(WGM21) | bit(COM2A0);
  TCCR2B = bit(CS20);
  OCR2A = 0;

  // Set up PORT A for input
  DDRA = 0x00;
  PORTA = 0x00;

  // Set up motor ctrl pins
  pinMode(pin_inA, OUTPUT);
  digitalWrite(pin_inA, LOW);
}

short readLocation() {
  short location;

  digitalWrite(pin_OE, LOW);

  digitalWrite(pin_SEL, LOW);
  location = PINA << 8;
  digitalWrite(pin_SEL, HIGH);
  location |= PINA;

  digitalWrite(pin_OE, HIGH);

  return location;
}

String input = "";
bool inputFinished = false;
void loop() {
  while (Serial.available()) {
    char in = Serial.read();

    if (in == '\r' || in == '\n') {
      inputFinished = true;
      break;
    }
    if (in != -1) {
      input.concat(in);
    }
  }

  if (inputFinished) {
    int target = input.toInt();
    pid.setTarget(target);
    Serial.println("Target (" + String(target) + ") Set");

    inputFinished = false;
    input = "";
  }

  short location = readLocation();
  int pwmControl = pid.calculatePID(location);
  setMotorSpeed(pwmControl);

  Serial.println(String(location) + " | " + String(pwmControl));
}
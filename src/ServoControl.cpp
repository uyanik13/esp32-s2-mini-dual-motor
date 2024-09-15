#include "ServoControl.h"
#include <Arduino.h>

#define SERVO_PIN 4

Servo servo;

void ServoControl::setup() {
    servo.attach(SERVO_PIN);
    servo.write(90); // Center position
}

void ServoControl::moveServo(int angle) {
    servo.write(angle);
    delay(200);
    Serial.print("Servo moved to: ");
    Serial.println(angle);
}

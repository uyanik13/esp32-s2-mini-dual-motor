#include "ServoControl.h"
#include <Arduino.h>

#define SERVO_PIN 6
#define CENTER_ANGLE 90
#define LEFT_ANGLE 0
#define RIGHT_ANGLE 180
#define MOVE_DELAY 15  // Short delay between angle steps for smooth movement
#define STEP_SIZE 1     // How much the servo moves per step
#define PWM_CHANNEL_SERVO 2
#define PWM_FREQUENCY 50  // Servo motors typically run at 50Hz
#define PWM_RESOLUTION 16 // 16-bit resolution
#define MIN_PULSE_WIDTH 544  // Minimum pulse width in microseconds for 0 degrees
#define MAX_PULSE_WIDTH 2400 // Maximum pulse width in microseconds for 180 degrees

ServoControl::ServoControl() {}

// Helper function to map angle to duty cycle
int ServoControl::mapAngleToDutyCycle(int angle) {
    int pulseWidth = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    int dutyCycle = (pulseWidth * (1 << PWM_RESOLUTION)) / (1000000 / PWM_FREQUENCY);
    return dutyCycle;
}

void ServoControl::setup() {
    // Setup PWM for the servo
    ledcSetup(PWM_CHANNEL_SERVO, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(SERVO_PIN, PWM_CHANNEL_SERVO);

    // Move servo to the center position initially
    moveServo(CENTER_ANGLE);
    Serial.println("Servo initialized at center position.");
}

void ServoControl::moveServo(int targetAngle) {
    if (targetAngle < LEFT_ANGLE || targetAngle > RIGHT_ANGLE) {
        Serial.println("Invalid angle. Must be between 0 and 180.");
        return;
    }

    Serial.print("Moving servo to: ");
    Serial.println(targetAngle);

    // Get current duty cycle based on the angle
    int dutyCycle = mapAngleToDutyCycle(targetAngle);
    ledcWrite(PWM_CHANNEL_SERVO, dutyCycle);

    Serial.print("Servo moved to: ");
    Serial.println(targetAngle);
}

void ServoControl::moveLeft() {
    Serial.println("Moving servo to the left...");
    moveServo(LEFT_ANGLE);
}

void ServoControl::moveRight() {
    Serial.println("Moving servo to the right...");
    moveServo(RIGHT_ANGLE);
}

void ServoControl::center() {
    Serial.println("Centering servo...");
    moveServo(CENTER_ANGLE);
}

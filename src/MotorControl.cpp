#include "MotorControl.h"
#include <Arduino.h>

// Motor control pins
#define FRONT_MOTOR_IN1 21
#define FRONT_MOTOR_IN2 17
#define BACK_MOTOR_IN1 33
#define BACK_MOTOR_IN2 18
#define FRONT_MOTOR_ENABLE 34
#define BACK_MOTOR_ENABLE 16
// Standby pin
#define MOTOR_STBY 15

// PWM configuration
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_BACK_MOTOR 3
#define PWM_CHANNEL_FRONT_MOTOR 4
#define TURN_REDUCTION_FACTOR 1  // Reduce turning speed for smoother turns

// Motor control flags
#define ENABLE_FRONT_MOTOR true  // Set to false to disable the front motor
#define ENABLE_BACK_MOTOR true  // Set to false to disable the back motor

#define TURN_DURATION 1000  // Motor turning duration

MotorControl::MotorControl() {}

void MotorControl::setup() {
    Serial.println("MotorControl: Initializing motor control pins...");
    setupPins();
    setupPWM();
    Serial.println("MotorControl: Pin setup complete.");
    checkMotorStatus();
}

void MotorControl::setupPins() {
    pinMode(FRONT_MOTOR_IN1, OUTPUT);
    pinMode(FRONT_MOTOR_IN2, OUTPUT);
    pinMode(BACK_MOTOR_IN1, OUTPUT);
    pinMode(BACK_MOTOR_IN2, OUTPUT);
    pinMode(FRONT_MOTOR_ENABLE, OUTPUT);
    pinMode(BACK_MOTOR_ENABLE, OUTPUT);
    pinMode(MOTOR_STBY, OUTPUT);
}

// Process WebSocket Input
void MotorControl::handleWebSocketInput(const String& direction, int speed, float angle) {
    Serial.printf("Handling WebSocket input - Direction: %s, Speed: %d, Angle: %.1f\n", direction.c_str(), speed, angle);

    // Eğer hız 0 ise motorları durdur
    if (speed == 0 || direction == "C") {
        stopAll();  // Hız 0 veya "C" yönünde motorları hemen durdur
        return;
    }

    // Hız maksimum sınırı aşarsa hızı sınırlayalım
    speed = constrain(speed, 0, 100); // Hızı 0 ile 100 arasında tut

    // Validation: Ensure angle is between 0 and 360
    if (angle < 0 || angle >= 360) {
        Serial.println("Invalid angle. Stopping motors.");
        stopAll();
        return;
    }

    // Determine which motor(s) should be active based on direction
    int motorOperation = whichMotorRuns(direction);

    // Adjust speed for respective motor limits
    int mappedBackMotorSpeed = map(constrain(speed, 0, 100), 0, 100, 0, MAX_BACK_MOTOR_SPEED);
    int mappedFrontMotorSpeed = map(constrain(speed, 0, 100), 0, 100, 0, MAX_FRONT_MOTOR_SPEED);

    // Validation based on which motor(s) should run
    if (motorOperation == PWM_CHANNEL_BACK_MOTOR && mappedBackMotorSpeed > MAX_BACK_MOTOR_SPEED) {
         Serial.println("Speed exceeds maximum limit for back motor. Reducing speed.");
         mappedBackMotorSpeed = MAX_BACK_MOTOR_SPEED;
    } else if (motorOperation == PWM_CHANNEL_FRONT_MOTOR && mappedFrontMotorSpeed > MAX_FRONT_MOTOR_SPEED) {
        Serial.println("Speed exceeds maximum limit for front motor. Reducing speed.");
        mappedFrontMotorSpeed = MAX_FRONT_MOTOR_SPEED;
    }

    // Process motor control based on validated inputs
    processMotorControl(direction, mappedBackMotorSpeed, mappedFrontMotorSpeed, angle);
}

void MotorControl::processMotorControl(const String& direction, int backMotorSpeed, int frontMotorSpeed, float angle) {
    // Hız aşımını engelle
    backMotorSpeed = constrain(backMotorSpeed, 0, MAX_BACK_MOTOR_SPEED);
    frontMotorSpeed = constrain(frontMotorSpeed, 0, MAX_FRONT_MOTOR_SPEED);

    // Hız sıfırsa tüm motorları durdur
    if (backMotorSpeed == 0 && frontMotorSpeed == 0) {
        stopAll();
        return;
    }

    // Update the last direction to track the current movement
    lastDirection = direction;

    // Process motor control based on the validated direction, speed, and angle
    if (direction == "N") {
        move(true, backMotorSpeed);  // Move forward with fine-tuned speed
        stopFrontMotor();
    } else if (direction == "S") {
        int reducedBackMotorSpeed = backMotorSpeed / 2;
        move(false, reducedBackMotorSpeed);  // Move backward with reduced speed
        stopFrontMotor();
    } else if (direction == "NE" || direction == "NW") {
        move(true, backMotorSpeed);
        turnFrontMotor(direction == "NE" ? false : true, frontMotorSpeed * TURN_REDUCTION_FACTOR);  // Turn front motor based on direction
    } else if (direction == "SE" || direction == "SW") {
        int reducedBackMotorSpeed = backMotorSpeed / 2;
        move(false, reducedBackMotorSpeed);
        turnFrontMotor(direction == "SE" ? false : true, frontMotorSpeed * TURN_REDUCTION_FACTOR);  // Turn front motor based on direction
    } else if (direction == "E") {
        move(true, backMotorSpeed);
        turnFrontMotor(false, frontMotorSpeed);
    } else if (direction == "W") {
        move(true, backMotorSpeed);
        turnFrontMotor(true, frontMotorSpeed);
    } else {
        Serial.println("Unknown direction. Stopping motors.");
        stopAll();
    }
}


// Turn left or right (only for the front motor)
void MotorControl::turnFrontMotor(bool left, int speed) {
    unsigned long currentTime = millis();

    Serial.printf("MotorControl: Turning front motor %s with speed %d\n", left ? "left" : "right", speed);
    int cappedSpeed = constrain(speed, 0, MAX_FRONT_MOTOR_SPEED);  // Limit speed to avoid strain on front motor

    if (left) {
        // Left turn logic
        setFrontMotorDirection(HIGH, LOW);
        lastTurnDirection = "left";
    } else {
        // Right turn logic
        setFrontMotorDirection(LOW, HIGH);
        lastTurnDirection = "right";
    }

    setFrontMotorSpeed(cappedSpeed);  // Apply speed to the front motor only
    motorTurning = true;
    frontMotorActive = cappedSpeed > 0;
    lastTurnTime = currentTime;
}

// Stop all motors
void MotorControl::stopAll() {
    if (!frontMotorActive && !backMotorActive) return;
    Serial.println("MotorControl: Stopping all motors...");
    setFrontMotorSpeed(0);
    setBackMotorSpeed(0);
    frontMotorActive = false;
    backMotorActive = false;
    setMotorStandby(true); 
    checkMotorStatus();
}

// Set front motor direction
void MotorControl::setFrontMotorDirection(int dir1, int dir2) {
    if (ENABLE_FRONT_MOTOR) {
        digitalWrite(FRONT_MOTOR_IN1, dir1);
        digitalWrite(FRONT_MOTOR_IN2, dir2);
    }
}

// Set back motor direction
void MotorControl::setBackMotorDirection(int dir1, int dir2) {
    if (ENABLE_BACK_MOTOR) {
        digitalWrite(BACK_MOTOR_IN1, dir1);
        digitalWrite(BACK_MOTOR_IN2, dir2);
    }
}

// Set front motor speed
void MotorControl::setFrontMotorSpeed(int speed) {
    if (ENABLE_FRONT_MOTOR) {
        setMotorStandby(false);
        int cappedSpeed = constrain(speed, 0, MAX_FRONT_MOTOR_SPEED);
        ledcWrite(PWM_CHANNEL_FRONT_MOTOR, abs(cappedSpeed));
        frontMotorActive = cappedSpeed > 0;
    }
}

// Set back motor speed
void MotorControl::setBackMotorSpeed(int speed) {
    setMotorStandby(false);

    int cappedSpeed = constrain(speed, 0, MAX_BACK_MOTOR_SPEED);

    if (ENABLE_BACK_MOTOR) {
        ledcWrite(PWM_CHANNEL_BACK_MOTOR, abs(cappedSpeed));
        backMotorActive = cappedSpeed > 0;
    }
}

// Move forward or backward
void MotorControl::move(bool forward, int targetSpeed) {
    int dir1 = forward ? HIGH : LOW;
    int dir2 = forward ? LOW : HIGH;

    // Set the motor direction immediately
    setBackMotorDirection(dir1, dir2);

    // Smoothly increase or decrease speed towards targetSpeed
    int speedIncrement = 5;  // Fine-tuning için hız artış/azalış miktarı
    int currentSpeed = lastSpeed;

    // Hızlandırma veya yavaşlatma
    if (currentSpeed < targetSpeed) {
        currentSpeed = min(currentSpeed + speedIncrement, targetSpeed);  // Hız artırma
    } else if (currentSpeed > targetSpeed) {
        currentSpeed = max(currentSpeed - speedIncrement, targetSpeed);  // Hız azaltma
    }

    // Apply the speed to the motor
    setBackMotorSpeed(currentSpeed);

    // Update the last speed to track the current state
    lastSpeed = currentSpeed;

    // Durma kontrolü: Eğer hedef hız 0 ise motoru durdur
    if (targetSpeed == 0) {
        stopAll();
    }
}


// Stop the front motor
void MotorControl::stopFrontMotor() {
    if (!frontMotorActive) return;
    setFrontMotorSpeed(0);
    frontMotorActive = false;

    // Optionally, set the front motor direction pins to LOW
    setFrontMotorDirection(LOW, LOW);

    // If both motors are inactive, set the standby mode
    if (!backMotorActive) {
        setMotorStandby(true);
    }
}

// Setup PWM channels
void MotorControl::setupPWM() {
    bool pwmBackSetup = ledcSetup(PWM_CHANNEL_BACK_MOTOR, PWM_FREQUENCY, PWM_RESOLUTION);
    bool pwmFrontSetup = ledcSetup(PWM_CHANNEL_FRONT_MOTOR, PWM_FREQUENCY, PWM_RESOLUTION);

    if (!pwmBackSetup || !pwmFrontSetup) {
        Serial.println("MotorControl: Error setting up PWM channels.");
        return;
    }

    if (ENABLE_BACK_MOTOR) {
        ledcAttachPin(BACK_MOTOR_ENABLE, PWM_CHANNEL_BACK_MOTOR);
    }
    if (ENABLE_FRONT_MOTOR) {
        ledcAttachPin(FRONT_MOTOR_ENABLE, PWM_CHANNEL_FRONT_MOTOR);
    }

    Serial.println("MotorControl: PWM setup complete.");
    checkMotorStatus();
}

// Check the current status of the motors
void MotorControl::checkMotorStatus() {
    if (ENABLE_FRONT_MOTOR) {
        Serial.printf("FRONT_MOTOR_IN1: %d, FRONT_MOTOR_IN2: %d\n", digitalRead(FRONT_MOTOR_IN1), digitalRead(FRONT_MOTOR_IN2));
    }
    if (ENABLE_BACK_MOTOR) {
        Serial.printf("BACK_MOTOR_IN1: %d, BACK_MOTOR_IN2: %d\n", digitalRead(BACK_MOTOR_IN1), digitalRead(BACK_MOTOR_IN2));
    }

    int backMotorPWM = ledcRead(PWM_CHANNEL_BACK_MOTOR);
    int frontMotorPWM = ledcRead(PWM_CHANNEL_FRONT_MOTOR);
    Serial.printf("Back Motor PWM: %d, Front Motor PWM: %d\n", backMotorPWM, frontMotorPWM);
}

// Set the motor standby mode
void MotorControl::setMotorStandby(bool standby) {
    digitalWrite(MOTOR_STBY, standby ? LOW : HIGH);
}

int MotorControl::whichMotorRuns(const String& direction) {
    if (direction == "N" || direction == "S") {
        return PWM_CHANNEL_BACK_MOTOR;
    } else {
        return PWM_CHANNEL_FRONT_MOTOR;
    }
}

//Get the last direction
String MotorControl::getLastDirection() {
    return lastDirection;
}

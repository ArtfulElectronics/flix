// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// PWM control for motors

#define USE_SERVO_LIB 0

#include <ESP32Servo.h>
#include "util.h"

float motors[4] = { 0.0f, 0.0f, 0.0f, 0.0f }; // normalized motor thrusts in range [0..1]
float pwm[4], duty[4]; // for printing to the console for troubleshooting purpose

#if USE_SERVO_LIB
Servo myESC[4];
#endif

static constexpr int motorPins[4] = {5, 4, 3, 1}; // default pin numbers
// Not documented but this is in Hz so the period is 20ms or 20000 usec.
static int pwmFrequency = 50;
static constexpr int pwmResolution = 10;
// Not documented but pwm* are measured in microseconds.
// For brushless motor ESC, the control pulse are 1ms to 2ms (or 1000 microseconds to 2000 microseconds).
// We keep pwmStop = 0 to OFF the entire control signal (disarm the motor) instead of keeping it at 1ms duty.
static constexpr int pwmStop = 1000;
static constexpr int pwmMin = 1000;
static constexpr int pwmMax = 2000; // -1 means duty cycle mode

const int MOT_RL = 0, MOT_RR = 1, MOT_FR = 2, MOT_FL = 3;

void setupMotors() {
	print("Setup motors\n");

#if USE_SERVO_LIB
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
#endif

	// Configure pins
	for (int i = 0; i < 4; i++) {
#if USE_SERVO_LIB
		myESC[i].setPeriodHertz(50);      // Standard 50Hz frequency for servos/ESCs
		myESC[i].attach(motorPins[i], 1000, 2000); // Attach pin with min/max pulse width in microseconds
#else
		if (motorPins[i] < 0) continue; // skip unassigned motors
		ledcAttach(motorPins[i], pwmFrequency, pwmResolution);
		pwmFrequency = ledcChangeFrequency(motorPins[i], pwmFrequency, pwmResolution); // when reconfiguring
#endif
	}
	sendMotors();
}

void sendMotors() {
	for (int i = 0; i < 4; i++) {
		if (motorPins[i] < 0) continue; // skip unassigned motors

		float thrust = constrain(motors[i], 0, 1);
		pwm[i] = mapf(thrust, 0, 1, pwmMin, pwmMax);

#if USE_SERVO_LIB
		myESC[i].writeMicroseconds(pwm[i]);
#else
		duty[i] = mapf(pwm[i], 0, 1000000 / pwmFrequency, 0, (1 << pwmResolution) - 1);
		ledcWrite(motorPins[i], /* getDutyCycle(motors[i]) */ duty[i]);
#endif
	}
}

int getDutyCycle(float value) {
	value = constrain(value, 0, 1);

	if (pwmMax >= 0) { // pwm mode
		float pwm = mapf(value, 0, 1, pwmMin, pwmMax);
		if (value == 0) pwm = pwmStop;
		float duty = mapf(pwm, 0, 1000000 / pwmFrequency, 0, (1 << pwmResolution) - 1);
		return round(duty);
	} else { // duty cycle mode
		return round(value * ((1 << pwmResolution) - 1));
	}
}

bool motorsActive() {
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(int n, float thrust) {
	print("Testing motor %d\n", n);
	motors[n] = thrust;
	delay(50); // ESP32 may need to wait until the end of the current cycle to change duty https://github.com/espressif/arduino-esp32/issues/5306
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	print("Done\n");
}

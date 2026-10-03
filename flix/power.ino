// Copyright (c) 2026 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Power management

#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include "filter.h"
#include "util.h"

float voltage = NAN;
LowPassFilter<float> voltageFilter(1);
int voltagePin = 0;
// Use a RV : RG voltage divider to get maximum 2S LiPo battery voltage (8.4V) into readable range.
// Ideally, RV : RG = 10K : 5K then [0, 8.4V] becomes [0, 2.8V].
const float RV = 10; // Resistor connecting to +BATT (k Ohm)
const float RG =  5; // Resistor connecting to GND (k Ohm)
float voltageScale = (RV + RG) / RG;

void setupPower() {
	REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA); // disable reset on low voltage
	if (digitalPinToAnalogChannel(voltagePin) == -1) voltagePin = -1; // test ADC pin
}

void readVoltage() {
	if (voltagePin < 0) return;

	static Rate rate(10);
	if (!rate) return;

	float v = analogReadMilliVolts(voltagePin) * voltageScale / 1000.0f;
	voltage = voltageFilter.update(v);
}

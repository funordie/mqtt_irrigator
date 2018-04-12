/*
 * hx711.c
 *
 *  Created on: Apr 11, 2018
 *      Author: ipaev
 */

#include "Arduino.h"
#include "hx711.h"
#include "Q2HX711.h"

//HX711 constructor (dout pin, sck pin)
HX711::HX711(byte output_pin, byte clock_pin) {
  m_LoadCell = new Q2HX711(output_pin, clock_pin);
}

HX711::~HX711() {
	delete(m_LoadCell);
}

void HX711::set_cal(hx711_cal_t * cal) {
	memcpy(&m_cal,   cal, sizeof(hx711_cal_t));
}

float HX711::get_weight() {

	float res = m_LoadCell->read();
	Serial.print("get_weight: RAW:");
	Serial.print(res);

	Serial.print(" Cal [factor:");
	Serial.print(m_cal.factor);

	Serial.print(" offset:");
	Serial.print(m_cal.offset);
	Serial.print("]");

	res = res*m_cal.factor + m_cal.offset;
	Serial.print(" result:");
	Serial.println(res);

    return res;
}

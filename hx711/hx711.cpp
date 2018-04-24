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
  m_cal.factor = 1;
  m_cal.offset = 0;
}

HX711::~HX711() {
	delete(m_LoadCell);
}

void HX711::set_cal(hx711_cal_t * cal) {
	memcpy(&m_cal,   cal, sizeof(hx711_cal_t));
}

long HX711::get_raw() {
    long raw = m_LoadCell->read();
    Serial.printf("HX711::get_raw:%lu\n", raw);
    return raw;
}

float HX711::get_weight() {

	long raw = get_raw();
	float res = raw * m_cal.factor + m_cal.offset;

    Serial.printf("HX711::get_weight: [factor:%f offset:%f] result:%f\n", m_cal.factor, m_cal.offset, res);

    return res;
}

/*
 * hx711.h
 *
 *  Created on: Apr 11, 2018
 *      Author: ipaev
 */

#ifndef HX711_HX711_H_
#define HX711_HX711_H_

#include "Q2HX711.h"

typedef struct {
	float factor;
	float offset;
}hx711_cal_t;

class HX711 {
public:
	HX711(byte output_pin, byte clock_pin);
	~HX711();
	float get_weight();
	void set_cal(hx711_cal_t *);
private:
	hx711_cal_t m_cal;
	Q2HX711 * m_LoadCell;
};
void hx711_set_cal(hx711_cal_t * cal);

#endif /* HX711_HX711_H_ */

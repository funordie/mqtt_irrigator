/*
 * mqtt_irrigator.h
 *
 *  Created on: Apr 11, 2018
 *      Author: ipaev
 */

#ifndef MQTT_IRRIGATOR_H_
#define MQTT_IRRIGATOR_H_

#include "hx711/hx711.h"
#include "include/wl_definitions.h"

#define VERSION_LENGTH 4

typedef struct {
  // This is for mere detection if they are your settings
  char version[VERSION_LENGTH];
  // The variables of your settings
  uint8_t moduleId[WL_MAC_ADDR_LENGTH];  // module id
  hx711_cal_t hx711_cal; //load cell calibration
}StoreStruc;

void loadConfig(StoreStruc *pStorage);


#endif /* MQTT_IRRIGATOR_H_ */

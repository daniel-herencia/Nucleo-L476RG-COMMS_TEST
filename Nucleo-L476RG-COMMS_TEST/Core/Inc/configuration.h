/*
 * configuration.h
 *
 *  Created on: 31 may. 2022
 *      Author: Usuari
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_
#include "definitions.h"
#include "flash.h"
#include <time.h>
#include <string.h>

static const uint8_t GYRO_ADDR = 0x68 << 1; //gyroscope address, 0x68 or 0x69 depending on the SA0 pin
static const uint8_t MAG_ADDR = 0x30 << 1; //magnetometer address
static const uint8_t BATTSENSOR_ADDR = 0x34 << 1; //battery sensor address

/*Only at the beginning, includes the Antenna deployment, check batteries, configure payloads*/
/*Will be executed every time we reboot the system*/
void init(I2C_HandleTypeDef *hi2c);

/*Initialize all the sensors*/
void initsensors(I2C_HandleTypeDef *hi2c);

/*Compute the level of battery and updates batterylevel*/
void checkbatteries(I2C_HandleTypeDef *hi2c);

/*Check the temperature of the 8 sensors, returns true if the temperature is between TEMP_MAX and TEMP_MIN*/
bool checktemperature(I2C_HandleTypeDef *hi2c);

/*Activate or deactivate the battery heater*/
void heater(int state);

/*Send a signal to a resistor to burn a wire and deploy COMMS Antenna
 *Check the switch to make sure the Antenna has been deployed properly
 *And write deployment_state = true in the EEPROM memory*/
void deployment(I2C_HandleTypeDef *hi2c, uint8_t nominal, uint8_t low, uint8_t critical);

/*Send a signal to a resistor to burn a wire and deploy PL2 Antenna
 *Once confirmed the proper deployment of the antenna,  write deploymentRF_state = true in the EEPROM memory*/
void deploymentRF(I2C_HandleTypeDef *hi2c);

/*We'll have a file (SPG4) which will tell us the position
 *Check if we are in the region of contact with GS */
void check_position(void);

/*Check battery level, temperatures,etc
 *Depending on the battery level, it returns a velue betweeen 0-3*/
int system_state(I2C_HandleTypeDef *hi2c, uint8_t nominal, uint8_t low, uint8_t critical);

/*Converts the RTC in Unix Time Format */
void HumanToUnixTime(RTC_HandleTypeDef *hrtc, uint32_t time32);
/*Converts Unix Time Format to RTC Type */
void UnixToHumanTime(uint32_t time, RTC_HandleTypeDef *hrtc);
/* Different causes for a system reset*/
typedef enum reset_cause_e
  {
      RESET_CAUSE_UNKNOWN = 0,
      RESET_CAUSE_LOW_POWER_RESET,
      RESET_CAUSE_WINDOW_WATCHDOG_RESET,
      RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
      RESET_CAUSE_SOFTWARE_RESET,
      RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
      RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
      RESET_CAUSE_BROWNOUT_RESET,
  } reset_cause_t;
 /* Returns the cause of a system reset*/
reset_cause_t reset_cause_get(void);



#endif /* INC_CONFIGURATION_H_ */

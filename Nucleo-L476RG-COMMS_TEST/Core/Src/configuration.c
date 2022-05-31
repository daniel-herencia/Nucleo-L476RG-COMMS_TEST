/*!
 * \file      configuration.c
 *
 * \brief     It contains the initializing functions and all the functions that check
 * 			  the satellite state
 *
 *
 * \created on: 01/10/2021
 *
 * \author    Pol Simon
 *
 * \author    David Reiss
 *
 * \modified on: 01/03/2022
 */

#include "configuration.h"

/**************************************************************************************
 *                                                                                    *
 * Function:  checkbatteries                                                 		  *
 * --------------------                                                               *
 * Checks the current battery level	and stores it in the NVM						  *
 *                                                                                    *
 * hi2c: I2C to read battery capacity							    				  *
 *															                          *
 * returns: Nothing									                              	  *
 *                                                                                    *
 **************************************************************************************/
void checkbatteries(I2C_HandleTypeDef *hi2c) {
	uint8_t percentage;
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Master_Transmit(hi2c, BATTSENSOR_ADDR, (uint8_t*) 0x06, 1,
			500); //we want to read from the register 0x06
	if (ret != HAL_OK) {

	} else {
		HAL_I2C_Master_Receive(hi2c, BATTSENSOR_ADDR, &percentage, 1, 500);
	}
	Write_Flash(BATT_LEVEL_ADDR, &percentage, 1);
}

/**************************************************************************************
 *                                                                                    *
 * Function:  deployment                                               		  		  *
 * --------------------                                                               *
 * Induces a current through a resistor in order to burn the nylon wire and deploy	  *
 * the comms antenna. After that, the function writes delpoyment_state = true in	  *
 * the memory																		  *
 *																					  *
 *  hi2c: I2C to read temperatures in system_state()			    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void deployment(I2C_HandleTypeDef *hi2c, uint8_t nominal, uint8_t low, uint8_t critical) {
	bool deployment = false;
	Read_Flash(NOMINAL_ADDR, &nominal, sizeof(nominal));
	Read_Flash(LOW_ADDR, &low, sizeof(low));
	Read_Flash(CRITICAL_ADDR, &critical, sizeof(critical));
	if (system_state(&hi2c,nominal,low,critical) == 0) {
		/*Give high voltage to the resistor to burn the wire, TBD TIME AND VOLTAGE*/
	}
	deployment = true;

	Write_Flash(DEPLOYMENT_STATE_ADDR, &deployment, 1); /*Must be stored in FLASH memory in order to keep it if the system is rebooted*/
}

/**************************************************************************************
 *                                                                                    *
 * Function:  deploymentRF                                               	  		  *
 * --------------------                                                               *
 * Induces a current through a resistor in order to burn the nylon wire and deploy	  *
 * the PL2 antenna. After that, the function writes delpoymentRF_state = true in the  *
 * memory																		  	  *
 *																					  *
 *  hi2c: I2C to read temperatures in system_state()			    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void deploymentRF(I2C_HandleTypeDef *hi2c) {

}

/**************************************************************************************
 *                                                                                    *
 * Function:  check_position                                               	  		  *
 * --------------------                                                               *
 * With the SPG4 file, checks if the satellite is in the contact range with GS  	  *
 *																					  *
 *  No input													    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void check_position() {
	//Region of contact with GS => Write_Flash(COMMS_STATE_ADDRESS, TRUE, 1);
}

/**************************************************************************************
 *                                                                                    *
 * Function:  init			                                               	  		  *
 * --------------------                                                               *
 * Simulates the "INIT" state: detumbling and deployment of the antennas. If all	  *
 * goes as expected, the next state is CHECK										  *
 *																					  *
 *  hi2c: I2C to read temperatures in system_state()			    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void init(I2C_HandleTypeDef *hi2c) {
	bool deployment_state, deploymentRF_state;
	Read_Flash(DEPLOYMENT_STATE_ADDR, &deployment_state, 1); //read the indicator of the deployment of comms antenna
	Read_Flash(DEPLOYMENTRF_STATE_ADDR, &deploymentRF_state, 1); //read the indicator of the deployment of PL2 antenna
	uint8_t currentStateinit,nominal, low, critical;

	Read_Flash(NOMINAL_ADDR, &nominal, sizeof(nominal));
	Read_Flash(LOW_ADDR, &low, sizeof(low));
	Read_Flash(CRITICAL_ADDR, &critical, sizeof(critical));

	// If battery level is low --> CONTINGENCY
	if (system_state(&hi2c,nominal,low,critical)>0)
		currentStateinit = CONTINGENCY;
	else {
		// If the deployment is not done yet
		if (!deployment_state)
			deployment(&hi2c,nominal,low,critical);
		//Just in the PocketQube with the RF antenna
		if (!deploymentRF_state)
			deploymentRF(&hi2c);

		currentStateinit = CHECK;
	}
	// Escribim a memoria el nou estat
	Write_Flash(CURRENT_STATE_ADDR, currentStateinit, sizeof(currentStateinit));
}

/**************************************************************************************
 *                                                                                    *
 * Function:  initsensors                                               	  		  *
 * --------------------                                                               *
 * Initializes both gyroscope and magnetometer sensors 								  *
 *																					  *
 *  hi2c: I2C to write in the registers of the sensors			    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void initsensors(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef ret;
	//GYROSCOPE CONFIGURATION
	ret = HAL_I2C_Master_Transmit(hi2c, GYRO_ADDR, (uint8_t*) 0x1A, 1, 1000); //write in the register 0x1A
	if (ret != HAL_OK) {

	} else {
//		HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, /*data to register 0x1A*/, 1, 1000);
	}
	ret = HAL_I2C_Master_Transmit(hi2c, GYRO_ADDR, (uint8_t*) 0x1B, 1, 1000); //write in the register 0x1B
	if (ret != HAL_OK) {

	} else {
//		HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, /*data to register 0x1B*/, 1, HAL_MAX_DELAY);
	}

	//MAGNETOMETER CONFIGURATION mirar a quin registre s'ha d'escriure
//	ret = HAL_I2C_Master_Transmit(&hi2c, MAG_ADDR, /**/, 1, HAL_MAX_DELAY);

}

/**************************************************************************************
 *                                                                                    *
 * Function:  system_state                                               	  		  *
 * --------------------                                                               *
 * Checks the current battery level and different temperature sensors of the		  *
 * satellite																		  *
 *																					  *
 *  hi2c: I2C to read from the temperature sensors				    				  *
 *															                          *
 *  returns: Value between 0-3					                              		  *
 *  		 0: If battery level between 100%-NOMINAL								  *
 *  		 1: If battery level between NOMINAL-LOW								  *
 *  		 2: If battery level between LOW-CRITICAL								  *
 *  		 3: If battery level lower than CRITICAL								  *
 *                                                                                    *
 **************************************************************************************/
int system_state(I2C_HandleTypeDef *hi2c, uint8_t nominal, uint8_t low, uint8_t critical) {
	uint8_t battery_capacity;

	/*If checktemperature returns false, there are different cases which must be distinguished:
	 * 	- More than three temperature sensors are hot => start rotating the satellite
	 * 	- Battery temperature is too hot => THIS CASE MUST BE STUDIED
	 * 	- MCU temperature out of operating range => THIS CASE MUST BE STUDIED */
	if (!checktemperature(hi2c)) /*rotate_satellite*/
	{
		//NOTIFY ADCS --> ROTATE_NOTI
		//xTaskNotify("Task ADCS", ROTATE_NOTI, eSetBits);
	}

	checkbatteries(hi2c);

	/*Read from memory the BATTERY LEVEL*/
	Read_Flash(BATT_LEVEL_ADDR, &battery_capacity, 1);

	// BATERRY LEVEL < CRITICAL --> 3 (WORSE CASE) = SURVIVAL STATE
	if (battery_capacity < critical)
		return 3;
	// CRITICAL <= BATERRY LEVEL < LOW --> 2 = SUNSAFE STATE
	else if (battery_capacity < low)
		return 2;
	// LOW <= BATERRY LEVEL < NOMINAL --> 1 = CONTINGENCY STATE
	else if (battery_capacity < nominal)
		return 1;
	// BATERRY LEVEL >= NOMINAL --> 0 (IDEAL CASE) = CHECK STATE
	else
		return 0;
}


/**************************************************************************************
 *                                                                                    *
 * Function:  checktemperature                                             	  		  *
 * --------------------                                                               *
 * Checks if all the temperatures are in their corresponding ranges. This function is *
 * also responsible for activating or deactivating the battery heater				  *
 *																					  *
 *  hi2c: I2C to read from the temperature sensors				    				  *
 *															                          *
 *  returns: False if more than 3 solar panels are too hot 							  *
 *  		 True otherwise								                              *
 *  		 																		  *
 **************************************************************************************/
bool checktemperature(I2C_HandleTypeDef *hi2c) {
	Temperatures temp;
	int N = 7; // Number of sensors not defined yet
	Flash_Read_Data(TEMP_ADDR, &temp.raw, sizeof(temp));
	int i, cont = 0;
	for (i = 1; i <= N; i++) {

		switch (i) {

		case 1:
			if (temp.fields.tempbatt <= TEMP_MIN)
				heater(1);
			else
				heater(0);
			break;

		case 2:
			if (temp.fields.temp1 > TEMP_MAX)
				cont++;
			break;

		case 3:
			if (temp.fields.temp2 > TEMP_MAX)
				cont++;
			break;

		case 4:
			if (temp.fields.temp3 > TEMP_MAX)
				cont++;
			break;

		case 5:
			if (temp.fields.temp4 > TEMP_MAX)
				cont++;
			break;

		case 6:
			if (temp.fields.temp5 > TEMP_MAX)
				cont++;
			break;

		case 7:
			if (temp.fields.temp6 > TEMP_MAX)
				cont++;
			break;

		default:
			break;
			//more cases should come as much as the final number of sensors
		}

		if (cont > 3)
			return false;
		else
			return true;
	}
}

void heater(int state) {

}

/**************************************************************************************
 *                                                                                    	*
 * Function:  HumanToUnixTime                                          	  		  	  	*
 * --------------------                                                               	*
 *  Read the actual time with the RTC and converts the time in Unix Format				*
 *  Saves the time in a uint32_t variable. This function will be used only in the 		*
 *  Payload Camera Task																	*
 *																					  	*
 *  hrtc: RTC to read from the date and time			    				      		*
 *															                          	*
 *  returns: nothing							      									*
 *  		 																		  	*
 **************************************************************************************/
void HumanToUnixTime(RTC_HandleTypeDef *hrtc, uint32_t time32) {
	//Get the time and date
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN);

	//Make the conversion from RTC Type to struct tm
	//January and February are counted as months 13 and 14 of the previous year
	time_t timestamp;
	struct tm currTime;
	currTime.tm_year = sDate.Year + 100;
    currTime.tm_mon = sDate.Month-1;
    currTime.tm_mday = sDate.Date;
	currTime.tm_hour = sTime.Hours-1;
    currTime.tm_min = sTime.Minutes;
    currTime.tm_sec = sTime.Seconds;

    // Convert from struct tm to time_t
    timestamp = mktime(&currTime);

    // Convert from time_t to uint32_t
    time32 = (uint32_t)timestamp;

    // Converts uint32_t to uint8_t[4]
    uint8_t data[4];
    memcpy(data, &time32, sizeof(time32));

    // Saves the Time in Unix format in the ADDR
    Write_Flash(SET_TIME_ADDR, &data, sizeof(data));
    // TEST
    uint32_t timeread;
    Read_Flash(SET_TIME_ADDR, &timeread, sizeof(timeread));
}

/**************************************************************************************
 *                                                                                    *
 * Function:  UnixToHumanTime                                            	  		  *
 * --------------------                                                               *
 * Converts the time from Unix Format to Human Readable Date, writes the time 		  *
 * (unix format) in the corresponding ADDR and makes an RTC_SetTime and RTC_SetDate	  *
 *																					  *
 *  hrtc: RTC to read from the date and time
 *  settime: The time in Unix format send by the GS			    				      *
 *															                          *
 *  returns: nothing							      									*
 *  		 																		  *
 **************************************************************************************/
void UnixToHumanTime(uint32_t settime, RTC_HandleTypeDef *hrtc) {
	time_t unix = settime;
	struct tm currTime;
	localtime_r(&unix, &currTime);
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	// Converts tm struct to RTC_TimeTypeDef, RTC_DateTypeDef
	sDate.Year = currTime.tm_year - 100;
	sDate.Month = currTime.tm_mon +1;
	sDate.Date = currTime.tm_mday;
	sTime.Hours= currTime.tm_hour + 1;
	sTime.Minutes = currTime.tm_min;
	sTime.Seconds = currTime.tm_sec;

	// Writes in the memory ADDR -> DONE BY COMMS
    // Write_Flash(RTC_TIME_ADDR, &settime, sizeof(settime));

    // Sets the new Time and Date
    HAL_RTC_SetTime(hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(hrtc, &sDate, RTC_FORMAT_BIN);
}

  /// @brief      Obtain the STM32 system reset cause
  /// @param      None
  /// @return     The system reset cause

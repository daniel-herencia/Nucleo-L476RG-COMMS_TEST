#include "vADCSTask.h"
#include "adcs.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
#include "sgp.h"
#include "stdlib.h"
#include "orbit_propagators_utils.h"
#include "igrf13.h"
#include "optimal_request.h"

uint32_t received_value = 0;
uint32_t CONTINGENCY_NOTI = 0;
uint32_t DETUMBLING_NOTI = 0;
uint32_t TUMBLING_NOTI = 0;
uint32_t portMAX_DELAY = 0;
uint32_t ULONG_MAX = 0;
uint32_t GS_NOTI = 0;
uint32_t NOTGS_NOTI = 0;
uint32_t POINTING_NOTI = 0;
uint32_t OBC_task = 0;
uint32_t TLE_NOTI;
orbit_t orbit;
tle_data tle;
int nPts;
float *xcoord, *ycoord, *zcoord, *r_eci, *v_eci, *xvel, *yvel, *zvel, lat, lon;
float actualunixtime, latmax, latmin, lonmax, lonmin, eSetBits, jday, *q_est, actual_time, dtime, previous_time;

int checkPositionValue;
ControlValues *control;
I2C_HandleTypeDef *hi2c1 = 0;
ADC_HandleTypeDef *hadc = 0;
mag_data *magData = 0;
gyro_data *gyroData = 0;
sun_vector *sunvector = 0;
RTC_HandleTypeDef *hrtc = 0;

void ADCSTask(void *pvParameters) {

	while(received_value != CONTINGENCY_NOTI){
		for (;;) {

			sensorData(hi2c1, hadc, magData, gyroData, sunvector);
			// Constantly checking the sat position
			checkPositionValue = checkposition(orbit, 2, actualunixtime, latmax, latmin, lonmax, lonmin);
			// if in the contact region
			if(checkPositionValue == 1){
				xTaskNotify(OBC_task, GS_NOTI, eSetBits);
			}
			// if NOT the contact region
			if(checkPositionValue == 0){
				xTaskNotify(OBC_task, NOTGS_NOTI, eSetBits);
			}

			xTaskNotifyWait(0,0,ULONG_MAX,&received_value,portMAX_DELAY);
			//if noti = Detumbling, we start the detumbling
			if(received_value == DETUMBLING_NOTI){

				sensorData(hi2c1, hadc, magData, gyroData, sunvector);
				//run until the satellite has stopped moving
				while(!checkGyro(hi2c1)){
					detumble(hi2c1);
				}
			}
			//if noti = tumbling, we start the tumbling
			if(received_value == TUMBLING_NOTI){

				sensorData(hi2c1, hadc, magData, gyroData, sunvector);
				while(checkGyro(hi2c1)){
					tumble(hi2c1);
				}
			}
			//if noti = tle, we update the tle data
			if(received_value == TLE_NOTI){

				updateTLE(&tle);

			}
			//if noti = pointing, call nadir algorithm
			if(received_value == POINTING_NOTI){

				//read data from the sensors
				sensorData(hi2c1, hadc, magData, gyroData, sunvector);
				HumanToUnixTime(hrtc, actualunixtime);
				jday = j_day(actualunixtime);
				sgp(orbit, nPts, &xvel, &yvel, &zvel, &xcoord, &ycoord, &zcoord, actualunixtime);
				vec3 xyz, llh, mag_eci, mag_ecef;
				r_eci[0] = xcoord[0];
				r_eci[1] = ycoord[0];
				r_eci[2] = zcoord[0];
				v_eci[0] = xvel[0];
				v_eci[1] = yvel[0];
				v_eci[2] = zvel[0];
				xyz.raw[0] = xcoord[0];
				xyz.raw[1] = ycoord[0];
				xyz.raw[2] = zcoord[0];
				ecef2llh(xyz, &llh);
				igrf12_ngdc(jday, llh, &mag_eci, &mag_ecef);
				optimal_request(control, q_est);
				actual_time = OS_Tick_GetCount();
				dtime = (actual_time*1.0f-previous_time*1.0f);
				previous_time = OS_Tick_GetCount();
				nadir_algorithm(hi2c1, control, dtime, q_est, r_eci, v_eci);

			}
		}
	}
}

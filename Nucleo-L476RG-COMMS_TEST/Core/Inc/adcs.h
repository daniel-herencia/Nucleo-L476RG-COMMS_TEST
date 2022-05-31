
#ifndef INC_ADCS_H_
#define INC_ADCS_H_


#include "stm32l4xx_hal.h"
#include "stdbool.h"
#include "sgp.h"
#include "optimal_request.h"

typedef struct __attribute__ ((__packed__)) gyro_aux {
	double gx_h;
	double gy_h;
	double gz_h;
}gyro_aux;

typedef struct __attribute__ ((__packed__)) gyro_data {
	double gx;
	double gy;
	double gz;
}gyro_data;

typedef struct __attribute__ ((__packed__)) mag_data {
	double mx;
	double my;
	double mz;
}mag_data;

typedef struct __attribute__ ((__packed__)) sun_vector {
	uint32_t x;
	uint32_t y;
	uint32_t z;
}sun_vector;


typedef struct __attribute__ ((__packed__)) ControlData {
	float quaternion_est[4];
}ControlData;


void detumble(I2C_HandleTypeDef *hi2c1);

void tumble(I2C_HandleTypeDef *hi2c1);

void AngularVelocity(I2C_HandleTypeDef *hi2c1, double *w);

void readPhotodiodes(ADC_HandleTypeDef *hadc, uint32_t photoData[6]);

void sunVector(ADC_HandleTypeDef *hadc, uint32_t *sunvector);

void MagneticField(I2C_HandleTypeDef *hi2c1, double *m);

void cross(double *A, double *B, double *res);

double norm(double A[]);

void CurrentToCoil(I2C_HandleTypeDef *hi2c1, double intensidad[3]);

bool checkGyro(I2C_HandleTypeDef *hi2c1);

void decimal_to_binary(int n, char *res);

void nadir_algorithm(I2C_HandleTypeDef *hi2c1, ControlValues *control, float dtime, float *q_est, float *r_eci, float *v_eci);

double gainConstant(void);

void sensorData(I2C_HandleTypeDef *hi2c1, ADC_HandleTypeDef *hadc, mag_data *magData, gyro_data *gyroData, sun_vector *sunVector);




#endif /* INC_ADCS_H_ */

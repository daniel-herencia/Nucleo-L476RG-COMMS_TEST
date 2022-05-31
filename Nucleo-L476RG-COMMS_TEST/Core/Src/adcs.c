#include "adcs.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
#include "sgp.h"
#include "stdlib.h"
#include "matrix_utils.h"
#include "optimal_request.h"

#define PI 3.14159265358979323846
#define asind(x) (asin( x ) / PI * 180)
#define atan2d(x,y) (atan2(x,y) / PI * 180)



/**************************************************************************************
 *                                                                                    *
 * Function:  cross                                                 		  		  *
 * --------------------                                                               *
 * Calculates the cross product between two vectors					  				  *
 *															                          *
 *  returns: vector with the cross product									          *
 *                                                                                    *
 **************************************************************************************/
void cross(double *A, double *B, double *res){

	res[0] = A[1]*B[2]-A[2]*B[1];
	res[1] = A[2]*B[0]-A[0]*B[2];
	res[2] = A[0]*B[1]-A[1]*B[0];

 }

void decimal_to_binary(int n, char *res)
{
  int c, d, t;
  char *p;

  t = 0;
  p = (char*)malloc(32+1);

  for (c = 8 ; c > 0 ; c--)
  {
	//  dividing n with 2^c
    d = n >> c;

    if (d & 1)
      *(p+t) = 1 + '0';
    else
      *(p+t) = 0 + '0';

    t++;
  }
  *(p+t) = '\0';

  res = p;
}


/**************************************************************************************
 *                                                                                    *
 * Function:  norm                                                 		  		      *
 * --------------------                                                               *
 * Calculates the norm of a vectors					  				                  *
 *															                          *
 *  returns: integer with the value of the norm of the vector						  *
 *                                                                                    *
 **************************************************************************************/
double norm(double A[3]){

	double vect_norm=0;
	vect_norm = sqrt((A[0]+A[1]+A[2]));
	return vect_norm;

}

double gainConstant(void){

	double w = 0;
	double T = 5551;
	double m = 0.250;
	double a = 0.05;
	double alfa = 0.645772;
	double k = 0;
	double inercia [3][3] = {{(m*a*a)/6, 0, 0}, {0, (m*a*a)/6, 0}, {0, 0, (m*a*a)/6}};
	w = (2*PI)/T;
	k = 2*w*(1+sin(alfa))*inercia[0][0];

	return k;

}

/**************************************************************************************
 *                                                                                    *
 * Function:  detumble                                                 		  		  *
 * --------------------                                                               *
 * Checks the gyroscope measurements and stabilizes the satellite. 					  *
 * It is called when the satellite is ejected from the deployer						  *
 *                                                                                    *
 *  hi2c: I2C to read outputs from gyroscope					    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void detumble(I2C_HandleTypeDef *hi2c1) {

	double k = gainConstant();
	double gyr_read[3], mag_read[3], intensity[3], magneticDipole[3], w[3], m[3], a;
	double maxMagneticDipole1 = 0.02359;
	double maxMagneticDipole2 = 0.03234;
	double maxIntensity1 = 0.14681;
	double maxIntensity2 = 0.1495;
	//shut down the magnetorquers to get a good reading of the magnetometers
	CurrentToCoil(hi2c1, 0);
	//read the data from the gyroscope
	AngularVelocity(hi2c1, w);
	//read the data from the magnetometer
	MagneticField(hi2c1,m);
	memcpy(gyr_read, w, 3);
	memcpy(mag_read, m, 3);
	//bdot algorithm
	cross(mag_read,gyr_read, magneticDipole);
	a = pow(norm(magneticDipole),2);
	magneticDipole[0] = -k*magneticDipole[0]/a;
	magneticDipole[1] = -k*magneticDipole[1]/a;
	magneticDipole[2] = -k*magneticDipole[2]/a;

	if(magneticDipole[0]>0){
		intensity[0] = (magneticDipole[0]*maxIntensity1)/maxMagneticDipole1;
	}else{
		intensity[0] = (magneticDipole[0]*maxIntensity2)/maxMagneticDipole2;
	}

	intensity[1] = (magneticDipole[1]*maxIntensity2)/maxMagneticDipole2;
	intensity[2] = (magneticDipole[2]*maxIntensity2)/maxMagneticDipole2;
	//pass the current to the magnetorquers
	CurrentToCoil(hi2c1, intensity);


}
/**************************************************************************************
 *                                                                                    *
 * Function:  tumble                                                 		  		  *
 * --------------------                                                               *
 * Checks the gyroscope measurements and destabilizes the satellite. 				  *
 * It is called when the satellite is experiencing a lot of heat and wants to cool dow*
 *                                                                                    *
 *  hi2c: I2C to read outputs from gyroscope					    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void tumble(I2C_HandleTypeDef *hi2c1) {

	double k = gainConstant();
	double gyr_read[3], mag_read[3], intensity[3], magneticDipole[3], w[3], m[3], a;
	double maxMagneticDipole1 = 0.02359;
	double maxMagneticDipole2 = 0.03234;
	double maxIntensity1 = 0.14681;
	double maxIntensity2 = 0.1495;
	//shut down the magnetorquers to get a good reading of the magnetometers
	CurrentToCoil(hi2c1, 0);
	//read the data from the gyroscope
	AngularVelocity(hi2c1, w);
	//read the data from the magnetometer
	MagneticField(hi2c1,m);
	memcpy(gyr_read, w, 3);
	memcpy(mag_read, m, 3);
	//bdot algorithm
	cross(mag_read,gyr_read, magneticDipole);
	a = pow(norm(magneticDipole),2);
	magneticDipole[0] = k*magneticDipole[0]/a;
	magneticDipole[1] = k*magneticDipole[1]/a;
	magneticDipole[2] = k*magneticDipole[2]/a;

	if(magneticDipole[0]>0){
		intensity[0] = (magneticDipole[0]*maxIntensity1)/maxMagneticDipole1;
	}else{
		intensity[0] = (magneticDipole[0]*maxIntensity2)/maxMagneticDipole2;
	}

	intensity[1] = (magneticDipole[1]*maxIntensity2)/maxMagneticDipole2;
	intensity[2] = (magneticDipole[2]*maxIntensity2)/maxMagneticDipole2;
	//pass the current to the magnetorquers
	CurrentToCoil(hi2c1, intensity);


}
/****************************************************************************/
/* AngularVelocity: This function read the data from the gyroscope 			*/
/****************************************************************************/
void AngularVelocity(I2C_HandleTypeDef *hi2c1, double *w){
	uint8_t gx[2];
	uint8_t gy[2];
	uint8_t gz[2];
	double s = 131;
	gyro_aux gyrox;
	HAL_I2C_Master_Transmit(hi2c1, 0x68<<1, 0x43, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x68<<1, gx, 2, 1000);
	gyrox.gx_h = (double)((uint8_t)gx[0]<<8|gx[1]); //Comprobar que els valors tenen sentit (la conversió)
	HAL_I2C_Master_Transmit(hi2c1, 0x68<<1, 0x45, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x68<<1, gy, 2, 1000);
	gyrox.gy_h = (double)((uint8_t)gy[0]<<8|gy[1]);
	HAL_I2C_Master_Transmit(hi2c1, 0x68<<1, 0x47, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x68<<1, gz, 2, 1000);
	gyrox.gz_h = (double)((uint8_t)gz[0]<<8|gz[1]);
	//After we have readed the data from the gyro we have to divide by 131
	//Check the datasheet for more information
	w[0] = gyrox.gx_h/s;
	w[1] = gyrox.gy_h/s;
	w[2] = gyrox.gz_h/s;

}
/**************************************************************************************
 *                                                                                    *
 * Function:  readPhotodiodes                                                 		  *
 * --------------------                                                               *
 * Obtains the output values from all the photodiodes, varying the selectors		  *
 * of the multiplexor (GPIOs PA11 and PA12) 										  *
 *                                                                                    *
 *  hadc: ADC to read outputs from the photodiodes				    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void readPhotodiodes(ADC_HandleTypeDef *hadc, uint32_t *photoData) { // I think the four ADC should be passed as parameters

	uint32_t data[6];
	//conversion value, we have 3.3 volts and 14 bits of ADC
	double conversionValue = 3.3/pow(2,14);
	//Side 1
	//choose the channel we want to read data from
	HAL_ADC_ConfigChannel(hadc, 0);
	HAL_ADC_Start(hadc);
	//poll the ADC
	HAL_ADC_PollForConversion(hadc, 1000);
	//get the data from the ADC channel
	data[0] = HAL_ADC_GetValue(hadc)*conversionValue;
	HAL_ADC_Stop(hadc);
	//Side 2
	//choose the channel we want to read data from
	HAL_ADC_ConfigChannel(hadc, 1);
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	//get the data from the ADC channel
	data[1] = HAL_ADC_GetValue(hadc)*conversionValue;
	HAL_ADC_Stop(hadc);
	//Side 3
	//choose the channel we want to read data from
	HAL_ADC_ConfigChannel(hadc, 2);
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	//get the data from the ADC channel
	data[2] = HAL_ADC_GetValue(hadc)*conversionValue;
	HAL_ADC_Stop(hadc);
	//Side 4
	//choose the channel we want to read data from
	HAL_ADC_ConfigChannel(hadc, 3);
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	//get the data from the ADC channel
	data[3] = HAL_ADC_GetValue(hadc)*conversionValue;
	HAL_ADC_Stop(hadc);
	//Side 5
	//choose the channel we want to read data from
	HAL_ADC_ConfigChannel(hadc, 4);
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	//get the data from the ADC channel
	data[4] = HAL_ADC_GetValue(hadc)*conversionValue;
	HAL_ADC_Stop(hadc);
	//Side 2
	//choose the channel we want to read data from
	HAL_ADC_ConfigChannel(hadc, 5);
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	//get the data from the ADC channel
	data[5] = HAL_ADC_GetValue(hadc)*conversionValue;
	HAL_ADC_Stop(hadc);

	//assign the data from the photoDiodes to the output vector
	photoData[0] = data[0];
	photoData[1] = data[1];
	photoData[2] = data[2];
	photoData[3] = data[3];
	photoData[4] = data[4];
	photoData[5] = data[5];




}

void sunVector(ADC_HandleTypeDef *hadc, uint32_t *sunvector){

	uint32_t *photodiodesData;
	//get the vector containing the data from the photodiodes
	readPhotodiodes(&hadc, &photodiodesData);
	//compute the sunvector
	//if the photodiodesData[0], which is the positive X side, is > than photodiodesData[3]
	//which is the negative X side, we obtain the value of the photodiodesData[0]
	//as the voltage received is much bigger and this will let us know the position of the sun
	if(photodiodesData[0]>photodiodesData[3]) sunvector[0] = photodiodesData[0];
	else sunvector[0] = -photodiodesData[3];
	if(photodiodesData[1]>photodiodesData[4]) sunvector[1] = photodiodesData[1];
	else sunvector[1] = -photodiodesData[4];
	if(photodiodesData[2]>photodiodesData[5]) sunvector[2] = photodiodesData[2];
	else sunvector[2] = -photodiodesData[5];

}


bool checkGyro(I2C_HandleTypeDef *hi2c1){

	double gyr_read[3], v[3];
	//get the data from the gyroscope
	AngularVelocity(hi2c1, v);
	memcpy(gyr_read, v, 3);
	//if the gyroscope values are less than 0.5 we will assume that the satellite has no angular velocity
	if( gyr_read[0]<=0.5 && gyr_read[1]<=0.5 && gyr_read[2]<=0.5 ){
		return true;
	}
	return false;
}

/****************************************************************************/
/* MagneticField: This function read the data from the magnetometer */
/****************************************************************************/
void MagneticField(I2C_HandleTypeDef *hi2c1, double *m){
	uint8_t x[2], y[2], z[2];
	double mx, my, mz;
	//Check the datasheet to see how the data comes
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x00, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, x[0], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x01, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, x[1], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x02, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, y[0], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x03, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, y[1], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x04, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, z[0], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x05, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, z[1], 1, 1000);
	mx = (double)((uint8_t)x[0]<<8|x[1]);
	my = (double)((uint8_t)y[0]<<8|y[1]);
	mz = (double)((uint8_t)z[0]<<8|z[1]);
	m[0] = mx;
	m[1] = my;
	m[2] = mz;

}


/*************************************************************************************/
/* CurrentToCoil: We use the intensity that we have calculated in the Bdot
*/
/* and we send it to the differents coils. We need to distinguish every coil */
/* LV1=+x LV2=+y LV3=-y LV4=-z LV5=+z LV6=-x
*/
/*************************************************************************************/
void CurrentToCoil(I2C_HandleTypeDef *hi2c1, double intensidad[3]){


	int auxCurrent[3];
	char *currentToApply, *data_1, *data_2, *signCurrent, address_x, address_y, address_z;
	char finalCurrent[16] = {1,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0};
	//conversion value, 1023 is the max value of intensity as the data is a 10 bits bus
	//0.150 is the max output intensity
	auxCurrent[0]=round(intensidad[0]*1023/0.150);
	auxCurrent[1]=round(intensidad[1]*1023/0.150);
	auxCurrent[2]=round(intensidad[2]*1023/0.150);
	uint8_t data[4] = {0x66, 0x00, 0x00, 0x90};


	if(sign(auxCurrent[0])<=0){

		// -x side
		signCurrent[0] = sign(auxCurrent[0])*auxCurrent[0];
		//convert the decimal number to binary
		decimal_to_binary(signCurrent[0], currentToApply);
		//address of the driver
		address_x = 0x75<<1;

	}else{
		// +x side
		//convert the decimal number to binary
		decimal_to_binary(auxCurrent[0], currentToApply);
		//address of the driver
		address_x= 0x70<<1;
	}
	for(int i=0; i<10;i++){
		finalCurrent[i+2] = currentToApply[i];
		//assign the first 10 bits to the first data bus
		data_1[i] = finalCurrent[i];

	}
	for(int j=0; j<4; j++){
		//assign the last 4 bits to the second data bus
		data_2[j] = finalCurrent[j+8];
	}
	data[1] = (uint8_t)*data_1;
	data[2] = (uint8_t)*data_2;
	HAL_I2C_Master_Transmit_DMA(hi2c1, address_x, data, 4);


	if(sign(auxCurrent[1])<=0){

		// -y side
		signCurrent[1] = sign(auxCurrent[1])*auxCurrent[1];
		//convert the decimal number to binary
		decimal_to_binary(signCurrent[1], currentToApply);
		//address of the driver
		address_y = 0x72<<1;

	}else{
		// +y side
		//convert the decimal number to binary
		decimal_to_binary(auxCurrent[1], currentToApply);
		//address of the driver
		address_y = 0x71<<1;
	}
	for(int i=0; i<10;i++){
		//assign the first 10 bits to the first data bus
		finalCurrent[i+2] = currentToApply[i];
		data_1[i] = finalCurrent[i];

	}
	for(int j=0; j<4; j++){
		//assign the last 4 bits to the second data bus
		data_2[j] = finalCurrent[j+8];
	}
	data[1] = (uint8_t)*data_1;
	data[2] = (uint8_t)*data_2;
	HAL_I2C_Master_Transmit_DMA(hi2c1, address_y, data, 4);


	if(sign(auxCurrent[2])<=0){

		// -z side
		signCurrent[1] = sign(auxCurrent[2])*auxCurrent[2];
		//convert the decimal number to binary
		decimal_to_binary(signCurrent[2], currentToApply);
		//address of the driver
		address_z = 0x73<<1;

	}else{
		// +z side
		//convert the decimal number to binary
		decimal_to_binary(auxCurrent[2], currentToApply);
		//address of the driver
		address_z = 0x74<<1;
	}
	for(int i=0; i<10;i++){
		//assign the first 10 bits to the first data bus
		finalCurrent[i+2] = currentToApply[i];
		data_1[i] = finalCurrent[i];

	}
	for(int j=0; j<4; j++){
		//assign the last 4 bits to the second data bus
		data_2[j] = finalCurrent[j+8];
	}
	data[1] = (uint8_t)*data_1;
	data[2] = (uint8_t)*data_2;

	HAL_I2C_Master_Transmit_DMA(hi2c1, address_z, data, 4);

}




void sensorData(I2C_HandleTypeDef *hi2c1, ADC_HandleTypeDef *hadc, mag_data *magData, gyro_data *gyroData, sun_vector *sunvector){

	double w[3], m[3];
	uint32_t *data;
	CurrentToCoil(hi2c1, 0);
	//get data from gyroscope
	AngularVelocity(hi2c1, w);
	//get data from magnetometer
	MagneticField(hi2c1, m);
	//get data from the photodiodes
	sunVector(hadc, data);
	//save the values from the gyroscope
	gyroData->gx = w[0];
	gyroData->gy = w[1];
	gyroData->gz = w[2];
	//save the values from the magnetometer
	magData->mx = m[0];
	magData->my = m[1];
	magData->mz = m[2];
	//save the values from the photodiodes
	sunvector->x = data[0];
	sunvector->y = data[1];
	sunvector->z = data[2];

}




void nadir_algorithm(I2C_HandleTypeDef *hi2c1, ControlValues *control, float dtime, float *q_est, float *r_eci, float *v_eci)
{
    ControlData	control_data;
    CalibSensorsData  calib_sens_data;
	float maxMagneticDipole1 = 0.02359;
	float maxMagneticDipole2 = 0.03234;
	float maxIntensity1 = 0.14681;
	float maxIntensity2 = 0.1495;
    volatile float angle_target[2] = {0.0f,0.0f}, angle_error[3], angle_der[2];
    volatile float w_target[3], w_error[3], w_der[3], torque_ideal[3], kp_w[3], mag_moment[3], w_actual[3];
    float  mtq_c_t_calibration[3], norm, norm2, euler[3],q_e2o[3], mag_read[3], intensity[3];
    static float w_error_prev[3], angle_error_prev[3] = {0.0f,0.0f,0.0f};

    poseci2Quat(r_eci, v_eci, q_e2o);
    estimated_quat2euler(euler, q_est, q_e2o);

    mag_read[0] = calib_sens_data.mag_cal1[0]*1E-9;
    mag_read[1] = calib_sens_data.mag_cal1[1]*1E-9;
    mag_read[2] = calib_sens_data.mag_cal1[2]*1E-9;

	angle_error[0] = euler[2] - angle_target[0];
	angle_error[1] = euler[1] - angle_target[1];
	angle_error[2] = 0;
	if(angle_error_prev[2] == 0){
		angle_error_prev[2] = 1;
	} else {
		angle_der[0] = (angle_error[0] - angle_error_prev[0])/dtime;
		angle_der[1] = (angle_error[1] - angle_error_prev[1])/dtime;
		w_target[0] = control->nadir_kp_angle[0]*angle_error[0] + control->nadir_kd_angle[0]*angle_der[0];
		w_target[1] = control->nadir_kp_angle[1]*angle_error[1] + control->nadir_kd_angle[1]*angle_der[1];
		w_target[2] = 0;

		w_actual[0] = calib_sens_data.gyro_cal[0];
		w_actual[1] = calib_sens_data.gyro_cal[1];
		w_actual[2] = calib_sens_data.gyro_cal[2];

		if((w_actual[2]) >40 || (w_actual[2])<-40) {
			kp_w[0] = 1E-3;
			kp_w[1] = 1E-3;
			kp_w[2] = 9E-5;
		} else {
			kp_w[0] = control->nadir_kp_w[0];
			kp_w[1] = control->nadir_kp_w[1];
			kp_w[2] = control->nadir_kp_w[2];
		}

		w_error[0] = w_target[0] - w_actual[0];
		w_error[1] = w_target[1] - w_actual[1];
		w_error[2] = w_target[2] - w_actual[2];
		w_der[0] = (w_error[0] - w_error_prev[0])/dtime;
		w_der[1] = (w_error[1] - w_error_prev[1])/dtime;
		w_der[2] = (w_error[2] - w_error_prev[2])/dtime;

		torque_ideal[0] = -(kp_w[0]*w_error[0] + control->nadir_kd_w[0]*w_der[0]);
		torque_ideal[1] = -(kp_w[1]*w_error[1] + control->nadir_kd_w[1]*w_der[1]);
		torque_ideal[2] = -(kp_w[2]*w_error[2] + control->nadir_kd_w[2]*w_der[2]);


		cross_product(torque_ideal, mag_read, mag_moment);
		norm = float_norm(mag_read);
		norm2 = 1/(norm*norm);

		mag_moment[0] = mag_moment[0]*norm2;
		mag_moment[1] = mag_moment[1]*norm2;
		mag_moment[2] = mag_moment[2]*norm2;

		if(mag_moment[0]>0){
			intensity[0] = (mag_moment[0]*maxIntensity1)/maxMagneticDipole1;
		}else{
			intensity[0] = (mag_moment[0]*maxIntensity2)/maxMagneticDipole2;
		}

		intensity[1] = (mag_moment[1]*maxIntensity2)/maxMagneticDipole2;
		intensity[2] = (mag_moment[2]*maxIntensity2)/maxMagneticDipole2;

		CurrentToCoil(hi2c1, intensity);


		control_data.quaternion_est[0] = q_est[0];
		control_data.quaternion_est[1] = q_est[1];
		control_data.quaternion_est[2] = q_est[2];
		control_data.quaternion_est[3] = q_est[3];

		w_error_prev[0] = w_error[0];
		w_error_prev[1] = w_error[1];
		w_error_prev[2] = w_error[2];
	}
	angle_error_prev[0]=angle_error[0];
	angle_error_prev[1]=angle_error[1];

}

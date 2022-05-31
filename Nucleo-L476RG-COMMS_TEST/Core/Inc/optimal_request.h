/*
 * optimal_request.h
 *
 *  Created on: 31 may. 2022
 *      Author: Usuari
 */

#ifndef INC_OPTIMAL_REQUEST_H_
#define INC_OPTIMAL_REQUEST_H_

typedef struct DeterminationData {
    float igrf_eci[3];
    float sun_pos_eci[3];
}DeterminationData;

typedef struct CalibSensorsData {
    float mag_cal1[3];
    float sun_vec_cal[3];
    float gyro_cal[3];
}CalibSensorsData;

typedef struct ControlValues {
    float sun_var;
    float gyro_var;
    float mag_var;
    float nadir_kd_w[3];
    float nadir_kp_w[3];
    float nadir_kd_angle[2];
    float nadir_kp_angle[2];
}ControlValues;


void optimal_request(ControlValues *control, float *q_est);

#endif /* INC_OPTIMAL_REQUEST_H_ */

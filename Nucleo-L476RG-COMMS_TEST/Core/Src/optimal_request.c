/*
 * optimal_request.c
 *
 *  Created on: 16 may. 2022
 *      Author: jose_
 */
#include "matrix_utils.h"
#include "optimal_request.h"

void optimal_request(ControlValues *control, float *q_est)
{
    DeterminationData   det_data;
    CalibSensorsData    calib_sens_data;

	float sensor_weight[2],b[3][2],r[3][2], db[3][2], eigenv[4], q_p[4];
	float temp[3], tempb[3], tempr[3], z[3] = {0.0f,0.0f,0.0f}, zb[3] = {0.0f,0.0f,0.0f}, ze[3] = {0.0f,0.0f,0.0f};
	Matrix3x3 B_mat, B_mat_t, S_mat, aux, Bb_mat, Bb_mat_t, Sb_mat, Be_mat, Be_mat_t, Se_mat, aux_mat;
	Matrix4x4 dK_n, V_n_n, W_n, Q_n, R_n, phi_n, K_n_n, K_n_n1, phi_n_trans;
	Matrix4x4 P_n_n, P_n_n1, Q_n1, aux1_4x4, aux2_4x4;
	float sigma, kappab, kappae, dm_n, m_n, rho_n;
	float size_a, aux7, aux8,c,s, n[3];
	static Matrix4x4 K_n1_n1, P_n1_n1;
	static float m_n1;
	static uint32_t previous_time;
	float dtime, mag_read[3];
	uint8_t eclipse = 0;
	static uint8_t iteration = 0;
	float w_norm;
	Matrix4x3 P;
	uint32_t actual_time;

	mag_read[0] = calib_sens_data.mag_cal1[0]*1E-9;
	mag_read[1] = calib_sens_data.mag_cal1[1]*1E-9;
	mag_read[2] = calib_sens_data.mag_cal1[2]*1E-9;
	if (iteration == 0) {
		previous_time = OS_Tick_GetCount();
		iteration++;
	} else {
		if(eclipse == 1) {
			sensor_weight[0] = 1.0f;
			sensor_weight[1] = 0.0f;
		} else {
			sensor_weight[0] = 1.0f/control->mag_var;
			sensor_weight[1] = 1.0f/control->sun_var;
		}

		actual_time = OS_Tick_GetCount();
		dtime = (actual_time*1.0f-previous_time*1.0f);
		previous_time = OS_Tick_GetCount();
		size_a = sizeof(sensor_weight)/sizeof(sensor_weight[0]);
		aux7 = float_norm(mag_read);
		aux8 = float_norm(calib_sens_data.sun_vec_cal);
		if(aux7==0) {
			b[0][0] = 0;
			b[1][0] = 0;
			b[2][0] = 0;
		} else {
			b[0][0] = mag_read[0]/aux7;
			b[1][0] = mag_read[1]/aux7;
			b[2][0] = mag_read[2]/aux7;
		}
		if(aux8==0) {
			b[0][1] = 0;
			b[1][1] = 0;
			b[2][1] = 0;
		} else {
			b[0][1] = calib_sens_data.sun_vec_cal[0]/aux8;
			b[1][1] = calib_sens_data.sun_vec_cal[1]/aux8;
			b[2][1] = calib_sens_data.sun_vec_cal[2]/aux8;
		}

		aux7 = float_norm(det_data.igrf_eci);
		aux8 = float_norm(det_data.sun_pos_eci);
		if(aux7==0) {
			r[0][0] = 0;
			r[1][0] = 0;
			r[2][0] = 0;
		} else {
			r[0][0] = det_data.igrf_eci[0]/aux7;
			r[1][0] = det_data.igrf_eci[1]/aux7;
			r[2][0] = det_data.igrf_eci[2]/aux7;
		}
		if(aux8==0) {
			r[0][1] = 0;
			r[1][1] = 0;
			r[2][1] = 0;
		} else {
			r[0][1] = det_data.sun_pos_eci[0]/aux8;
			r[1][1] = det_data.sun_pos_eci[1]/aux8;
			r[2][1] = det_data.sun_pos_eci[2]/aux8;
		}

		//Compute B
		memset(&B_mat, 0, sizeof(B_mat));
		for(int i = 0; i < size_a; i++) {
			B_mat.col1[0] = B_mat.col1[0]+sensor_weight[i]*b[0][i]*r[0][i];
			B_mat.col1[1] = B_mat.col1[1]+sensor_weight[i]*b[1][i]*r[0][i];
			B_mat.col1[2] = B_mat.col1[2]+sensor_weight[i]*b[2][i]*r[0][i];

			B_mat.col2[0] = B_mat.col2[0]+sensor_weight[i]*b[0][i]*r[1][i];
			B_mat.col2[1] = B_mat.col2[1]+sensor_weight[i]*b[1][i]*r[1][i];
			B_mat.col2[2] = B_mat.col2[2]+sensor_weight[i]*b[2][i]*r[1][i];

			B_mat.col3[0] = B_mat.col3[0]+sensor_weight[i]*b[0][i]*r[2][i];
			B_mat.col3[1] = B_mat.col3[1]+sensor_weight[i]*b[1][i]*r[2][i];
			B_mat.col3[2] = B_mat.col3[2]+sensor_weight[i]*b[2][i]*r[2][i];
		}
		//Compute sigma
		sigma = matrix_trace(&B_mat);
		//Compute S
		matrix_trans3x3(&B_mat,&B_mat_t);
		matrix_sum3x3(&B_mat,&B_mat_t,&S_mat,1,1);
		//Compute z
		for(int i = 0; i < size_a; i++) {
			tempb[0] = b[0][i];
			tempb[1] = b[1][i];
			tempb[2] = b[2][i];
			tempr[0] = r[0][i];
			tempr[1] = r[1][i];
			tempr[2] = r[2][i];
			cross_product(tempb,tempr,temp);
			z[0] = z[0]+sensor_weight[i]*temp[0];
			z[1] = z[1]+sensor_weight[i]*temp[1];
			z[2] = z[2]+sensor_weight[i]*temp[2];
		}
		//Compute dK_0
		eye3x3(&aux,-sigma);
		matrix_sum3x3(&S_mat,&aux,&aux,1,1);
		concat_matrix(&dK_n,&aux,z,sigma);
		// dK_n validated

		//Define V
		db[0][0] = sqrt(control->mag_var);
		db[1][0] = sqrt(control->mag_var);
		db[2][0] = sqrt(control->mag_var);

		db[0][1] = sqrt(control->sun_var);
		db[1][1] = sqrt(control->sun_var);
		db[2][1] = sqrt(control->sun_var);
		//Compute Bb
		memset(&Bb_mat,0,sizeof(Bb_mat));
		for(int i = 0; i < size_a; i++) {
			Bb_mat.col1[0] = Bb_mat.col1[0]+sensor_weight[i]*db[0][i]*r[0][i];
			Bb_mat.col1[1] = Bb_mat.col1[1]+sensor_weight[i]*db[1][i]*r[0][i];
			Bb_mat.col1[2] = Bb_mat.col1[2]+sensor_weight[i]*db[2][i]*r[0][i];

			Bb_mat.col2[0] = Bb_mat.col2[0]+sensor_weight[i]*db[0][i]*r[1][i];
			Bb_mat.col2[1] = Bb_mat.col2[1]+sensor_weight[i]*db[1][i]*r[1][i];
			Bb_mat.col2[2] = Bb_mat.col2[2]+sensor_weight[i]*db[2][i]*r[1][i];

			Bb_mat.col3[0] = Bb_mat.col3[0]+sensor_weight[i]*db[0][i]*r[2][i];
			Bb_mat.col3[1] = Bb_mat.col3[1]+sensor_weight[i]*db[1][i]*r[2][i];
			Bb_mat.col3[2] = Bb_mat.col3[2]+sensor_weight[i]*db[2][i]*r[2][i];
		}
		//compute kappab
		kappab = matrix_trace(&Bb_mat);
		//Compute Sb
		matrix_trans3x3(&Bb_mat,&Bb_mat_t);
		matrix_sum3x3(&Bb_mat,&Bb_mat_t,&Sb_mat,1,1);
		//Compute zb
		for(int i = 0; i < size_a; i++) {
			tempb[0] = db[0][i];
			tempb[1] = db[1][i];
			tempb[2] = db[2][i];
			tempr[0] = r[0][i];
			tempr[1] = r[1][i];
			tempr[2] = r[2][i];
			cross_product(tempb,tempr,temp);
			zb[0] = zb[0]+sensor_weight[i]*temp[0];
			zb[1] = zb[1]+sensor_weight[i]*temp[1];
			zb[2] = zb[2]+sensor_weight[i]*temp[2];
		}
		//Compute V
		eye3x3(&aux,-kappab);
		matrix_sum3x3(&Sb_mat,&aux,&aux,1,1);
		concat_matrix(&V_n_n,&aux,zb,kappab);

		//Compute R_n
		cov4x4(&V_n_n,&R_n);
		//Define W
		//Compute Be
		aux7 = sqrt(control->gyro_var);
		aux_mat.col1[0] = 0;
		aux_mat.col1[1] = aux7;
		aux_mat.col1[2] = -aux7;

		aux_mat.col2[0] = -aux7;
		aux_mat.col2[1] = 0;
		aux_mat.col2[2] = aux7;

		aux_mat.col3[0] = aux7;
		aux_mat.col3[1] = -aux7;
		aux_mat.col3[2] = 0;

		matrix_prod3x3(&aux_mat,&B_mat,&Be_mat);

		//Compute kappae
		kappae = matrix_trace(&Be_mat);
		//Compute Se
		matrix_trans3x3(&Be_mat,&Be_mat_t);
		matrix_sum3x3(&Be_mat,&Be_mat_t,&Se_mat,1,1);

		//Compute Ze
		matrix_sum3x3(&Be_mat_t,&Be_mat,&Be_mat,1,-1);
		ze[0] = Be_mat.col2[2];
		ze[1] = Be_mat.col3[0];
		ze[2] = Be_mat.col1[1];

		//Compute W_n
		eye3x3(&aux,-kappae);
		matrix_sum3x3(&Se_mat,&aux,&aux,1,1);
		concat_matrix(&W_n,&aux,ze,kappae);

		W_n.col1[0] *= dtime;
		W_n.col2[0] *= dtime;
		W_n.col3[0] *= dtime;
		W_n.col4[0] *= dtime;

		W_n.col1[1] *= dtime;
		W_n.col2[1] *= dtime;
		W_n.col3[1] *= dtime;
		W_n.col4[1] *= dtime;

		W_n.col1[2] *= dtime;
		W_n.col2[2] *= dtime;
		W_n.col3[2] *= dtime;
		W_n.col4[2] *= dtime;

		W_n.col1[3] *= dtime;
		W_n.col2[3] *= dtime;
		W_n.col3[3] *= dtime;
		W_n.col4[3] *= dtime;

		//Compute Q_n
		cov4x4(&W_n,&Q_n);

		//Compute m
		dm_n = 0;
		for(int i = 0; i < size_a; i++) {
			dm_n += sensor_weight[i];
		}

		w_norm = float_norm(calib_sens_data.gyro_cal);
		c = cos(0.5*w_norm*dtime);
		s = sin(0.5*w_norm*dtime);
		if(w_norm == 0){
			n[0] = 0;
			n[1] = 0;
			n[2] = 0;
		} else {
			n[0] = calib_sens_data.gyro_cal[0]/w_norm;
			n[1] = calib_sens_data.gyro_cal[1]/w_norm;
			n[2] = calib_sens_data.gyro_cal[2]/w_norm;
		}
		phi_n.col1[0] = c;
		phi_n.col1[1] = -n[2]*s;
		phi_n.col1[2] = n[1]*s;
		phi_n.col1[3] = -n[0]*s;

		phi_n.col2[0] = n[2]*s;
		phi_n.col2[1] = c;
		phi_n.col2[2] = -n[0]*s;
		phi_n.col2[3] = -n[1]*s;

		phi_n.col3[0] = -n[1]*s;
		phi_n.col3[1] = n[0]*s;
		phi_n.col3[2] = c;
		phi_n.col3[3] = -n[2]*s;

		phi_n.col4[0] = n[0]*s;
		phi_n.col4[1] = n[1]*s;
		phi_n.col4[2] = n[2]*s;
		phi_n.col4[3] = c;

		//Transition matrix
		if(iteration == 1) {
			K_n_n = dK_n;
			matrix_norm4x4(&K_n_n);
			P_n_n = R_n;
			m_n = 1;

			iteration++;
		} else {
			//Time update
			matrix_prod4x4(&phi_n,&K_n1_n1,&aux1_4x4);
			matrix_trans4x4(&phi_n,&phi_n_trans);
			matrix_prod4x4(&aux1_4x4,&phi_n_trans,&K_n_n1);

			matrix_prod4x4(&phi_n,&P_n1_n1,&aux1_4x4);
			matrix_prod4x4(&aux1_4x4,&phi_n_trans,&aux2_4x4);
			matrix_sum4x4(&aux2_4x4,&Q_n1,&P_n_n1,1.0,1.0);

			rho_n = m_n1*m_n1*matrix_trace4x4(&P_n_n1)/(m_n1*m_n1*matrix_trace4x4(&P_n_n1)+dm_n*dm_n*matrix_trace4x4(&R_n));
			if(eclipse == 1){
				rho_n = 0;
			} else {
				rho_n = 1;
			}
			//m_n
			m_n = (1.0-rho_n)*m_n1 + rho_n*dm_n;
			//K
			matrix_sum4x4(&K_n_n1,&dK_n,&K_n_n,(1-rho_n),rho_n);
			matrix_norm4x4(&K_n_n);

			//P
			matrix_sum4x4(&P_n_n1,&R_n,&P_n_n,pow(1-rho_n,2),pow(rho_n,2));
			matrix_norm4x4(&P_n_n);

		}
		//Update state for the next iteration
		K_n1_n1 = K_n_n;
		m_n1 = m_n;
		P_n1_n1 = P_n_n;
		Q_n1 = Q_n;

		//Compute lambda_max
		aux2_4x4.col1[0] = K_n_n.col1[0]*1E12;
		aux2_4x4.col1[1] = K_n_n.col1[1]*1E12;
		aux2_4x4.col1[2] = K_n_n.col1[2]*1E12;
		aux2_4x4.col1[3] = K_n_n.col1[3]*1E12;

		aux2_4x4.col2[0] = K_n_n.col2[0]*1E12;
		aux2_4x4.col2[1] = K_n_n.col2[1]*1E12;
		aux2_4x4.col2[2] = K_n_n.col2[2]*1E12;
		aux2_4x4.col2[3] = K_n_n.col2[3]*1E12;

		aux2_4x4.col3[0] = K_n_n.col3[0]*1E12;
		aux2_4x4.col3[1] = K_n_n.col3[1]*1E12;
		aux2_4x4.col3[2] = K_n_n.col3[2]*1E12;
		aux2_4x4.col3[3] = K_n_n.col3[3]*1E12;

		aux2_4x4.col4[0] = K_n_n.col4[0]*1E12;
		aux2_4x4.col4[1] = K_n_n.col4[1]*1E12;
		aux2_4x4.col4[2] = K_n_n.col4[2]*1E12;
		aux2_4x4.col4[3] = K_n_n.col4[3]*1E12;

		fix_and_sub(&aux1_4x4,&aux2_4x4);
		matrix_sum4x4(&aux2_4x4,&aux1_4x4,&K_n_n,1.0,-1.0);

		K_n_n.col1[0] = K_n_n.col1[0]*1E-12;
		K_n_n.col1[1] = K_n_n.col1[1]*1E-12;
		K_n_n.col1[2] = K_n_n.col1[2]*1E-12;
		K_n_n.col1[3] = K_n_n.col1[3]*1E-12;

		K_n_n.col2[0] = K_n_n.col2[0]*1E-12;
		K_n_n.col2[1] = K_n_n.col2[1]*1E-12;
		K_n_n.col2[2] = K_n_n.col2[2]*1E-12;
		K_n_n.col2[3] = K_n_n.col2[3]*1E-12;

		K_n_n.col3[0] = K_n_n.col3[0]*1E-12;
		K_n_n.col3[1] = K_n_n.col3[1]*1E-12;
		K_n_n.col3[2] = K_n_n.col3[2]*1E-12;
		K_n_n.col3[3] = K_n_n.col3[3]*1E-12;

		K_n_n.col4[0] = K_n_n.col4[0]*1E-12;
		K_n_n.col4[1] = K_n_n.col4[1]*1E-12;
		K_n_n.col4[2] = K_n_n.col4[2]*1E-12;
		K_n_n.col4[3] = K_n_n.col4[3]*1E-12;

		eigen4x4(&K_n_n, eigenv);

		if (eclipse == 1) {
			quaternion_norm(q_est);
			P.col1[0] = -q_est[1];
			P.col1[1] = q_est[0];
			P.col1[2] = q_est[3];
			P.col1[3] = -q_est[2];

			P.col2[0] = -q_est[2];
			P.col2[1] = -q_est[3];
			P.col2[2] = q_est[0];
			P.col2[3] = q_est[1];

			P.col3[0] = -q_est[1];
			P.col3[1] = q_est[0];
			P.col3[2] = q_est[3];
			P.col3[3] = -q_est[2];

			q_p[0] = 0.5*(P.col1[0]*calib_sens_data.gyro_cal[0]+P.col2[0]*calib_sens_data.gyro_cal[1]+P.col3[0]*calib_sens_data.gyro_cal[2]);
			q_p[1] = 0.5*(P.col1[1]*calib_sens_data.gyro_cal[0]+P.col2[1]*calib_sens_data.gyro_cal[1]+P.col3[1]*calib_sens_data.gyro_cal[2]);
			q_p[2] = 0.5*(P.col1[2]*calib_sens_data.gyro_cal[0]+P.col2[2]*calib_sens_data.gyro_cal[1]+P.col3[2]*calib_sens_data.gyro_cal[2]);
			q_p[3] = 0.5*(P.col1[3]*calib_sens_data.gyro_cal[0]+P.col2[3]*calib_sens_data.gyro_cal[1]+P.col3[3]*calib_sens_data.gyro_cal[2]);

			q_est[0] = q_est[0] * q_p[0];
			q_est[1] = q_est[1] * q_p[1];
			q_est[2] = q_est[2] * q_p[2];
			q_est[3] = q_est[3] * q_p[3];

		} else {
			q_est[0] = eigenv[3];
			q_est[1] = -eigenv[0];
			q_est[2] = -eigenv[1];
			q_est[3] = -eigenv[2];
		}
	}
}

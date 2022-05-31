

#ifndef INC_MATRIX_UTILS_H_
#define INC_MATRIX_UTILS_H_

/*
 * matrix_utils.h
 *
 *  Created on: 16 may. 2022
 *      Author: jose_
 */

//#include <lapacke.h>
//#include <cblas-netlib.h>
//#include <cblas.h>
#include <math.h>
#include <stdint.h>

/*typedef struct float {
    float w;
    float x;
    float y;
    float z;
}float;*/

typedef struct Matrix3x3 {
    float col1[3];
    float col2[3];
    float col3[3];
}Matrix3x3;

typedef struct Matrix4x4 {
    float col1[4];
    float col2[4];
    float col3[4];
    float col4[4];
}Matrix4x4;

typedef struct Matrix4x3 {
    float col1[4];
    float col2[4];
    float col3[4];
}Matrix4x3;

void cross_product(float *vec_a, float *vec_b, float *res);
float dot_product(float *vec_a, float *vec_b);
float float_norm(float *vec);
float float_norm4(float *vec);
//void quaternion_norm(float *q);
void quat_to_dcm(float *q, Matrix3x3 *m);
float matrix_trace(Matrix3x3 *m);
float matrix_trace4x4(Matrix4x4 *m);
void matrix_sum3x3(Matrix3x3 *m1, Matrix3x3 *m2, Matrix3x3 *res, float alpha, float beta);
void matrix_sum4x4(Matrix4x4 *m1, Matrix4x4 *m2, Matrix4x4 *res, float alpha, float beta);
void matrix_prod3x3(Matrix3x3 *m1, Matrix3x3 *m2, Matrix3x3 *res);
void matrix_prod4x4(Matrix4x4 *m1, Matrix4x4 *m2, Matrix4x4 *res);
float matrix_determinant3x3(Matrix3x3 *m);
void matrix_trans3x3(Matrix3x3 *m, Matrix3x3 *res);
void matrix_trans4x4(Matrix4x4 *m, Matrix4x4 *res);
void array_to_mat3x3(Matrix3x3 *m, float *v);
void array_to_mat4x4(Matrix4x4 *m, float *v);
void mat3x3_to_array(Matrix3x3 *m, float *v);
void mat4x4_to_array(Matrix4x4 *m, float *v);
void cov4x4(Matrix4x4 *m, Matrix4x4 *res);
float eigenvalues4x4(Matrix4x4 *m, float *eigvector);
float eigen4x4(Matrix4x4 *m, float *res);
float matrix_norm4x4(Matrix4x4 *m);
void concat_matrix(Matrix4x4 *res, Matrix3x3 *m, float v[3], float e);
void eye3x3(Matrix3x3 *m, float alpha);
void eye4x4(Matrix4x4 *m, float alpha);
void fix_and_sub(Matrix4x4 *res, Matrix4x4 *m);
void quaternion_norm(float *q);
void estimated_quat2euler(float *euler, float *q_e2b, float *q_e2o);
void quaternion_mult(float *q1, float *q2, float *q3);
void unwind(float *q, float *q_prev);
void quat2angle(float *q, float *euler);
void poseci2Quat(float *r_eci, float *v_eci, float *q);
void matrix2Quat(Matrix3x3 *A, float *q);



#endif /* INC_MATRIX_UTILS_H_ */

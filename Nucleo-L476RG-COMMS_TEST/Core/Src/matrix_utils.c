/*
 * matrix_utils.c
 *
 *  Created on: 16 may. 2022
 *      Author: jose_
 */


#include "matrix_utils.h"
#include <string.h>

void cross_product(float *vec_a, float *vec_b, float *res)
{
    res[0] = vec_a[1] * vec_b[2] - vec_a[2] * vec_b[1];
    res[1] = vec_a[2] * vec_b[0] - vec_a[0] * vec_b[2];
    res[2] = vec_a[0] * vec_b[1] - vec_a[1] * vec_b[0];
}

float dot_product(float *vec_a, float *vec_b)
{
    return vec_a[0]*vec_b[0]+vec_a[1]*vec_b[1]+vec_a[2]*vec_b[2];
}

float float_norm(float *vec)
{
    return (sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2])));
}

float float_norm4(float *vec)
{
    return (sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]) + (vec[3] * vec[3])));
}


void quat_to_dcm(float *q, Matrix3x3 *m)
{
    quaternion_norm(q);
    m->col1[0] = (q[1]*q[1]) - (q[2]*q[2]) + (q[3]*q[3]) + (q[3]*q[3]);
    m->col1[1] = (2*q[1]*q[2]) - (2*q[3]*q[0]);
    m->col1[2] = (2*q[1]*q[3]) + (2*q[2]*q[0]);

    m->col2[0] = (2*q[1]*q[2]) + (2*q[3]*q[0]);
    m->col2[1] = -(q[1]*q[1]) + (q[2]*q[2]) - (q[3]*q[3]) + (q[3]*q[3]);
    m->col2[2] = (2*q[2]*q[3]) - (2*q[1]*q[0]);

    m->col3[0] = (2*q[1]*q[3]) - (2*q[2]*q[0]);
    m->col3[1] = (2*q[2]*q[3]) + (2*q[1]*q[0]);
    m->col3[2] = -(q[1]*q[1]) - (q[2]*q[2]) + (q[3]*q[3]) + (q[3]*q[3]);
}

float matrix_trace(Matrix3x3 *m)
{
    return m->col1[0]+m->col2[1]+m->col3[2];
}

float matrix_trace4x4(Matrix4x4 *m)
{
    return m->col1[0]+m->col2[1]+m->col3[2]+m->col4[3];
}

void matrix_sum3x3(Matrix3x3 *m1, Matrix3x3 *m2, Matrix3x3 *res, float alpha, float beta)
{
    res->col1[0] = alpha*m1->col1[0] + beta*m2->col1[0];
    res->col1[1] = alpha*m1->col1[1] + beta*m2->col1[1];
    res->col1[2] = alpha*m1->col1[2] + beta*m2->col1[2];

    res->col2[0] = alpha*m1->col2[0] + beta*m2->col2[0];
    res->col2[1] = alpha*m1->col2[1] + beta*m2->col2[1];
    res->col2[2] = alpha*m1->col2[2] + beta*m2->col2[2];

    res->col3[0] = alpha*m1->col3[0] + beta*m2->col3[0];
    res->col3[1] = alpha*m1->col3[1] + beta*m2->col3[1];
    res->col3[2] = alpha*m1->col3[2] + beta*m2->col3[2];
}

void matrix_sum4x4(Matrix4x4 *m1, Matrix4x4 *m2, Matrix4x4 *res, float alpha, float beta)
{
    res->col1[0] = alpha*m1->col1[0] + beta*m2->col1[0];
    res->col1[1] = alpha*m1->col1[1] + beta*m2->col1[1];
    res->col1[2] = alpha*m1->col1[2] + beta*m2->col1[2];
    res->col1[3] = alpha*m1->col1[3] + beta*m2->col1[3];

    res->col2[0] = alpha*m1->col2[0] + beta*m2->col2[0];
    res->col2[1] = alpha*m1->col2[1] + beta*m2->col2[1];
    res->col2[2] = alpha*m1->col2[2] + beta*m2->col2[2];
    res->col2[3] = alpha*m1->col2[3] + beta*m2->col2[3];

    res->col3[0] = alpha*m1->col3[0] + beta*m2->col3[0];
    res->col3[1] = alpha*m1->col3[1] + beta*m2->col3[1];
    res->col3[2] = alpha*m1->col3[2] + beta*m2->col3[2];
    res->col3[3] = alpha*m1->col3[3] + beta*m2->col3[3];

    res->col4[0] = alpha*m1->col4[0] + beta*m2->col4[0];
    res->col4[1] = alpha*m1->col4[1] + beta*m2->col4[1];
    res->col4[2] = alpha*m1->col4[2] + beta*m2->col4[2];
    res->col4[3] = alpha*m1->col4[3] + beta*m2->col4[3];
}

void matrix_prod3x3(Matrix3x3 *m1, Matrix3x3 *m2, Matrix3x3 *res){
    res->col1[0] = m1->col1[0]*m2->col1[0]+m1->col2[0]*m2->col1[1]+m1->col3[0]*m2->col1[2];
    res->col1[1] = m1->col1[1]*m2->col1[0]+m1->col2[1]*m2->col1[1]+m1->col3[1]*m2->col1[2];
    res->col1[2] = m1->col1[2]*m2->col1[0]+m1->col2[2]*m2->col1[1]+m1->col3[2]*m2->col1[2];

    res->col2[0] = m1->col1[0]*m2->col2[0]+m1->col2[0]*m2->col2[1]+m1->col3[0]*m2->col2[2];
    res->col2[1] = m1->col1[1]*m2->col2[0]+m1->col2[1]*m2->col2[1]+m1->col3[1]*m2->col2[2];
    res->col2[2] = m1->col1[2]*m2->col2[0]+m1->col2[2]*m2->col2[1]+m1->col3[2]*m2->col2[2];

    res->col3[0] = m1->col1[0]*m2->col3[0]+m1->col2[0]*m2->col3[1]+m1->col3[0]*m2->col3[2];
    res->col3[1] = m1->col1[1]*m2->col3[0]+m1->col2[1]*m2->col3[1]+m1->col3[1]*m2->col3[2];
    res->col3[2] = m1->col1[2]*m2->col3[0]+m1->col2[2]*m2->col3[1]+m1->col3[2]*m2->col3[2];
}

void matrix_prod4x4(Matrix4x4 *m1, Matrix4x4 *m2, Matrix4x4 *res)
{
    res->col1[0] = m1->col1[0]*m2->col1[0]+m1->col2[0]*m2->col1[1]+m1->col3[0]*m2->col1[2]+m1->col4[0]*m2->col1[3];
    res->col1[1] = m1->col1[1]*m2->col1[0]+m1->col2[1]*m2->col1[1]+m1->col3[1]*m2->col1[2]+m1->col4[1]*m2->col1[3];
    res->col1[2] = m1->col1[2]*m2->col1[0]+m1->col2[2]*m2->col1[1]+m1->col3[2]*m2->col1[2]+m1->col4[2]*m2->col1[3];
    res->col1[3] = m1->col1[3]*m2->col1[0]+m1->col2[3]*m2->col1[1]+m1->col3[3]*m2->col1[2]+m1->col4[3]*m2->col1[3];

    res->col2[0] = m1->col1[0]*m2->col2[0]+m1->col2[0]*m2->col2[1]+m1->col3[0]*m2->col2[2]+m1->col4[0]*m2->col2[3];
    res->col2[1] = m1->col1[1]*m2->col2[0]+m1->col2[1]*m2->col2[1]+m1->col3[1]*m2->col2[2]+m1->col4[1]*m2->col2[3];
    res->col2[2] = m1->col1[2]*m2->col2[0]+m1->col2[2]*m2->col2[1]+m1->col3[2]*m2->col2[2]+m1->col4[2]*m2->col2[3];
    res->col2[3] = m1->col1[3]*m2->col2[0]+m1->col2[3]*m2->col2[1]+m1->col3[3]*m2->col2[2]+m1->col4[3]*m2->col2[3];

    res->col3[0] = m1->col1[0]*m2->col3[0]+m1->col2[0]*m2->col3[1]+m1->col3[0]*m2->col3[2]+m1->col4[0]*m2->col3[3];
    res->col3[1] = m1->col1[1]*m2->col3[0]+m1->col2[1]*m2->col3[1]+m1->col3[1]*m2->col3[2]+m1->col4[1]*m2->col3[3];
    res->col3[2] = m1->col1[2]*m2->col3[0]+m1->col2[2]*m2->col3[1]+m1->col3[2]*m2->col3[2]+m1->col4[2]*m2->col3[3];
    res->col3[3] = m1->col1[3]*m2->col3[0]+m1->col2[3]*m2->col3[1]+m1->col3[3]*m2->col3[2]+m1->col4[3]*m2->col3[3];

    res->col4[0] = m1->col1[0]*m2->col4[0]+m1->col2[0]*m2->col4[1]+m1->col3[0]*m2->col4[2]+m1->col4[0]*m2->col4[3];
    res->col4[1] = m1->col1[1]*m2->col4[0]+m1->col2[1]*m2->col4[1]+m1->col3[1]*m2->col4[2]+m1->col4[1]*m2->col4[3];
    res->col4[2] = m1->col1[2]*m2->col4[0]+m1->col2[2]*m2->col4[1]+m1->col3[2]*m2->col4[2]+m1->col4[2]*m2->col4[3];
    res->col4[3] = m1->col1[3]*m2->col4[0]+m1->col2[3]*m2->col4[1]+m1->col3[3]*m2->col4[2]+m1->col4[3]*m2->col4[3];
}

float matrix_determinant3x3(Matrix3x3 *m)
{

    float a,b,c,d,e,f,g,h,i;

    a = m->col1[0];
    b = m->col2[0];
    c = m->col3[0];
    d = m->col1[1];
    e = m->col2[1];
    f = m->col3[1];
    g = m->col1[2];
    h = m->col2[2];
    i = m->col3[2];

    return a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g);
}

void matrix_trans3x3(Matrix3x3 *m, Matrix3x3 *res)
{
    res->col1[0] = m->col1[0];
    res->col1[1] = m->col2[0];
    res->col1[2] = m->col3[0];

    res->col2[0] = m->col1[1];
    res->col2[1] = m->col2[1];
    res->col2[2] = m->col3[1];

    res->col3[0] = m->col1[2];
    res->col3[1] = m->col2[2];
    res->col3[2] = m->col3[2];
}

void matrix_trans4x4(Matrix4x4 *m, Matrix4x4 *res)
{
    res->col1[0] = m->col1[0];
    res->col1[1] = m->col2[0];
    res->col1[2] = m->col3[0];
    res->col1[3] = m->col4[0];

    res->col2[0] = m->col1[1];
    res->col2[1] = m->col2[1];
    res->col2[2] = m->col3[1];
    res->col2[3] = m->col4[1];

    res->col3[0] = m->col1[2];
    res->col3[1] = m->col2[2];
    res->col3[2] = m->col3[2];
    res->col3[3] = m->col4[2];

    res->col4[0] = m->col1[3];
    res->col4[1] = m->col2[3];
    res->col4[2] = m->col3[3];
    res->col4[3] = m->col4[3];
}

void array_to_mat3x3(Matrix3x3 *m, float *v)
{
    m->col1[0] = v[0];
    m->col2[0] = v[1];
    m->col3[0] = v[2];
    m->col1[1] = v[3];
    m->col2[1] = v[4];
    m->col3[1] = v[5];
    m->col1[2] = v[6];
    m->col2[2] = v[7];
    m->col3[2] = v[8];
}

void array_to_mat4x4(Matrix4x4 *m, float *v)
{
    m->col1[0] = v[0];
    m->col2[0] = v[1];
    m->col3[0] = v[2];
    m->col4[0] = v[3];
    m->col1[1] = v[4];
    m->col2[1] = v[5];
    m->col3[1] = v[6];
    m->col4[1] = v[7];
    m->col1[2] = v[8];
    m->col2[2] = v[9];
    m->col3[2] = v[10];
    m->col4[2] = v[11];
    m->col1[3] = v[12];
    m->col2[3] = v[13];
    m->col3[3] = v[14];
    m->col4[3] = v[15];
}

void mat3x3_to_array(Matrix3x3 *m, float *v)
{
    v[0] = m->col1[0];
    v[1] = m->col2[0];
    v[2] = m->col3[0];
    v[3] = m->col1[1];
    v[4] = m->col2[1];
    v[5] = m->col3[1];
    v[6] = m->col1[2];
    v[7] = m->col2[2];
    v[8] = m->col3[2];
}

void mat4x4_to_array(Matrix4x4 *m, float *v)
{
    v[0] = m->col1[0];
    v[1] = m->col2[0];
    v[2] = m->col3[0];
    v[3] = m->col4[0];
    v[4] = m->col1[1];
    v[5] = m->col2[1];
    v[6] = m->col3[1];
    v[7] = m->col4[1];
    v[8] = m->col1[2];
    v[9] = m->col2[2];
    v[10] = m->col3[2];
    v[11] = m->col4[2];
    v[12] = m->col1[3];
    v[13] = m->col2[3];
    v[14] = m->col3[3];
    v[15] = m->col4[3];
}


void cov4x4(Matrix4x4 *m, Matrix4x4 *res)
{
    // C = 1/n*X'*X-1/n²*(1'*X)'*(1'*X)
    Matrix4x4 ones,aux1,aux2,aux3;
    //float mat[16],aux[16] ={0} ,ones[16] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    int n = 4;
    float alpha = -1.0/n;
    float beta = 1.0/(n-1);

    ones.col1[0] = 1;
    ones.col1[1] = 1;
    ones.col1[2] = 1;
    ones.col1[3] = 1;

    ones.col2[0] = 1;
    ones.col2[1] = 1;
    ones.col2[2] = 1;
    ones.col2[3] = 1;

    ones.col3[0] = 1;
    ones.col3[1] = 1;
    ones.col3[2] = 1;
    ones.col3[3] = 1;

    ones.col4[0] = 1;
    ones.col4[1] = 1;
    ones.col4[2] = 1;
    ones.col4[3] = 1;

    matrix_prod4x4(&ones,m,&aux1);
    matrix_sum4x4(&aux1,m,&aux2,alpha,1.0);
    matrix_trans4x4(&aux2,&aux1);
    matrix_prod4x4(&aux1,&aux2,&aux3);

    res->col1[0] = beta*aux3.col1[0];
    res->col1[1] = beta*aux3.col1[1];
    res->col1[2] = beta*aux3.col1[2];
    res->col1[3] = beta*aux3.col1[3];

    res->col2[0] = beta*aux3.col2[0];
    res->col2[1] = beta*aux3.col2[1];
    res->col2[2] = beta*aux3.col2[2];
    res->col2[3] = beta*aux3.col2[3];

    res->col3[0] = beta*aux3.col3[0];
    res->col3[1] = beta*aux3.col3[1];
    res->col3[2] = beta*aux3.col3[2];
    res->col3[3] = beta*aux3.col3[3];

    res->col4[0] = beta*aux3.col4[0];
    res->col4[1] = beta*aux3.col4[1];
    res->col4[2] = beta*aux3.col4[2];
    res->col4[3] = beta*aux3.col4[3];
}

float eigenvalues4x4(Matrix4x4 *m, float *eigvector)
{
    int n = 4,i;
    float A[n][n],norm;
    float z[n],x[n],e[n],zmax,emax,k;

    A[0][0] = m->col1[0];
    A[0][1] = m->col2[0];
    A[0][2] = m->col3[0];
    A[0][3] = m->col4[0];

    A[1][0] = m->col1[1];
    A[1][1] = m->col2[1];
    A[1][2] = m->col3[1];
    A[1][3] = m->col4[1];

    A[2][0] = m->col1[2];
    A[2][1] = m->col2[2];
    A[2][2] = m->col3[2];
    A[2][3] = m->col4[2];

    A[3][0] = m->col1[3];
    A[3][1] = m->col2[3];
    A[3][2] = m->col3[3];
    A[3][3] = m->col4[3];


    x[0] = 0.5;
    x[1] = 0.5;
    x[2] = 0.5;
    x[3] = 0.5;

    i = 0;

    do{
        z[0] = A[0][0]*x[0]+A[0][1]*x[1]+A[0][2]*x[2]+A[0][3]*x[3];
        z[1] = A[1][0]*x[0]+A[1][1]*x[1]+A[1][2]*x[2]+A[1][3]*x[3];
        z[2] = A[2][0]*x[0]+A[2][1]*x[1]+A[2][2]*x[2]+A[2][3]*x[3];
        z[3] = A[3][0]*x[0]+A[3][1]*x[1]+A[3][2]*x[2]+A[3][3]*x[3];
        zmax = fabs(z[0]);
        if((fabs(z[0]))>zmax)zmax = fabs(z[0]);
        if((fabs(z[1]))>zmax)zmax = fabs(z[1]);
        if((fabs(z[2]))>zmax)zmax = fabs(z[2]);
        if((fabs(z[3]))>zmax)zmax = fabs(z[3]);
        k=1/zmax;
        z[0]=z[0]*k;
        z[1]=z[1]*k;
        z[2]=z[2]*k;
        z[3]=z[3]*k;
        e[0]=fabs((fabs(z[0]))-(fabs(x[0])));
        e[1]=fabs((fabs(z[1]))-(fabs(x[1])));
        e[2]=fabs((fabs(z[2]))-(fabs(x[2])));
        e[3]=fabs((fabs(z[3]))-(fabs(x[3])));
        emax=e[0];
        if(e[0]>emax)emax =e[0];
        if(e[1]>emax)emax =e[1];
        if(e[2]>emax)emax =e[2];
        if(e[3]>emax)emax =e[3];
        x[0]=z[0];
        x[1]=z[1];
        x[2]=z[2];
        x[3]=z[3];
        i++;
    }while(emax>0.0000001);
    norm = 1/float_norm4(z);
    eigvector[0] = z[0]*norm;
    eigvector[1] = z[1]*norm;
    eigvector[2] = z[2]*norm;
    eigvector[3] = z[3]*norm;
    return zmax;
}


float matrix_norm4x4(Matrix4x4 *m)
{
    float norma = 1, eigvector[4];
    Matrix4x4 m2;
    matrix_prod4x4(m,m,&m2);
    norma = sqrt(eigenvalues4x4(&m2,eigvector));
    m->col1[0] = m->col1[0]/norma;
    m->col1[1] = m->col1[1]/norma;
    m->col1[2] = m->col1[2]/norma;
    m->col1[3] = m->col1[3]/norma;

    m->col2[0] = m->col2[0]/norma;
    m->col2[1] = m->col2[1]/norma;
    m->col2[2] = m->col2[2]/norma;
    m->col2[3] = m->col2[3]/norma;

    m->col3[0] = m->col3[0]/norma;
    m->col3[1] = m->col3[1]/norma;
    m->col3[2] = m->col3[2]/norma;
    m->col3[3] = m->col3[3]/norma;

    m->col4[0] = m->col4[0]/norma;
    m->col4[1] = m->col4[1]/norma;
    m->col4[2] = m->col4[2]/norma;
    m->col4[3] = m->col4[3]/norma;

    return norma;
}


void concat_matrix(Matrix4x4 *res, Matrix3x3 *m, float v[3], float e)
{
/******************
*   | m     v |   *
*   | v'    e |   *
*******************/
    res->col1[0] = m->col1[0];
    res->col1[1] = m->col1[1];
    res->col1[2] = m->col1[2];
    res->col1[3] = v[0];

    res->col2[0] = m->col2[0];
    res->col2[1] = m->col2[1];
    res->col2[2] = m->col2[2];
    res->col2[3] = v[1];

    res->col3[0] = m->col3[0];
    res->col3[1] = m->col3[1];
    res->col3[2] = m->col3[2];
    res->col3[3] = v[2];

    res->col4[0] = v[0];
    res->col4[1] = v[1];
    res->col4[2] = v[2];
    res->col4[3] = e;

}

void eye3x3(Matrix3x3 *m, float alpha)
{
    m->col1[0] = alpha;
    m->col1[1] = 0;
    m->col1[2] = 0;

    m->col2[0] = 0;
    m->col2[1] = alpha;
    m->col2[2] = 0;

    m->col3[0] = 0;
    m->col3[1] = 0;
    m->col3[2] = alpha;
}


void eye4x4(Matrix4x4 *m, float alpha)
{
    m->col1[0] = alpha;
    m->col1[1] = 0;
    m->col1[2] = 0;
    m->col1[3] = 0;

    m->col2[0] = 0;
    m->col2[1] = alpha;
    m->col2[2] = 0;
    m->col2[3] = 0;

    m->col3[0] = 0;
    m->col3[1] = 0;
    m->col3[2] = alpha;
    m->col3[3] = 0;

    m->col4[0] = 0;
    m->col4[1] = 0;
    m->col4[2] = 0;
    m->col4[3] = alpha;
}

void fix_and_sub(Matrix4x4 *res, Matrix4x4 *m)
{
    res->col1[0] = m->col1[0] - trunc(m->col1[0]);
    res->col1[1] = m->col1[1] - trunc(m->col1[1]);
    res->col1[2] = m->col1[2] - trunc(m->col1[2]);
    res->col1[3] = m->col1[3] - trunc(m->col1[3]);

    res->col2[0] = m->col2[0] - trunc(m->col2[0]);
    res->col2[1] = m->col2[1] - trunc(m->col2[1]);
    res->col2[2] = m->col2[2] - trunc(m->col2[2]);
    res->col2[3] = m->col2[3] - trunc(m->col2[3]);

    res->col3[0] = m->col3[0] - trunc(m->col3[0]);
    res->col3[1] = m->col3[1] - trunc(m->col3[1]);
    res->col3[2] = m->col3[2] - trunc(m->col3[2]);
    res->col3[3] = m->col3[3] - trunc(m->col3[3]);

    res->col4[0] = m->col4[0] - trunc(m->col4[0]);
    res->col4[1] = m->col4[1] - trunc(m->col4[1]);
    res->col4[2] = m->col4[2] - trunc(m->col4[2]);
    res->col4[3] = m->col4[3] - trunc(m->col4[3]);
}


void quaternion_norm(float *q)
{
	float qnorm, aux;
	//int iter = 0;

	qnorm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    aux = 1/sqrt(qnorm);
    q[0] = q[0]*aux;
    q[1] = q[1]*aux;
    q[2] = q[2]*aux;
    q[3] = q[3]*aux;
	/*while (qnorm > 1.00001 || iter++ < 5) {
		if (qnorm > 1+1E-12) {
			aux = (3 + qnorm)/(1+3*qnorm);
			q[0] = aux*q[0];
			q[1] = aux*q[1];
			q[2] = aux*q[2];
			q[3] = aux*q[3];
		} else {
			if(isnan(qnorm)) {
				q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
			} else {
				aux = 1/sqrt(qnorm);
				q[0] = q[0]*aux;
				q[1] = q[1]*aux;
				q[2] = q[2]*aux;
				q[3] = q[3]*aux;
			}
		}
		qnorm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	}*/
}


/* QR decomposition funciton here */
void qr_decomp(float *A, float *Q, float *R)
{
    int i, j;
    float Q_[4][4];
    float R_[4][4];
    float A_[4][4];
    memcpy(A_, A, sizeof(A_));
    memset(R_, 0, sizeof(R_));
    memset(Q_, 0, sizeof(Q_));
    float current_column[4], current_unit_vector[4], current_dot_product, norm, norm_inv;
    for(i = 0; i < 4; i++) {
        current_column[0] = A_[i][0];
        current_column[1] = A_[i][1];
        current_column[2] = A_[i][2];
        current_column[3] = A_[i][3];
        for(j = 0; j < i; j++) {
            // TODO: Allocate current_unit_vector one time, and then copy *into* this
            // vector, saving a malloc and free.
            current_unit_vector[0] = Q_[j][0];
            current_unit_vector[1] = Q_[j][1];
            current_unit_vector[2] = Q_[j][2];
            current_unit_vector[3] = Q_[j][3];
            /* Dot product between current_unit_vector and current_column */
            current_dot_product =   (current_unit_vector[0]*current_column[0]) +
                                    (current_unit_vector[1]*current_column[1]) +
                                    (current_unit_vector[2]*current_column[2]) +
                                    (current_unit_vector[3]*current_column[3]) ;

            /* Multiply current_unit_vector by the scalar current_dot_product */
            current_unit_vector[0] *= current_dot_product;
            current_unit_vector[1] *= current_dot_product;
            current_unit_vector[2] *= current_dot_product;
            current_unit_vector[3] *= current_dot_product;

            /* Substract current_column to current_unit_vector */
            current_column[0] -= current_unit_vector[0];
            current_column[1] -= current_unit_vector[1];
            current_column[2] -= current_unit_vector[2];
            current_column[3] -= current_unit_vector[3];
            /* Store the value of r */
            R_[i][j] = current_dot_product;
        }
        norm = float_norm4(current_column);
        R_[i][i] = norm;
        norm_inv = 1.0 / norm;
        current_column[0] *= norm_inv;
        current_column[1] *= norm_inv;
        current_column[2] *= norm_inv;
        current_column[3] *= norm_inv;
        Q_[i][0] = current_column[0];
        Q_[i][1] = current_column[1];
        Q_[i][2] = current_column[2];
        Q_[i][3] = current_column[3];
    }
    memcpy(Q, Q_, sizeof(Q_));
    memcpy(R, R_, sizeof(R_));
    memcpy(A, A_, sizeof(A_));
}

void compute_eigenvectors4x4(float *A, float eigenval, float tol, float *vector)
{
    float Q_[4][4], R_[4][4], A_[4][4];
    float curr_vec[4], prev_vec[4], rhs_vec[4], back_substitute, norm, norm_inv;
    int i,j,iter;
    memcpy(A_, A, sizeof(A_));
    A_[0][0] -= (eigenval+tol);
    A_[1][1] -= (eigenval+tol);
    A_[2][2] -= (eigenval+tol);
    A_[3][3] -= (eigenval+tol);
    curr_vec[0] = 1; curr_vec[1] = 1; curr_vec[2] = 1; curr_vec[3] = 1;
    iter = 0;
    do {
        prev_vec[0] = curr_vec[0]; prev_vec[1] = curr_vec[1]; prev_vec[2] = curr_vec[2]; prev_vec[3] = curr_vec[3];
        qr_decomp(A_[0], Q_[0], R_[0]);
        for (i = 0; i < 4; i++)
            rhs_vec[i] = Q_[i][0] * prev_vec[0] + Q_[i][1] * prev_vec[1] + Q_[i][2] * prev_vec[2] + Q_[i][3] * prev_vec[3];

        for(i = 3; i >= 0; i--) {
            back_substitute = 0;
            for(j = i+1; j <= 3; j++) {
                back_substitute += curr_vec[j] * R_[i][j];
            }
            curr_vec[i] = (rhs_vec[i] - back_substitute) / R_[i][i];
        }
        if(curr_vec[0] < 0) {
            for(j = 0; j < 4; j++) {
                curr_vec[j] = -curr_vec[j];
            }
        }
        norm = float_norm4(curr_vec);
        norm_inv = 1.0/norm;
        curr_vec[0] *= norm_inv; curr_vec[1] *= norm_inv; curr_vec[2] *= norm_inv; curr_vec[3] *= norm_inv;

    }while(!((fabs(curr_vec[0] - prev_vec[0]) < tol)  &&
             (fabs(curr_vec[1] - prev_vec[1]) < tol)  &&
             (fabs(curr_vec[2] - prev_vec[2]) < tol)  &&
             (fabs(curr_vec[3] - prev_vec[3]) < tol))
             && ++iter < 4);
    /* Now curr_vec is the actual solution */
    vector[0] = curr_vec[0]; vector[1] = curr_vec[1]; vector[2] = curr_vec[2]; vector[3] = curr_vec[3];
}

/* Returns the largest eigenvalue and its associates eigenvector of a 4x4 matrix */
float eigen4x4(Matrix4x4 *m, float *res)
{
    /* job=0: eigen values only
        1: both eigen values and eigen vectors
    float w[n*2]: work space
    */
    float A[4][4], B[4][4], Q[4][4], R[4][4];
    float eigenval, tol = 0.001;
    int i, j;

    A[0][0] = m->col1[0] + tol;
    A[1][0] = m->col2[0] + tol;
    A[2][0] = m->col3[0] + tol;
    A[3][0] = m->col4[0] + tol;

    A[0][1] = m->col1[1] + tol;
    A[1][1] = m->col2[1] + tol;
    A[2][1] = m->col3[1] + tol;
    A[3][1] = m->col4[1] + tol;

    A[0][2] = m->col1[2] + tol;
    A[1][2] = m->col2[2] + tol;
    A[2][2] = m->col3[2] + tol;
    A[3][2] = m->col4[2] + tol;

    A[0][3] = m->col1[3] + tol;
    A[1][3] = m->col2[3] + tol;
    A[2][3] = m->col3[3] + tol;
    A[3][3] = m->col4[3] + tol;

    memcpy(B, A, sizeof(A));

    int iter = 0;
    while(++iter < 5) {
        qr_decomp(A[0], Q[0], R[0]); /* A will be solved into eigenvalue matrix */
        /* Casting into Matrix4x4... Lets check if this works */
        matrix_prod4x4((Matrix4x4 *) Q, (Matrix4x4 *) R, (Matrix4x4 *)A);
        for (i = 0; i < 4; i++) {
            for(j = 0; j < i; j++) {
                if(fabs(A[i][j]) > tol) {
                    continue;
                }
            }
        }
        break;
    }
    /* Now get the diagional and the larger eigenval */
    eigenval = 0;
    for (i = 0; i < 4; i++) {
        if (A[i][i] > eigenval) {
            eigenval = A[i][i];
        }
    }
    if(eigenval == 0 || isnan(eigenval) || fabs(eigenval) > 1e6) {
    	eigenval = 1;
    }

    /* Now compute the eigenvector for this eigenvalue */
    compute_eigenvectors4x4(B[0], eigenval, tol, res);

    return 0;
}

void estimated_quat2euler(float *euler, float *q_e2b, float *q_e2o)
{
    static float q_o2b_prev[4] = {0.09f,0.0f,0.0f,0.0f};
    float qnorm;
    float q_o2e[4], q_o2b[4];

    q_o2e[0] = q_e2o[0];
    q_o2e[1] = -q_e2o[1];
    q_o2e[2] = -q_e2o[2];
    q_o2e[3] = -q_e2o[3];

    quaternion_mult(q_o2e, q_e2b, q_o2b);

    qnorm = sqrt((q_o2b_prev[0] * q_o2b_prev[0]) + (q_o2b_prev[1] * q_o2b_prev[1]) + (q_o2b_prev[2] * q_o2b_prev[2]) + (q_o2b_prev[3] * q_o2b_prev[3]));
    if(qnorm != 0.0) {
        unwind(q_o2b,q_o2b_prev);
    }
    quat2angle(q_o2b,euler);
    euler[0] = euler[0]*180/M_PI;
    euler[1] = euler[1]*180/M_PI;
    euler[2] = euler[2]*180/M_PI;

    q_o2b_prev[0] = q_o2b[0];
    q_o2b_prev[1] = q_o2b[1];
    q_o2b_prev[2] = q_o2b[2];
    q_o2b_prev[3] = q_o2b[3];
}

void quaternion_mult(float *q1, float *q2, float *q3)
{
    float q1s,q2s,q3s;
    float q1v[3],q2v[3],q3v[3],cross3[3];

    q1s = q1[0];
    q1v[0] = q1[1];
    q1v[1] = q1[2];
    q1v[2] = q1[3];

    q2s = q2[0];
    q2v[0] = q2[1];
    q2v[1] = q2[2];
    q2v[2] = q2[3];

    q3s = q1s*q2s - dot_product(q1v, q2v);
    cross_product(q1v, q2v, cross3);
    q3v[0] = q2s*q1v[0]+q1s*q2v[0]+cross3[0];
    q3v[1] = q2s*q1v[1]+q1s*q2v[1]+cross3[1];
    q3v[2] = q2s*q1v[2]+q1s*q2v[2]+cross3[2];

    q3[0] = q3s;
    q3[1] = q3v[0];
    q3[2] = q3v[1];
    q3[3] = q3v[2];
}

void unwind(float *q, float *q_prev)
{
    float anglethreshold, angle, vnorm;
    float vecpart[3], prevecpart[3];

    anglethreshold = (90.0 * M_PI) / 180.0;
    vecpart[0] = q[1];
    vecpart[1] = q[2];
    vecpart[2] = q[3];

    prevecpart[0] = q_prev[1];
    prevecpart[1] = q_prev[2];
    prevecpart[2] = q_prev[3];

    vnorm = float_norm(vecpart);
    vecpart[0] = vecpart[0]/vnorm;
    vecpart[1] = vecpart[1]/vnorm;
    vecpart[2] = vecpart[2]/vnorm;

    vnorm = float_norm(prevecpart);
    prevecpart[0] = prevecpart[0]/vnorm;
    prevecpart[1] = prevecpart[1]/vnorm;
    prevecpart[2] = prevecpart[2]/vnorm;

    angle = acos(dot_product(vecpart,prevecpart));
    if(angle > anglethreshold) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
}

void quat2angle(float *q, float *euler)
{
    float phi, theta, psi;
    quaternion_norm(q);
    phi = atan2((q[2]*q[3])+(q[0]*q[1]),0.5-((q[1]*q[1])+(q[2]*q[2])));
    theta = asin(-2*((q[1]*q[3])-(q[0]*q[2])));
    psi = atan2((q[1]*q[2])+(q[0]*q[3]),0.5-((q[2]*q[2])+(q[3]*q[3])));

    euler[0] = psi;
    euler[1] = theta;
    euler[2] = phi;
}


void poseci2Quat(float *r, float *v, float *q)
{
    float norms, qnorm;
    float z[3],cross[3],x[3],y[3];
    Matrix3x3 A;
    static float q_prev[4] = {0.0f,0.0f,0.0f,0.0f};

    norms = float_norm(r);
    z[0] = -r[0]/norms;
    z[1] = -r[1]/norms;
    z[2] = -r[2]/norms;
    cross_product(r,v,cross);
    norms = float_norm(cross);
    y[0] = -cross[0]/norms;
    y[1] = -cross[1]/norms;
    y[2] = -cross[2]/norms;
    cross_product(y,z,x);
    A.col1[0] = x[0];
    A.col2[0] = x[1];
    A.col3[0] = x[2];

    A.col1[1] = y[0];
    A.col2[1] = y[1];
    A.col3[1] = y[2];

    A.col1[2] = z[0];
    A.col2[2] = z[1];
    A.col3[2] = z[2];

    matrix2Quat(&A,q);
    qnorm = sqrt((q_prev[0] * q_prev[0]) + (q_prev[1] * q_prev[1]) + (q_prev[2] * q_prev[2]) + (q_prev[3] * q_prev[3]));
    if(qnorm != 0.0) {
        unwind(q,q_prev);
    }
}

void matrix2Quat(Matrix3x3 *A, float *q)
{
    float trA, qscal;
    float keyval[4];
    uint8_t max = 0;
    trA = matrix_trace(A);
    keyval[0] = A->col1[0];
    keyval[1] = A->col2[1];
    keyval[2] = A->col3[2];
    keyval[3] = trA;

    for(int i = 1; i < 4; i++){
        if (keyval[i]>keyval[max]) {
            max = i;
        }
    }

    switch (max) {
        case 0:
            q[0] = 1 + 2*A->col1[0] - trA;
            q[1] = A->col2[0] + A->col1[1];
            q[2] = A->col3[0] + A->col1[2];
            q[3] = A->col3[1] - A->col2[2];
            break;
        case 1:
            q[0] = A->col1[1] + A->col2[0];
            q[1] = 1 + 2*A->col2[1] - trA;
            q[2] = A->col3[1] + A->col2[2];
            q[3] = A->col1[2] - A->col3[0];
            break;
        case 2:
            q[0] = A->col1[2] + A->col3[0];
            q[1] = A->col2[2] + A->col3[1];
            q[2] = 1 + 2*A->col3[2] - trA;
            q[3] = A->col2[0] - A->col1[1];
            break;
        case 3:
            q[0] = A->col3[1] - A->col2[2];
            q[1] = A->col1[2] - A->col3[0];
            q[2] = A->col2[0] - A->col1[1];
            q[3] = 1 + trA;
            break;
    }
    quaternion_norm(q);
    if (q[3] < 0) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }

    qscal = q[3];
    q[3] = q[2];
    q[2] = q[1];
    q[1] = q[0];
    q[0] = qscal;

}


// This file has been automatically generated
// DO NOT EDIT!

#include <math.h>

const int STATE_DIM = 4;
const int CONTROL_DIM = 3;
const int MEASURE_DIM = 1;



static inline void f(const float *x, const float *u, float delta_t, float *out) {
    (void)x;
    (void)u;
    (void)delta_t;

   out[0] = delta_t*(-0.5*u[0]*x[1] - 0.5*u[1]*x[2] - 0.5*u[2]*x[3]) + x[0];
   out[1] = delta_t*(0.5*u[0]*x[0] - 0.5*u[1]*x[3] + 0.5*u[2]*x[2]) + x[1];
   out[2] = delta_t*(0.5*u[0]*x[3] + 0.5*u[1]*x[0] - 0.5*u[2]*x[1]) + x[2];
   out[3] = delta_t*(-0.5*u[0]*x[2] + 0.5*u[1]*x[1] + 0.5*u[2]*x[0]) + x[3];

}

static inline void F(const float *x, const float *u, float delta_t, float *out) {
    (void)x;
    (void)u;
    (void)delta_t;

   out[0] = 1;
   out[1] = 0.5*delta_t*u[0];
   out[2] = 0.5*delta_t*u[1];
   out[3] = 0.5*delta_t*u[2];
   out[4] = -0.5*delta_t*u[0];
   out[5] = 1;
   out[6] = -0.5*delta_t*u[2];
   out[7] = 0.5*delta_t*u[1];
   out[8] = -0.5*delta_t*u[1];
   out[9] = 0.5*delta_t*u[2];
   out[10] = 1;
   out[11] = -0.5*delta_t*u[0];
   out[12] = -0.5*delta_t*u[2];
   out[13] = -0.5*delta_t*u[1];
   out[14] = 0.5*delta_t*u[0];
   out[15] = 1;

}

static inline void h(const float *x, float *out) {
    (void)x;

   out[0] = 0;

}

static inline void H(const float *x, float *out) {
    (void)x;

   out[0] = 2*x[3];
   out[1] = 2*x[2];
   out[2] = -2*x[1];
   out[3] = -2*x[0];

}

static inline void meas_transf(const float *attitude, const float *z, float *out) {
    (void)attitude;
    (void)z;

   out[0] = atan2f((attitude[0]*z[0] + attitude[2]*z[2] - attitude[3]*z[1])*attitude[3] + (attitude[0]*z[1] - attitude[1]*z[2] + attitude[3]*z[0])*attitude[0] - (attitude[0]*z[2] + attitude[1]*z[1] - attitude[2]*z[0])*attitude[1] - (-attitude[1]*z[0] - attitude[2]*z[1] - attitude[3]*z[2])*attitude[2], (attitude[0]*z[0] + attitude[2]*z[2] - attitude[3]*z[1])*attitude[0] - (attitude[0]*z[1] - attitude[1]*z[2] + attitude[3]*z[0])*attitude[3] + (attitude[0]*z[2] + attitude[1]*z[1] - attitude[2]*z[0])*attitude[2] - (-attitude[1]*z[0] - attitude[2]*z[1] - attitude[3]*z[2])*attitude[1]);

}

static inline void z_inertial(const float *attitude, const float *z, float *out) {
    (void)attitude;
    (void)z;

   out[0] = (attitude[0]*z[0] + attitude[2]*z[2] - attitude[3]*z[1])*attitude[0] - (attitude[0]*z[1] - attitude[1]*z[2] + attitude[3]*z[0])*attitude[3] + (attitude[0]*z[2] + attitude[1]*z[1] - attitude[2]*z[0])*attitude[2] - (-attitude[1]*z[0] - attitude[2]*z[1] - attitude[3]*z[2])*attitude[1];
   out[1] = (attitude[0]*z[0] + attitude[2]*z[2] - attitude[3]*z[1])*attitude[3] + (attitude[0]*z[1] - attitude[1]*z[2] + attitude[3]*z[0])*attitude[0] - (attitude[0]*z[2] + attitude[1]*z[1] - attitude[2]*z[0])*attitude[1] - (-attitude[1]*z[0] - attitude[2]*z[1] - attitude[3]*z[2])*attitude[2];
   out[2] = -(attitude[0]*z[0] + attitude[2]*z[2] - attitude[3]*z[1])*attitude[2] + (attitude[0]*z[1] - attitude[1]*z[2] + attitude[3]*z[0])*attitude[1] + (attitude[0]*z[2] + attitude[1]*z[1] - attitude[2]*z[0])*attitude[0] - (-attitude[1]*z[0] - attitude[2]*z[1] - attitude[3]*z[2])*attitude[3];

}

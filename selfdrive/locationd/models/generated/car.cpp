#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_992729493706288594) {
   out_992729493706288594[0] = delta_x[0] + nom_x[0];
   out_992729493706288594[1] = delta_x[1] + nom_x[1];
   out_992729493706288594[2] = delta_x[2] + nom_x[2];
   out_992729493706288594[3] = delta_x[3] + nom_x[3];
   out_992729493706288594[4] = delta_x[4] + nom_x[4];
   out_992729493706288594[5] = delta_x[5] + nom_x[5];
   out_992729493706288594[6] = delta_x[6] + nom_x[6];
   out_992729493706288594[7] = delta_x[7] + nom_x[7];
   out_992729493706288594[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5285575722055005222) {
   out_5285575722055005222[0] = -nom_x[0] + true_x[0];
   out_5285575722055005222[1] = -nom_x[1] + true_x[1];
   out_5285575722055005222[2] = -nom_x[2] + true_x[2];
   out_5285575722055005222[3] = -nom_x[3] + true_x[3];
   out_5285575722055005222[4] = -nom_x[4] + true_x[4];
   out_5285575722055005222[5] = -nom_x[5] + true_x[5];
   out_5285575722055005222[6] = -nom_x[6] + true_x[6];
   out_5285575722055005222[7] = -nom_x[7] + true_x[7];
   out_5285575722055005222[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1123052732529277220) {
   out_1123052732529277220[0] = 1.0;
   out_1123052732529277220[1] = 0.0;
   out_1123052732529277220[2] = 0.0;
   out_1123052732529277220[3] = 0.0;
   out_1123052732529277220[4] = 0.0;
   out_1123052732529277220[5] = 0.0;
   out_1123052732529277220[6] = 0.0;
   out_1123052732529277220[7] = 0.0;
   out_1123052732529277220[8] = 0.0;
   out_1123052732529277220[9] = 0.0;
   out_1123052732529277220[10] = 1.0;
   out_1123052732529277220[11] = 0.0;
   out_1123052732529277220[12] = 0.0;
   out_1123052732529277220[13] = 0.0;
   out_1123052732529277220[14] = 0.0;
   out_1123052732529277220[15] = 0.0;
   out_1123052732529277220[16] = 0.0;
   out_1123052732529277220[17] = 0.0;
   out_1123052732529277220[18] = 0.0;
   out_1123052732529277220[19] = 0.0;
   out_1123052732529277220[20] = 1.0;
   out_1123052732529277220[21] = 0.0;
   out_1123052732529277220[22] = 0.0;
   out_1123052732529277220[23] = 0.0;
   out_1123052732529277220[24] = 0.0;
   out_1123052732529277220[25] = 0.0;
   out_1123052732529277220[26] = 0.0;
   out_1123052732529277220[27] = 0.0;
   out_1123052732529277220[28] = 0.0;
   out_1123052732529277220[29] = 0.0;
   out_1123052732529277220[30] = 1.0;
   out_1123052732529277220[31] = 0.0;
   out_1123052732529277220[32] = 0.0;
   out_1123052732529277220[33] = 0.0;
   out_1123052732529277220[34] = 0.0;
   out_1123052732529277220[35] = 0.0;
   out_1123052732529277220[36] = 0.0;
   out_1123052732529277220[37] = 0.0;
   out_1123052732529277220[38] = 0.0;
   out_1123052732529277220[39] = 0.0;
   out_1123052732529277220[40] = 1.0;
   out_1123052732529277220[41] = 0.0;
   out_1123052732529277220[42] = 0.0;
   out_1123052732529277220[43] = 0.0;
   out_1123052732529277220[44] = 0.0;
   out_1123052732529277220[45] = 0.0;
   out_1123052732529277220[46] = 0.0;
   out_1123052732529277220[47] = 0.0;
   out_1123052732529277220[48] = 0.0;
   out_1123052732529277220[49] = 0.0;
   out_1123052732529277220[50] = 1.0;
   out_1123052732529277220[51] = 0.0;
   out_1123052732529277220[52] = 0.0;
   out_1123052732529277220[53] = 0.0;
   out_1123052732529277220[54] = 0.0;
   out_1123052732529277220[55] = 0.0;
   out_1123052732529277220[56] = 0.0;
   out_1123052732529277220[57] = 0.0;
   out_1123052732529277220[58] = 0.0;
   out_1123052732529277220[59] = 0.0;
   out_1123052732529277220[60] = 1.0;
   out_1123052732529277220[61] = 0.0;
   out_1123052732529277220[62] = 0.0;
   out_1123052732529277220[63] = 0.0;
   out_1123052732529277220[64] = 0.0;
   out_1123052732529277220[65] = 0.0;
   out_1123052732529277220[66] = 0.0;
   out_1123052732529277220[67] = 0.0;
   out_1123052732529277220[68] = 0.0;
   out_1123052732529277220[69] = 0.0;
   out_1123052732529277220[70] = 1.0;
   out_1123052732529277220[71] = 0.0;
   out_1123052732529277220[72] = 0.0;
   out_1123052732529277220[73] = 0.0;
   out_1123052732529277220[74] = 0.0;
   out_1123052732529277220[75] = 0.0;
   out_1123052732529277220[76] = 0.0;
   out_1123052732529277220[77] = 0.0;
   out_1123052732529277220[78] = 0.0;
   out_1123052732529277220[79] = 0.0;
   out_1123052732529277220[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3399954509693334481) {
   out_3399954509693334481[0] = state[0];
   out_3399954509693334481[1] = state[1];
   out_3399954509693334481[2] = state[2];
   out_3399954509693334481[3] = state[3];
   out_3399954509693334481[4] = state[4];
   out_3399954509693334481[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3399954509693334481[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3399954509693334481[7] = state[7];
   out_3399954509693334481[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7186565621096537770) {
   out_7186565621096537770[0] = 1;
   out_7186565621096537770[1] = 0;
   out_7186565621096537770[2] = 0;
   out_7186565621096537770[3] = 0;
   out_7186565621096537770[4] = 0;
   out_7186565621096537770[5] = 0;
   out_7186565621096537770[6] = 0;
   out_7186565621096537770[7] = 0;
   out_7186565621096537770[8] = 0;
   out_7186565621096537770[9] = 0;
   out_7186565621096537770[10] = 1;
   out_7186565621096537770[11] = 0;
   out_7186565621096537770[12] = 0;
   out_7186565621096537770[13] = 0;
   out_7186565621096537770[14] = 0;
   out_7186565621096537770[15] = 0;
   out_7186565621096537770[16] = 0;
   out_7186565621096537770[17] = 0;
   out_7186565621096537770[18] = 0;
   out_7186565621096537770[19] = 0;
   out_7186565621096537770[20] = 1;
   out_7186565621096537770[21] = 0;
   out_7186565621096537770[22] = 0;
   out_7186565621096537770[23] = 0;
   out_7186565621096537770[24] = 0;
   out_7186565621096537770[25] = 0;
   out_7186565621096537770[26] = 0;
   out_7186565621096537770[27] = 0;
   out_7186565621096537770[28] = 0;
   out_7186565621096537770[29] = 0;
   out_7186565621096537770[30] = 1;
   out_7186565621096537770[31] = 0;
   out_7186565621096537770[32] = 0;
   out_7186565621096537770[33] = 0;
   out_7186565621096537770[34] = 0;
   out_7186565621096537770[35] = 0;
   out_7186565621096537770[36] = 0;
   out_7186565621096537770[37] = 0;
   out_7186565621096537770[38] = 0;
   out_7186565621096537770[39] = 0;
   out_7186565621096537770[40] = 1;
   out_7186565621096537770[41] = 0;
   out_7186565621096537770[42] = 0;
   out_7186565621096537770[43] = 0;
   out_7186565621096537770[44] = 0;
   out_7186565621096537770[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7186565621096537770[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7186565621096537770[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7186565621096537770[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7186565621096537770[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7186565621096537770[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7186565621096537770[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7186565621096537770[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7186565621096537770[53] = -9.8100000000000005*dt;
   out_7186565621096537770[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7186565621096537770[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7186565621096537770[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7186565621096537770[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7186565621096537770[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7186565621096537770[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7186565621096537770[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7186565621096537770[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7186565621096537770[62] = 0;
   out_7186565621096537770[63] = 0;
   out_7186565621096537770[64] = 0;
   out_7186565621096537770[65] = 0;
   out_7186565621096537770[66] = 0;
   out_7186565621096537770[67] = 0;
   out_7186565621096537770[68] = 0;
   out_7186565621096537770[69] = 0;
   out_7186565621096537770[70] = 1;
   out_7186565621096537770[71] = 0;
   out_7186565621096537770[72] = 0;
   out_7186565621096537770[73] = 0;
   out_7186565621096537770[74] = 0;
   out_7186565621096537770[75] = 0;
   out_7186565621096537770[76] = 0;
   out_7186565621096537770[77] = 0;
   out_7186565621096537770[78] = 0;
   out_7186565621096537770[79] = 0;
   out_7186565621096537770[80] = 1;
}
void h_25(double *state, double *unused, double *out_2688066850896918279) {
   out_2688066850896918279[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1024106204748103192) {
   out_1024106204748103192[0] = 0;
   out_1024106204748103192[1] = 0;
   out_1024106204748103192[2] = 0;
   out_1024106204748103192[3] = 0;
   out_1024106204748103192[4] = 0;
   out_1024106204748103192[5] = 0;
   out_1024106204748103192[6] = 1;
   out_1024106204748103192[7] = 0;
   out_1024106204748103192[8] = 0;
}
void h_24(double *state, double *unused, double *out_2651501169572462520) {
   out_2651501169572462520[0] = state[4];
   out_2651501169572462520[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2258749862360183052) {
   out_2258749862360183052[0] = 0;
   out_2258749862360183052[1] = 0;
   out_2258749862360183052[2] = 0;
   out_2258749862360183052[3] = 0;
   out_2258749862360183052[4] = 1;
   out_2258749862360183052[5] = 0;
   out_2258749862360183052[6] = 0;
   out_2258749862360183052[7] = 0;
   out_2258749862360183052[8] = 0;
   out_2258749862360183052[9] = 0;
   out_2258749862360183052[10] = 0;
   out_2258749862360183052[11] = 0;
   out_2258749862360183052[12] = 0;
   out_2258749862360183052[13] = 0;
   out_2258749862360183052[14] = 1;
   out_2258749862360183052[15] = 0;
   out_2258749862360183052[16] = 0;
   out_2258749862360183052[17] = 0;
}
void h_30(double *state, double *unused, double *out_1410236447382453585) {
   out_1410236447382453585[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1153445151891343262) {
   out_1153445151891343262[0] = 0;
   out_1153445151891343262[1] = 0;
   out_1153445151891343262[2] = 0;
   out_1153445151891343262[3] = 0;
   out_1153445151891343262[4] = 1;
   out_1153445151891343262[5] = 0;
   out_1153445151891343262[6] = 0;
   out_1153445151891343262[7] = 0;
   out_1153445151891343262[8] = 0;
}
void h_26(double *state, double *unused, double *out_8240198162061100891) {
   out_8240198162061100891[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4765609523622159416) {
   out_4765609523622159416[0] = 0;
   out_4765609523622159416[1] = 0;
   out_4765609523622159416[2] = 0;
   out_4765609523622159416[3] = 0;
   out_4765609523622159416[4] = 0;
   out_4765609523622159416[5] = 0;
   out_4765609523622159416[6] = 0;
   out_4765609523622159416[7] = 1;
   out_4765609523622159416[8] = 0;
}
void h_27(double *state, double *unused, double *out_7170514541007791929) {
   out_7170514541007791929[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3328208463691768173) {
   out_3328208463691768173[0] = 0;
   out_3328208463691768173[1] = 0;
   out_3328208463691768173[2] = 0;
   out_3328208463691768173[3] = 1;
   out_3328208463691768173[4] = 0;
   out_3328208463691768173[5] = 0;
   out_3328208463691768173[6] = 0;
   out_3328208463691768173[7] = 0;
   out_3328208463691768173[8] = 0;
}
void h_29(double *state, double *unused, double *out_6895320478723286040) {
   out_6895320478723286040[0] = state[1];
}
void H_29(double *state, double *unused, double *out_643213807576951078) {
   out_643213807576951078[0] = 0;
   out_643213807576951078[1] = 1;
   out_643213807576951078[2] = 0;
   out_643213807576951078[3] = 0;
   out_643213807576951078[4] = 0;
   out_643213807576951078[5] = 0;
   out_643213807576951078[6] = 0;
   out_643213807576951078[7] = 0;
   out_643213807576951078[8] = 0;
}
void h_28(double *state, double *unused, double *out_1292229053449076841) {
   out_1292229053449076841[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5725612824646481652) {
   out_5725612824646481652[0] = 1;
   out_5725612824646481652[1] = 0;
   out_5725612824646481652[2] = 0;
   out_5725612824646481652[3] = 0;
   out_5725612824646481652[4] = 0;
   out_5725612824646481652[5] = 0;
   out_5725612824646481652[6] = 0;
   out_5725612824646481652[7] = 0;
   out_5725612824646481652[8] = 0;
}
void h_31(double *state, double *unused, double *out_2412872788612412390) {
   out_2412872788612412390[0] = state[8];
}
void H_31(double *state, double *unused, double *out_993460242871142764) {
   out_993460242871142764[0] = 0;
   out_993460242871142764[1] = 0;
   out_993460242871142764[2] = 0;
   out_993460242871142764[3] = 0;
   out_993460242871142764[4] = 0;
   out_993460242871142764[5] = 0;
   out_993460242871142764[6] = 0;
   out_993460242871142764[7] = 0;
   out_993460242871142764[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_992729493706288594) {
  err_fun(nom_x, delta_x, out_992729493706288594);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5285575722055005222) {
  inv_err_fun(nom_x, true_x, out_5285575722055005222);
}
void car_H_mod_fun(double *state, double *out_1123052732529277220) {
  H_mod_fun(state, out_1123052732529277220);
}
void car_f_fun(double *state, double dt, double *out_3399954509693334481) {
  f_fun(state,  dt, out_3399954509693334481);
}
void car_F_fun(double *state, double dt, double *out_7186565621096537770) {
  F_fun(state,  dt, out_7186565621096537770);
}
void car_h_25(double *state, double *unused, double *out_2688066850896918279) {
  h_25(state, unused, out_2688066850896918279);
}
void car_H_25(double *state, double *unused, double *out_1024106204748103192) {
  H_25(state, unused, out_1024106204748103192);
}
void car_h_24(double *state, double *unused, double *out_2651501169572462520) {
  h_24(state, unused, out_2651501169572462520);
}
void car_H_24(double *state, double *unused, double *out_2258749862360183052) {
  H_24(state, unused, out_2258749862360183052);
}
void car_h_30(double *state, double *unused, double *out_1410236447382453585) {
  h_30(state, unused, out_1410236447382453585);
}
void car_H_30(double *state, double *unused, double *out_1153445151891343262) {
  H_30(state, unused, out_1153445151891343262);
}
void car_h_26(double *state, double *unused, double *out_8240198162061100891) {
  h_26(state, unused, out_8240198162061100891);
}
void car_H_26(double *state, double *unused, double *out_4765609523622159416) {
  H_26(state, unused, out_4765609523622159416);
}
void car_h_27(double *state, double *unused, double *out_7170514541007791929) {
  h_27(state, unused, out_7170514541007791929);
}
void car_H_27(double *state, double *unused, double *out_3328208463691768173) {
  H_27(state, unused, out_3328208463691768173);
}
void car_h_29(double *state, double *unused, double *out_6895320478723286040) {
  h_29(state, unused, out_6895320478723286040);
}
void car_H_29(double *state, double *unused, double *out_643213807576951078) {
  H_29(state, unused, out_643213807576951078);
}
void car_h_28(double *state, double *unused, double *out_1292229053449076841) {
  h_28(state, unused, out_1292229053449076841);
}
void car_H_28(double *state, double *unused, double *out_5725612824646481652) {
  H_28(state, unused, out_5725612824646481652);
}
void car_h_31(double *state, double *unused, double *out_2412872788612412390) {
  h_31(state, unused, out_2412872788612412390);
}
void car_H_31(double *state, double *unused, double *out_993460242871142764) {
  H_31(state, unused, out_993460242871142764);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)

#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_992729493706288594);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5285575722055005222);
void car_H_mod_fun(double *state, double *out_1123052732529277220);
void car_f_fun(double *state, double dt, double *out_3399954509693334481);
void car_F_fun(double *state, double dt, double *out_7186565621096537770);
void car_h_25(double *state, double *unused, double *out_2688066850896918279);
void car_H_25(double *state, double *unused, double *out_1024106204748103192);
void car_h_24(double *state, double *unused, double *out_2651501169572462520);
void car_H_24(double *state, double *unused, double *out_2258749862360183052);
void car_h_30(double *state, double *unused, double *out_1410236447382453585);
void car_H_30(double *state, double *unused, double *out_1153445151891343262);
void car_h_26(double *state, double *unused, double *out_8240198162061100891);
void car_H_26(double *state, double *unused, double *out_4765609523622159416);
void car_h_27(double *state, double *unused, double *out_7170514541007791929);
void car_H_27(double *state, double *unused, double *out_3328208463691768173);
void car_h_29(double *state, double *unused, double *out_6895320478723286040);
void car_H_29(double *state, double *unused, double *out_643213807576951078);
void car_h_28(double *state, double *unused, double *out_1292229053449076841);
void car_H_28(double *state, double *unused, double *out_5725612824646481652);
void car_h_31(double *state, double *unused, double *out_2412872788612412390);
void car_H_31(double *state, double *unused, double *out_993460242871142764);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
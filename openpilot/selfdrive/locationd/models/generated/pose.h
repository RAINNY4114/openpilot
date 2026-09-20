#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6167525615574876253);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8657746229211315654);
void pose_H_mod_fun(double *state, double *out_8244082865274323662);
void pose_f_fun(double *state, double dt, double *out_6288059084794879923);
void pose_F_fun(double *state, double dt, double *out_7236874212108329750);
void pose_h_4(double *state, double *unused, double *out_6261259594144104730);
void pose_H_4(double *state, double *unused, double *out_6499476752447183532);
void pose_h_10(double *state, double *unused, double *out_6242140905112775300);
void pose_H_10(double *state, double *unused, double *out_630116466754361676);
void pose_h_13(double *state, double *unused, double *out_8966555842540160286);
void pose_H_13(double *state, double *unused, double *out_8734993495930035283);
void pose_h_14(double *state, double *unused, double *out_5640486197908056385);
void pose_H_14(double *state, double *unused, double *out_7984026464922883555);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
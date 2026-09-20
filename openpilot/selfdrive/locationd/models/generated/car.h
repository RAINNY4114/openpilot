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
void car_err_fun(double *nom_x, double *delta_x, double *out_445477681596107833);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1730746270203870227);
void car_H_mod_fun(double *state, double *out_8266434048742633287);
void car_f_fun(double *state, double dt, double *out_5010589441685349139);
void car_F_fun(double *state, double dt, double *out_629448728715182792);
void car_h_25(double *state, double *unused, double *out_395058084693636622);
void car_H_25(double *state, double *unused, double *out_5276618199782253489);
void car_h_24(double *state, double *unused, double *out_2083897173295378813);
void car_H_24(double *state, double *unused, double *out_3099403776175103516);
void car_h_30(double *state, double *unused, double *out_5803567298700278226);
void car_H_30(double *state, double *unused, double *out_2758285241275004862);
void car_h_26(double *state, double *unused, double *out_9219840588349348242);
void car_H_26(double *state, double *unused, double *out_9018121518656309713);
void car_h_27(double *state, double *unused, double *out_998373969370114139);
void car_H_27(double *state, double *unused, double *out_4933048553075429773);
void car_h_29(double *state, double *unused, double *out_6764891287450558302);
void car_H_29(double *state, double *unused, double *out_2248053896960612678);
void car_h_28(double *state, double *unused, double *out_8126811998091528518);
void car_H_28(double *state, double *unused, double *out_7330452914030143252);
void car_h_31(double *state, double *unused, double *out_4919549963995348585);
void car_H_31(double *state, double *unused, double *out_8802414452819890427);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
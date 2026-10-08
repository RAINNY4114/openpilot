#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8008231329674625977);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9148252894232034006);
void pose_H_mod_fun(double *state, double *out_1922727016412823836);
void pose_f_fun(double *state, double dt, double *out_4569588112805388687);
void pose_F_fun(double *state, double dt, double *out_4952891530857479207);
void pose_h_4(double *state, double *unused, double *out_8707903708277267811);
void pose_H_4(double *state, double *unused, double *out_7075245401451904571);
void pose_h_10(double *state, double *unused, double *out_5731269717084300037);
void pose_H_10(double *state, double *unused, double *out_7323133101098852149);
void pose_h_13(double *state, double *unused, double *out_2954187921383884135);
void pose_H_13(double *state, double *unused, double *out_8159224846925314244);
void pose_h_14(double *state, double *unused, double *out_5785405054235647194);
void pose_H_14(double *state, double *unused, double *out_7408257815918162516);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
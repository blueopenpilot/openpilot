#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7621931960387363190);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5740643755297289225);
void gnss_H_mod_fun(double *state, double *out_3754482379115474220);
void gnss_f_fun(double *state, double dt, double *out_3214597710273968771);
void gnss_F_fun(double *state, double dt, double *out_5275588445702707408);
void gnss_h_6(double *state, double *sat_pos, double *out_170926249370392253);
void gnss_H_6(double *state, double *sat_pos, double *out_3742869924876393196);
void gnss_h_20(double *state, double *sat_pos, double *out_1863480405032848687);
void gnss_H_20(double *state, double *sat_pos, double *out_6580296787323519817);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3420059550442289419);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2065618174834969080);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3420059550442289419);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2065618174834969080);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
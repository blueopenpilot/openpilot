#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2220497270947833429);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4694480141159607186);
void gnss_H_mod_fun(double *state, double *out_8762642917035623636);
void gnss_f_fun(double *state, double dt, double *out_3481548129316230265);
void gnss_F_fun(double *state, double dt, double *out_4057106452161948183);
void gnss_h_6(double *state, double *sat_pos, double *out_2042956050660128342);
void gnss_H_6(double *state, double *sat_pos, double *out_6208909197614479079);
void gnss_h_20(double *state, double *sat_pos, double *out_2432843798433092424);
void gnss_H_20(double *state, double *sat_pos, double *out_5201508118287945176);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8574522793821478815);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8833501221798871914);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8574522793821478815);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8833501221798871914);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
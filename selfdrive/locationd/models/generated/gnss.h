#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3447090500816114356);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8035028355194170582);
void gnss_H_mod_fun(double *state, double *out_2818950684216781197);
void gnss_f_fun(double *state, double dt, double *out_3087561945035795840);
void gnss_F_fun(double *state, double dt, double *out_8128229715354435733);
void gnss_h_6(double *state, double *sat_pos, double *out_6299004301386785170);
void gnss_H_6(double *state, double *sat_pos, double *out_4160133055002661189);
void gnss_h_20(double *state, double *sat_pos, double *out_7323928502644680378);
void gnss_H_20(double *state, double *sat_pos, double *out_1240025102187636063);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4309803303408579533);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2751727956889753603);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4309803303408579533);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2751727956889753603);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
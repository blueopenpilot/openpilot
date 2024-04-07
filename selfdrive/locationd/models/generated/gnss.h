#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_8229174994289557138);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6217284632776378574);
void gnss_H_mod_fun(double *state, double *out_5011568525599502586);
void gnss_f_fun(double *state, double dt, double *out_3473355585470786110);
void gnss_F_fun(double *state, double dt, double *out_5105028025396229212);
void gnss_h_6(double *state, double *sat_pos, double *out_3612315771256631339);
void gnss_H_6(double *state, double *sat_pos, double *out_3598249379002766159);
void gnss_h_20(double *state, double *sat_pos, double *out_583828302854464326);
void gnss_H_20(double *state, double *sat_pos, double *out_2729544730293299337);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2465849935794505934);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1235251322282411141);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2465849935794505934);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1235251322282411141);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
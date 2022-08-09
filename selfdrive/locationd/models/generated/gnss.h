#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6435077088755969679);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5461030829569705127);
void gnss_H_mod_fun(double *state, double *out_1113439463888521147);
void gnss_f_fun(double *state, double dt, double *out_3853921857578862715);
void gnss_F_fun(double *state, double dt, double *out_1990023416032551193);
void gnss_h_6(double *state, double *sat_pos, double *out_5372040911404388122);
void gnss_H_6(double *state, double *sat_pos, double *out_1538307200137441102);
void gnss_h_20(double *state, double *sat_pos, double *out_5942081968181849174);
void gnss_H_20(double *state, double *sat_pos, double *out_3657082352704797406);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4161012093764852918);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3960385618526437650);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4161012093764852918);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3960385618526437650);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
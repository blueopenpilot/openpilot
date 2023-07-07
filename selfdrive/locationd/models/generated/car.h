#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4702752847547446396);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7477561154065216360);
void car_H_mod_fun(double *state, double *out_2627163213642550557);
void car_f_fun(double *state, double dt, double *out_2949320639431583799);
void car_F_fun(double *state, double dt, double *out_8341996880936103292);
void car_h_25(double *state, double *unused, double *out_7940820690712606148);
void car_H_25(double *state, double *unused, double *out_5496306684859195057);
void car_h_24(double *state, double *unused, double *out_8680464193662503182);
void car_H_24(double *state, double *unused, double *out_4664942968776231447);
void car_h_30(double *state, double *unused, double *out_8414235386947951111);
void car_H_30(double *state, double *unused, double *out_2977973726351946430);
void car_h_26(double *state, double *unused, double *out_1956308648912788502);
void car_H_26(double *state, double *unused, double *out_9208934069976300335);
void car_h_27(double *state, double *unused, double *out_4106881865583096322);
void car_H_27(double *state, double *unused, double *out_754379655168003213);
void car_h_29(double *state, double *unused, double *out_8578567303724458);
void car_H_29(double *state, double *unused, double *out_2467742382037554246);
void car_h_28(double *state, double *unused, double *out_5088965958918394434);
void car_H_28(double *state, double *unused, double *out_7550141399107084820);
void car_h_31(double *state, double *unused, double *out_7783634022361477003);
void car_H_31(double *state, double *unused, double *out_5465660722982234629);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
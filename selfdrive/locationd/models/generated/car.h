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
void car_err_fun(double *nom_x, double *delta_x, double *out_1070714730182928934);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1165789664415453656);
void car_H_mod_fun(double *state, double *out_585124696828107175);
void car_f_fun(double *state, double dt, double *out_7399759131319460689);
void car_F_fun(double *state, double dt, double *out_6459431986806209049);
void car_h_25(double *state, double *unused, double *out_1968240230009004181);
void car_H_25(double *state, double *unused, double *out_5940609591942357782);
void car_h_24(double *state, double *unused, double *out_8990479235979821569);
void car_H_24(double *state, double *unused, double *out_3278069295697998609);
void car_h_30(double *state, double *unused, double *out_1693046167724498292);
void car_H_30(double *state, double *unused, double *out_5811270644799117712);
void car_h_26(double *state, double *unused, double *out_911359153084199359);
void car_H_26(double *state, double *unused, double *out_2199106273068301558);
void car_h_27(double *state, double *unused, double *out_7310107465895034744);
void car_H_27(double *state, double *unused, double *out_3636507332998692801);
void car_h_29(double *state, double *unused, double *out_9128189602153199105);
void car_H_29(double *state, double *unused, double *out_1923144606129141768);
void car_h_28(double *state, double *unused, double *out_2308434198234189099);
void car_H_28(double *state, double *unused, double *out_3159254410940388806);
void car_h_31(double *state, double *unused, double *out_6671637246709537703);
void car_H_31(double *state, double *unused, double *out_5971255553819318210);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
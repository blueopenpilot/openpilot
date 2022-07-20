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
void car_err_fun(double *nom_x, double *delta_x, double *out_5379945950210905137);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6100139837532875915);
void car_H_mod_fun(double *state, double *out_8072557983707861366);
void car_f_fun(double *state, double dt, double *out_3587256842187338616);
void car_F_fun(double *state, double dt, double *out_673445270312808434);
void car_h_25(double *state, double *unused, double *out_5795691796466611661);
void car_H_25(double *state, double *unused, double *out_1186979588426458488);
void car_h_24(double *state, double *unused, double *out_1165476922925981880);
void car_H_24(double *state, double *unused, double *out_3412687372405327050);
void car_h_30(double *state, double *unused, double *out_7091429700375966071);
void car_H_30(double *state, double *unused, double *out_8103669929918075243);
void car_h_26(double *state, double *unused, double *out_2949657553689358892);
void car_H_26(double *state, double *unused, double *out_2554523730447597736);
void car_h_27(double *state, double *unused, double *out_6772766789684224867);
void car_H_27(double *state, double *unused, double *out_5928906618117650332);
void car_h_29(double *state, double *unused, double *out_6733128343877477231);
void car_H_29(double *state, double *unused, double *out_8613901274232467427);
void car_h_28(double *state, double *unused, double *out_1130036918603268032);
void car_H_28(double *state, double *unused, double *out_3531502257162936853);
void car_h_31(double *state, double *unused, double *out_4011174800524398071);
void car_H_31(double *state, double *unused, double *out_1217625550303418916);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
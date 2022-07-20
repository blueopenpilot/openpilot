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
void car_err_fun(double *nom_x, double *delta_x, double *out_6934071491810961021);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5348077128676617066);
void car_H_mod_fun(double *state, double *out_4159593663802874514);
void car_f_fun(double *state, double dt, double *out_4252897372159381429);
void car_F_fun(double *state, double dt, double *out_6678692079888871950);
void car_h_25(double *state, double *unused, double *out_422800814981887828);
void car_H_25(double *state, double *unused, double *out_6712579314593980538);
void car_h_24(double *state, double *unused, double *out_7125751847150321414);
void car_H_24(double *state, double *unused, double *out_4486871530615111976);
void car_h_30(double *state, double *unused, double *out_4487868591059444478);
void car_H_30(double *state, double *unused, double *out_204111026897636217);
void car_h_26(double *state, double *unused, double *out_7108536964396069770);
void car_H_26(double *state, double *unused, double *out_6055725250483668634);
void car_h_27(double *state, double *unused, double *out_180515069694589689);
void car_H_27(double *state, double *unused, double *out_1970652284902788694);
void car_h_29(double *state, double *unused, double *out_3128241060050074650);
void car_H_29(double *state, double *unused, double *out_714342371212028401);
void car_h_28(double *state, double *unused, double *out_2856270981749742996);
void car_H_28(double *state, double *unused, double *out_4368056645857502173);
void car_h_31(double *state, double *unused, double *out_5737408863670873035);
void car_H_31(double *state, double *unused, double *out_6681933352717020110);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
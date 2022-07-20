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
void car_err_fun(double *nom_x, double *delta_x, double *out_9182764351597017505);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6627670019370406735);
void car_H_mod_fun(double *state, double *out_1096019154694829530);
void car_f_fun(double *state, double dt, double *out_4339405799423572566);
void car_F_fun(double *state, double dt, double *out_346308560399216339);
void car_h_25(double *state, double *unused, double *out_2234606782992478127);
void car_H_25(double *state, double *unused, double *out_6386336540791844398);
void car_h_24(double *state, double *unused, double *out_3192731147905955648);
void car_H_24(double *state, double *unused, double *out_7313898205951198184);
void car_h_30(double *state, double *unused, double *out_1863696515286893737);
void car_H_30(double *state, double *unused, double *out_3868003582284595771);
void car_h_26(double *state, double *unused, double *out_7118272656719947028);
void car_H_26(double *state, double *unused, double *out_8318904214043650994);
void car_h_27(double *state, double *unused, double *out_8537932094985248772);
void car_H_27(double *state, double *unused, double *out_1644409511100652554);
void car_h_29(double *state, double *unused, double *out_8262738032700742883);
void car_H_29(double *state, double *unused, double *out_3357772237970203587);
void car_h_28(double *state, double *unused, double *out_2906049274931237065);
void car_H_28(double *state, double *unused, double *out_8440171255039734161);
void car_h_31(double *state, double *unused, double *out_1959412720707972238);
void car_H_31(double *state, double *unused, double *out_7692696111810299518);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
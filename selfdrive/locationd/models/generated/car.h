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
void car_err_fun(double *nom_x, double *delta_x, double *out_4219125439312905275);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8640473231899427946);
void car_H_mod_fun(double *state, double *out_6403599997404205727);
void car_f_fun(double *state, double dt, double *out_8494721620328521265);
void car_F_fun(double *state, double dt, double *out_7925611537347342114);
void car_h_25(double *state, double *unused, double *out_8319850071724330228);
void car_H_25(double *state, double *unused, double *out_3040935136717207570);
void car_h_24(double *state, double *unused, double *out_5122675272519993604);
void car_H_24(double *state, double *unused, double *out_868285537711708004);
void car_h_30(double *state, double *unused, double *out_8595044134008836117);
void car_H_30(double *state, double *unused, double *out_5559268095224456197);
void car_h_26(double *state, double *unused, double *out_2716758646450121029);
void car_H_26(double *state, double *unused, double *out_700568182156848654);
void car_h_27(double *state, double *unused, double *out_6629402236988649268);
void car_H_27(double *state, double *unused, double *out_7782862166408399414);
void car_h_29(double *state, double *unused, double *out_9024150106849758680);
void car_H_29(double *state, double *unused, double *out_6069499439538848381);
void car_h_28(double *state, double *unused, double *out_6815668569060056703);
void car_H_28(double *state, double *unused, double *out_987100422469317807);
void car_h_31(double *state, double *unused, double *out_2233954507100698029);
void car_H_31(double *state, double *unused, double *out_1326776284390200130);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
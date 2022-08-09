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
void car_err_fun(double *nom_x, double *delta_x, double *out_4341611149106626338);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6563642220978654068);
void car_H_mod_fun(double *state, double *out_1862272950594096159);
void car_f_fun(double *state, double dt, double *out_4060615684978379852);
void car_F_fun(double *state, double dt, double *out_7512294491876261158);
void car_h_25(double *state, double *unused, double *out_5848465558922810029);
void car_H_25(double *state, double *unused, double *out_2604461143443793040);
void car_h_24(double *state, double *unused, double *out_7826187274481621079);
void car_H_24(double *state, double *unused, double *out_431811544438293474);
void car_h_30(double *state, double *unused, double *out_2211016030668787679);
void car_H_30(double *state, double *unused, double *out_5122794101951041667);
void car_h_26(double *state, double *unused, double *out_7212689298329633324);
void car_H_26(double *state, double *unused, double *out_1137042175430263184);
void car_h_27(double *state, double *unused, double *out_8885251610367142714);
void car_H_27(double *state, double *unused, double *out_2948030790150616756);
void car_h_29(double *state, double *unused, double *out_2790657236992057966);
void car_H_29(double *state, double *unused, double *out_5633025446265433851);
void car_h_28(double *state, double *unused, double *out_4219625227456378339);
void car_H_28(double *state, double *unused, double *out_550626429195903277);
void car_h_31(double *state, double *unused, double *out_6123659621207315918);
void car_H_31(double *state, double *unused, double *out_1763250277663614660);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
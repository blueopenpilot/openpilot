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
void car_err_fun(double *nom_x, double *delta_x, double *out_6300774429342187872);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7490438056241178548);
void car_H_mod_fun(double *state, double *out_8524316325449410804);
void car_f_fun(double *state, double dt, double *out_3805144880965371850);
void car_F_fun(double *state, double dt, double *out_5441793289359834623);
void car_h_25(double *state, double *unused, double *out_4139015218604585631);
void car_H_25(double *state, double *unused, double *out_3381867638513875071);
void car_h_24(double *state, double *unused, double *out_2169929365393919604);
void car_H_24(double *state, double *unused, double *out_1156159854535006509);
void car_h_30(double *state, double *unused, double *out_3753924276279937728);
void car_H_30(double *state, double *unused, double *out_3534822702977741684);
void car_h_26(double *state, double *unused, double *out_8755597543940783373);
void car_H_26(double *state, double *unused, double *out_7123370957387931295);
void car_h_27(double *state, double *unused, double *out_1102229167160252946);
void car_H_27(double *state, double *unused, double *out_1360059391177316773);
void car_h_29(double *state, double *unused, double *out_7690057093636548109);
void car_H_29(double *state, double *unused, double *out_4045054047292133868);
void car_h_28(double *state, double *unused, double *out_158719314258590511);
void car_H_28(double *state, double *unused, double *out_5435702352761764834);
void car_h_31(double *state, double *unused, double *out_3863821156320079742);
void car_H_31(double *state, double *unused, double *out_3351221676636914643);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
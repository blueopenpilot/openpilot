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
void car_err_fun(double *nom_x, double *delta_x, double *out_3756630299165339778);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1504140271042908252);
void car_H_mod_fun(double *state, double *out_862615575614504655);
void car_f_fun(double *state, double dt, double *out_8976431745750036532);
void car_F_fun(double *state, double dt, double *out_8449402309040632318);
void car_h_25(double *state, double *unused, double *out_7280144249566122930);
void car_H_25(double *state, double *unused, double *out_5359450340106912024);
void car_h_24(double *state, double *unused, double *out_1595796958856727745);
void car_H_24(double *state, double *unused, double *out_3186800741101412458);
void car_h_30(double *state, double *unused, double *out_7529150295889406336);
void car_H_30(double *state, double *unused, double *out_6170603392111022837);
void car_h_26(double *state, double *unused, double *out_6361406494813264599);
void car_H_26(double *state, double *unused, double *out_8663976309867712625);
void car_h_27(double *state, double *unused, double *out_3639637011742573612);
void car_H_27(double *state, double *unused, double *out_8345366703911447748);
void car_h_29(double *state, double *unused, double *out_2527477028228560691);
void car_H_29(double *state, double *unused, double *out_5660372047796630653);
void car_h_28(double *state, double *unused, double *out_8865544227577898559);
void car_H_28(double *state, double *unused, double *out_3305615625859022261);
void car_h_31(double *state, double *unused, double *out_601128401187926255);
void car_H_31(double *state, double *unused, double *out_6010618483090822339);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
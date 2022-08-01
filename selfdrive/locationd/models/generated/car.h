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
void car_err_fun(double *nom_x, double *delta_x, double *out_8546428699701313973);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6276729264926819565);
void car_H_mod_fun(double *state, double *out_7387032395725791226);
void car_f_fun(double *state, double dt, double *out_7992121703102262561);
void car_F_fun(double *state, double dt, double *out_6052186748360722795);
void car_h_25(double *state, double *unused, double *out_5253203141964689396);
void car_H_25(double *state, double *unused, double *out_6823297006306510332);
void car_h_24(double *state, double *unused, double *out_4448088281771825364);
void car_H_24(double *state, double *unused, double *out_2478580357452164122);
void car_h_30(double *state, double *unused, double *out_9114886618393191048);
void car_H_30(double *state, double *unused, double *out_4304964047799261705);
void car_h_26(double *state, double *unused, double *out_6406977070144503880);
void car_H_26(double *state, double *unused, double *out_7881943748528985060);
void car_h_27(double *state, double *unused, double *out_8127123413026519497);
void car_H_27(double *state, double *unused, double *out_6479727359599686616);
void car_h_29(double *state, double *unused, double *out_8728541570639883051);
void car_H_29(double *state, double *unused, double *out_3794732703484869521);
void car_h_28(double *state, double *unused, double *out_2901216197191194550);
void car_H_28(double *state, double *unused, double *out_8877131720554400095);
void car_h_31(double *state, double *unused, double *out_5202243027854662809);
void car_H_31(double *state, double *unused, double *out_7255735646295633584);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
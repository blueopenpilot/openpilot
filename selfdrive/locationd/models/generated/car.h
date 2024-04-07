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
void car_err_fun(double *nom_x, double *delta_x, double *out_4793426118989952565);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6104751408298337559);
void car_H_mod_fun(double *state, double *out_6067088922067929627);
void car_f_fun(double *state, double dt, double *out_1262970013950409256);
void car_F_fun(double *state, double dt, double *out_8280434690380054427);
void car_h_25(double *state, double *unused, double *out_3171541189463855340);
void car_H_25(double *state, double *unused, double *out_2582795789210079086);
void car_h_24(double *state, double *unused, double *out_4459677364439223850);
void car_H_24(double *state, double *unused, double *out_2112338307166740362);
void car_h_30(double *state, double *unused, double *out_3446735251748361229);
void car_H_30(double *state, double *unused, double *out_1944900540917529112);
void car_h_26(double *state, double *unused, double *out_738825703499674061);
void car_H_26(double *state, double *unused, double *out_1158707529663977138);
void car_h_27(double *state, double *unused, double *out_4651469294038202300);
void car_H_27(double *state, double *unused, double *out_278693530266414105);
void car_h_29(double *state, double *unused, double *out_4050051136424838746);
void car_H_29(double *state, double *unused, double *out_1434669196603136928);
void car_h_28(double *state, double *unused, double *out_8569367563836024369);
void car_H_28(double *state, double *unused, double *out_6517068213672667502);
void car_h_31(double *state, double *unused, double *out_6635325407527154392);
void car_H_31(double *state, double *unused, double *out_2613441751087039514);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
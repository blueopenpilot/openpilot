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
void car_err_fun(double *nom_x, double *delta_x, double *out_1093100257688424678);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7128642225905965818);
void car_H_mod_fun(double *state, double *out_5507642244994157908);
void car_f_fun(double *state, double dt, double *out_8894714846457968717);
void car_F_fun(double *state, double dt, double *out_1917681428035716347);
void car_h_25(double *state, double *unused, double *out_3196602313497850705);
void car_H_25(double *state, double *unused, double *out_6748259161611645143);
void car_h_24(double *state, double *unused, double *out_5915140457056590213);
void car_H_24(double *state, double *unused, double *out_94053703503459386);
void car_h_30(double *state, double *unused, double *out_3471796375782356594);
void car_H_30(double *state, double *unused, double *out_6877598108754885213);
void car_h_26(double *state, double *unused, double *out_6076201696591054245);
void car_H_26(double *state, double *unused, double *out_7956981593223850249);
void car_h_27(double *state, double *unused, double *out_9074887801056565793);
void car_H_27(double *state, double *unused, double *out_9052361420555310124);
void car_h_29(double *state, double *unused, double *out_8473469643443202239);
void car_H_29(double *state, double *unused, double *out_7681019926284690459);
void car_h_28(double *state, double *unused, double *out_4145949056817660876);
void car_H_28(double *state, double *unused, double *out_8802093875859534906);
void car_h_31(double *state, double *unused, double *out_6753481078947960919);
void car_H_31(double *state, double *unused, double *out_6717613199734684715);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
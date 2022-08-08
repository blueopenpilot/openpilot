#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_176423441279690910);
void live_err_fun(double *nom_x, double *delta_x, double *out_395211094905203923);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6850906830985321928);
void live_H_mod_fun(double *state, double *out_2691954792996225698);
void live_f_fun(double *state, double dt, double *out_3501586882147416184);
void live_F_fun(double *state, double dt, double *out_3915126576134129947);
void live_h_4(double *state, double *unused, double *out_2264850770044554603);
void live_H_4(double *state, double *unused, double *out_2654413244157288387);
void live_h_9(double *state, double *unused, double *out_3285427884274290167);
void live_H_9(double *state, double *unused, double *out_2895602890786879032);
void live_h_10(double *state, double *unused, double *out_6183432949031568228);
void live_H_10(double *state, double *unused, double *out_5611567429780513543);
void live_h_12(double *state, double *unused, double *out_5583145542829180464);
void live_H_12(double *state, double *unused, double *out_7673869652189250182);
void live_h_31(double *state, double *unused, double *out_8056048439282908788);
void live_H_31(double *state, double *unused, double *out_8027311389195287725);
void live_h_32(double *state, double *unused, double *out_7764141664932751882);
void live_H_32(double *state, double *unused, double *out_3504568180820147535);
void live_h_13(double *state, double *unused, double *out_4193995487034710069);
void live_H_13(double *state, double *unused, double *out_4714218927543448096);
void live_h_14(double *state, double *unused, double *out_3285427884274290167);
void live_H_14(double *state, double *unused, double *out_2895602890786879032);
void live_h_33(double *state, double *unused, double *out_557239165020941256);
void live_H_33(double *state, double *unused, double *out_4876754384556430121);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
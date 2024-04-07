#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_628998680425083766);
void live_err_fun(double *nom_x, double *delta_x, double *out_2969844009246816994);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6027517651849506129);
void live_H_mod_fun(double *state, double *out_6084748629706323649);
void live_f_fun(double *state, double dt, double *out_8886463940381424804);
void live_F_fun(double *state, double dt, double *out_1566583358217669108);
void live_h_4(double *state, double *unused, double *out_4975326173156221970);
void live_H_4(double *state, double *unused, double *out_7234840091630714929);
void live_h_9(double *state, double *unused, double *out_7102841024691433696);
void live_H_9(double *state, double *unused, double *out_3924685046814389217);
void live_h_10(double *state, double *unused, double *out_6448122657399315695);
void live_H_10(double *state, double *unused, double *out_4270753952895197954);
void live_h_12(double *state, double *unused, double *out_5263026406180287830);
void live_H_12(double *state, double *unused, double *out_3544775668396386195);
void live_h_35(double *state, double *unused, double *out_2590326542821036459);
void live_H_35(double *state, double *unused, double *out_7845241924706229311);
void live_h_32(double *state, double *unused, double *out_4358040035946591900);
void live_H_32(double *state, double *unused, double *out_7714077139765488842);
void live_h_13(double *state, double *unused, double *out_974357172067582595);
void live_H_13(double *state, double *unused, double *out_2502389047595765410);
void live_h_14(double *state, double *unused, double *out_7102841024691433696);
void live_H_14(double *state, double *unused, double *out_3924685046814389217);
void live_h_33(double *state, double *unused, double *out_8528125957484447016);
void live_H_33(double *state, double *unused, double *out_4694684920067371707);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
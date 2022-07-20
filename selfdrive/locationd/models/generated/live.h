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
void live_H(double *in_vec, double *out_6020406760525914227);
void live_err_fun(double *nom_x, double *delta_x, double *out_9138946898377382373);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1766051097771838348);
void live_H_mod_fun(double *state, double *out_7757453199381973644);
void live_f_fun(double *state, double dt, double *out_6862179030632812745);
void live_F_fun(double *state, double dt, double *out_5806104157926584478);
void live_h_4(double *state, double *unused, double *out_4436485360583885785);
void live_H_4(double *state, double *unused, double *out_6123477027156270371);
void live_h_9(double *state, double *unused, double *out_6246065433276342271);
void live_H_9(double *state, double *unused, double *out_1163741908108177099);
void live_h_10(double *state, double *unused, double *out_1526413025883775463);
void live_H_10(double *state, double *unused, double *out_6038771845523629986);
void live_h_12(double *state, double *unused, double *out_6580810709086629191);
void live_H_12(double *state, double *unused, double *out_5942008669510548249);
void live_h_31(double *state, double *unused, double *out_2208032687669611621);
void live_H_31(double *state, double *unused, double *out_4289214318851193830);
void live_h_32(double *state, double *unused, double *out_501696487008393722);
void live_H_32(double *state, double *unused, double *out_1244939009647749363);
void live_h_13(double *state, double *unused, double *out_1162208089473883596);
void live_H_13(double *state, double *unused, double *out_5054413420910978744);
void live_h_14(double *state, double *unused, double *out_6246065433276342271);
void live_H_14(double *state, double *unused, double *out_1163741908108177099);
void live_h_33(double *state, double *unused, double *out_7457613365932383325);
void live_H_33(double *state, double *unused, double *out_7439771323490051434);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
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
void live_H(double *in_vec, double *out_1048484788621772472);
void live_err_fun(double *nom_x, double *delta_x, double *out_2871014111866515098);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6489216175832462602);
void live_H_mod_fun(double *state, double *out_1652960104217739196);
void live_f_fun(double *state, double dt, double *out_9221450714756977716);
void live_F_fun(double *state, double dt, double *out_9213270367014775596);
void live_h_4(double *state, double *unused, double *out_6182698249901673968);
void live_H_4(double *state, double *unused, double *out_8378509393902577523);
void live_h_9(double *state, double *unused, double *out_4560294093115925512);
void live_H_9(double *state, double *unused, double *out_8137319747272986878);
void live_h_10(double *state, double *unused, double *out_7031730563044228516);
void live_H_10(double *state, double *unused, double *out_5893418408071830239);
void live_h_12(double *state, double *unused, double *out_7742411024452946790);
void live_H_12(double *state, double *unused, double *out_3359052985870615728);
void live_h_35(double *state, double *unused, double *out_1687077408921857505);
void live_H_35(double *state, double *unused, double *out_5011847336529970147);
void live_h_32(double *state, double *unused, double *out_3262888130587692097);
void live_H_32(double *state, double *unused, double *out_4880682551589229678);
void live_h_13(double *state, double *unused, double *out_6744278987412836798);
void live_H_13(double *state, double *unused, double *out_8476575654478173343);
void live_h_14(double *state, double *unused, double *out_4560294093115925512);
void live_H_14(double *state, double *unused, double *out_8137319747272986878);
void live_h_33(double *state, double *unused, double *out_4007001513162065970);
void live_H_33(double *state, double *unused, double *out_1861290331891112543);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
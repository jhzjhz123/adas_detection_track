#ifndef DISTANCE_MEASURE_H
#define DISTANCE_MEASURE_H

#include "common.h"

HV_S32 distance_measure(HV_S32 i_in, HV_S32 j_in, HV_S32 &x_distance, HV_S32 &y_distance);

HV_HVID distance_measure_mat(HV_S32 i_in, HV_S32 j_in, HV_S32 &x_distance, HV_S32 &y_distance);

HV_S32 invert_distance_measure(HV_S32 x_distance, HV_S32 y_distance, HV_S32 &i_out, HV_S32 &j_out);

HV_HVID get_real_position(HV_S32 i_in, HV_S32 j_in, HV_F32 &y_world, HV_F32 &z_world, HV_S32 &i_out, HV_S32 & j_out, HV_F32 *camera_world_matrix);

HV_HVID store_correct_img(HV_BYTE *img, HV_S32 width, HV_S32 height, HV_F32 *camera_world_matrix);

HV_HVID perspective_mat(HV_F32 A[][9], HV_S32 equ, HV_S32 var, HV_F32* ans);

HV_HVID get_perspective_matrix();

HV_HVID perspective_transform(HV_S32 i_in, HV_S32 j_in, HV_S32 &i_out, HV_S32 &j_out, HV_F32 *mat);

HV_HVID img_perspective_transform(HV_BYTE *img_in, HV_BYTE *img_out);

#endif
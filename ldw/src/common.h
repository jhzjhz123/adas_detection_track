#ifndef COMMON_H
#define COMMON_H

#include <opencv2/opencv.hpp>
#include <iostream>
//#include <iterator>
//#include <vector>
//#include <string>
//#include <io.h>
#include <stdio.h>

#ifdef _WIN32
#include <io.h>
#endif

#include "../hvapi/hvType.h"

#define _CRT_SECURE_NO_WARNINGS

typedef HV_BYTE byte;
using namespace std;
using namespace cv;
#define ORI_IMG_WIDTH 1280
#define ORI_IMG_HEIGHT 720
#define SCALE_RATIO 1
#define IMG_WIDTH 1280/SCALE_RATIO
#define IMG_HEIGHT 720/SCALE_RATIO
#define FILTER_EN 1

#define max(a, b)  (((a) > (b)) ? (a) : (b))
#define min(a, b)  (((a) < (b)) ? (a) : (b))

//#define DIST_0 12	//distance of first segment in meter
//#define DIST_1 20
//#define DIST_2 30
//#define DIST_4 40
#define SEGMENT_NUM 5
//#define CAMERA_POSITION -128	//distance between camera and mid of the vehicle, camera on right is negative

#define LANE_DIST 3.5

//#define ROI_TOP 500/SCALE_RATIO	//425
#define ROI_DOWN 718/SCALE_RATIO	//650
#define ROI_TOP_LEFT 2
#define ROI_TOP_RIGHT 1277/SCALE_RATIO
#define ROI_DOWN_LEFT 2
#define ROI_DOWN_RIGHT 1277/SCALE_RATIO
#define MAX_VAL 99999
#define MIN_VAL 0.0001

//#define SEG_DIST_MID 13		//can use decimal
//#define SEG_DIST_SIDE 13
//#define DETECT_DIST_MID 40
//#define DETECT_DIST_SIDE 40

//use coordinate directly	//500/425, 530/480
//#define SEG_DIST_MID 500/SCALE_RATIO		//can use decimal
//#define DETECT_DIST_MID 425/SCALE_RATIO
//#define SEG_DIST_SIDE 425/SCALE_RATIO
//#define DETECT_DIST_SIDE 425/SCALE_RATIO

typedef struct image
{
	HV_BYTE *y_in;
	HV_BYTE *u_in;
	HV_BYTE *v_in;
	HV_S32 width;
	HV_S32 height;
}image_t;

typedef struct lane
{
	HV_S32 detected;
	//pos in ori image, just for debug
	HV_S32 ori_bot_j, ori_top_j, ori_mid_j;
	HV_S32 bot_j, mid_j, top_j;
	HV_S32 degree;
	//j=a0+a1*i+a2*i*2, in bird view coordinate
	HV_F32 a0, a1, a2, a3;
	//input: 1:solid, 2: dash, 3: roadside, 4: ldrs, 5:lsrd
	HV_S32 lane_type;	//0: solidline, 1: dashed line, 2: roadside
	HV_S32 type_change_pos;
	HV_S32 tracking_num;	
	HV_S32 hold_num;
	HV_S32 bot_end, top_end;
}lane_t;

typedef struct zebra_area
{
	HV_S32 top;
	HV_S32 bot;
	HV_S32 left;
	HV_S32 right;
}zebra_area_t;

typedef struct stop_line
{
	HV_S32 detected;
	HV_F32 a0;
	HV_F32 a1;
	HV_S32 degree;
}stop_line_t;

typedef struct point
{
	HV_F32 i;
	HV_F32 j;
	HV_S32 degree;
}point_t;



#include "lane_detect.h"
#include "distance_measure.h"
#include "lane_tracking.h"
#include "kalman.h"
#include "lane_class.h"

HV_HVID initial_parameter();

HV_HVID initial_lane(lane_t *lane[4]);

HV_HVID initial_lane_based_on_ref(lane_t *lane, lane_t *ref_lane, HV_F32 offset_dist);

HV_HVID update_ref_lane(lane_t *cur_lanes[], lane_t *ref_lanes[]);

HV_S32 get_pixel(image_t img_in, HV_S32 i, HV_S32 j);

HV_S32 get_pixel(HV_BYTE *img_in, HV_S32 i, HV_S32 j);

HV_HVID set_pixel(image_t img_in, HV_S32 i, HV_S32 j, HV_S32 val);

HV_HVID matrix_multiply(HV_F32 *a_in, HV_F32 *b_in, HV_F32 *out, HV_S32 dim0, HV_S32 dim1, HV_S32 dim2);

HV_HVID matrix_multiply(HV_S32 *a_in, HV_S32 *b_in, HV_S32 *out, HV_S32 dim0, HV_S32 dim1, HV_S32 dim2);

HV_HVID matrix_trans(HV_S32 *in, HV_S32 *out, HV_S32 dim0, HV_S32 dim1);

HV_HVID matrix_inv(HV_F32 *a_in, HV_F32 *a_out);

HV_HVID draw_point(HV_S32 ori_size, HV_BYTE *img_in, HV_BYTE *uv_in, HV_S32 i_in, HV_S32 j_in, HV_BYTE y_val, HV_BYTE u_val, HV_BYTE v_val);

HV_HVID draw_lane(HV_S32 ori_size, HV_BYTE *img_in, HV_BYTE *uv_in, lane_t *lane, HV_BYTE y_val, HV_BYTE u_val, HV_BYTE v_val);

HV_HVID draw_lane_world_param(HV_BYTE *img_in, lane_t *lane);

HV_HVID init_image(image_t *img_in, HV_S32 width, HV_S32 height);

HV_HVID release_image(image_t *img_in);

HV_S32 calc_angle(HV_S32 height, HV_S32 j_0, HV_S32 j_1);

HV_S32 grad_to_angle(HV_F32 grad);

HV_F32 angle_to_grad(HV_S32 angle);

vector<string> get_files(string path);

HV_HVID mat_to_yuv(HV_BYTE *yuv_img, Mat *mat_img, HV_S32 width, HV_S32 height);

HV_HVID yuv_to_mat(Mat *mat_img, HV_BYTE *yuv_img);

image_t yuv_to_img(HV_BYTE *yuv_img);

HV_HVID img_scaling(HV_BYTE *img_in, HV_BYTE *img_out);

HV_S32 get_lane_pos(lane_t *lane, HV_S32 i);

bool sort_degree(const lane_t &lane1, const lane_t &lane2);

HV_S32 calc_lane_diff(lane_t lane0, lane_t lane1);

HV_HVID draw_zebra(HV_BYTE* img_in, vector<zebra_area_t> *zebra_areas);

HV_HVID transfer_to_out_param(lane_t cur_lane, HV_F32 out_param[7]);

HV_HVID save_width_map(HV_BYTE *width_img);

HV_HVID draw_stop_line(HV_BYTE *img_in, stop_line_t *stop_line);

HV_HVID sort_lane(vector<lane_t> *lanes);

#endif
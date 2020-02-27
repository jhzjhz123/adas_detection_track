#include "lane_class.h"
#include "../hvapi/hvLanedet.h"


lane_detect_c::lane_detect_c(HV_INIT_USERDATA * pUserData)
{
	mat_input_en_ = 1;
	//camera_yaw = -5.194, camera_pitch = -0, camera_roll = -1;	//201
	//camera_yaw_ = -3.28, camera_pitch_ = 0.31, camera_roll_ = 1.64;	//veran
	//camera_yaw = 4.763175, camera_pitch = -3.5, camera_roll = -1.88;	//tiguan
	//camera_yaw = -0.468, camera_pitch = -5.322, camera_roll = 0.6838;	//srx

	camera_yaw_ = (*pUserData).camera_yaw, camera_pitch_ = (*pUserData).camera_pitch, camera_roll_ = (*pUserData).camera_roll;
	g_k1_ = (*pUserData).g_k1, g_k2_ = (*pUserData).g_k2, g_k3_ = (*pUserData).g_k3;
	g_p1_ = (*pUserData).g_p1, g_p2_ = (*pUserData).g_p2;
	g_fx_ = (*pUserData).g_fx, g_fy_ = (*pUserData).g_fy;
	g_cx_ = (*pUserData).g_cx, g_cy_ = (*pUserData).g_cy;
	g_camera_position_ = (*pUserData).g_camera_position * 256;
	g_camera_height_ = (*pUserData).g_camera_height * 256;
	g_camera_head_ = (*pUserData).g_camera_head * 256;

    hv_clock = pUserData->hv_clock;
    hv_printf = pUserData->hv_printf;
    hv_rand = pUserData->hv_rand;

	HV_S32 i;
	for (i = 0; i < 4; i++)
	{
		cur_lanes_[i] = new lane_t;
		ref_lanes_[i] = new lane_t;
		tracking_lanes_[i] = new lane_t;
		tracking_lanes_[i]->detected = 0;
	}

	initial_param();

}

lane_detect_c::~lane_detect_c()
{
	HV_S32 i;
	for (i = 0; i < 4; i++)
	{
		delete cur_lanes_[i];
		delete ref_lanes_[i];
		delete tracking_lanes_[i];
	}
}

HV_HVID lane_detect_c::initial_param()
{
	extern HV_F32 g_k1, g_k2, g_k3;
	extern HV_F32 g_p1, g_p2;
	extern HV_S32 g_fx, g_fy;
	extern HV_S32 g_cx, g_cy;
	extern HV_S32 g_camera_position;
	extern HV_S32 g_camera_height;
	extern HV_S32 g_camera_head;

	/*update the paramater in distance_measure.cpp*/
	g_k1 = g_k1_, g_k2 = g_k2_, g_k3 = g_k3_;
	g_p1 = g_p1_, g_p2 = g_p2_;
	g_fx = g_fx_, g_fy = g_fy_;
	g_cx = g_cx_, g_cy = g_cy_;
	g_camera_position = g_camera_position_;
	g_camera_height = g_camera_height_;
	g_camera_head = g_camera_head_;

	initial_parameter();
	extern HV_S32 g_ref_lane_dist;
	extern HV_F32 g_y_dist_in_perspective;
	HV_S32 width = IMG_WIDTH;
	g_ref_lane_dist = 3.5*width / 2 / g_y_dist_in_perspective;

	extern HV_S32 g_vanish_i, g_vanish_j;
	extern HV_F32 g_sin_camera_yaw, g_cos_camera_yaw, g_sin_camera_pitch, g_cos_camera_pitch,
		g_sin_camera_roll, g_cos_camera_roll, g_sin_vehicle_pitch, g_cos_vehicle_pitch,
		g_sin_vehicle_roll, g_cos_vehicle_roll;

	g_sin_camera_yaw = sin(camera_yaw_*3.1415926 / 180);
	g_cos_camera_yaw = cos(camera_yaw_*3.1415926 / 180);
	g_sin_camera_pitch = sin(camera_pitch_*3.1415926 / 180);
	g_cos_camera_pitch = cos(camera_pitch_*3.1415926 / 180);
	g_sin_camera_roll = sin(camera_roll_*3.1415926 / 180);
	g_cos_camera_roll = cos(camera_roll_*3.1415926 / 180);


	extern HV_F32 camera_world_matrix[];
	extern HV_F32 world_camera_matrix[];
	HV_F32 pitch_matrix[9] = { g_cos_camera_pitch*g_cos_vehicle_pitch - g_sin_camera_pitch*g_sin_camera_pitch, 0, g_sin_camera_pitch*g_cos_vehicle_pitch + g_cos_camera_pitch*g_sin_vehicle_pitch,
		0, 1, 0,
		-g_sin_camera_pitch*g_cos_vehicle_pitch - g_cos_camera_pitch*g_sin_vehicle_pitch, 0, g_cos_camera_pitch*g_cos_vehicle_pitch - g_sin_camera_pitch*g_sin_vehicle_pitch };
	HV_F32 row_matrix[9] = { 1, 0, 0,
		0, g_cos_camera_roll*g_cos_vehicle_roll - g_sin_camera_roll*g_sin_vehicle_roll , -g_sin_camera_roll*g_cos_vehicle_roll - g_cos_camera_roll*g_sin_vehicle_roll,
		0, g_sin_camera_roll*g_cos_vehicle_roll + g_cos_camera_roll*g_sin_vehicle_roll, g_cos_camera_roll*g_cos_vehicle_roll + g_sin_camera_roll*g_sin_vehicle_roll };
	HV_F32 yaw_matrix[9] = { g_cos_camera_yaw, -g_sin_camera_yaw, 0,
		g_sin_camera_yaw, g_cos_camera_yaw, 0,
		0, 0, 1 };
	HV_F32 temp_matrix[9];
	matrix_multiply(row_matrix, pitch_matrix, temp_matrix, 3, 3, 3);
	matrix_multiply(yaw_matrix, temp_matrix, camera_world_matrix, 3, 3, 3);



	matrix_inv(camera_world_matrix, world_camera_matrix);
	//calc farmost point on image coordinate
	invert_distance_measure(999999999, 0, g_vanish_i, g_vanish_j);
	get_perspective_matrix();

	HV_S32 temp_j;
	extern HV_F32 g_x_dist_in_perspective;
	extern HV_S32 g_roi_top;
	invert_distance_measure(g_x_dist_in_perspective * 256, 0, g_roi_top, temp_j);
	g_roi_top /= SCALE_RATIO;




	extern HV_S32 g_seg_dist_mid, g_detect_dist_mid, g_seg_dist_side, g_detect_dist_side;


	invert_distance_measure(20 * 256, 0, g_seg_dist_mid, temp_j);
	invert_distance_measure(45 * 256, 0, g_detect_dist_mid, temp_j);
	g_seg_dist_mid /= SCALE_RATIO;
	g_detect_dist_mid /= SCALE_RATIO;
	//g_seg_dist_mid = g_detect_dist_mid;
	g_seg_dist_side = g_detect_dist_mid;
	g_detect_dist_side = g_detect_dist_mid;

	HV_S32 vanish_i0, vanish_i1, vanish_j0, vanish_j1;
	extern HV_F32 g_perspective_mat[9];
	perspective_transform(IMG_HEIGHT - 1, 0, vanish_i0, vanish_j0, g_perspective_mat);
	perspective_transform(IMG_HEIGHT*0.75, 0, vanish_i1, vanish_j1, g_perspective_mat);
	extern HV_F32 vanish_a0, vanish_a1;
	vanish_a1 = HV_F32(vanish_j0 - vanish_j1) / HV_F32(vanish_i0 - vanish_i1);
	vanish_a0 = HV_F32(vanish_j0) - HV_F32(vanish_a1*vanish_i0);

	initial_lane(ref_lanes_);

	//get default cross point
	HV_F32 tan_yaw = g_sin_camera_yaw / g_cos_camera_yaw;
	HV_F32 tan_pitch = g_sin_camera_pitch / g_cos_camera_pitch;
	cross_i_ = g_cy_ - tan_pitch * g_fy_;
	cross_j_ = g_cx_ + tan_yaw * g_fx_;
}

HV_HVID lane_detect_c::lane_detect_frame(HV_HVID *frame_input)
{
	clock_t start, end;

	//extern HV_F32 camera_world_matrix[];
	//store_correct_img((HV_BYTE*)frame_input, 1280, 720, camera_world_matrix);

	extern HV_COMMON_API g_common_api;
	start = g_common_api.hv_clock();



	end = g_common_api.hv_clock();
	//cout << end - start << ": transfer input" << endl;

	extern HV_S32 g_framenum;

	start = g_common_api.hv_clock();
	lane_detect((HV_BYTE*)frame_input, cur_lanes_, ref_lanes_);
	end = g_common_api.hv_clock();
	//cout << end - start << ": detect" << endl;
	//return;

	

	start = g_common_api.hv_clock();
	if (g_framenum == 52)
	{
		int qqq = 3;
	}
	update_ref_lanes(cur_lanes_, ref_lanes_);

	extern HV_S32 g_ref_lane_dist;
	extern HV_F32 g_lane_dist_filter;
	HV_S32 cur_ref_lane_dist;
	if (ref_lanes_[1]->detected == 1 && ref_lanes_[2]->detected == 1)
	{
		cur_ref_lane_dist = calc_lane_dist(*ref_lanes_[2], *ref_lanes_[1]);
		g_ref_lane_dist = g_lane_dist_filter*g_ref_lane_dist + (1 - g_lane_dist_filter)*cur_ref_lane_dist;
	}
	//use 280 as default
	g_ref_lane_dist = max(220 / SCALE_RATIO, min(340 / SCALE_RATIO, g_ref_lane_dist));

	end = g_common_api.hv_clock();
	//cout << end - start << ": update" << endl;



	start = g_common_api.hv_clock();

#ifndef video_input
	//draw four lanes
	/*for (HV_S32 i = 0; i < 4; i++)
	{
		if (ref_lanes_[i]->lane_type == 0)
			draw_lane(1, (HV_BYTE*)frame_input, 0, ref_lanes_[i], 80, 128, 255);
		else if (ref_lanes_[i]->lane_type == 1)
			draw_lane(1, (HV_BYTE*)frame_input, 0, ref_lanes_[i], 255, 0, 0);
		else if (ref_lanes_[i]->lane_type == 2)
			draw_lane(1, (HV_BYTE*)frame_input, 0, ref_lanes_[i], 0, 255, 128);
	}*/
#endif

	point_t cur_cross = get_cross_point((HV_BYTE*)frame_input);
	if (cur_cross.i != -1 && abs(cur_cross.i - cross_i_)<100 && (cur_cross.j - cross_j_)<100)
	{
		cross_i_ = 0.995 * cross_i_ + 0.005 * cur_cross.i;
		cross_j_ = 0.995 * cross_j_ + 0.005 * cur_cross.j;
		HV_F32 tan_pitch = (g_cy_ - cross_i_) / g_fy_;
		HV_F32 tan_yaw = (cross_j_ - g_cx_) / g_fx_;
		camera_pitch_ = atan(tan_pitch) * 180 / 3.1415926;
		camera_yaw_ = atan(tan_yaw) * 180 / 3.1415926;
	}

	//delete[] buffer_in;
	end = g_common_api.hv_clock();
	//cout << end - start << ": transfer output" << endl;

}

point_t lane_detect_c::get_cross_point(HV_BYTE* frame_input)
{
	for (int k = 0; k < 7; k++)
	{
		//draw_point(frame_input, 0, cross_i_+k, cross_j_, 255, 0, 0);
	}

	point_t ret;
	ret.i = -1;
	ret.j = -1;
	if (ref_lanes_[1]->detected == 0 || ref_lanes_[2]->detected == 0)
		return ret;
	//mid two lanes should long enough
	if(ref_lanes_[1]->bot_end-ref_lanes_[1]->top_end<250|| ref_lanes_[2]->bot_end - ref_lanes_[2]->top_end<250)
		return ret;
	//mid two lanes should be straight
	if(abs(ref_lanes_[1]->a2)>1e-4|| abs(ref_lanes_[2]->a2)>1e-4)
		return ret;
	//mid two lanes should parallel to vehicle
	if (abs(ref_lanes_[1]->a1)>0.2 || abs(ref_lanes_[2]->a1)>0.2)
		return ret;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 min_j_dist = width - 1;
	HV_S32 cross_i = -1, cross_j = -1;
	extern HV_F32 g_perspective_mat[9];
	extern HV_F32 g_inv_perspective_mat[9];
	extern HV_S32 g_roi_top;

	HV_S32 i_bird, j_bird;
	HV_S32 i00, j00, i01, j01, i02, j02;
	HV_S32 i10, j10, i11, j11, i12, j12;
	i_bird = 0;
	j_bird = ref_lanes_[1]->a0 + ref_lanes_[1]->a1*i_bird + ref_lanes_[1]->a2*i_bird*i_bird;
	perspective_transform(i_bird, j_bird, i00, j00, g_inv_perspective_mat);
	j_bird = ref_lanes_[2]->a0 + ref_lanes_[2]->a1*i_bird + ref_lanes_[2]->a2*i_bird*i_bird;
	perspective_transform(i_bird, j_bird, i10, j10, g_inv_perspective_mat);

	i_bird = height*4/5;
	j_bird = ref_lanes_[1]->a0 + ref_lanes_[1]->a1*i_bird + ref_lanes_[1]->a2*i_bird*i_bird;
	perspective_transform(i_bird, j_bird, i01, j01, g_inv_perspective_mat);
	j_bird = ref_lanes_[2]->a0 + ref_lanes_[2]->a1*i_bird + ref_lanes_[2]->a2*i_bird*i_bird;
	perspective_transform(i_bird, j_bird, i11, j11, g_inv_perspective_mat);

	i00 = HV_F32(i00);
	i01 = HV_F32(i01);
	j00 = HV_F32(j00);
	j01 = HV_F32(j01);
	i10 = HV_F32(i10);
	i11 = HV_F32(i11);
	j10 = HV_F32(j10);
	j11 = HV_F32(j11);
	//just use two points to get a straight line
	HV_F32 cur_a01 = HV_F32(j00 - j01) / HV_F32(i00 - i01);
	HV_F32 cur_a00 = HV_F32(j00) - cur_a01*HV_F32(i00);
	HV_F32 cur_a11 = HV_F32(j10 - j11) / HV_F32(i10 - i11);
	HV_F32 cur_a10 = HV_F32(j10) - cur_a11*HV_F32(i10);
	if (cur_a11 == cur_a01)
		return ret;
	cross_i = (cur_a10 - cur_a00) / (cur_a01 - cur_a11);
	cross_j = cur_a00 + cur_a01 * cross_i;
	if (cross_i>=0&&cross_i<height-6&&cross_j>=0&&cross_j<width)
	{
		for (int k = 0; k < 7; k++)
		{
			//draw_point(frame_input, 0, cross_i + k, cross_j, 80, 128, 255);
		}
		ret.i = cross_i;
		ret.j = cross_j;
	}
	return ret;
}
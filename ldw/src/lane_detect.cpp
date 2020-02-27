#include "lane_detect.h"
#include "../hvapi/hvLanedet.h"

//#define SAVE_EDGE
//#define FPGA

HV_S32 g_ref_lane_dist;	//lane distance in bird view image in pixel
HV_F32 g_lane_dist_filter = 0.8;
HV_S32 g_lane_detect_thresh = 20;
HV_S32 g_bot_lane_detect_thresh = 30;
//FILE *fp_edge = fopen("e:\\yuv\\lane\\edge.yuv", "wb");
//FILE *fp_candidate = fopen("e:\\yuv\\lane\\candidate.yuv", "wb");

HV_COMMON_API g_common_api;

extern HV_S32 g_vanish_i, g_vanish_j;
HV_S32 g_framenum;


HV_BYTE g_ori_bird_img[IMG_WIDTH*IMG_HEIGHT];
//HV_BYTE g_ori_img[IMG_WIDTH*IMG_HEIGHT];


HV_S32 g_canny_thred;	//threshold of graident for edge

HV_F32 filter_beta = 0.5;
HV_S32 filter_left_left_bot_j = 0, filter_left_left_bot_angle = 0, filter_left_left_top_angle = 0;
HV_S32 filter_left_bot_j = 0, filter_left_bot_angle = 0, filter_left_top_angle = 0;
HV_S32 filter_right_bot_j = 0, filter_right_bot_angle = 0, filter_right_top_angle = 0;
HV_S32 filter_right_right_bot_j = 0, filter_right_right_bot_angle = 0, filter_right_right_top_angle = 0;

HV_F32 vanish_a0, vanish_a1;	//vanish line of bird image
HV_S32 g_roi_top = 425;



HV_HVID lane_detect(HV_BYTE *img_in, lane_t *cur_lanes[4], lane_t *ref_lanes[4])
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 i, j, k;
	clock_t start, end;

	for (k = 0; k < 4; k++)
	{
		cur_lanes[k]->detected = 0;
	}



	/*HV_BYTE *scaling_img = new HV_BYTE[IMG_WIDTH*IMG_HEIGHT*3/2];
	img_scaling(img_in, scaling_img);*/

	


	memset(g_ori_bird_img, 0, width*height*sizeof(HV_BYTE));
	//copy ori image to find solid/dash lane
	//memcpy(g_ori_img, img_in, width*height*sizeof(HV_BYTE));



	start = g_common_api.hv_clock();
	vector<point_t> *candidate_points[4];
	for (int i = 0; i < 4; i++)
	{
		candidate_points[i] = new vector<point_t>;
	}
	vector<lane_t> *candidate_lanes = new vector<lane_t>;
	canny_transfer(img_in, candidate_points, ref_lanes);

	get_candidate_lanes(candidate_points, candidate_lanes);

	//sort(candidate_lanes->begin(), candidate_lanes->end(), sort_degree);
	sort_lane(candidate_lanes);

	delete_overlapped_lanes(candidate_lanes);

	//draw candidate lanes
	/*for (i = 0; i < candidate_lanes->size(); i++)
	{
		draw_lane(0, img_in, 0, &(*candidate_lanes)[i], 255, 255, 255);
	}*/
	//fwrite(img_in, width*height * 3 / 2, sizeof(byte), fp_candidate);

	end = g_common_api.hv_clock();
	//g_common_api.hv_printf("%d: canny\n", end - start);
	if (g_framenum == 72)
	{
		int qqq = 3;
	}
	//select four lanes
	select_four_lanes(candidate_lanes, cur_lanes, ref_lanes);


	/*if (cur_lanes[1]->detected == 0)
		cur_lanes[0]->detected = 0;
	if (cur_lanes[2]->detected == 0)
		cur_lanes[3]->detected = 0;*/

	start = g_common_api.hv_clock();
	get_lane_type(cur_lanes);
	end = g_common_api.hv_clock();
	//g_common_api.hv_printf("%d: lant type\n", end - start);

	for (int k = 0; k < 4; k++)
	{
		candidate_points[k]->clear();
		delete candidate_points[k];
	}
	candidate_lanes->clear();
	delete candidate_lanes;
	//delete[] scaling_img;
	end = g_common_api.hv_clock();
	//cout << end - start << ": get candidate lanes" << endl;

}

HV_HVID canny_transfer(HV_BYTE *img_in, vector<point_t> *candidate_points[4], lane_t *ref_lanes[4])
{
	extern HV_S32 g_i_offset;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;

	//each line remain one pixel
	//should add fitting area in fitting lane
	//slow!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	HV_S32 mult_val = 0;
#ifdef _WIN32
	mult_val = 1;
#else
	mult_val = 3;
#endif
	for (int i = 0; i < height - g_i_offset; i++)
	{
		HV_S32 cur_start = 0, cur_end = 0, has_roadside=0;
		for (int j = 0; j < width-1; j++)
		{
			if (g_framenum == 31 && i == 489 && j == 849)
			{
				int qqq = 3;
			}
			if (img_in[mult_val*(i*width + j)] <20 && img_in[mult_val*(i*width + j + 1)] >=20)
			{
				cur_start = j;
				has_roadside = 0;
			}
			else if (img_in[mult_val*(i*width + j)] >= 20 && img_in[mult_val*(i*width + j + 1)] <20)
			{
				// HV_S32 cur_value = img_in[mult_val*(i*width + (j + cur_start >> 1))];
				// //roadside prior
				// if (has_roadside == 1)
				// 	cur_value = 100;
				// memset(img_in + mult_val*(i*width + cur_start + 1), 0, mult_val*((j - cur_start) * sizeof(HV_BYTE)));
				
				// //output middle of the lane
				// img_in[mult_val*(i*width + (j + cur_start >> 1))] = cur_value;
				// //output inner side of the lane
				// /*if(j<width/2)
				// 	img_in[mult_val*(i*width + j)] = cur_value;
				// else
				// 	img_in[mult_val*(i*width + cur_start)] = cur_value;*/
			}
			
			if (img_in[mult_val*(i*width + j)] > 75 && img_in[mult_val*(i*width + j)] < 125)
				has_roadside = 1;
		}
	}

	//fwrite(img_in, width*height, sizeof(HV_BYTE), fp_edge);
	//bird transfer
	img_perspective_transform(img_in, g_ori_bird_img);

	//store remain points
	for (int i = height - 1; i >= 0; i--)
	{
		//ref lane position on current i
		int ref_j[4];
		for (int k = 0; k < 4; k++)
		{
			ref_j[k] = ref_lanes[k]->a0 + ref_lanes[k]->a1*i + ref_lanes[k]->a2*i*i;
		}
		//segment position for candidate points on current i
		int seg_j[3];
		for (int k = 0; k < 3; k++)
		{
			seg_j[k] = ref_j[k] + ref_j[k + 1] >> 1;
		}
		int seg_index = 3;
		for (int j = 0; j < width; j++)
		{
			if (g_ori_bird_img[i*width + j] == 0)
				continue;
			for (int k = 0; k < 3; k++)
			{
				if (j < seg_j[k])
				{
					seg_index = k;
					break;
				}
			}
			point_t cur_point;
			cur_point.i = i;
			cur_point.j = j;
			candidate_points[seg_index]->push_back(cur_point);
		}
	}


	//fwrite(g_ori_bird_img, width*height, sizeof(HV_BYTE), fp_edge);


}

HV_HVID get_candidate_lanes(vector<point_t> *candidate_points[4], vector<lane_t> *candidate_lanes)
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	for (int area_index = 0; area_index < 4; area_index++)
	{
		HV_S32 points_num = candidate_points[area_index]->size();
		if (points_num == 0)
			continue;

		HV_S32 loop_num = 0;
		HV_S32 invalid_loop_num = 0;
		//fitting straight
		while (loop_num < 30)
		{
			HV_S32 point0 = g_common_api.hv_rand() % points_num;
			HV_S32 point1 = g_common_api.hv_rand() % points_num;
			HV_F32 i0 = (*candidate_points[area_index])[point0].i;
			HV_F32 j0 = (*candidate_points[area_index])[point0].j;
			HV_F32 i1 = (*candidate_points[area_index])[point1].i;
			HV_F32 j1 = (*candidate_points[area_index])[point1].j;
			invalid_loop_num++;
			if (invalid_loop_num > 200)
				break;
			if (abs(i0 - i1)<25 || abs(j0 - j1)>50)
				continue;
			//cur iter is mid lane
			loop_num++;
			HV_F32 cur_a1 = HV_F32(j0 - j1) / HV_F32(i0 - i1);
			HV_F32 cur_a0 = HV_F32(j0) - cur_a1*HV_F32(i0);
			HV_F32 cur_a2 = 0;
			lane_t cur_fitting_lane = fitting_lane(cur_a0, cur_a1, cur_a2, 0, area_index, 3);
			if (cur_fitting_lane.detected == 1)
				candidate_lanes->push_back(cur_fitting_lane);
		}

		//continue;
		//fitting curve
		loop_num = 0;
		invalid_loop_num = 0;
		while (loop_num < 30)
		{
			HV_S32 point0 = g_common_api.hv_rand() % points_num;
			HV_S32 point1 = g_common_api.hv_rand() % points_num;
			HV_S32 point2 = g_common_api.hv_rand() % points_num;
			HV_F32 i0 = (*candidate_points[area_index])[point0].i;
			HV_F32 j0 = (*candidate_points[area_index])[point0].j;
			HV_F32 i1 = (*candidate_points[area_index])[point1].i;
			HV_F32 j1 = (*candidate_points[area_index])[point1].j;
			HV_F32 i2 = (*candidate_points[area_index])[point2].i;
			HV_F32 j2 = (*candidate_points[area_index])[point2].j;
			invalid_loop_num++;
			if (invalid_loop_num > 500)
				break;
			if (abs(i0 - i1) < 50 || abs(i0 - i2) < 50 || abs(i1 - i2) < 50
				|| abs(j0 - j1) > 50 || abs(j0 - j2) > 50 || abs(j1 - j2) > 50)
				continue;
			loop_num++;
			HV_F32 cur_a0, cur_a1, cur_a2;
			curve_fitting(cur_a0, cur_a1, cur_a2, i0, j0, i1, j1, i2, j2);
			lane_t cur_fitting_lane = fitting_lane(cur_a0, cur_a1, cur_a2, 0, area_index, 2);
			if (cur_fitting_lane.detected == 1)
				candidate_lanes->push_back(cur_fitting_lane);
		}
	}
}

//hold_en=0 fitting in g_edge_img, hold_en=1 fitting in g_unfilted_edge_img
lane_t fitting_lane(HV_F32 cur_a0, HV_F32 cur_a1, HV_F32 cur_a2, HV_S32 hold_en, HV_S32 area_index, HV_S32 fitting_area)
{
	if (g_framenum == 17 && cur_a0 > 1511.5&&cur_a0<1511.6&&cur_a1>-2.88&&cur_a1 < -2.89)
	{
		int qqq = 3;
	}
	lane_t return_lane;
	return_lane.detected = 0;
	extern HV_F32 g_inv_perspective_mat[9], g_perspective_mat[9];
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 det_thresh = g_lane_detect_thresh / 2 / SCALE_RATIO;
	//HV_S32 fitting_area = hold_en==0?1:10;
	//fitting_area can use 0 because store wide edge image
	//HV_S32 fitting_area = 1;
	extern HV_F32 g_x_min_dist_in_perspective, g_x_dist_in_perspective;
	//20m
	HV_S32 seg_pos = height - (17 - g_x_min_dist_in_perspective)*height / (g_x_dist_in_perspective - g_x_min_dist_in_perspective);
	HV_S32 i, j, k;
	HV_S32 cur_fitting_num = 0;
	HV_S32 cur_bot_fitting_num = 0;

	HV_F32 top_grad, bot_grad;	//both where lane reach border of the img
	HV_F32 temp_val;

	HV_S32 bottest_pos = -1, toppest_pos = MAX_VAL;

	//calc intersection point of cur lane and vanish lane
	HV_S32 inter_i = get_inter_lane_in_bird(cur_a0, cur_a1, cur_a2);

	top_grad = cur_a1 + 2 * cur_a2 * 0;
	bot_grad = cur_a1 + 2 * cur_a2 * height;

	HV_S32 bot_angle, top_angle;
	if (bot_grad != 0)
		bot_angle = grad_to_angle(-1 / bot_grad);
	else
		bot_angle = 90;
	if (top_grad != 0)
		top_angle = grad_to_angle(-1 / top_grad);
	else
		top_angle = 90;
	//strict angle of the lane

	//more error detect in rainy day when bot_angle restrict is not severe, miss some curve road otherwise
	//annotate to avoid missing when vehicle is declining
	//if (bot_angle<70 || bot_angle>110)
	//	return return_lane;

	//calc the gradient at bottom
	HV_S32 dash_num = 0, roadside_num=0;
	int top_end = -1, bot_end = -1;
	HV_S32 last_fit_i = -1, last_fit_j = -1;
	HV_S32 end_fitting = 0;
	for (i = height - 1; i >= 0; i--)
	{
		if (i == 650)
		{
			HV_S32 qqq = 3;
		}
		HV_S32 cur_j = cur_a0 + cur_a1*HV_F32(i) + cur_a2*HV_F32(i)*HV_F32(i);
		if (cur_j < fitting_area || cur_j >= width - fitting_area)
			continue;
		for (j = cur_j - fitting_area; j <= cur_j + fitting_area; j++)
		{
			if (g_ori_bird_img[i*width + j] != 0)
			{
				if (last_fit_i != -1 && abs(i - last_fit_i) + abs(j - last_fit_j)>200 / SCALE_RATIO)
				{
					end_fitting = 1;
					break;
				}
				if (bot_end == -1)
					bot_end = i;
				top_end = i;
				cur_fitting_num++;
				last_fit_i = i;
				last_fit_j = j;
				break;
			}
		}
		if (end_fitting == 1)
			break;
	}
	//side lanes should have bigger degree
	extern HV_F32 g_y_dist_in_perspective;
	HV_S32 i_ori, j_ori, i_bird, j_bird;
	perspective_transform(height, width / 2, i_bird, j_bird, g_perspective_mat);
	j_bird = cur_a0 + cur_a1*i_bird + cur_a2*i_bird*i_bird;
	perspective_transform(i_bird, j_bird, i_ori, j_ori, g_inv_perspective_mat);

	//*2 if bot exceeds image
	if (j_ori < 0 || j_ori > width)
		cur_fitting_num *= 2;

	//add degree for side lanes
	/*if (area_index==0 || area_index==3)
		cur_fitting_num *= 2;*/

	if (cur_fitting_num > det_thresh)
	{
		return_lane.a0 = cur_a0, return_lane.a1 = cur_a1, return_lane.a2 = cur_a2;
		return_lane.a3 = 0;
		return_lane.bot_j = cur_a0 + cur_a1*height + cur_a2*height*height;
		return_lane.mid_j = cur_a0 + cur_a1*height / 2 + cur_a2*height*height / 4;
		return_lane.top_j = cur_a0;
		//return_lane.bot_angle = bot_angle;
		//return_lane.top_angle = top_angle;		//pos on border of the image
		return_lane.detected = 1;
		return_lane.tracking_num = 999;
		return_lane.bot_end = bot_end;
		return_lane.top_end = top_end;

		//degree will be bigger when angle difference is small
		//degree decrease to 0.5 when angle_diff is 25
		//cur_lane.degree = cur_fitting_num*(-0.02*angle_diff + 1);
		//cur_lane.bot_degree = cur_bot_fitting_num*(-0.02*angle_diff + 1);
		return_lane.degree = cur_fitting_num;
		return_lane.lane_type = 0;
		/*if(dash_num*32/cur_fitting_num>4)
			return_lane.lane_type = 1;
		if (roadside_num * 32 / cur_fitting_num>4)
			return_lane.lane_type = 2;*/


		HV_S32 i_bird, j_bird;
		HV_S32 i_ori, j_ori;
		//perspective_transform(719, width / 2, i_bird, j_bird, g_perspective_mat);
		j_bird = return_lane.a0 + return_lane.a1*bot_end + return_lane.a2*bot_end*bot_end;
		perspective_transform(bot_end, j_bird, i_ori, return_lane.ori_bot_j, g_inv_perspective_mat);

		//perspective_transform(572, width / 2, i_bird, j_bird, g_perspective_mat);
		//j_bird = return_lane.a0 + return_lane.a1*i_bird + return_lane.a2*i_bird*i_bird;
		//perspective_transform(i_bird, j_bird, i_ori, return_lane.ori_mid_j, g_inv_perspective_mat);

		//perspective_transform(426, width / 2, i_bird, j_bird, g_perspective_mat);
		j_bird = return_lane.a0 + return_lane.a1*top_end + return_lane.a2*top_end*top_end;
		perspective_transform(top_end, j_bird, i_ori, return_lane.ori_top_j, g_inv_perspective_mat);
		return_lane.detected = 1;

	}
	return return_lane;
}

HV_HVID delete_overlapped_lanes(vector<lane_t> *candidate_lanes)
{
	HV_S32 i, j, k0, k1;
	extern HV_F32 g_y_dist_in_perspective;
	for (k0 = 0; k0 < candidate_lanes->size(); k0++)
	{
		lane_t ref_lane = (*candidate_lanes)[k0];
		for (k1 = k0 + 1; k1 < candidate_lanes->size(); k1++)
		{
			lane_t cur_lane = (*candidate_lanes)[k1];
			HV_S32 cur_diff = calc_lane_diff(ref_lane, cur_lane);
			if (cur_diff < 400 / g_y_dist_in_perspective)
			{
				candidate_lanes->erase(candidate_lanes->begin() + k1);
				k1--;
			}
		}
	}
}

HV_HVID select_four_lanes(vector<lane_t> *candidate_lanes, lane_t *cur_lanes[4], lane_t *ref_lanes[4])
{
	if (candidate_lanes->size() == 0)
		return;
	HV_S32 lane_num = candidate_lanes->size();
	HV_S32 i, j, k;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	extern HV_F32 g_y_dist_in_perspective;
	//ref pixel distance between two lanes
	//HV_S32 ref_dist = 3.5*width / 2 / g_y_dist_in_perspective;
	HV_S32 min_ref_dist = g_ref_lane_dist*0.5, max_ref_dist = g_ref_lane_dist*1.5;
	HV_S32 tracking_thresh = 10;
	HV_F32 parallel_degree, close_degree;
	HV_S32 cur_ref_lane_dist = 3.5*width / 2 / g_y_dist_in_perspective;

	//none valid ref lane, compare to default
	//if (ref_lanes[0]->detected == 0 && ref_lanes[1]->detected == 0 && ref_lanes[2]->detected == 0 && ref_lanes[3]->detected == 0)
	
	//if mid lane all missing, compare to default, no meaning only tracking side lane
	if (ref_lanes[1]->detected == 0 && ref_lanes[2]->detected == 0)
	{
		//find most matching lane to any ref, other lanes use this lane as ref
		//most important procedure, better than old method, which find each lane based on correspond ref
		HV_S32 max_matching_degree = 0;
		lane_t cur_lane = (*candidate_lanes)[0];
		if (cur_lane.degree <= g_lane_detect_thresh)
			return;
		HV_S32 matching_index = 0;
		HV_S32 min_val = abs(cur_lane.bot_j - ref_lanes[0]->bot_j);
		for (k = 1; k<4; k++)
		{
			HV_S32 cur_val = abs(cur_lane.bot_j - ref_lanes[k]->bot_j);
			if (cur_val < min_val)
			{
				min_val = cur_val;
				matching_index = k;
			}
		}
		memcpy(cur_lanes[matching_index], &(*candidate_lanes)[0], sizeof(lane_t));
		lane_t cur_ref_lane;
		memcpy(&cur_ref_lane, cur_lanes[matching_index], sizeof(lane_t));
		candidate_lanes->erase(candidate_lanes->begin());

		//find other three lanes based on first matching lane
		HV_S32 changeable[4] = { 0 };	//whther four position around matching lane is fixed
		HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
		memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
		for (i = 0; i < candidate_lanes->size(); i++)
		{
			lane_t cur_lane = (*candidate_lanes)[i];
			//parallel_degrees only store valid lanes
			if (cur_lane.degree < g_lane_detect_thresh)
				continue;
			if (i == 5 && k == 2)
			{
				HV_S32 qqq = 3;
			}
			lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
			//if (parallel_degree>0.5)
				parallel_degrees[i] = cur_lane.degree;
		}
		//each iter find remain lane with biggest degree
		for (i = 0; i < candidate_lanes->size(); i++)
		{
			//find current biggest parallel degree
			HV_S32 max_degree = 0, max_index = -1;
			for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
			{
				if (parallel_degrees[i1] > max_degree)
				{
					max_degree = parallel_degrees[i1];
					max_index = i1;
				}
			}
			if (max_degree == 0)
				break;
			lane_t cur_lane = (*candidate_lanes)[max_index];
			parallel_degrees[max_index] = 0;
			HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
			//must right on the already matching lane
			if (cur_dist>min_ref_dist&&cur_dist < min_ref_dist * 2)
			{
				if (matching_index < 3 && (cur_lanes[matching_index + 1]->detected == 0 ||
					(changeable[2] == 1 && calc_lane_dist(*cur_lanes[matching_index + 2], cur_lane)>min_ref_dist)
					))
				{
					memcpy(cur_lanes[matching_index + 1], &cur_lane, sizeof(lane_t));
					changeable[2] = 0, changeable[3] = 0;
				}
			}
			//may on right or right right
			else if (cur_dist >= min_ref_dist * 2 && cur_dist <= max_ref_dist)
			{
				if (matching_index < 2)
				{
					//just when right two lanes all empty will set changeable to 1
					if (cur_lanes[matching_index + 1]->detected == 0 && cur_lanes[matching_index + 2]->detected == 0)
					{
						memcpy(cur_lanes[matching_index + 1], &cur_lane, sizeof(lane_t));
						memcpy(cur_lanes[matching_index + 2], &cur_lane, sizeof(lane_t));
						changeable[2] = 1, changeable[3] = 1;
					}
					else if (cur_lanes[matching_index + 1]->detected == 0
						&& calc_lane_dist(*cur_lanes[matching_index + 2], cur_lane)>min_ref_dist)
					{
						memcpy(cur_lanes[matching_index + 1], &cur_lane, sizeof(lane_t));
					}
					else if (cur_lanes[matching_index + 2]->detected == 0
						&& calc_lane_dist(cur_lane, *cur_lanes[matching_index + 1])>min_ref_dist)
					{
						memcpy(cur_lanes[matching_index + 2], &cur_lane, sizeof(lane_t));
					}
				}
				//can just on right
				else if (matching_index < 3)
				{
					if (cur_lanes[matching_index + 1]->detected == 0)
					{
						memcpy(cur_lanes[matching_index + 1], &cur_lane, sizeof(lane_t));
					}
				}
			}
			//must on right right
			else if (cur_dist > max_ref_dist&&cur_dist<2 * max_ref_dist)
			{
				if (matching_index < 2 && (cur_lanes[matching_index + 2]->detected == 0 ||
					(changeable[3] == 1 && calc_lane_dist(cur_lane, *cur_lanes[matching_index + 1])>min_ref_dist)))
				{
					memcpy(cur_lanes[matching_index + 2], &cur_lane, sizeof(lane_t));
					changeable[2] = 0, changeable[3] = 0;
				}
			}
			//must on left
			else if (cur_dist<-min_ref_dist&&cur_dist > -min_ref_dist * 2)
			{
				if (matching_index >0 && (cur_lanes[matching_index - 1]->detected == 0
					|| (changeable[1] == 1 && calc_lane_dist(cur_lane, *cur_lanes[matching_index - 2])>min_ref_dist)))
				{
					memcpy(cur_lanes[matching_index - 1], &cur_lane, sizeof(lane_t));
					changeable[0] = 0, changeable[1] = 0;
				}
			}
			//may on left or left left
			else if (cur_dist <= -min_ref_dist * 2 && cur_dist >= -max_ref_dist)
			{
				if (matching_index >1)
				{
					//just when right two lanes all empty will set changeable to 1
					if (cur_lanes[matching_index - 1]->detected == 0 && cur_lanes[matching_index - 2]->detected == 0)
					{
						memcpy(cur_lanes[matching_index - 1], &cur_lane, sizeof(lane_t));
						memcpy(cur_lanes[matching_index - 2], &cur_lane, sizeof(lane_t));
						changeable[0] = 1, changeable[1] = 1;
					}
					else if (cur_lanes[matching_index - 1]->detected == 0
						&& calc_lane_dist(cur_lane, *cur_lanes[matching_index - 2])>min_ref_dist)
					{
						memcpy(cur_lanes[matching_index - 1], &cur_lane, sizeof(lane_t));
					}
					else if (cur_lanes[matching_index - 2]->detected == 0
						&& calc_lane_dist(*cur_lanes[matching_index - 1], cur_lane)>min_ref_dist)
					{
						memcpy(cur_lanes[matching_index - 2], &cur_lane, sizeof(lane_t));
					}
				}
				//can just on left
				else if (matching_index >0)
				{
					if (cur_lanes[matching_index - 1]->detected == 0)
					{
						memcpy(cur_lanes[matching_index - 1], &cur_lane, sizeof(lane_t));
					}
				}
			}
			//must on left left
			else if (cur_dist < -max_ref_dist&&cur_dist>-2 * max_ref_dist)
			{
				if (matching_index >1 && (cur_lanes[matching_index - 2]->detected == 0 ||
					(changeable[0] == 1 && calc_lane_dist(*cur_lanes[matching_index - 1], cur_lane)>min_ref_dist)))
				{
					memcpy(cur_lanes[matching_index - 2], &cur_lane, sizeof(lane_t));
					changeable[0] = 0, changeable[1] = 0;
				}
			}
		}
		//one lane in two position, delete side one
		if (changeable[0] == 1 && changeable[1] == 1)
		{
			cur_lanes[0]->detected = 0;
		}
		if (changeable[2] == 1 && changeable[3] == 1)
		{
			cur_lanes[3]->detected = 0;
		}
		delete[] parallel_degrees;

	}
	//exist valid ref lane, find each valid ref lane's best matching........................
	else
	{
		if (g_framenum == 43)
		{
			HV_S32 qqq = 3;
		}
		//matching lanes with valid ref lane
		for (k = 0; k < 4; k++)
		{
			if (ref_lanes[k]->detected == 0)
				continue;
			lane_t cur_ref_lane;
			memcpy(&cur_ref_lane, ref_lanes[k], sizeof(lane_t));
			HV_S32 max_degree = 0, max_index = -1;
			//matching to both default and ref, 
			//if correspond position to default changes should chang position in cur_lanes
			for (i = 0; i < candidate_lanes->size(); i++)
			{
				if (g_framenum == 18 && k == 3 && i == 2)
				{
					HV_S32 qqq = 3;
				}
				lane_t cur_lane;
				memcpy(&cur_lane, &(*candidate_lanes)[i], sizeof(lane_t));
				lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
				//far lanes will not consider
				if (close_degree < 0.6)
					close_degree = 0;
				HV_S32 cur_degree = cur_lane.degree*close_degree*parallel_degree;
				if (cur_degree > max_degree)
				{
					max_degree = cur_degree;
					max_index = i;
				}
			}
			if (max_index == -1)
				continue;

		

			if ((*candidate_lanes)[max_index].degree > g_lane_detect_thresh / 2)
			{
				memcpy(cur_lanes[k], &(*candidate_lanes)[max_index], sizeof(lane_t));
				candidate_lanes->erase(candidate_lanes->begin() + max_index);
			}
		}
		if (g_framenum == 43)
		{
			int qqq = 3;
		}

		//if mid two lanes all matching, decide whether exist a lane between
		if (cur_lanes[1]->detected == 1 && cur_lanes[2]->detected == 1
			&& cur_lanes[2]->bot_j - cur_lanes[1]->bot_j>2 * min_ref_dist)
		{
			lane_t cur_ref_lane;
			if (cur_lanes[1]->degree > cur_lanes[2]->degree)
				memcpy(&cur_ref_lane, cur_lanes[1], sizeof(lane_t));
			else
				memcpy(&cur_ref_lane, cur_lanes[2], sizeof(lane_t));
			HV_S32 max_degree = 0, max_index = -1;
			for (i = 0; i < candidate_lanes->size(); i++)
			{
				lane_t cur_lane = (*candidate_lanes)[i];
				HV_S32 left_dist = calc_lane_dist(cur_lane, *cur_lanes[1]);
				HV_S32 right_dist = calc_lane_dist(*cur_lanes[2], cur_lane);
				if (left_dist < min_ref_dist || right_dist < min_ref_dist
					||left_dist > max_ref_dist || right_dist > max_ref_dist
					|| cur_lane.degree < g_lane_detect_thresh)
					continue;
				lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
				HV_S32 cur_degree = 0;
				//if (parallel_degree>0.5)
					cur_degree = cur_lane.degree*parallel_degree;
				if (cur_degree > max_degree)
				{
					max_degree = cur_degree;
					max_index = i;
				}
			}
			if (max_index != -1)
			{
				//add left lane
				if ((*candidate_lanes)[max_index].bot_j<width / 2)
				{
					memcpy(cur_lanes[0], cur_lanes[1], sizeof(lane_t));
					memcpy(cur_lanes[1], &(*candidate_lanes)[max_index], sizeof(lane_t));
				}
				else
				{
					memcpy(cur_lanes[3], cur_lanes[2], sizeof(lane_t));
					memcpy(cur_lanes[2], &(*candidate_lanes)[max_index], sizeof(lane_t));
				}
			}

		}

		if (g_framenum == 60)
		{
			HV_S32 qqq = 3;
		}
		//matching remain lanes
		//left two lanes all missing should matching to right lane together
		if (cur_lanes[0]->detected == 0 && cur_lanes[1]->detected == 0)
		{
			if (cur_lanes[2]->detected == 1)
			{
				HV_S32 changeable[2] = { 0 };
				lane_t cur_ref_lane = *cur_lanes[2];
				HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
				memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					lane_t cur_lane = (*candidate_lanes)[i];
					//parallel_degrees only store valid lanes
					if (cur_lane.degree < g_lane_detect_thresh)
						continue;
					if (i == 5 && k == 2)
					{
						HV_S32 qqq = 3;
					}
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist > -min_ref_dist|| cur_dist < -2 * max_ref_dist)
						continue;
					//lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
					//if (parallel_degree>0.5)
						parallel_degrees[i] = cur_lane.degree;
				}
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					HV_S32 max_degree = 0, max_index = -1;
					for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
					{
						if (parallel_degrees[i1] > max_degree)
						{
							max_degree = parallel_degrees[i1];
							max_index = i1;
						}
					}
					if (max_index == -1)
						continue;
					lane_t cur_lane = (*candidate_lanes)[max_index];
					parallel_degrees[max_index] = 0;
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					//only on left
					if (cur_dist < -min_ref_dist&&cur_dist > -2 * min_ref_dist)
					{
						if (cur_lanes[1]->detected == 0
							|| (changeable[1] == 1 && calc_lane_dist(cur_lane, *cur_lanes[0])>min_ref_dist))
						{
							memcpy(cur_lanes[1], &cur_lane, sizeof(lane_t));
							if (changeable[1] == 0)
							{
								changeable[0] = 0;
								changeable[1] = 0;
							}
						}
					}
					//may on left or left left
					else if (cur_dist <= -2 * min_ref_dist&&cur_dist >= -max_ref_dist)
					{
						if (cur_lanes[0]->detected == 0 && cur_lanes[1]->detected == 0)
						{
							memcpy(cur_lanes[0], &cur_lane, sizeof(lane_t));
							memcpy(cur_lanes[1], &cur_lane, sizeof(lane_t));
							changeable[0] = 1, changeable[1] = 1;
						}
						else if (cur_lanes[0]->detected == 0 && calc_lane_dist(*cur_lanes[1], cur_lane)>min_ref_dist)
						{
							memcpy(cur_lanes[0], &cur_lane, sizeof(lane_t));
						}
						else if (cur_lanes[1]->detected == 0 && calc_lane_dist(cur_lane, *cur_lanes[0])>min_ref_dist)
						{
							memcpy(cur_lanes[1], &cur_lane, sizeof(lane_t));
						}
					}
					//only on left left
					else if (cur_dist < -max_ref_dist&&cur_dist > -2 * max_ref_dist)
					{
						if (cur_lanes[0]->detected == 0
							|| (changeable[0] == 1 && calc_lane_dist(*cur_lanes[1], cur_lane)>min_ref_dist))
						{
							memcpy(cur_lanes[0], &cur_lane, sizeof(lane_t));
							if (changeable[0] == 0)
							{
								changeable[0] = 0;
								changeable[1] = 0;
							}
						}
					}
				}
				//one lane in two position, delete side one
				if (changeable[0] == 1 && changeable[1] == 1)
				{
					cur_lanes[0]->detected = 0;
				}
				delete[] parallel_degrees;
			}
		}
		//left only matching to right
		else if (cur_lanes[1]->detected == 0)
		{
			if (cur_lanes[2]->detected == 1)
			{
				lane_t cur_ref_lane = *cur_lanes[2];
				HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
				memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					lane_t cur_lane = (*candidate_lanes)[i];
					//parallel_degrees only store valid lanes
					if (cur_lane.degree < g_lane_detect_thresh)
						continue;
					if (i == 5 && k == 2)
					{
						HV_S32 qqq = 3;
					}
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist > -min_ref_dist || cur_dist < -max_ref_dist)
						continue;
					//lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
					//if (parallel_degree>0.5)
						parallel_degrees[i] = cur_lane.degree;
				}
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					HV_S32 max_degree = 0, max_index = -1;
					for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
					{
						if (parallel_degrees[i1] > max_degree)
						{
							max_degree = parallel_degrees[i1];
							max_index = i1;
						}
					}
					if (max_index == -1)
						continue;
					lane_t cur_lane = (*candidate_lanes)[max_index];
					parallel_degrees[max_index] = 0;
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist < -min_ref_dist&&cur_dist > -max_ref_dist)
					{
						memcpy(cur_lanes[1], &cur_lane, sizeof(lane_t));
						break;
					}
				}
				delete[] parallel_degrees;
			}
		}
		//left left only matching to left
		else if (cur_lanes[0]->detected == 0)
		{
			if (cur_lanes[1]->detected == 1)
			{
				lane_t cur_ref_lane = *cur_lanes[1];
				HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
				memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					lane_t cur_lane = (*candidate_lanes)[i];
					//parallel_degrees only store valid lanes
					if (cur_lane.degree < g_lane_detect_thresh)
						continue;
					if (i == 5 && k == 2)
					{
						HV_S32 qqq = 3;
					}
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist > -min_ref_dist || cur_dist < -max_ref_dist)
						continue;
					//lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
					//if (parallel_degree>0.5)
						parallel_degrees[i] = cur_lane.degree;
				}
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					HV_S32 max_degree = 0, max_index = -1;
					for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
					{
						if (parallel_degrees[i1] > max_degree)
						{
							max_degree = parallel_degrees[i1];
							max_index = i1;
						}
					}
					if (max_index == -1)
						continue;
					lane_t cur_lane = (*candidate_lanes)[max_index];
					parallel_degrees[max_index] = 0;
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist < -min_ref_dist&&cur_dist > -max_ref_dist)
					{
						memcpy(cur_lanes[0], &cur_lane, sizeof(lane_t));
						break;
					}
				}
				delete[] parallel_degrees;
			}
		}
		//left left matching and left miss
		//has bug in if (cur_dist<min_ref_dist || cur_dist>max_ref_dist)
		//annotate now, not consider this case
		/*
		else if (cur_lanes[1]->detected == 0)
		{
			//matching to right or left left
			if (cur_lanes[2]->detected == 1||cur_lanes[0]->detected==1)
			{
				lane_t cur_ref_lane;
				if (cur_lanes[2]->detected == 1)
					cur_ref_lane = *cur_lanes[2];
				else
					cur_ref_lane = *cur_lanes[0];
				HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
				memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					lane_t cur_lane = (*candidate_lanes)[i];
					//parallel_degrees only store valid lanes
					if (cur_lane.degree < g_lane_detect_thresh)
						continue;
					if (i == 5 && k == 2)
					{
						HV_S32 qqq = 3;
					}
					lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
					if (parallel_degree>0.5)
						parallel_degrees[i] = cur_lane.degree*parallel_degree;
				}
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					HV_S32 max_degree = 0, max_index = -1;
					for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
					{
						if (parallel_degrees[i1] > max_degree)
						{
							max_degree = parallel_degrees[i1];
							max_index = i1;
						}
					}
					if (max_index == -1)
						continue;
					lane_t cur_lane = (*candidate_lanes)[max_index];
					parallel_degrees[max_index] = 0;
					HV_S32 cur_dist;
					if (cur_lanes[2]->detected == 1)
						cur_dist= calc_lane_dist(cur_ref_lane, cur_lane);
					else
						cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist > min_ref_dist&&cur_dist < max_ref_dist)
					{
						HV_S32 matching_en = 1;
						//if right and left left all matching already, should also decide to left left
						if (cur_lanes[2]->detected == 1 && cur_lanes[0]->detected == 1)
						{
							cur_dist = calc_lane_dist(cur_lane, *cur_lanes[0]);
							if (cur_dist<min_ref_dist || cur_dist>max_ref_dist)
								matching_en = 0;
						}
						if (matching_en == 1)
						{
							memcpy(cur_lanes[1], &cur_lane, sizeof(lane_t));
							break;
						}
					}
				}
				delete[] parallel_degrees;
			}
		}*/

		//right two lanes all missing
		//just matching to left
		if (cur_lanes[2]->detected == 0 && cur_lanes[3]->detected == 0)
		{
			if (cur_lanes[1]->detected == 1)
			{
				HV_S32 changeable[2] = { 0 };
				lane_t cur_ref_lane = *cur_lanes[1];
				HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
				memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					lane_t cur_lane = (*candidate_lanes)[i];
					//parallel_degrees only store valid lanes
					if (cur_lane.degree < g_lane_detect_thresh)
						continue;
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist < min_ref_dist || cur_dist > 2 * max_ref_dist)
						continue;
					//lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
					//if (parallel_degree>0.5)
						parallel_degrees[i] = cur_lane.degree;
				}
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					HV_S32 max_degree = 0, max_index = -1;
					for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
					{
						if (parallel_degrees[i1] > max_degree)
						{
							max_degree = parallel_degrees[i1];
							max_index = i1;
						}
					}
					if (max_index == -1)
						continue;
					lane_t cur_lane = (*candidate_lanes)[max_index];
					parallel_degrees[max_index] = 0;
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					//only on right
					if (cur_dist > min_ref_dist&&cur_dist < 2 * min_ref_dist)
					{
						if (cur_lanes[2]->detected == 0
							|| (changeable[0] == 1 && calc_lane_dist(*cur_lanes[3], cur_lane)>min_ref_dist))
						{
							memcpy(cur_lanes[2], &cur_lane, sizeof(lane_t));
							if (changeable[0] == 0)
							{
								changeable[0] = 0;
								changeable[1] = 0;
							}
						}
					}
					//may on right or right right
					else if (cur_dist >= 2 * min_ref_dist&&cur_dist <= max_ref_dist)
					{
						if (cur_lanes[2]->detected == 0 && cur_lanes[3]->detected == 0)
						{
							memcpy(cur_lanes[2], &cur_lane, sizeof(lane_t));
							memcpy(cur_lanes[3], &cur_lane, sizeof(lane_t));
							changeable[0] = 1, changeable[1] = 1;
						}
						else if (cur_lanes[2]->detected == 0 && calc_lane_dist(*cur_lanes[3], cur_lane)>min_ref_dist)
						{
							memcpy(cur_lanes[2], &cur_lane, sizeof(lane_t));
						}
						else if (cur_lanes[3]->detected == 0 && calc_lane_dist(cur_lane, *cur_lanes[2])>min_ref_dist)
						{
							memcpy(cur_lanes[3], &cur_lane, sizeof(lane_t));
						}
					}
					//only on right right
					else if (cur_dist > max_ref_dist&&cur_dist < 2 * max_ref_dist)
					{
						if (cur_lanes[3]->detected == 0
							|| (changeable[3] == 1 && calc_lane_dist(cur_lane, *cur_lanes[2])>min_ref_dist))
						{
							memcpy(cur_lanes[3], &cur_lane, sizeof(lane_t));
							if (changeable[1] == 0)
							{
								changeable[0] = 0;
								changeable[1] = 0;
							}
						}
					}


				}
				//one lane in two position, delete side one
				if (changeable[0] == 1 && changeable[1] == 1)
				{
					cur_lanes[3]->detected = 0;
				}
				delete[] parallel_degrees;
			}
		}
		//just right miss, matching to left
		else if (cur_lanes[2]->detected == 0)
		{
			if (g_framenum == 60)
			{
				HV_S32 qqq = 3;
			}
			if (cur_lanes[1]->detected == 1)
			{
				lane_t cur_ref_lane = *cur_lanes[1];
				HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
				memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					if (i == 10)
					{
						HV_S32 qqq = 3;
					}
					lane_t cur_lane = (*candidate_lanes)[i];
					//parallel_degrees only store valid lanes
					if (cur_lane.degree < g_lane_detect_thresh)
						continue;
					if (g_framenum==52&&i == 0)
					{
						HV_S32 qqq = 3;
					}

					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist < min_ref_dist || cur_dist>max_ref_dist)
						continue;
					//lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
					//if (parallel_degree>0.5)
						parallel_degrees[i] = cur_lane.degree;
				}
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					HV_S32 max_degree = 0, max_index = -1;
					for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
					{
						if (parallel_degrees[i1] > max_degree)
						{
							max_degree = parallel_degrees[i1];
							max_index = i1;
						}
					}
					if (max_index == -1)
						continue;
					lane_t cur_lane = (*candidate_lanes)[max_index];
					parallel_degrees[max_index] = 0;
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist > min_ref_dist&&cur_dist < max_ref_dist)
					{
						memcpy(cur_lanes[2], &cur_lane, sizeof(lane_t));
						break;
					}
				}
				delete[] parallel_degrees;
			}
		}
		//just right right miss, matching to right
		else if (cur_lanes[3]->detected == 0)
		{
			if (g_framenum == 3)
			{
				HV_S32 qqq = 3;
			}
			if (cur_lanes[2]->detected == 1)
			{
				lane_t cur_ref_lane = *cur_lanes[2];
				HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
				memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					if (i == 10)
					{
						HV_S32 qqq = 3;
					}
					lane_t cur_lane = (*candidate_lanes)[i];
					//parallel_degrees only store valid lanes
					if (cur_lane.degree < g_lane_detect_thresh)
						continue;
					if (i == 5 && k == 2)
					{
						HV_S32 qqq = 3;
					}
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist < min_ref_dist || cur_dist>max_ref_dist)
						continue;
					//lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
					//if (parallel_degree>0.5)
						parallel_degrees[i] = cur_lane.degree;
				}
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					HV_S32 max_degree = 0, max_index = -1;
					for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
					{
						if (parallel_degrees[i1] > max_degree)
						{
							max_degree = parallel_degrees[i1];
							max_index = i1;
						}
					}
					if (max_index == -1)
						continue;
					lane_t cur_lane = (*candidate_lanes)[max_index];
					parallel_degrees[max_index] = 0;
					HV_S32 cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					if (cur_dist > min_ref_dist&&cur_dist < max_ref_dist)
					{
						memcpy(cur_lanes[3], &cur_lane, sizeof(lane_t));
						break;
					}
				}
				delete[] parallel_degrees;
			}
		}
		//just right miss
		//has bug in if (cur_dist<min_ref_dist || cur_dist>max_ref_dist)
		//annotate now, not consider this case
		/*
		else if (cur_lanes[2]->detected == 0)
		{
			if (g_framenum == 5)
			{
				HV_S32 qqq = 3;
			}
			//matching to left or right right
			if (cur_lanes[1]->detected == 1||cur_lanes[3]->detected==1)
			{
				lane_t cur_ref_lane;
				if(cur_lanes[1]->detected==1)
					cur_ref_lane = *cur_lanes[1];
				else
					cur_ref_lane = *cur_lanes[3];
				HV_S32 *parallel_degrees = new HV_S32[candidate_lanes->size()];
				memset(parallel_degrees, 0, candidate_lanes->size()*sizeof(HV_S32));
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					lane_t cur_lane = (*candidate_lanes)[i];
					//parallel_degrees only store valid lanes
					if (cur_lane.degree < g_lane_detect_thresh)
						continue;
					if (i == 10)
					{
						HV_S32 qqq = 3;
					}
					lane_diff_weight(cur_lane, cur_ref_lane, parallel_degree, close_degree);
					if (parallel_degree>0.5)
						parallel_degrees[i] = cur_lane.degree*parallel_degree;
				}
				for (i = 0; i < candidate_lanes->size(); i++)
				{
					HV_S32 max_degree = 0, max_index = -1;
					for (HV_S32 i1 = 0; i1 < candidate_lanes->size(); i1++)
					{
						if (parallel_degrees[i1] > max_degree)
						{
							max_degree = parallel_degrees[i1];
							max_index = i1;
						}
					}
					if (max_index == -1)
						continue;
					lane_t cur_lane = (*candidate_lanes)[max_index];
					parallel_degrees[max_index] = 0;
					HV_S32 cur_dist;
					if (cur_lanes[1]->detected == 1)
						cur_dist = calc_lane_dist(cur_lane, cur_ref_lane);
					else
						cur_dist = calc_lane_dist(cur_ref_lane, cur_lane);
					//decide the distance to ref
					if (cur_dist > min_ref_dist&&cur_dist < max_ref_dist)
					{
						HV_S32 matching_en = 1;
						//if left and right right all matching already, should also decide to right right
						if (cur_lanes[1]->detected == 1 && cur_lanes[3]->detected == 1)
						{
							cur_dist = calc_lane_dist(*cur_lanes[3], cur_lane);
							if (cur_dist<min_ref_dist || cur_dist>max_ref_dist)
								matching_en = 0;
						}
						if (matching_en == 1)
						{
							memcpy(cur_lanes[2], &cur_lane, sizeof(lane_t));
							break;
						}
					}

				}
				delete[] parallel_degrees;
			}
		}
		*/
		//if mid both lost but side detected should delete side
		//or mid will keep lost until side lost, since there is no case for 0 detected and 123 lost
		if (cur_lanes[1]->detected == 0 && cur_lanes[2]->detected == 0)
		{
			cur_lanes[0]->detected = 0;
			cur_lanes[3]->detected = 0;
		}
	}


	//update ref_lane_dist
	HV_S32 last_index = -1;
	HV_S32 sum_dist = 0, sum_val = 0;
	//use all lanes
	/*for (k = 0; k < 4; k++)
	{
		if (cur_lanes[k]->detected == 0)
			continue;
		if (last_index != -1)
		{
			sum_dist += calc_lane_dist(*cur_lanes[k], *cur_lanes[last_index]);
			sum_val += k - last_index;
		}
		last_index = k;
	}
	if (sum_val != 0)
		cur_ref_lane_dist = sum_dist / sum_val;
	g_ref_lane_dist = g_lane_dist_filter*g_ref_lane_dist + (1 - g_lane_dist_filter)*cur_ref_lane_dist;*/

	//use mid two lanes
	//should move after update ref lane
	//if (cur_lanes[1]->detected == 1 && cur_lanes[2]->detected == 1)
	//{
	//	cur_ref_lane_dist = calc_lane_dist(*cur_lanes[2], *cur_lanes[1]);
	//	g_ref_lane_dist = g_lane_dist_filter*g_ref_lane_dist + (1 - g_lane_dist_filter)*cur_ref_lane_dist;
	//}
	////use 280 as default
	//g_ref_lane_dist = max(220/SCALE_RATIO, min(340/SCALE_RATIO, g_ref_lane_dist));
}

//delete cur lane with wrong position
HV_HVID filter_cur_lanes(lane_t *cur_lanes[4], lane_t *ref_lanes[4])
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 min_ref_dist = g_ref_lane_dist*0.5;
	HV_S32 max_ref_dist = g_ref_lane_dist*1.5;
	HV_S32 i, j, k;
	//if just detect one side lane, decide whether move to middle
	if (cur_lanes[1]->detected == 0 && cur_lanes[2]->detected == 0)
	{
		if (cur_lanes[0]->detected == 1)
		{
			if (cur_lanes[0]->bot_j > width / 2 - g_ref_lane_dist * 3 / 2)
			{
				memcpy(cur_lanes[1], cur_lanes[0], sizeof(lane_t));
				cur_lanes[0]->detected = 0;
			}
		}
		if (cur_lanes[3]->detected == 1)
		{
			if (cur_lanes[3]->bot_j < width / 2 + g_ref_lane_dist * 3 / 2)
			{
				memcpy(cur_lanes[2], cur_lanes[3], sizeof(lane_t));
				cur_lanes[3]->detected = 0;
			}
		}
	}
	//still no mid lane, return
	if (cur_lanes[1]->detected == 0 && cur_lanes[2]->detected == 0)
		return;
	//all compare to middle lanes with biggest degree
	lane_t ref_lane;
	HV_S32 ref_index = -1;
	if (cur_lanes[1]->detected == 1 && cur_lanes[2]->detected == 1)
	{
		if (cur_lanes[1]->degree > cur_lanes[2]->degree)
		{
			memcpy(&ref_lane, cur_lanes[1], sizeof(lane_t));
			ref_index = 1;
		}
		else
		{
			memcpy(&ref_lane, cur_lanes[2], sizeof(lane_t));
			ref_index = 2;
		}
	}
	else if (cur_lanes[1]->detected == 1)
	{
		memcpy(&ref_lane, cur_lanes[1], sizeof(lane_t));
		ref_index = 1;
	}
	else
	{
		memcpy(&ref_lane, cur_lanes[2], sizeof(lane_t));
		ref_index = 2;
	}

	for (k = 0; k < 4; k++)
	{
		if (k == ref_index || cur_lanes[k]->detected == 0)
			continue;
		lane_t cur_lane;
		memcpy(&cur_lane, cur_lanes[k], sizeof(lane_t));
		HV_S32 cur_dist;
		if (k < ref_index)
			cur_dist = calc_lane_dist(ref_lane, cur_lane);
		else
			cur_dist = calc_lane_dist(cur_lane, ref_lane);
		HV_S32 cur_min_ref_dist = abs(k - ref_index)*min_ref_dist;
		HV_S32 cur_max_ref_dist = abs(k - ref_index)*max_ref_dist;

		HV_F32 parallel_degree, close_degree;
		lane_diff_weight(cur_lane, ref_lane, parallel_degree, close_degree);

		if (cur_dist<cur_min_ref_dist || cur_dist>cur_max_ref_dist || parallel_degree<0.3)
			cur_lanes[k]->detected = 0;

		//side lanes should also compare to nearest middle lane
		if (k == 0 && ref_index != 1 && cur_lanes[1]->detected == 1)
		{
			cur_dist = calc_lane_dist(*cur_lanes[1], cur_lane);
			if (cur_dist<min_ref_dist || cur_dist>max_ref_dist)
				cur_lanes[k]->detected = 0;
		}
		if (k == 3 && ref_index != 2 && cur_lanes[2]->detected == 1)
		{
			cur_dist = calc_lane_dist(cur_lane, *cur_lanes[2]);
			if (cur_dist<min_ref_dist || cur_dist>max_ref_dist)
				cur_lanes[k]->detected = 0;
		}

	}

	//decide whether best lane change position
	//ref lane should change position to tracking
	HV_S32 cur_bot_pos = ref_lane.bot_j;
	HV_S32 ref_left_pos = width / 2 + (ref_index - 2)*g_ref_lane_dist;
	HV_S32 ref_right_pos = width / 2 + (ref_index - 1)*g_ref_lane_dist;
	extern Kalman g_lanes_kalman[4], g_temp_lanes_kalman[4];
	extern HV_S32 g_temp_tracking_num[4];
	//move left
	if (ref_index > 0 && cur_bot_pos < ref_left_pos)
	{
		memcpy(cur_lanes[0], cur_lanes[1], sizeof(lane_t));
		memcpy(cur_lanes[1], cur_lanes[2], sizeof(lane_t));
		memcpy(cur_lanes[2], cur_lanes[3], sizeof(lane_t));
		memcpy(ref_lanes[0], ref_lanes[1], sizeof(lane_t));
		memcpy(ref_lanes[1], ref_lanes[2], sizeof(lane_t));
		memcpy(ref_lanes[2], ref_lanes[3], sizeof(lane_t));
		memcpy(g_lanes_kalman + 0, g_lanes_kalman + 1, sizeof(Kalman));
		memcpy(g_lanes_kalman + 1, g_lanes_kalman + 2, sizeof(Kalman));
		memcpy(g_lanes_kalman + 2, g_lanes_kalman + 3, sizeof(Kalman));
		memcpy(g_temp_lanes_kalman + 0, g_temp_lanes_kalman + 1, sizeof(Kalman));
		memcpy(g_temp_lanes_kalman + 1, g_temp_lanes_kalman + 2, sizeof(Kalman));
		memcpy(g_temp_lanes_kalman + 2, g_temp_lanes_kalman + 3, sizeof(Kalman));
		g_temp_tracking_num[0] = g_temp_tracking_num[1];
		g_temp_tracking_num[1] = g_temp_tracking_num[2];
		g_temp_tracking_num[2] = g_temp_tracking_num[3];
		cur_lanes[3]->detected = 0;
	}
	//move right
	else if (ref_index<3 && cur_bot_pos>ref_right_pos)
	{
		memcpy(cur_lanes[3], cur_lanes[2], sizeof(lane_t));
		memcpy(cur_lanes[2], cur_lanes[1], sizeof(lane_t));
		memcpy(cur_lanes[1], cur_lanes[0], sizeof(lane_t));
		memcpy(ref_lanes[3], ref_lanes[2], sizeof(lane_t));
		memcpy(ref_lanes[2], ref_lanes[1], sizeof(lane_t));
		memcpy(ref_lanes[1], ref_lanes[0], sizeof(lane_t));
		memcpy(g_lanes_kalman + 3, g_lanes_kalman + 2, sizeof(Kalman));
		memcpy(g_lanes_kalman + 2, g_lanes_kalman + 1, sizeof(Kalman));
		memcpy(g_lanes_kalman + 1, g_lanes_kalman + 0, sizeof(Kalman));
		memcpy(g_temp_lanes_kalman + 3, g_temp_lanes_kalman + 2, sizeof(Kalman));
		memcpy(g_temp_lanes_kalman + 2, g_temp_lanes_kalman + 1, sizeof(Kalman));
		memcpy(g_temp_lanes_kalman + 1, g_temp_lanes_kalman + 0, sizeof(Kalman));
		g_temp_tracking_num[3] = g_temp_tracking_num[2];
		g_temp_tracking_num[2] = g_temp_tracking_num[1];
		g_temp_tracking_num[1] = g_temp_tracking_num[0];
		cur_lanes[0]->detected = 0;
	}
}

//return valus is bigger when two lane are closer
//used to multiply on degree to find most matching lane to the reference
HV_HVID lane_diff_weight(lane_t lane0, lane_t lane1, HV_F32 &parallel_degree, HV_F32 &close_degree)
{
	parallel_degree = 1;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 bot_pos = min(lane0.bot_end, lane1.bot_end);
	HV_S32 top_pos = max(lane0.top_end, lane1.top_end);
	extern HV_F32 g_y_dist_in_perspective;
	//should at least compare mid part
	/*bot_pos = max(height * 3 / 4, bot_pos);
	top_pos = min(height / 2, top_pos);*/
	if (bot_pos <= top_pos + 40 / SCALE_RATIO)
	{
		close_degree = 0;
		parallel_degree = 0;
		return;
	}
	HV_S32 sum_val = 0;
	for (int i = bot_pos; i >= top_pos; i -- )
	{
		HV_S32 cur_j0 = lane0.a0 + lane0.a1*i + lane0.a2*i*i;
		HV_S32 cur_j1 = lane1.a0 + lane1.a1*i + lane1.a2*i*i;
		sum_val += abs(cur_j0 - cur_j1);
	}
	HV_S32 mean_val = sum_val / (bot_pos - top_pos + 1);
	HV_F32 world_mean_dist = HV_F32(mean_val)*g_y_dist_in_perspective / HV_F32(width / 2);
	close_degree = 1 - 0.5*world_mean_dist;
	close_degree = max(0, close_degree);
	return;



	////should based on whether two lane is parallel but not the sum of the distance
	//HV_S32 width = IMG_WIDTH;
	//HV_S32 height = IMG_HEIGHT;
	//HV_F32 degree;
	//extern HV_F32 g_y_dist_in_perspective;
	//HV_S32 *dist_table = new HV_S32[height];

	//HV_S32 i, j, k;
	//k = 0;
	////calc mean and standard deviation
	//HV_S32 sum_val = 0, sum_num = 0;
	////whether to use weighted mean????????????????????????????????????????
	//for (i = height - 1; i >= 0; i-=2)
	//{
	//	HV_S32 cur_j0 = lane0.a0 + lane0.a1*i + lane0.a2*i*i;
	//	HV_S32 cur_j1 = lane1.a0 + lane1.a1*i + lane1.a2*i*i;
	//	dist_table[k] = abs(cur_j0 - cur_j1);
	//	sum_val += abs(cur_j0 - cur_j1);
	//	k++;
	//}
	//HV_S32 mean_dist = sum_val * 2 / height;
	//sum_val = 0;
	//k = 0;
	//for (i = height - 1; i >= 0; i -= 2)
	//{
	//	sum_val += abs(dist_table[k] - mean_dist);
	//	k++;
	//}
	//HV_S32 mean_var = sum_val * 2 / height;
	////closer lanes will have smaller parallel_degree when mean_var is identical

	//HV_F32 world_mean_dist= HV_F32(mean_dist)*g_y_dist_in_perspective / HV_F32(width / 2);
	//close_degree = 1 - 0.5*world_mean_dist;
	//close_degree = max(0, close_degree);
	//HV_F32 world_mean_var= HV_F32(mean_var)*g_y_dist_in_perspective / HV_F32(width / 2);
	//parallel_degree = 1 - 0.5*world_mean_var;
	//parallel_degree = max(0, parallel_degree);

	////don't consider parallel degree
	//parallel_degree = 1;

	//HV_F32 bot0 = HV_F32(width / 2 - lane0.bot_j)*g_y_dist_in_perspective / HV_F32(width / 2);
	//HV_F32 bot1 = HV_F32(width / 2 - lane1.bot_j)*g_y_dist_in_perspective / HV_F32(width / 2);
	//HV_F32 bot_diff = abs(bot0 - bot1);
	//close_degree = min(close_degree, 1 - bot_diff);
	//close_degree = max(0, close_degree);

	//delete[] dist_table;
	//return;




}

//cur_lane on the right will return positive value
HV_S32 calc_lane_dist(lane_t cur_lane, lane_t ref_lane)
{
	//HV_S32 bot_ref = ref_lane.bot_j;
	//HV_S32 bot_cur = cur_lane.bot_j;
	//HV_S32 mid_ref = ref_lane.mid_j;
	//HV_S32 mid_cur = cur_lane.mid_j;
	//HV_S32 top_ref = ref_lane.top_j;
	//HV_S32 top_cur = cur_lane.top_j;
	////return (bot_cur - bot_ref + top_cur - top_ref) / 2;
	//return (bot_cur - bot_ref + mid_cur - mid_ref) / 2;


	//should use two lane's overlap part
	HV_S32 bot_pos = min(ref_lane.bot_end, cur_lane.bot_end);
	HV_S32 top_pos = max(ref_lane.top_end, cur_lane.top_end);
	if (bot_pos <= top_pos + 20 / SCALE_RATIO)
	{
		return 1000;
	}
	HV_S32 bot_ref = ref_lane.a0 + ref_lane.a1 * bot_pos + ref_lane.a2 * bot_pos * bot_pos;
	HV_S32 top_ref = ref_lane.a0 + ref_lane.a1 * top_pos + ref_lane.a2 * top_pos * top_pos;
	HV_S32 bot_cur = cur_lane.a0 + cur_lane.a1 * bot_pos + cur_lane.a2 * bot_pos * bot_pos;
	HV_S32 top_cur = cur_lane.a0 + cur_lane.a1 * top_pos + cur_lane.a2 * top_pos * top_pos;
	return (bot_cur - bot_ref + top_cur - top_ref) / 2;
}


HV_HVID curve_fitting(HV_F32 &a0, HV_F32 &a1, HV_F32 &a2, HV_F32 i0, HV_F32 j0, HV_F32 i1, HV_F32 j1, HV_F32 i2, HV_F32 j2)
{
	HV_F32 matrix_x[9];
	matrix_x[0] = 1, matrix_x[1] = i0, matrix_x[2] = i0 * i0;
	matrix_x[3] = 1, matrix_x[4] = i1, matrix_x[5] = i1*i1;
	matrix_x[6] = 1, matrix_x[7] = i2, matrix_x[8] = i2*i2;
	HV_F32 matrix_x_inv[9];
	matrix_inv(matrix_x, matrix_x_inv);
	HV_F32 matrix_y[3];
	matrix_y[0] = j0, matrix_y[1] = j1, matrix_y[2] = j2;
	HV_F32 matrix_a[3];
	matrix_multiply(matrix_x_inv, matrix_y, matrix_a, 3, 3, 1);
	a0 = matrix_a[0];
	a1 = matrix_a[1];
	a2 = matrix_a[2];
}

HV_HVID get_lane_type(lane_t *cur_lanes[4])
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 fitting_area = 3;
	for (int k = 0; k < 4; k++)
	{
		lane_t *cur_lane = cur_lanes[k];
		if (cur_lane->detected == 0)
			continue;
		cur_lane->type_change_pos = -1;
		HV_S32 dash_num = 0, roadside_num = 0;
		HV_S32 fitting_num = 0;
		//used to get type change position
		HV_S32 *type_index = new HV_S32[height];
		memset(type_index, -1, height*sizeof(HV_S32));
		for (int i = cur_lane->bot_end; i >= cur_lane->top_end; i--)
		{
			type_index[i] = 0;
			HV_S32 cur_j = cur_lane->a0 + cur_lane->a1*HV_F32(i) + cur_lane->a2*HV_F32(i)*HV_F32(i);
			if (cur_j < fitting_area || cur_j >= width - fitting_area)
				continue;
			for (int j = cur_j - fitting_area; j <= cur_j + fitting_area; j++)
			{
				if (g_ori_bird_img[i*width + j] != 0)
				{
					fitting_num++;
					if (g_ori_bird_img[i*width + j] >= 75 && g_ori_bird_img[i*width + j] < 125)
					{
						roadside_num++;
					}
					else if (g_ori_bird_img[i*width + j] >= 125 && g_ori_bird_img[i*width + j] < 175)
					{
						dash_num++;
						type_index[i] = 1;
					}
					else if (g_ori_bird_img[i*width + j] >= 175 && g_ori_bird_img[i*width + j] < 225
						&& k >= 2)
					{
						dash_num++;
						type_index[i] = 1;
					}
					else if (g_ori_bird_img[i*width + j] >= 225 && g_ori_bird_img[i*width + j]
						&& k <= 1)
					{
						dash_num++;
						type_index[i] = 1;
					}
					break;
				}
			}
		}
		if (dash_num * 32 / fitting_num>4)
			cur_lane->lane_type = 1;
		if (roadside_num * 32 / fitting_num>4)
			cur_lane->lane_type = 2;
		//get pos when change to solid, just for mid two lanes
		HV_S32 change_pos = 0, solid_num = 0;
		if (cur_lane->lane_type == 1&&k!=0&&k!=3)
		{
			for (int i = cur_lane->top_end; i <= cur_lane->bot_end; i++)
			{
				if (type_index[i] == 0)
					solid_num++;
				if (type_index[i] == 1)
				{
					change_pos = i;
					break;
				}
			}
			if(solid_num>5)
				cur_lane->type_change_pos = change_pos;
		}
		delete[] type_index;

		continue;

		/*
		//find solid/dash lane in ori image
		if (k != 1 && k != 2)
			continue;
		HV_S32* lane_widths = new HV_S32[height];
		memset(lane_widths, 0, height*sizeof(HV_S32));
		extern HV_F32 g_perspective_mat[9];
		extern HV_F32 g_inv_perspective_mat[9];
		extern HV_S32 cur_frame_num;
		if (cur_frame_num == 85 && k==1)
		{
			int qqq = 3;
		}
		for (int i = height - 1; i > g_roi_top; i--)
		{
			if (i == 511)
			{
				int qqq = 3;
			}
			HV_S32 i_bird, j_bird;
			perspective_transform(i, width / 2, i_bird, j_bird, g_perspective_mat);
			if (i_bird<cur_lane->top_end || i_bird>cur_lane->bot_end)
				continue;
			j_bird = cur_lane->a0 + cur_lane->a1*i_bird + cur_lane->a2*i_bird*i_bird;
			HV_S32 i_ori, j_ori;
			perspective_transform(i_bird, j_bird, i_ori, j_ori, g_inv_perspective_mat);
			//find inner position of lane on current row
			HV_S32 j_inner = j_ori;
			if (k == 1)
			{
				//find right position
				for (int j = j_ori; j < width; j++)
				{
					if (g_ori_img[i_ori*width + j] < 25)
					{
						j_inner = j;
						break;
					}
				}
			}
			else if (k == 2)
			{
				//find left position
				for (int j = j_ori; j >= 0; j--)
				{
					if (g_ori_img[i_ori*width + j] < 25)
					{
						j_inner = j;
						break;
					}
				}
			}
			//calc width between inner point and lane point
			HV_S32 j_bird_inner;
			perspective_transform(i_ori, j_inner, i_bird, j_bird_inner, g_perspective_mat);
			if(i_bird>=0 && i_bird < height)
				lane_widths[i_bird] = abs(j_bird_inner - j_bird);
		}
		//use 30cm as threshold to distinguish thin lane and wide lane
		extern HV_F32 g_y_dist_in_perspective;
		HV_S32 width_thresh = width / 2 / g_y_dist_in_perspective*0.30;
		HV_S32 sum_val = 0, sum_num = 0;
		for (int i = 0; i < height; i++)
		{
			if (lane_widths[i] > 5)
			{
				sum_val += lane_widths[i];
				sum_num++;
			}
		}
		HV_S32 mean_val = (sum_val + (sum_num >> 1)) / sum_num;
		HV_S32 big_val = 0, big_num = 0, small_val = 0, small_num = 0;
		for (int i = 0; i < height; i++)
		{
			if (lane_widths[i] >= mean_val)
			{
				big_val += lane_widths[i];
				big_num++;
			}
			else if (lane_widths[i] > 5)
			{
				small_val += lane_widths[i];
				small_num++;
			}
		}
		HV_S32 big_mean = (big_val + (big_num >> 1)) / big_num;
		HV_S32 small_mean = (small_val + (small_num >> 1)) / small_num;
		if (big_num * 3 > sum_num&&small_num * 3 > sum_num&&small_mean * 2 < big_mean)
		{
			cur_lane->lane_type = 1;
		}
		int qqq = 3;
		delete[] lane_widths;
		*/
	}
	
}

//get intersection point of cur lane and border of ori image in bird image
//calc intersection to j=vanish_a0+vanish_a1*i and (1280-j)=......
//vanish_a0 and vanish_a1 is left side of the image in bird view
HV_S32 get_inter_lane_in_bird(HV_F32 cur_a0, HV_F32 cur_a1, HV_F32 cur_a2)
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 inter_i = -1, inter_j = -1;
	HV_F32 temp_val;
	HV_F32 result_i, result_j;
	if (cur_a2 != 0)
	{
		temp_val = (cur_a1 - vanish_a1)*(cur_a1 - vanish_a1) - 4 * cur_a2*(cur_a0 - vanish_a0);
		if (temp_val >= 0)
		{
			result_i = (vanish_a1 - cur_a1 + sqrt(temp_val)) / 2 / cur_a2;
			result_j = vanish_a0 + vanish_a1*result_i;
			if (result_i >= 0 && result_i < height&&result_j >= 0 && result_j < width)
				inter_i = result_i, inter_j = result_j;
			result_i = (vanish_a1 - cur_a1 - sqrt(temp_val)) / 2 / cur_a2;
			result_j = vanish_a0 + vanish_a1*result_i;
			if (result_i >= 0 && result_i < height&&result_j >= 0 && result_j < width)
				inter_i = result_i, inter_j = result_j;
		}
		temp_val = (cur_a1 + vanish_a1)*(cur_a1 + vanish_a1) - 4 * cur_a2*(cur_a0 + vanish_a0 - width);
		if (temp_val >= 0)
		{
			result_i = (-vanish_a1 - cur_a1 + sqrt(temp_val)) / 2 / cur_a2;
			result_j = width - vanish_a0 - vanish_a1*result_i;
			if (result_i >= 0 && result_i < height&&result_j >= 0 && result_j < width)
				inter_i = result_i, inter_j = result_j;
			result_i = (-vanish_a1 - cur_a1 - sqrt(temp_val)) / 2 / cur_a2;
			result_j = width - vanish_a0 - vanish_a1*result_i;
			if (result_i >= 0 && result_i < height&&result_j >= 0 && result_j < width)
				inter_i = result_i, inter_j = result_j;
		}
	}
	else if(cur_a1 - vanish_a1!=0)
	{
		result_i = (cur_a0 - vanish_a0) / (vanish_a1 - cur_a1);
		result_j = cur_a0 + cur_a1*result_i;
		if (result_i >= 0 && result_i < height&&result_j >= 0 && result_j < width)
			inter_i = result_i, inter_j = result_j;
		result_i = HV_F32(width - cur_a0 - vanish_a0) / (cur_a1 + vanish_a1);
		result_j = cur_a0 + cur_a1*result_i;
		if (result_i >= 0 && result_i < height&&result_j >= 0 && result_j < width)
			inter_i = result_i, inter_j = result_j;
	}
	return inter_i;
}

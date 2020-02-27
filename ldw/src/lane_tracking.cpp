#include "lane_tracking.h"

HV_S32 g_vanish_i, g_vanish_j;

HV_S32 g_variance_r = 1;	//variance of measure error in kalman filter
HV_S32 g_variance_q = 256;	//q smaller than r means more trust to predict result
HV_S32 g_tracking_thresh = 3;	//thresh to regard as valid

Kalman g_lanes_kalman[4];
Kalman g_temp_lanes_kalman[4];
HV_S32 g_temp_tracking_num[4];

HV_HVID kalman_filtering(HV_S32 &cur_i, HV_S32 last_i, HV_S32 &variance)
{
	if (cur_i == last_i)
		return;
	HV_S32 i, j;
	HV_S32 a = 1;
	HV_S32 p_k, x_k = last_i, z_k = cur_i;
	HV_F32 k;
	p_k = variance;
	HV_S32 h = 1;
	HV_S32 ii[4] = { 256,0,0,256 };
	HV_S32 temp1[4], temp2[4], temp3[4], temp4[4];
	
	p_k += g_variance_q;
	k = HV_F32(p_k*h) / HV_F32(h*p_k*h + g_variance_r);
	p_k = (1 - k*h)*p_k;
	cur_i = last_i + k*(z_k - h*last_i);

}

HV_HVID kalman_filtering(HV_F32 &cur_i, HV_F32 last_i, HV_S32 &variance)
{
	if (cur_i == last_i)
		return;
	HV_S32 i, j;
	HV_F32 p_k, x_k = last_i, z_k = cur_i;
	HV_F32 k;
	p_k = variance;
	HV_F32 h = 1;

	p_k += g_variance_q;
	k = HV_F32(p_k*h) / HV_F32(h*p_k*h + g_variance_r);
	p_k = (1 - k*h)*p_k;
	cur_i = last_i + k*(z_k - h*last_i);

}

HV_HVID update_ref_lanes(lane_t *cur_lanes[4], lane_t *ref_lanes[4])
{
	/*for (int k = 0; k < 4; k++)
	{
		memcpy(ref_lanes[k], cur_lanes[k], sizeof(lane_t));
		ref_lanes[k]->tracking_num = 10;
	}
	filter_ref_lanes(ref_lanes);
	return;*/


	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 i, j, k;
	HV_F32 filter_num = 0.8;
	extern HV_F32 g_y_dist_in_perspective;
	extern HV_S32 g_framenum;
	if (g_framenum == 62)
	{
		HV_S32 qqq = 3;
	}
	for (k = 0; k < 4; k++)
	{
		if (cur_lanes[k]->detected == 1)
		{
			lane_t *cur_lane = cur_lanes[k];
			lane_t *ref_lane = ref_lanes[k];
			if (g_framenum == 95&&k==1)
			{
				HV_S32 qqq = 3;
			}
			HV_S32 tracking_en = tracking_success(*cur_lane, *ref_lane);
			if (tracking_en == 1)
			{
				HV_S32 cur_tracking_num = ref_lane->tracking_num+1;
				//kalman filter
				KalmanPredict(g_lanes_kalman + k);
				float measure_state[3] = { cur_lane->a0, cur_lane->a1, cur_lane->a2 };
				KalmanCorrect(g_lanes_kalman + k, measure_state);

				//annotate to close kalman
				cur_lane->a0 = g_lanes_kalman[k].state_post[0];
				cur_lane->a1 = g_lanes_kalman[k].state_post[2];
				cur_lane->a2 = g_lanes_kalman[k].state_post[4];

				//update ref lane
				memcpy(ref_lane, cur_lane, sizeof(lane_t));
				ref_lane->tracking_num = cur_tracking_num;
				cur_lane->tracking_num= cur_tracking_num;
				ref_lanes[k]->hold_num = 0;

				//store kalman output in cur lanes to show
				//ref lane store detected lane and will used in next frames detect
				/*cur_lane->a0 = lanes_kalman[k].state_post[0];
				cur_lane->a1 = lanes_kalman[k].state_post[2];
				cur_lane->a2 = lanes_kalman[k].state_post[4];*/
			}
			//should both tracking cur lane and ref lane until cur lane exceed g_tracking_thresh
			/*not tracking ref lane, use cur lane directly*/
			else
			{
				
				//new lane's bot position should in right position
				if (k == 2)
				{
					HV_S32 qqq = 3;
				}
				

				lane_t hold_fitting_lane;
				hold_fitting_lane.detected = 0;
				if (ref_lanes[k]->tracking_num > g_tracking_thresh)
				{
					lane_t *hold_lane = ref_lanes[k];
					//just predict lane param
					//update param will cause lane shifting
					//g_lanes_kalman[k].state_post[0] += g_lanes_kalman[k].state_post[1];
					//g_lanes_kalman[k].state_post[2] += g_lanes_kalman[k].state_post[3];
					//g_lanes_kalman[k].state_post[4] += g_lanes_kalman[k].state_post[5];
					hold_lane->a0 = g_lanes_kalman[k].state_post[0];
					hold_lane->a1 = g_lanes_kalman[k].state_post[2];
					hold_lane->a2 = g_lanes_kalman[k].state_post[4];
					//update ref position and fitting
					hold_fitting_lane = fitting_lane(hold_lane->a0, hold_lane->a1, hold_lane->a2, 1, k, 4);
					//should tracking both ref lane and cur lane
					if (hold_fitting_lane.detected == 1)
					{
						lane_t temp_lane;
						temp_lane.a0 = g_temp_lanes_kalman[k].state_post[0];
						temp_lane.a1 = g_temp_lanes_kalman[k].state_post[2];
						temp_lane.a2 = g_temp_lanes_kalman[k].state_post[4];
						tracking_en = tracking_success(*cur_lane, temp_lane);
						if (tracking_en == 1)
						{
							KalmanPredict(g_temp_lanes_kalman + k);
							float measure_state[3] = { cur_lane->a0, cur_lane->a1, cur_lane->a2 };
							KalmanCorrect(g_temp_lanes_kalman + k, measure_state);
							g_temp_tracking_num[k]++;
							//cur lane tracking success, substitute ref lane
							//dont't need to compare degree, because cur lane is detected and ref lane is lost
							//cur lane's degree must bigger
							if (g_temp_tracking_num[k] > g_tracking_thresh)
							{
								memcpy(ref_lane, cur_lane, sizeof(lane_t));
								ref_lanes[k]->tracking_num = 1;
								ref_lanes[k]->hold_num = 0;
								float cur_state[6] = { cur_lane->a0, 0, cur_lane->a1, 0, cur_lane->a2 , 0 };
								init_Kalman(g_lanes_kalman + k, cur_state);
							}
						}
						//cur lane tracking failed, keep ref lane
						else
						{
							float cur_state[6] = { cur_lane->a0, 0, cur_lane->a1, 0, cur_lane->a2 , 0 };
							g_temp_tracking_num[k] = 0;
							init_Kalman(g_temp_lanes_kalman + k, cur_state);
						}
					}
				}
				
				//ref lane isn't valid, init kalman param for ref lane and temp lane
				if(hold_fitting_lane.detected ==0)
				{
					memcpy(ref_lane, cur_lane, sizeof(lane_t));
					ref_lanes[k]->tracking_num = 1;
					ref_lanes[k]->hold_num = 0;
					float cur_state[6] = { cur_lane->a0, 0, cur_lane->a1, 0, cur_lane->a2 , 0 };
					init_Kalman(g_lanes_kalman + k, cur_state);
					g_temp_tracking_num[k] = 0;
					init_Kalman(g_temp_lanes_kalman + k, cur_state);
				}
			}
			
		}
		//whether to keep for several frames?????????
		else
		{
			//should detect in last predict position first
			
			lane_t hold_fitting_lane;
			hold_fitting_lane.detected = 0;
			if (ref_lanes[k]->tracking_num > g_tracking_thresh
				&&ref_lanes[k]->hold_num<10)
			{
				lane_t *hold_lane = ref_lanes[k];
				//just predict lane param
				//update param will cause lane shifting
				//g_lanes_kalman[k].state_post[0] += g_lanes_kalman[k].state_post[1];
				//g_lanes_kalman[k].state_post[2] += g_lanes_kalman[k].state_post[3];
				//g_lanes_kalman[k].state_post[4] += g_lanes_kalman[k].state_post[5];
				hold_lane->a0 = g_lanes_kalman[k].state_post[0];
				hold_lane->a1 = g_lanes_kalman[k].state_post[2];
				hold_lane->a2 = g_lanes_kalman[k].state_post[4];
				//update ref position and fitting
				hold_fitting_lane = fitting_lane(hold_lane->a0, hold_lane->a1, hold_lane->a2, 1, k, 4);
			}

			if (hold_fitting_lane.detected == 1)
			{
				ref_lanes[k]->hold_num++;
				continue;
			}
			

			//keep for several frames, will cause maintain in intersection
			/*
			if (ref_lanes[k]->tracking_num > g_tracking_thresh
				&&ref_lanes[k]->hold_num < 20)
			{
				ref_lanes[k]->hold_num++;
				continue;
			}
			*/

			//tracking failure lane just reset to default here
			//if one lane detected, next frame will just use this lane as reference
			//if next frame detect none lane, all ref lanes will defaulted
			//so default all ref here is okay, next frame will only use ref which detected is 1
			ref_lanes[k]->detected = 0;
			ref_lanes[k]->a0 = width / 2 - width / 2 * (5.25 - 3.5 * k) / g_y_dist_in_perspective;
			ref_lanes[k]->a1 = 0, ref_lanes[k]->a2 = 0, ref_lanes[k]->a3 = 0;
			ref_lanes[k]->bot_j = ref_lanes[k]->a0;
			ref_lanes[k]->mid_j = ref_lanes[k]->a0;
			ref_lanes[k]->top_j = ref_lanes[k]->a0;
			//ref_lanes[k]->bot_angle = 90;
			//ref_lanes[k]->top_angle = 90;
			ref_lanes[k]->lane_type = -1;
			ref_lanes[k]->tracking_num = 0;
			ref_lanes[k]->hold_num = 0;
		}
	}
	if (g_framenum == 21)
	{
		HV_S32 qqq = 3;
	}
	filter_ref_lanes(ref_lanes);
}

HV_S32 tracking_success(lane_t lane0, lane_t lane1)
{
	extern HV_S32 g_framenum;
	if (lane1.detected == 0)
		return 0;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 track_success = 0;
	extern HV_F32 g_x_min_dist_in_perspective, g_x_dist_in_perspective;
	//pos of 15 m
	HV_S32 roi_top = height - (15 - g_x_min_dist_in_perspective)*height / (g_x_dist_in_perspective - g_x_min_dist_in_perspective);
	HV_S32 i, j, k;
	HV_S32 sum_bot_val = 0, sum_val=0;
	HV_S32 bot_pos = min(lane0.bot_end, lane1.bot_end);
	HV_S32 top_pos = max(lane0.top_end, lane1.top_end);
	//temp tracking's top/bot_end is invalid
	if (lane1.bot_end<0 || lane1.bot_end>height)
	{
		bot_pos = lane0.bot_end;
		top_pos = lane0.top_end;
	}
	//tracking should use extend lane when not overlapping
	bot_pos = max(bot_pos, top_pos + 40);
	/*if (bot_pos <= top_pos+40/SCALE_RATIO)
	{
		return 0;
	}*/
	for (i = bot_pos; i >= top_pos; i --)
	{
		if (i == 363)
		{
			HV_S32 qqq = 3;
		}
		HV_S32 cur_j0 = lane0.a0 + lane0.a1*i + lane0.a2*i*i;
		HV_S32 cur_j1 = lane1.a0 + lane1.a1*i + lane1.a2*i*i;
		sum_val += abs(cur_j0 - cur_j1);

	}
	//sum distance in bird view image
	HV_S32 mean_val = sum_val / (bot_pos - top_pos + 1);
	if (mean_val <= 50/SCALE_RATIO)
		track_success = 1;
	return track_success;
}

HV_HVID filter_ref_lanes(lane_t *ref_lanes[4])
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	extern HV_S32 g_ref_lane_dist;
	HV_S32 min_ref_dist = g_ref_lane_dist*0.5;
	HV_S32 max_ref_dist = g_ref_lane_dist*1.5;
	HV_S32 i, j, k;
	//if just detect one side lane, decide whether move to middle
	if (ref_lanes[1]->detected == 0 && ref_lanes[2]->detected == 0)
	{
		if (ref_lanes[0]->detected == 1)
		{
			if (ref_lanes[0]->bot_j > width / 2 - g_ref_lane_dist * 3 / 2)
			{
				memcpy(ref_lanes[1], ref_lanes[0], sizeof(lane_t));
				memcpy(g_lanes_kalman + 1, g_lanes_kalman + 0, sizeof(Kalman));
				memcpy(g_temp_lanes_kalman + 1, g_temp_lanes_kalman + 0, sizeof(Kalman));
				g_temp_tracking_num[1] = g_temp_tracking_num[0];
				ref_lanes[0]->detected = 0;
			}
		}
		else if (ref_lanes[3]->detected == 1)
		{
			if (ref_lanes[3]->bot_j < width / 2 + g_ref_lane_dist * 3 / 2)
			{
				memcpy(ref_lanes[2], ref_lanes[3], sizeof(lane_t));
				memcpy(g_lanes_kalman + 2, g_lanes_kalman + 3, sizeof(Kalman));
				memcpy(g_temp_lanes_kalman + 2, g_temp_lanes_kalman + 3, sizeof(Kalman));
				g_temp_tracking_num[2] = g_temp_tracking_num[3];
				ref_lanes[3]->detected = 0;
			}
		}
	}
	//still no mid lane, return
	if (ref_lanes[1]->detected == 0 && ref_lanes[2]->detected == 0)
		return;
	//all compare to middle lanes with biggest degree
	lane_t ref_lane;
	HV_S32 ref_index = -1;
	if (ref_lanes[1]->detected == 1 && ref_lanes[2]->detected == 1)
	{
		if (ref_lanes[1]->degree > ref_lanes[2]->degree)
		{
			memcpy(&ref_lane, ref_lanes[1], sizeof(lane_t));
			ref_index = 1;
		}
		else
		{
			memcpy(&ref_lane, ref_lanes[2], sizeof(lane_t));
			ref_index = 2;
		}
	}
	else if (ref_lanes[1]->detected == 1)
	{
		memcpy(&ref_lane, ref_lanes[1], sizeof(lane_t));
		ref_index = 1;
	}
	else
	{
		memcpy(&ref_lane, ref_lanes[2], sizeof(lane_t));
		ref_index = 2;
	}

	for (k = 0; k < 4; k++)
	{
		if (k == ref_index || ref_lanes[k]->detected == 0)
			continue;

		lane_t cur_lane;
		memcpy(&cur_lane, ref_lanes[k], sizeof(lane_t));
		HV_S32 cur_dist;
		//side lanes should also compare to nearest middle lane
		if (k == 0 && ref_index != 1 && ref_lanes[1]->detected == 1)
		{
			cur_dist = calc_lane_dist(*ref_lanes[1], cur_lane);
			if (cur_dist<min_ref_dist || cur_dist>max_ref_dist)
				ref_lanes[k]->detected = 0;
		}
		if (k == 3 && ref_index != 2 && ref_lanes[2]->detected == 1)
		{
			cur_dist = calc_lane_dist(cur_lane, *ref_lanes[2]);
			if (cur_dist<min_ref_dist || cur_dist>max_ref_dist)
				ref_lanes[k]->detected = 0;
		}

		if (k < ref_index)
			cur_dist = calc_lane_dist(ref_lane, cur_lane);
		else
			cur_dist = calc_lane_dist(cur_lane, ref_lane);

		//no overlap will not delete
		if (cur_dist == 1000)
			continue;
		HV_S32 cur_min_ref_dist = abs(k - ref_index)*min_ref_dist;
		HV_S32 cur_max_ref_dist = abs(k - ref_index)*max_ref_dist;

		HV_F32 parallel_degree, close_degree;
		lane_diff_weight(cur_lane, ref_lane, parallel_degree, close_degree);

		if (cur_dist<cur_min_ref_dist || cur_dist>cur_max_ref_dist || parallel_degree<0.2)
			ref_lanes[k]->detected = 0;

		

	}

	//decide whether best lane change position
	//ref lane should change position to tracking
	/*
	HV_S32 cur_bot_pos = ref_lane.bot_j;
	HV_S32 ref_left_pos = width / 2 + (ref_index - 2)*g_ref_lane_dist;
	HV_S32 ref_right_pos = width / 2 + (ref_index - 1)*g_ref_lane_dist;
	extern Kalman g_lanes_kalman[4], g_temp_lanes_kalman[4];
	extern HV_S32 g_temp_tracking_num[4];
	//move left
	if (ref_index > 0 && cur_bot_pos < ref_left_pos)
	{
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
		ref_lanes[3]->detected = 0;
	}
	//move right
	else if (ref_index<3 && cur_bot_pos>ref_right_pos)
	{
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
		ref_lanes[0]->detected = 0;
	}
	*/

	//just change position when middle lane cross mid position
	//avoid change position continuous when vehicle change lane
	//move left
	extern Kalman g_lanes_kalman[4], g_temp_lanes_kalman[4];
	extern HV_S32 g_temp_tracking_num[4];
	if (ref_lanes[2]->detected == 1&&ref_lanes[2]->bot_j < width / 2)
	{
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
		ref_lanes[3]->detected = 0;
	}
	//move right
	else if (ref_lanes[1]->detected == 1 && ref_lanes[1]->bot_j > width / 2)
	{
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
		ref_lanes[0]->detected = 0;
	}

	//update ref lane's position, to get segment position in next frame
	if (ref_lanes[0]->detected == 0&&ref_lanes[1]->detected==1)
	{
		ref_lanes[0]->a0 = ref_lanes[1]->a0 - g_ref_lane_dist;
		ref_lanes[0]->a1 = ref_lanes[1]->a1;
		ref_lanes[0]->a2 = ref_lanes[1]->a2;
	}
	if (ref_lanes[1]->detected == 0 && ref_lanes[2]->detected == 1)
	{
		ref_lanes[1]->a0 = ref_lanes[2]->a0 - g_ref_lane_dist;
		ref_lanes[1]->a1 = ref_lanes[2]->a1;
		ref_lanes[1]->a2 = ref_lanes[2]->a2;
	}
	if (ref_lanes[2]->detected == 0 && ref_lanes[1]->detected == 1)
	{
		ref_lanes[2]->a0 = ref_lanes[1]->a0 + g_ref_lane_dist;
		ref_lanes[2]->a1 = ref_lanes[1]->a1;
		ref_lanes[2]->a2 = ref_lanes[1]->a2;
	}
	if (ref_lanes[3]->detected == 0 && ref_lanes[2]->detected == 1)
	{
		ref_lanes[3]->a0 = ref_lanes[2]->a0 + g_ref_lane_dist;
		ref_lanes[3]->a1 = ref_lanes[2]->a1;
		ref_lanes[3]->a2 = ref_lanes[2]->a2;
	}
}
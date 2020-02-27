#include "common.h"

HV_S32 g_grad_table[511 * 511];
HV_S32 g_angle_table[511 * 511];
//use coordinate directly	//500/425, 530/480
HV_S32 g_seg_dist_mid = 530 / SCALE_RATIO;		//can use decimal
HV_S32 g_detect_dist_mid = 480 / SCALE_RATIO;
HV_S32 g_seg_dist_side = 480 / SCALE_RATIO;
HV_S32 g_detect_dist_side = 480 / SCALE_RATIO;

HV_HVID initial_parameter()
{
	HV_S32 i, j;
	HV_F32 pi = 3.1415;
	HV_F32 small = 2.2e-16;
	for (i = 0; i < 511; i++)
	{
		for (j = 0; j < 511; j++)
		{
			g_grad_table[i * 511 + j] = sqrt((i - 255)*(i - 255) + (j - 255)*(j - 255));
			float angle = 0;
			//float angleScale = (float)(TDECIM_CELL/PI);			
			HV_F32 x = i - 255, y = j - 255, x2 = x*x, y2 = y*y, a;
			if (y2 <= x2)
				a = (x < 0 ? pi : y >= 0 ? 0 : pi * 2) +
				x*y*(x2 + 0.43157974*y2) / (x2*x2 + y2*(0.76443945*x2 + 0.05831938*y2) + (float)small);
			else
			{
				a = (y >= 0 ? pi*0.5 : pi*1.5) -
					x*y*(y2 + 0.43157974*x2) / (y2*y2 + x2*(0.76443945*y2 + 0.05831938*x2) + (float)small);
			}
			angle = (float)a * 180 / pi;//0-2*PI
			g_angle_table[i * 511 + j] = angle;
			if (i == 500 && j == 254)
			{
				HV_S32 qqq = 3;
			}
		}
	}
	
}

HV_HVID initial_lane(lane_t *lane[4])
{
	extern HV_S32 g_vanish_i, g_vanish_j;
	extern HV_S32 g_camera_position;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	extern HV_F32 g_y_dist_in_perspective;
	HV_S32 k;
	for (k = 0; k < 4; k++)
	{
		lane[k]->a0 = width / 2 - width / 2 * (5.25 - 3.5 * k) / g_y_dist_in_perspective;
		lane[k]->a1 = 0, lane[k]->a2 = 0, lane[k]->a3 = 0;
		lane[k]->bot_j = lane[k]->a0;
		lane[k]->mid_j = lane[k]->a0;
		lane[k]->top_j = lane[k]->a0;
		//lane[k]->bot_angle = 90;
		//lane[k]->top_angle = 90;
		lane[k]->lane_type = -1;
		lane[k]->detected = 0;
		lane[k]->tracking_num = 0;
	}
}

//initial lane on offset_dist distance away from ref_lane
//not update segment i position, angle use same value
HV_HVID initial_lane_based_on_ref(lane_t *lane, lane_t *ref_lane, HV_F32 offset_dist)
{
	lane->detected = 0;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	extern HV_F32 g_y_dist_in_perspective;
	lane->a0 = ref_lane->a0 + width / 2 * offset_dist / g_y_dist_in_perspective;
	lane->a1 = ref_lane->a1, lane->a2 = ref_lane->a2, lane->a3 = ref_lane->a3;
	lane->lane_type = -1;
}

//update ref lane when current lane detect in bottom seg
HV_HVID update_ref_lane(lane_t *cur_lanes[], lane_t *ref_lanes[])
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	extern HV_F32 g_y_dist_in_perspective;
	HV_S32 k;
	//just detected ref lane will used in next frame, so copy all is okay
	if (cur_lanes[0]->detected == 1 || cur_lanes[1]->detected == 1 || cur_lanes[2]->detected == 1 || cur_lanes[3]->detected == 1)
	{
		for (HV_S32 k = 0; k < 4; k++)
		{
			memcpy(ref_lanes[k], cur_lanes[k], sizeof(lane_t));
		}
	}
	else
	{
		for (k = 0; k < 4; k++)
		{
			ref_lanes[k]->a0 = width / 2 - width / 2 * (5.25 - 3.5 * k) / g_y_dist_in_perspective;
			ref_lanes[k]->a1 = 0, ref_lanes[k]->a2 = 0, ref_lanes[k]->a3 = 0;
			ref_lanes[k]->bot_j = ref_lanes[k]->a0;
			ref_lanes[k]->mid_j = ref_lanes[k]->a0;
			ref_lanes[k]->top_j = ref_lanes[k]->a0;
			//ref_lanes[k]->bot_angle = 90;
			//ref_lanes[k]->top_angle = 90;
			ref_lanes[k]->lane_type = -1;
			ref_lanes[k]->detected = 0;
		}
	}
}

//return y value in (i,j) position
HV_S32 get_pixel(image_t img_in, HV_S32 i, HV_S32 j)
{
	return *(img_in.y_in + max(0, min(img_in.height - 1, i))*img_in.width + max(0, min(img_in.width - 1, j)));
}

HV_S32 get_pixel(HV_BYTE *img_in, HV_S32 i, HV_S32 j)
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	return *(img_in + max(0, min(height - 1, i))*width + max(0, min(width - 1, j)));
}

HV_HVID set_pixel(image_t img_in, HV_S32 i, HV_S32 j, HV_S32 val)
{
	*(img_in.y_in + max(0, min(img_in.height - 1, i))*img_in.width + max(0, min(img_in.width - 1, j)))=val;
}

HV_HVID matrix_multiply(HV_F32 *a_in, HV_F32 *b_in, HV_F32 *out, HV_S32 dim0, HV_S32 dim1, HV_S32 dim2)
{
	HV_S32 i, j, k;
	memset(out, 0, dim0 * dim2 * sizeof(HV_F32));
	for (i = 0; i < dim0; i++)
	{
		for (j = 0; j < dim2; j++)
		{
			for (k = 0; k < dim1; k++)
			{
				out[i*dim2 + j] += a_in[i*dim1 + k] * b_in[k*dim2 + j];
			}
		}
	}
}

//a_in is dim0*dim1, b_in is dim1*dim2, out is dim0*dim2
HV_HVID matrix_multiply(HV_S32 *a_in, HV_S32 *b_in, HV_S32 *out, HV_S32 dim0, HV_S32 dim1, HV_S32 dim2)
{
	HV_S32 i, j, k;
	memset(out, 0, dim0 * dim2 * sizeof(HV_S32));
	for (i = 0; i < dim0; i++)
	{
		for (j = 0; j < dim2; j++)
		{
			for (k = 0; k < dim1; k++)
			{
				out[i*dim2 + j] += a_in[i*dim1 + k] * b_in[k*dim2 + j];
			}
		}
	}
}

//in is dim0*dim1, out is dim1*dim0
HV_HVID matrix_trans(HV_S32 *in, HV_S32 *out, HV_S32 dim0, HV_S32 dim1)
{
	HV_S32 temp, i, j;
	for (i = 0; i < dim1; i++)
	{
		for (j = 0; j < dim0; j++)
		{
			out[i*dim0 + j] = in[j*dim1 + i];
		}
	}
}

//just for 3x3 matrix
HV_HVID matrix_inv(HV_F32 *a_in, HV_F32 *a_out)
{
	HV_F32 a1 = a_in[0], b1 = a_in[1], c1 = a_in[2], a2 = a_in[3], b2 = a_in[4], c2 = a_in[5], a3 = a_in[6], b3 = a_in[7], c3 = a_in[8];
	HV_F32 temp = a1*(b2*c3 - c2*b3) - a2*(b1*c3 - c1*b3) + a3*(b1*c2 - c1*b2);
	a_out[0] = (b2*c3 - c2*b3) / temp;
	a_out[1] = (c1*b3-b1*c3) / temp;
	a_out[2] = (b1*c2-c1*b2) / temp;
	a_out[3] = (c2*a3-a2*c3) / temp;
	a_out[4] = (a1*c3-c1*a3) / temp;
	a_out[5] = (a2*c1-a1*c2) / temp;
	a_out[6] = (a2*b3-b2*a3) / temp;
	a_out[7] = (b1*a3-a1*b3) / temp;
	a_out[8] = (a1*b2-a2*b1) / temp;

}

HV_HVID draw_point(HV_S32 ori_size, HV_BYTE *img_in, HV_BYTE *uv_in, HV_S32 i_in, HV_S32 j_in, HV_BYTE y_val, HV_BYTE u_val, HV_BYTE v_val)
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 ratio = 1;
	if (ori_size == 1)
	{
		ratio = SCALE_RATIO;
	}
	width *= ratio;
	height *= ratio;
	if (j_in <= 0 || j_in >= width - 1)
		return;
	HV_S32 cur_i = i_in - i_in % 2;
	HV_S32 cur_j = j_in - j_in % 2;
	//draw 2*k pixels in horizontal
	for (HV_S32 k = 0; k < 4; k++)
	{
		HV_S32 i_draw = cur_i + 0;
		HV_S32 j_draw = cur_j + 2 * k;
		i_draw = max(0, min(height - 2, i_draw));
		j_draw = max(0, min(width - 2, j_draw));
		img_in[i_draw*width + j_draw] = y_val;
		img_in[(i_draw + 1)*width + j_draw] = y_val;
		img_in[i_draw*width + j_draw + 1] = y_val;
		img_in[(i_draw + 1)*width + j_draw + 1] = y_val;
#ifdef _WIN32
		img_in[width*height+i_draw / 2 * width / 2 + j_draw / 2] = u_val;
		img_in[width*height*5/4+i_draw / 2 * width / 2 + j_draw / 2] = v_val;
#else
		uv_in[i_draw / 2 * width + 2 * (j_draw / 2)] = u_val;
		uv_in[i_draw / 2 * width + 2 * (j_draw / 2)+1] = v_val;
#endif

	}
}

HV_HVID draw_lane(HV_S32 ori_size, HV_BYTE *img_in, HV_BYTE *uv_in, lane_t *lane, HV_BYTE y_val, HV_BYTE u_val, HV_BYTE v_val)
{
	if (lane->detected == 0||lane->tracking_num<0)
		return;

	HV_S32 i, j;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 ratio = 1;
	if (ori_size == 1)
	{
		ratio  = SCALE_RATIO;
	}
	HV_S32 lane_type = lane->lane_type;

	extern HV_F32 g_perspective_mat[9];
	extern HV_F32 g_inv_perspective_mat[9];
	extern HV_S32 g_roi_top;
	for (i = height - 1; i > g_roi_top; i--)
	{
		/*if (lane->lane_type == 1 && (i/20)%2==0)
			continue;*/
		HV_S32 i_bird, j_bird;
		perspective_transform(i, width / 2, i_bird, j_bird, g_perspective_mat);
		if (i_bird<lane->top_end || i_bird>lane->bot_end)
			continue;
		j_bird = lane->a0 + lane->a1*i_bird + lane->a2*i_bird*i_bird;
		HV_S32 i_ori, j_ori;
		perspective_transform(i_bird, j_bird, i_ori, j_ori, g_inv_perspective_mat);
		//calc point on small image, but draw coordinate should recover to input image
		i_ori *= ratio;
		j_ori *= ratio;
		if(i_bird<lane->type_change_pos && lane->type_change_pos!=-1&&lane_type==1)
			draw_point(ori_size, img_in, uv_in, i_ori, j_ori, 80, 128, 255);
		else
			draw_point(ori_size, img_in, uv_in, i_ori, j_ori, y_val, u_val, v_val);
	}

	/*for (i = 0; i < height; i++)
	{
		if (lane->lane_type == 1 && (i / 20) % 2 == 0)
			continue;
		j= lane->a0 + lane->a1*i + lane->a2*i*i;
		HV_S32 i_ori, j_ori;
		perspective_transform(i, j, i_ori, j_ori, g_inv_perspective_mat);
		draw_point(img_in, i_ori, j_ori, u_val, v_val);
	}*/

	
}

HV_HVID draw_lane_world_param(HV_BYTE *img_in, lane_t *lane)
{
	if (lane->detected == 0)
		return;
	HV_S32 width = ORI_IMG_WIDTH;
	HV_S32 height = ORI_IMG_HEIGHT;
	HV_S32 i, j;
	for (i = height - 1; i > g_detect_dist_mid*SCALE_RATIO; i--)
	{
		HV_S32 cur_x, cur_y;
		distance_measure(i, ORI_IMG_WIDTH / 2, cur_x, cur_y);
		HV_F32 double_x = HV_F32(cur_x) / 256;
		HV_F32 double_y = lane->a0+lane->a1*double_x+lane->a2*double_x*double_x;
		cur_y = double_y * 256;
		HV_S32 cur_i, cur_j;
		invert_distance_measure(cur_x, cur_y, cur_i, cur_j);
		cur_i = i;
		//draw_point(img_in, cur_i, cur_j, 0, 255);
	}
}


HV_HVID init_image(image_t *img_in, HV_S32 width, HV_S32 height)
{

	img_in->height = height;
	img_in->width = width;
	img_in->y_in = new HV_BYTE[width*height];
	memset(img_in->y_in, 0, sizeof(HV_BYTE)*width*height);
	img_in->u_in = new HV_BYTE[width*height];
	memset(img_in->u_in, 0, sizeof(HV_BYTE)*width*height);
	img_in->v_in = new HV_BYTE[width*height];
	memset(img_in->v_in, 0, sizeof(HV_BYTE)*width*height);

}

HV_HVID release_image(image_t *img_in)
{
	delete[] img_in->y_in;
	delete[] img_in->u_in;
	delete[] img_in->v_in;
}

//j_0 is the down j
//angle is between the line and horizontal, top_left and bot_right is smaller than 90
HV_S32 calc_angle(HV_S32 height, HV_S32 j_0, HV_S32 j_1)
{
	HV_S32 cur_width, cur_height, down_pos, up_pos;
	up_pos = j_1;
	down_pos = j_0;
	cur_width = down_pos - up_pos;
	cur_height = height;
	while (max(cur_width, cur_height) >= 255 || min(cur_width, cur_height)<-255)
	{
		cur_width /= 2;
		cur_height /= 2;
	}
	HV_S32 angle = g_angle_table[(cur_width + 255) * 511 + cur_height + 255];
	return angle;
}

//grad is a of cur line, top_left and bot_right is negative
//angle is between the line and horizontal, top_left and bot_right is smaller than 90
HV_S32 grad_to_angle(HV_F32 grad)
{
	HV_S32 cur_width = 100;
	HV_S32 cur_height = -cur_width*grad;
	while (max(cur_width, cur_height) >= 255 || min(cur_width, cur_height)<-255)
	{
		cur_width /= 2;
		cur_height /= 2;
	}
	HV_S32 angle = g_angle_table[(cur_width + 255) * 511 + cur_height + 255];
	if (angle > 180)
		angle -= 180;
	return angle;
}

HV_F32 angle_to_grad(HV_S32 angle)
{
	HV_F32 grad;
	while (angle < 0)
		angle += 180;
	while (angle >= 180)
		angle -= 180;
	if (cos_table[angle] != 0)
	{
		HV_F32 tan_angle = sin_table[angle] / cos_table[angle];
		grad = -tan_angle;
	}
	else
	{
		grad = MAX_VAL;
	}
	return grad;
}

#ifdef _WIN32
vector<string> get_files(string path)
{
	vector<string> file_name;
	intptr_t lf;
	_finddata_t file;
	string p;
	if ((lf = _findfirst(p.assign(path).append("\\*").c_str(), &file)) == -1)
	{
		cout << path << "not found!!!!!!!!!!!!!" << endl;
	}
	else
	{
		while (_findnext(lf, &file) == 0)
		{
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;
			file_name.push_back(p.assign(path).append("\\").append(file.name));
		}
	}
	_findclose(lf);
	return file_name;
}
#endif

HV_HVID mat_to_yuv(HV_BYTE *yuv_img, Mat *mat_img, HV_S32 width, HV_S32 height)
{
	HV_S32 i, j;
	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			yuv_img[i*width + j] = (*mat_img).data[3 * (i*width + j)];
		}
	}
	for (i = 0; i < height / 2; i++)
	{
		for (j = 0; j < width / 2; j++)
		{
			yuv_img[width*height + i*width / 2 + j] = (*mat_img).data[3 * (i * 2 * width + j * 2) + 1];
			yuv_img[width*height * 5 / 4 + i*width / 2 + j] = (*mat_img).data[3 * (i * 2 * width + j * 2) + 2];
		}
	}
}

HV_HVID yuv_to_mat(Mat *mat_img, HV_BYTE *yuv_img)
{
	HV_S32 width = ORI_IMG_WIDTH;
	HV_S32 height = ORI_IMG_HEIGHT;
	HV_S32 i, j;
	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			(*mat_img).data[3 * (i*width + j)] = yuv_img[i*width + j];
			(*mat_img).data[3 * (i*width + j) + 1] = yuv_img[width*height + i / 2 * width / 2 + j / 2];
			(*mat_img).data[3 * (i*width + j) + 2] = yuv_img[width*height * 5 / 4 + i / 2 * width / 2 + j / 2];
		}
	}
}

image_t yuv_to_img(HV_BYTE *yuv_img)
{
	image_t img_out;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 i, j;
	img_out.width = width;
	img_out.height = height;
	img_out.y_in = yuv_img;
	img_out.u_in = yuv_img + width*height;
	img_out.v_in = yuv_img + width*height * 5 / 4;
	return img_out;
}

HV_HVID img_scaling(HV_BYTE *img_in, HV_BYTE *img_out)
{
	HV_S32 input_width = ORI_IMG_WIDTH;
	HV_S32 input_height = ORI_IMG_HEIGHT;
	HV_S32 output_width = IMG_WIDTH;
	HV_S32 output_height = IMG_HEIGHT;
	HV_S32 filter[32 * 7];
	HV_S32 v_step, h_step;

	HV_S32 i, j, k;
	HV_S32 cur_ratio = SCALE_RATIO;
	if (SCALE_RATIO >= 2)
	{
		//scale y
		for (i = 0; i < output_height; i++)
		{
			for (j = 0; j < output_width; j++)
			{
				if (i == 135 && j == 16)
				{
					HV_S32 qqq = 3;
				}
				img_out[i*output_width + j] = img_in[(cur_ratio * i)*input_width + (cur_ratio * j)]
					+ img_in[(cur_ratio * i + 1)*input_width + (cur_ratio * j)]
					+ img_in[(cur_ratio * i)*input_width + (cur_ratio * j + 1)]
					+ img_in[(cur_ratio * i + 1)*input_width + (cur_ratio * j + 1)] >> 2;
			}
		}
		//scale uv
	}
	else
	{
		memcpy(img_out, img_in, input_width*input_height*3/2);
	}
}

HV_S32 get_lane_pos(lane_t *lane, HV_S32 i)
{
	//HV_S32 width = IMG_WIDTH ;
	//HV_S32 height = IMG_HEIGHT ;
	//HV_F32 bot_a = angle_to_grad(lane->bot_angle);
	//HV_S32 cur_j;
	//if (i >= lane->mid_i)
	//{
	//	HV_S32 cur_j = lane->bot_j + (height - i - 1) / bot_a;
	//	return cur_j;
	//}

	//HV_S32 seg_j= lane->bot_j + (height - lane->mid_i - 1) / bot_a;
	//HV_S32 bot_angle = lane->bot_angle;
	//HV_S32 top_angle = lane->top_angle;
	//HV_S32 cur_i = i;
	//HV_S32 cur_angle = ((i - lane->top_i)*bot_angle + (lane->mid_i - i)*top_angle) / (lane->mid_i - lane->top_i);
	//HV_F32 cur_a = angle_to_grad(cur_angle);
	//cur_j = seg_j + (lane->mid_i - i) / cur_a;

	//return cur_j;
	return 0;
}

bool sort_degree(const lane_t &lane1, const lane_t &lane2)
{
	return lane1.degree > lane2.degree;
}

HV_S32 calc_lane_diff(lane_t lane0, lane_t lane1)
{
	HV_S32 diff = abs(lane0.bot_j - lane1.bot_j) + abs(lane0.mid_j - lane1.mid_j);
	//extern HV_F32 g_y_dist_in_perspective;
	//diff *= g_y_dist_in_perspective;
	return diff;
}

HV_HVID draw_zebra(HV_BYTE* img_in, vector<zebra_area_t> *zebra_areas)
{
	HV_S32 i, j, k;
	for (k = 0; k < zebra_areas->size();k++)
	{
		zebra_area_t cur_area = (*zebra_areas)[k];
		for (j = cur_area.left; j <= cur_area.right; j++)
		{
			//draw_point(img_in, cur_area.top, j, 0, 0);
			//draw_point(img_in, cur_area.bot, j, 0, 0);
		}
		for (i = cur_area.top; i <= cur_area.bot; i++)
		{
			//draw_point(img_in, i, cur_area.left, 0, 0);
			//draw_point(img_in, i, cur_area.right, 0, 0);
		}
	}
}

HV_HVID transfer_to_out_param(lane_t cur_lane, HV_F32 out_param[7])
{
	extern HV_F32 g_y_dist_in_perspective;
	extern HV_F32 g_x_dist_in_perspective;
	extern HV_F32 g_x_min_dist_in_perspective;
	out_param[3] = 0;
	HV_F32 width = IMG_WIDTH;
	HV_F32 height = IMG_HEIGHT;
	HV_F32 i0 = 0, i1 = height / 2, i2 = height;
	HV_F32 j0, j1, j2;
	j0 = cur_lane.a0 + cur_lane.a1*i0 + cur_lane.a2*i0*i0;
	j1 = cur_lane.a0 + cur_lane.a1*i1 + cur_lane.a2*i1*i1;
	j2 = cur_lane.a0 + cur_lane.a1*i2 + cur_lane.a2*i2*i2;
	i0 = (height - i0) / height*(g_x_dist_in_perspective - g_x_min_dist_in_perspective) + g_x_min_dist_in_perspective;
	i1 = (height - i1) / height*(g_x_dist_in_perspective - g_x_min_dist_in_perspective) + g_x_min_dist_in_perspective;
	i2 = (height - i2) / height*(g_x_dist_in_perspective - g_x_min_dist_in_perspective) + g_x_min_dist_in_perspective;
	j0 = (width / 2 - j0) / width * 2 * g_y_dist_in_perspective;
	j1 = (width / 2 - j1) / width * 2 * g_y_dist_in_perspective;
	j2 = (width / 2 - j2) / width * 2 * g_y_dist_in_perspective;
	curve_fitting(out_param[0], out_param[1], out_param[2], i0, j0, i1, j1, i2, j2);
	out_param[4] = (height - cur_lane.top_end) / height*(g_x_dist_in_perspective - g_x_min_dist_in_perspective) + g_x_min_dist_in_perspective;
	out_param[5] = (height - cur_lane.bot_end) / height*(g_x_dist_in_perspective - g_x_min_dist_in_perspective) + g_x_min_dist_in_perspective;
	if (cur_lane.type_change_pos != -1)
		out_param[6] = (height - cur_lane.type_change_pos) / height*(g_x_dist_in_perspective - g_x_min_dist_in_perspective) + g_x_min_dist_in_perspective;
	else
		out_param[6] = -1;
}

HV_HVID save_width_map(HV_BYTE *width_img)
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	FILE *fp_width_img = fopen("e:\\1.bin", "wb");
	
	fwrite(width_img, width*height, sizeof(HV_BYTE), fp_width_img);
	fclose(fp_width_img);
}

HV_HVID draw_stop_line(HV_BYTE *img_in, stop_line_t *stop_line)
{
	HV_S32 i, j;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;

	extern HV_S32 g_roi_top;
	for (j = 0; j < width; j++)
	{
		HV_S32 cur_i = (HV_F32(j) - stop_line->a0) / stop_line->a1;
		//draw_point(img_in, cur_i, j, 128, 255);
		//draw_point(img_in, max(0, cur_i - 1), j, 128, 255);
		//draw_point(img_in, max(0, cur_i - 2), j, 128, 255);
		//draw_point(img_in, max(0, cur_i - 3), j, 128, 255);
	}
}

HV_HVID sort_lane(vector<lane_t> *lanes)
{
	HV_S32 lane_num = lanes->size();
	for (HV_S32 i = 0; i < lane_num - 1; i++)
	{
		for (HV_S32 j = 0; j < lane_num - 1 - i; j++)
		{
			if ((*lanes)[j].degree < (*lanes)[j + 1].degree)
				swap((*lanes)[j], (*lanes)[j + 1]);
		}
	}
}
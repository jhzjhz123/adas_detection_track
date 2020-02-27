#include "distance_measure.h"

//FILE *fptemp = fopen("e:\\yuv\\lane\\correct.yuv", "wb");

//pitch angle is arctan((cy-vanish_line)/fy)
//yaw angle is arctan((center_point-cx)/fx)

//left/clockwise/down is positive

HV_S32 g_i_offset = 0;	//top cut position

//201 video
HV_F32 g_sin_camera_yaw = -0.1079, g_cos_camera_yaw = 0.9942;
HV_F32 g_sin_camera_pitch = 0.0008, g_cos_camera_pitch = 0.9999;
HV_F32 g_sin_camera_roll = 0.007976, g_cos_camera_roll = 0.9999;
HV_F32 g_sin_vehicle_pitch = 0, g_cos_vehicle_pitch = 1;	//identical to pitch angle, negative when head is up
HV_F32 g_sin_vehicle_roll = 0, g_cos_vehicle_roll = 1;

//veran
HV_F32 g_k1 = -0.48571, g_k2 = 0.292982, g_k3 = -0.54362;
HV_F32 g_p1 = -0.00503, g_p2 = -0.00544;
HV_S32 g_fx = 1479, g_fy = 1472;
HV_S32 g_cx = 671, g_cy = 377;
HV_S32 g_camera_position = -92;
HV_S32 g_camera_height = 345;
HV_S32 g_camera_head = 0;

//srx
//HV_F32 g_k1 = -0.48571, g_k2 = 0.292982, g_k3 = -0.54362;
//HV_F32 g_p1 = -0.00503, g_p2 = -0.00544;
//HV_S32 g_fx = 1468, g_fy = 1470;
//HV_S32 g_cx = 653, g_cy = 306;
//HV_S32 g_camera_position = -26;
//HV_S32 g_camera_height = 372;

//tiguan
//HV_F32 g_k1 = -0.2907, g_k2 = -0.0529, g_k3 = -0.8340;
//HV_F32 g_p1 = 0.002612, g_p2 = 0.00926;
//HV_S32 g_fx = 1458, g_fy = 1463;
//HV_S32 g_cx = 648, g_cy = 229;
//HV_S32 g_camera_position = 38;
//HV_S32 g_camera_height = 348;

HV_F32 camera_world_matrix[9];
HV_F32 world_camera_matrix[9];

HV_F32 g_perspective_mat[9];
HV_F32 g_inv_perspective_mat[9];
HV_F32 g_y_dist_in_perspective = 8;
HV_F32 g_x_dist_in_perspective = 40;
HV_F32 g_x_min_dist_in_perspective = 0;


//HV_S32 g_focal_length = 360;	//focal length in pixel
//HV_S32 g_camera_height = 384;	//camera height in meter *256
//HV_S32 g_tan_theta = 68;	//tan_theta*256, theta is angle between camera and horizontal, positive when camera adown

//calc distance of single light
HV_S32 distance_measure(HV_S32 i_in, HV_S32 j_in, HV_S32 &x_distance, HV_S32 &y_distance)
{
	HV_S32 i_correct, j_correct;
	HV_F32 y_world, z_world;
	//i_in = 413; j_in = 506;
	get_real_position(i_in, j_in, y_world, z_world, i_correct, j_correct, camera_world_matrix);
	if (z_world > 0)
	{
		x_distance = -1;
		return -1;
	}
	x_distance = g_camera_height / abs(z_world);
	x_distance -= g_camera_head;
	y_distance = x_distance * y_world + g_camera_position;
	return 0;
}

HV_HVID distance_measure_mat(HV_S32 i_in, HV_S32 j_in, HV_S32 &x_distance, HV_S32 &y_distance)
{
	HV_S32 i_bird, j_bird;
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	perspective_transform(i_in, j_in, i_bird, j_bird, g_perspective_mat);
	//j_bird=0, y_dist is g_y_dist_in_perspective*256
	//g_perspective_mat has already considered camera position
	y_distance = (width / 2 - j_bird) * 256 * g_y_dist_in_perspective / (width / 2);
	x_distance = (height - i_bird) * 256 * (g_x_dist_in_perspective - g_x_min_dist_in_perspective) / height+g_x_min_dist_in_perspective*256;
}

//input distance, output position in input image
HV_S32 invert_distance_measure(HV_S32 x_distance, HV_S32 y_distance, HV_S32 &i_out, HV_S32 &j_out)
{
	//transfer to distance to the camera
	x_distance += g_camera_head;
	y_distance -= g_camera_position;
	HV_F32 world_matrix[3] = { x_distance, y_distance, -g_camera_height };
	HV_F32 camera_matrix[3];
	matrix_multiply(world_camera_matrix, world_matrix, camera_matrix, 3, 3, 1);
	HV_F32 y_correct = camera_matrix[1] / camera_matrix[0];
	HV_F32 z_correct = camera_matrix[2] / camera_matrix[0];
	//distortion transfer

	HV_F32 double_i_out = HV_F32(g_cy) - z_correct*HV_F32(g_fx);
	HV_F32 double_j_out = HV_F32(g_cx) - y_correct*HV_F32(g_fy);
	i_out = double_i_out;
	j_out = double_j_out;

	return 0;
}

//eliminate distortion and camera deviation
//y_world is y/x in world coordinate
HV_HVID get_real_position(HV_S32 i_in, HV_S32 j_in, HV_F32 &y_world, HV_F32 &z_world, HV_S32 &i_out, HV_S32 & j_out, HV_F32 *camera_world_matrix)
{
	HV_F32 y_in = (HV_F32(g_cx - j_in)) / HV_F32(g_fx);
	HV_F32 z_in = (HV_F32(g_cy - i_in)) / HV_F32(g_fy);
	HV_F32 r = pow(y_in, 2) + pow(z_in, 2);
	//avoid error when pixel is outof image, this will cause r too big and cause error in dedistortion
	r = min(0.14, r);
	//y/x in camera coordinate
	//dedistortion will cause error when pixel is outof the image
	HV_F32 y_correct = y_in / (1 + g_k1*r + g_k2*r*r + g_k3*r*r*r);
	HV_F32 z_correct = z_in / (1 + g_k1*r + g_k2*r*r + g_k3*r*r*r);
	//HV_F32 y_correct = y_in;
	//HV_F32 z_correct = z_in;

	HV_F32 camera_matrix[3] = { 1, HV_F32(y_correct), HV_F32(z_correct) };
	HV_F32 world_matrix[3];
	matrix_multiply(camera_world_matrix, camera_matrix, world_matrix, 3, 3, 1);
	y_world = world_matrix[1] / world_matrix[0];
	z_world = world_matrix[2] / world_matrix[0];
	i_out = g_cy - z_world * g_fy;
	j_out = g_cx - y_world * g_fx;
	//i_out = z_correct * g_fy + g_cy;
	//j_out = y_correct * g_fx + g_cx;
}

HV_HVID store_correct_img(HV_BYTE *img, HV_S32 width, HV_S32 height, HV_F32 *camera_world_matrix)
{
	//transfer from camera coordinate to world coordinate
	HV_BYTE *correct_img = new HV_BYTE[width * height];
	memset(correct_img, 0, width * height * sizeof(HV_BYTE));
	HV_S32 i, j, k, i1, j1, k1;
	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			if (i == 50 && j == 50)
			{
				HV_S32 qqq = 3;
			}
			HV_S32 i_out, j_out;
			HV_F32 y_world, z_world;
			get_real_position(i, j, y_world, z_world, i_out, j_out, camera_world_matrix);
			i_out = max(0, min(height - 1, i_out));
			j_out = max(0, min(width - 1, j_out));
			correct_img[i_out*width + j_out] = img[i*width + j];
		}
	}

	//fwrite(correct_img, width, height, fptemp);
	delete[] correct_img;
}

HV_HVID perspective_mat(HV_F32 A[][9], HV_S32 equ, HV_S32 var, HV_F32* ans)
{
	HV_S32 row, col;
	for (row = 0, col = 0; col<var&&row<equ; col++, row++)
	{
		HV_S32 max_r = row;
		for (HV_S32 i = row + 1; i<equ; i++)
		{
			if ((1e-12)<fabs(A[i][col]) - fabs(A[max_r][col]))
			{
				max_r = i;
			}
		}
		if (max_r != row)
			for (HV_S32 j = 0; j<var + 1; j++)
				swap(A[row][j], A[max_r][j]);
		for (HV_S32 i = row + 1; i<equ; i++)
		{
			if (fabs(A[i][col])<(1e-12))
				continue;
			HV_F32 tmp = -A[i][col] / A[row][col];
			for (HV_S32 j = col; j<var + 1; j++)
			{
				A[i][j] += tmp*A[row][j];
			}
		}
	}
	for (HV_S32 i = var - 1; i >= 0; i--)
	{
		HV_F32 tmp = 0;
		for (HV_S32 j = i + 1; j<var; j++)
		{
			tmp += A[i][j] * (*(ans + j));
		}
		ans[i] = (A[i][var] - tmp) / A[i][i];
	}
}

HV_HVID get_perspective_matrix()
{
	//get mat from ori to bird view
	//bird image position
	HV_F32 u0 = 0, u1 = IMG_WIDTH-1, u2 = 0, u3 = IMG_WIDTH - 1;
	HV_F32 v0 = 0, v1 = 0, v2 = IMG_HEIGHT-1, v3 = IMG_HEIGHT - 1;
	HV_F32 x0, x1, x2, x3;
	HV_F32 y0, y1, y2, y3;
	HV_S32 temp_i, temp_j;
	//ori image position
	invert_distance_measure(g_x_dist_in_perspective * 256, g_y_dist_in_perspective * 256, temp_i, temp_j);
	x0 = HV_S32(temp_j) / SCALE_RATIO;
	y0 = HV_S32(temp_i) / SCALE_RATIO;
	invert_distance_measure(g_x_dist_in_perspective * 256, -g_y_dist_in_perspective * 256, temp_i, temp_j);
	x1 = HV_S32(temp_j) / SCALE_RATIO;
	y1 = HV_S32(temp_i) / SCALE_RATIO;
	invert_distance_measure(g_x_min_dist_in_perspective * 256, g_y_dist_in_perspective * 256, temp_i, temp_j);
	x2 = HV_S32(temp_j) / SCALE_RATIO;
	y2 = HV_S32(temp_i) / SCALE_RATIO;
	invert_distance_measure(g_x_min_dist_in_perspective * 256, -g_y_dist_in_perspective * 256, temp_i, temp_j);
	x3 = HV_S32(temp_j) / SCALE_RATIO;
	y3 = HV_S32(temp_i) / SCALE_RATIO;
	HV_F32 A[8][9] = {
		{ x0, y0, 1, 0, 0, 0, -x0*u0, -y0*u0, u0 },
		{ x1, y1, 1, 0, 0, 0, -x1*u1, -y1*u1, u1 },
		{ x2, y2, 1, 0, 0, 0, -x2*u2, -y2*u2, u2 },
		{ x3, y3, 1, 0, 0, 0, -x3*u3, -y3*u3, u3 },
		{ 0, 0, 0, x0, y0, 1, -x0*v0, -y0*v0, v0 },
		{ 0, 0, 0, x1, y1, 1, -x1*v1, -y1*v1, v1 },
		{ 0, 0, 0, x2, y2, 1, -x2*v2, -y2*v2, v2 },
		{ 0, 0, 0, x3, y3, 1, -x3*v3, -y3*v3, v3 },
	};
	perspective_mat(A, 8, 8, g_perspective_mat);
	*(g_perspective_mat + 8) = 1;
	//get mat form bird view to ori
	HV_F32 temp_val;
	temp_val = u0; u0 = x0; x0 = temp_val;
	temp_val = u1; u1 = x1; x1 = temp_val;
	temp_val = u2; u2 = x2; x2 = temp_val;
	temp_val = u3; u3 = x3; x3 = temp_val;
	temp_val = v0; v0 = y0; y0 = temp_val;
	temp_val = v1; v1 = y1; y1 = temp_val;
	temp_val = v2; v2 = y2; y2 = temp_val;
	temp_val = v3; v3 = y3; y3 = temp_val;
	HV_F32 B[8][9] = {
		{ x0, y0, 1, 0, 0, 0, -x0*u0, -y0*u0, u0 },
		{ x1, y1, 1, 0, 0, 0, -x1*u1, -y1*u1, u1 },
		{ x2, y2, 1, 0, 0, 0, -x2*u2, -y2*u2, u2 },
		{ x3, y3, 1, 0, 0, 0, -x3*u3, -y3*u3, u3 },
		{ 0, 0, 0, x0, y0, 1, -x0*v0, -y0*v0, v0 },
		{ 0, 0, 0, x1, y1, 1, -x1*v1, -y1*v1, v1 },
		{ 0, 0, 0, x2, y2, 1, -x2*v2, -y2*v2, v2 },
		{ 0, 0, 0, x3, y3, 1, -x3*v3, -y3*v3, v3 },
	};
	perspective_mat(B, 8, 8, g_inv_perspective_mat);
	*(g_inv_perspective_mat + 8) = 1;
}

HV_HVID perspective_transform(HV_S32 i_in, HV_S32 j_in, HV_S32 &i_out, HV_S32 &j_out, HV_F32 *mat)
{
	if (i_in == 276 && j_in == 284)
	{
		HV_S32 qqq = 3;
	}
	HV_F32 D = j_in*mat[6] + i_in*mat[7] + mat[8];
	j_out = (j_in*mat[0] + i_in*mat[1] + mat[2]) / D;
	i_out = (j_in*mat[3] + i_in*mat[4] + mat[5]) / D;
}

HV_HVID img_perspective_transform(HV_BYTE *img_in, HV_BYTE *img_out)
{
	HV_S32 width = IMG_WIDTH;
	HV_S32 height = IMG_HEIGHT;
	HV_S32 i, j;


	for (i = 0; i < height - g_i_offset; i++)
	{
		for (j = 0; j < width; j++)
		{
			if (i == 600&&j==1000)
			{
				HV_S32 qqq = 3;
			}
			//map from ori to bird
#ifdef _WIN32
			if (img_in[i*width + j] <20)
				continue;
			if (i == 320)
			{
				HV_S32 qqq = 3;
			}
			HV_S32 i_out, j_out;
			//don't need to multiply scale_ratio here, because points used to calc perspective_mat is multiplied already
			perspective_transform((i + g_i_offset), j, i_out, j_out, g_perspective_mat);
			if (i_out>0 && i_out<height && j_out>0 && j_out<width)
			{
				img_out[i_out*width + j_out] = img_in[i*width + j];
			}
#else
			if (img_in[3*(i*width + j)] <20)
				continue;
			HV_S32 i_out, j_out;
			perspective_transform(i + g_i_offset, j, i_out, j_out, g_perspective_mat);
			if (i_out>0 && i_out<720 && j_out>0 && j_out<1280)
			{
				img_out[i_out*width + j_out] = img_in[3*(i*width + j)];
			}
#endif
		}
	}
}
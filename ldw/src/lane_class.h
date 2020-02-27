#ifndef LANE_CLASS_H
#define LANE_CLASS_H

#include "common.h"
#include "../hvapi/hvType.h"

class lane_detect_c {
public:
	lane_detect_c(HV_INIT_USERDATA * pUserData);
	~lane_detect_c();
	HV_HVID initial_param();
	HV_HVID lane_detect_frame(HV_HVID *frame_input);

	signed long(HV_API * hv_clock) ();
	int(HV_API * hv_printf) (const char *format, ...);
	int(HV_API *hv_rand) ();

	point_t get_cross_point(HV_BYTE* frame_input);

//protected:
	lane_t *cur_lanes_[4], *ref_lanes_[4], *tracking_lanes_[4];
	HV_F32 camera_yaw_, camera_pitch_, camera_roll_;
	HV_F32 g_k1_, g_k2_, g_k3_;
	HV_F32 g_p1_, g_p2_;
	HV_S32 g_fx_, g_fy_;
	HV_S32 g_cx_, g_cy_;
	HV_S32 g_camera_position_;
	HV_S32 g_camera_height_;
	HV_S32 g_camera_head_;
	HV_S32 zebra_en_, stop_line_en_, mat_input_en_;
	HV_F32 cross_i_, cross_j_;

};

#endif

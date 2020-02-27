#ifndef LANE_TRACKING_H
#define LANE_TRACKING_H

#include "common.h"

//HV_HVID get_vanish_point(line_t *left_lane, line_t *right_lane, HV_S32 &i_out, HV_S32 &j_out, HV_S32 last_i, HV_S32 last_j);

HV_HVID kalman_filtering(HV_F32 &cur_i, HV_F32 last_i, HV_S32 &variance);

HV_HVID kalman_filtering(HV_S32 &cur_i, HV_S32 last_i, HV_S32 &variance);

HV_HVID update_ref_lanes(lane_t *cur_lanes[4], lane_t *ref_lanes[4]);

HV_S32 tracking_success(lane_t lane0, lane_t lane1);

HV_HVID filter_ref_lanes(lane_t *ref_lanes[4]);

#endif

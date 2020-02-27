#include "hv_lane.h"

HV_LANEDET_DONE_CB g_call_back_func;
extern HV_COMMON_API g_common_api;
extern HV_S32 g_roi_top;


static HV_S32 HV_LaneDet_RegisterDoneCB(HV_LANEDET_DONE_CB callback)
{
	g_call_back_func = callback;
	return 1;
}

static HV_S32 HV_DrawPoint(HV_S32 *points, HV_PTR pData, HV_S32 lane_index)

{
    HV_VIDEO_OUTPUTINFO *p_out_info = (HV_VIDEO_OUTPUTINFO *)pData;
    lane_t cur_lane;

    cur_lane.detected = p_out_info->detected[lane_index];
    cur_lane.lane_type = p_out_info->lane_type[lane_index];
    cur_lane.a0 = p_out_info->a0[lane_index];
    cur_lane.a1 = p_out_info->a1[lane_index];
    cur_lane.a2 = p_out_info->a2[lane_index];
    cur_lane.bot_end = p_out_info->bot_end[lane_index];
    cur_lane.top_end = p_out_info->top_end[lane_index];

    HV_S32 width = IMG_WIDTH;
    HV_S32 height = IMG_HEIGHT;
    extern HV_F32 g_perspective_mat[9];
    extern HV_F32 g_inv_perspective_mat[9];

    for (int i = height - 1; i > g_roi_top; i--)
    {
        HV_S32 i_bird, j_bird;
        perspective_transform(i, width / 2, i_bird, j_bird, g_perspective_mat);
        HV_S32 temp_i = HV_S32(i_bird);
        if (temp_i<cur_lane.top_end || temp_i>cur_lane.bot_end)
            continue;
        j_bird = cur_lane.a0 + cur_lane.a1*i_bird + cur_lane.a2*i_bird*i_bird;
        //move to inner side of the lane
        if (lane_index<2)
            j_bird += 5;
        else
            j_bird -= 5;
        HV_S32 i_ori, j_ori;
        perspective_transform(i_bird, j_bird, i_ori, j_ori, g_inv_perspective_mat);
        points[max(0, min(height - 1, i_ori))] = j_ori;
    }
    return g_roi_top;
}

static HV_S32 HV_DrawLane(HV_PTR buffer[], HV_PTR pData)
{
    HV_VIDEO_OUTPUTINFO *p_out_info = (HV_VIDEO_OUTPUTINFO *)pData;
    lane_t lanes[4];

    for (int i = 0; i < 4; i++)
    {
        lanes[i].detected = p_out_info->detected[i];
        lanes[i].lane_type = p_out_info->lane_type[i];
        lanes[i].a0 = p_out_info->a0[i];
        lanes[i].a1 = p_out_info->a1[i];
        lanes[i].a2 = p_out_info->a2[i];
		lanes[i].bot_end = p_out_info->bot_end[i];
		lanes[i].top_end = p_out_info->top_end[i];
		lanes[i].tracking_num = p_out_info->tracking_num[i];
		lanes[i].type_change_pos = p_out_info->type_change_pos[i];
		

		if(lanes[i].lane_type==0)
			draw_lane(1, (byte*)buffer[0], (byte*)buffer[1], &lanes[i], 80, 128, 255);
		if (lanes[i].lane_type == 1)
			draw_lane(1, (byte*)buffer[0], (byte*)buffer[1], &lanes[i], 255, 0, 0);
		if (lanes[i].lane_type == 2)
			draw_lane(1, (byte*)buffer[0], (byte*)buffer[1], &lanes[i], 0, 255, 128);
    }

    return 0;
}

static HV_U32 HVLaneDetGetParam(HV_HANDLE hLaneDet, HV_S32 uParamID, HV_PTR pData)
{
    lane_detect_c *lane_detect = (lane_detect_c*)hLaneDet;
    HV_U32 Result = 0;

    return Result;
}

HV_S32 HV_API hvGetLaneDetAPI(HV_LANE_DETAPI * pDecHandle, HV_U32 uFlag)
{
	HV_S32 ret = 0;
    
	pDecHandle->Init = Init;
	pDecHandle->SetInputData = SetInputData;
	pDecHandle->GetOutputData = GetOutputData;
    pDecHandle->GetParam = HVLaneDetGetParam;
	pDecHandle->Uninit = Uninit;
    pDecHandle->LaneDetRegisterDoneCB = HV_LaneDet_RegisterDoneCB;
    pDecHandle->DrawLane = HV_DrawLane;
    pDecHandle->DrawPoint = HV_DrawPoint;

	return ret;
}

HV_U32 Init(HV_HANDLE * phLaneDet, HV_INIT_USERDATA * pUserData)
{
	HV_U32 ret = 0;
	lane_detect_c *lane_detect = new lane_detect_c(pUserData);

    g_common_api.hv_clock = pUserData->hv_clock;
    g_common_api.hv_printf = pUserData->hv_printf;
    g_common_api.hv_rand = pUserData->hv_rand;

	*phLaneDet = (HV_HANDLE)lane_detect;
	return ret;
}

HV_U32 SetInputData(HV_HANDLE hLaneDet, HV_CODECBUFFER * pInput)
{
	HV_U32 ret = 0;
	extern HV_COMMON_API g_common_api;
	clock_t start, end;
	start = g_common_api.hv_clock();
	lane_detect_c *lane_detect = (lane_detect_c*)hLaneDet;
	lane_detect->lane_detect_frame(pInput->Buffer);
	end = g_common_api.hv_clock();
	//g_common_api.hv_printf("%d: frame\n", end - start);
	//g_call_back_func(&hLaneDet);
	return ret;
}

HV_U32 GetOutputData(HV_HANDLE hLaneDet, HV_VIDEO_BUFFER * pOutBuffer, HV_VIDEO_OUTPUTINFO * pOutInfo)
{
	HV_U32 ret = 0;
	lane_detect_c *lane_detect = (lane_detect_c*)hLaneDet;
	pOutInfo->camera_pitch = lane_detect->camera_pitch_;
	pOutInfo->camera_yaw = lane_detect->camera_yaw_;
	for (HV_S32 k = 0; k < 4; k++)
	{
		pOutInfo->detected[k] = lane_detect->ref_lanes_[k]->detected;
		pOutInfo->lane_type[k] = lane_detect->ref_lanes_[k]->lane_type;
		pOutInfo->a0[k] = lane_detect->ref_lanes_[k]->a0;
		pOutInfo->a1[k] = lane_detect->ref_lanes_[k]->a1;
		pOutInfo->a2[k] = lane_detect->ref_lanes_[k]->a2;
		pOutInfo->top_end[k] = lane_detect->ref_lanes_[k]->top_end;
		pOutInfo->bot_end[k] = lane_detect->ref_lanes_[k]->bot_end;
		pOutInfo->tracking_num[k] = lane_detect->ref_lanes_[k]->tracking_num;
		pOutInfo->type_change_pos[k] = lane_detect->ref_lanes_[k]->type_change_pos;
	}
	HV_F32 out_param[7];
    lane_t cur_lane;

	int qqq = 3;

    for (HV_S32 k = 0; k < 4; k++)
    {
        cur_lane.a0 = pOutInfo->a0[k];
        cur_lane.a1 = pOutInfo->a1[k];
        cur_lane.a2 = pOutInfo->a2[k];
		cur_lane.top_end = pOutInfo->top_end[k];
		cur_lane.bot_end = pOutInfo->bot_end[k];
		cur_lane.type_change_pos = pOutInfo->type_change_pos[k];

        transfer_to_out_param(cur_lane, out_param);

        pOutInfo->wrda0[k] = out_param[0];
        pOutInfo->wrda1[k] = out_param[1];
        pOutInfo->wrda2[k] = out_param[2];
		pOutInfo->wr_top_end[k] = out_param[4];
		pOutInfo->wr_bot_end[k] = out_param[5];
		pOutInfo->wr_type_change_pos[k] = out_param[6];
    }

    pOutInfo->roi_top = g_roi_top;
    return ret;
}

HV_U32 Uninit(HV_HANDLE hLaneDet)
{
	HV_U32 ret = 0;
	lane_detect_c *lane_detect = (lane_detect_c*)hLaneDet;
	free(lane_detect);
	return ret;
}

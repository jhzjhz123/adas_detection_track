#ifndef HV_LANE_H
#define HV_LANE_H

#include "../hvapi/hvLanedet.h"
#include "common.h"

HV_U32 Init(HV_HANDLE * phLaneDet, HV_INIT_USERDATA * pUserData);

HV_U32 SetInputData(HV_HANDLE hLaneDet, HV_CODECBUFFER * pInput);

HV_U32 GetOutputData(HV_HANDLE hLaneDet, HV_VIDEO_BUFFER * pOutBuffer, HV_VIDEO_OUTPUTINFO * pOutInfo);

HV_U32 Uninit(HV_HANDLE hLaneDet);

#endif

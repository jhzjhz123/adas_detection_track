#ifndef __HVLANEDET_H__
#define __HVLANEDET_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hvType.h"
#include "hvLanedet.h"

#define HV_PID_COMMON_BASE      0x40000000/*!< The base param ID for common */
#define HV_INDEX_LANE_DET       0x100000
#define HV_PID_LANE_DET_BASE   HV_PID_COMMON_BASE | HV_INDEX_LANE_DET

typedef int(*HV_LANEDET_DONE_CB)(HV_HANDLE * phLaneDet);


typedef enum
{
    /* for get paramete */
    HV_PID_LANE_DET_GET_FIRSTFRAME = HV_PID_LANE_DET_BASE | 0x0001,  /*!<it will force the video to output the first frame immediately,the parameter is a Boolean(LONG integer)*/
    //HV_PID_LANE_DET_GET_WORLDCOORDINATES = HV_PID_LANE_DET_BASE | 0x0002,

    /* for set paramete */
    HV_PID_LANE_DET_SET_FIRSTFRAME = HV_PID_LANE_DET_BASE | 0x0100,  /*!<it will force the video to output the first frame immediately,the parameter is a Boolean(LONG integer)*/

    HV_PID_LANE_DET_MAX          = HV_MAX_ENUM_VALUE
}
HVLANEDETPARAMETERID;

/**
 * Defination of color format
 */
typedef enum
{
	HV_COLOR_YUV_PLANER444			= 0,		/*!< YUV planer mode:444  vertical sample is 1, horizontal is 1  */
	HV_COLOR_YUV_PLANER422_12		= 1,		/*!< YUV planer mode:422, vertical sample is 1, horizontal is 2  */
	HV_COLOR_YUV_PLANER422_21		= 2,		/*!< YUV planer mode:422  vertical sample is 2, horizontal is 1  */
	HV_COLOR_YUV_PLANER420			= 3,		/*!< YUV planer mode:420  vertical sample is 2, horizontal is 2  */
	HV_COLOR_YUV_PLANER411			= 4,		/*!< YUV planer mode:411  vertical sample is 1, horizontal is 4  */
	HV_COLOR_GRAY_PLANERGRAY		= 5,		/*!< gray planer mode, just Y value								 */
	HV_COLOR_YUYV422_PACKED			= 6,		/*!< YUV packed mode:422, vertical sample is 1, horizontal is 2, data: Y0 U0 Y1 V0  */
	HV_COLOR_YVYU422_PACKED			= 7,		/*!< YUV packed mode:422, vertical sample is 1, horizontal is 2, data: Y0 V0 Y1 U0  */
	HV_COLOR_UYVY422_PACKED			= 8,		/*!< YUV packed mode:422, vertical sample is 1, horizontal is 2, data: U0 Y0 V0 Y1  */
	HV_COLOR_VYUY422_PACKED			= 9,		/*!< YUV packed mode:422, vertical sample is 1, horizontal is 2, data: V0 Y0 U0 Y1  */
	HV_COLOR_YUV444_PACKED			= 10,		/*!< YUV packed mode:444, vertical sample is 1, horizontal is 1, data: Y U V	*/
	HV_COLOR_YUV_420_PACK			= 11, 		/*!< YUV planer mode:420  vertical sample is 2, horizontal is 2  , Y planar, UV Packed*/
	HV_COLOR_YUV_420_PACK_2			= 35, 		/*!< YUV planer mode:420  vertical sample is 2, horizontal is 2  , Y planar, VU Packed*/
	HV_COLOR_YVU_PLANER420			= 12,
	HV_COLOR_YVU_PLANER422_12		= 13,
	HV_COLOR_YUYV422_PACKED_2		= 14,		/*!< YUV packed mode:422, vertical sample is 1, horizontal is 2, data: Y1 U0 Y0 V0  */
	HV_COLOR_YVYU422_PACKED_2		= 15,		/*!< YUV packed mode:422, vertical sample is 1, horizontal is 2, data: Y1 V0 Y0 U0  */
	HV_COLOR_UYVY422_PACKED_2		= 16,		/*!< YUV packed mode:422, vertical sample is 1, horizontal is 2, data: U0 Y1 V0 Y0  */
	HV_COLOR_VYUY422_PACKED_2		= 17,		/*!< YUV packed mode:422, vertical sample is 1, horizontal is 2, data: V0 Y1 U0 Y0  */
	HV_COLOR_RGB565_PACKED			= 30,		/*!< RGB packed mode, data: B:5 G:6 R:5   						 */
	HV_COLOR_RGB555_PACKED			= 31,		/*!< RGB packed mode, data: B:5 G:5 R:5   						 */
	HV_COLOR_RGB888_PACKED			= 32,		/*!< RGB packed mode, data: B G R		 						 */
	HV_COLOR_RGB32_PACKED			= 33,		/*!< RGB packed mode, data: B G R A								 */
	HV_COLOR_RGB888_PLANER			= 34,		/*!< RGB planer mode											 */
	HV_COLOR_YUV_PLANER420_NV12		= 36,		/*!< YUV planer mode:420  vertical sample is 2, horizontal is 2  */
	HV_COLOR_TYPE_MAX				= HV_MAX_ENUM_VALUE
}  HV_IV_COLORTYPE;



/**
 * Video data buffer, usually used as iutput or output of video codec.
 */
typedef struct
{
	HV_PBYTE 			Buffer[3];			/*!< Buffer pointer */
	HV_S32				Stride[3];			/*!< Buffer stride */
	HV_IV_COLORTYPE		ColorType;			/*!< Color Type */
	HV_S64				Time;				/*!< The time of the buffer */
} HV_VIDEO_BUFFER;

/*
  lane parameter
*/
typedef struct
{
	HV_S32 detected[4];
	HV_S32 lane_type[4];
	HV_F32 a0[4];
	HV_F32 a1[4];
	HV_F32 a2[4];
    HV_F32 wrda0[4];
    HV_F32 wrda1[4];
    HV_F32 wrda2[4];
	HV_F32 wr_top_end[4];
	HV_F32 wr_bot_end[4];
	HV_F32 wr_type_change_pos[4];
	HV_F32 top_end[4];
	HV_F32 bot_end[4];
	HV_S32 tracking_num[4];
	HV_F32 type_change_pos[4];
	HV_F32 camera_yaw;
	HV_F32 camera_pitch;
	HV_S32 roi_top;
}HV_VIDEO_OUTPUTINFO;



/**
 * HV Lane Detect Function Set.
 */
typedef struct
{
    /**
     * Init the video decoder module and return decorder handle
     * \param hLaneDet [OUT] Return the lane detect handle
     * \param pUserData [IN] The init param. It is memory operator or alloced memory
     * \retval HV_ERR_NONE Succeeded.
     */
    HV_U32 (HV_API * Init) (HV_HANDLE * phLaneDet, HV_INIT_USERDATA * pUserData);

    /**
     * Set lane detect video data as input.
     * \param hDec [IN]] The hLaneDet Handle which was created by Init function.
     * \param pInput [IN] The input buffer param.
     * \retval HV_ERR_NONE Succeeded.
     */
    HV_U32 (HV_API * SetInputData) (HV_HANDLE hLaneDet, HV_CODECBUFFER * pInput);

    /**
     * Get the uncompressed yuv video data
     * \param hLaneDet [IN]] The hLaneDet Handle which was created by Init function.
     * \param pOutBuffer [OUT] The hLaneDet module filled the buffer pointer and stride.
     * \param pOutInfo [OUT] The dec module filled video format and used the input size.
     *               pOutInfo->InputUsed is total used the input size.
     * \retval  HV_ERR_NONE Succeeded.
     *               HV_ERR_INPUT_BUFFER_SMALL. The input was finished or the input data was not enought.
     */
    HV_U32 (HV_API * GetOutputData) (HV_HANDLE hLaneDet, HV_VIDEO_BUFFER * pOutBuffer, HV_VIDEO_OUTPUTINFO * pOutInfo);

    /**
     * Set the param for special target.
     * \param hLaneDet [IN]] The hLaneDet Handle which was created by Init function.
     * \param uParamID [IN] The param ID.
     * \param pData [IN] The param value depend on the ID>
     * \retval HV_ERR_NONE Succeeded.
     */
    HV_U32 (HV_API * SetParam) (HV_HANDLE hLaneDet, HV_S32 uParamID, HV_PTR pData);

    /**
     * Get the param for special target.
     * \param hLaneDet [IN]] The hLaneDet Handle which was created by Init function.
     * \param uParamID [IN] The param ID.
     * \param pData [OUT] The param value depend on the ID>
     * \retval HV_ERR_NONE Succeeded.
     */
    HV_U32 (HV_API * GetParam) (HV_HANDLE hLaneDet, HV_S32 uParamID, HV_PTR pData);

    /**
     * Uninit the lane detect.
     * \param hLaneDet [IN]] The hLaneDet Handle which was created by Init function.
     * \retval HV_ERR_NONE Succeeded.
     */
    HV_U32 (HV_API * Uninit) (HV_HANDLE hLaneDet);

    /**
     * Lane detect.register callback fun
     * \param callback [IN]] The hLaneDet callback function.
     * \retval HV_ERR_NONE Succeeded.
     */
    HV_S32 (HV_API * LaneDetRegisterDoneCB)(HV_LANEDET_DONE_CB callback);

    /**
     * Draw Lane fun
     * \param buffer [IN]] Out image buffer addr
     * \param p_out_info. Out image info
     * \retval HV_ERR_NONE Succeeded.
     */
    HV_S32 (HV_API * DrawLane)(HV_PTR buffer[], HV_PTR pData);

    /**
     * Draw Lane fun
     * \param points [OUT] Out lane points
     * \param pData. [IN]IN image info
     * \param lane_index. [IN] in lane index
     * \retval HV_ERR_NONE Succeeded.
     */
    HV_S32 (HV_API * DrawPoint)(HV_S32 *points, HV_PTR pData, HV_S32 lane_index);
} HV_LANE_DETAPI;

typedef struct
{
	signed long(HV_API * hv_clock) ();
	int(HV_API * hv_printf) (const char *format, ...);
	int(HV_API *hv_rand) ();
}HV_COMMON_API;


#ifdef __cplusplus
}
#endif

/**
* Get Lane detect API interface
* \param pDecHandle [IN/OUT] Return the Lane detect API handle.
* \param uFlag,reserved
* \retval HV_ERR_OK Succeeded.
*/
HV_S32 HV_API hvGetLaneDetAPI (HV_LANE_DETAPI * pDecHandle, HV_U32 uFlag);

#endif

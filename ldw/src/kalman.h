#ifndef KALMAN_H
#define KALMAN_H

#include "common.h"

#define MAX_TRACE_NUM 100

#define STATEMENT_NUM   6
#define MEASUREMENT_NUM 3
const HV_S32 MAX_CALMAN_NUM = STATEMENT_NUM>=MEASUREMENT_NUM ? STATEMENT_NUM : MEASUREMENT_NUM;

typedef struct//B=0ÎÞ¿ØÖÆkalmanÂË²¨
{
	HV_S32 MP;					/* number of measurement vector dimensions */
	HV_S32 DP;					/* number of state vector dimensions */
	float state_prio[STATEMENT_NUM];//X
	float state_post[STATEMENT_NUM];
	float transition_matrix[STATEMENT_NUM*STATEMENT_NUM];//A
	float process_noise_cov[STATEMENT_NUM*STATEMENT_NUM];//Q
	//float process_noise_cov_cur[STATEMENT_NUM*STATEMENT_NUM];
	float measurement_matrix[MEASUREMENT_NUM*STATEMENT_NUM];//H
	float measurement_noise_cov[MEASUREMENT_NUM*MEASUREMENT_NUM];//R
	//float measurement_noise_cov_cur[MEASUREMENT_NUM*MEASUREMENT_NUM];
	float error_cov_prio[STATEMENT_NUM*STATEMENT_NUM];//P
	float error_cov_post[STATEMENT_NUM*STATEMENT_NUM];
	float gain_matrix[STATEMENT_NUM*MEASUREMENT_NUM];//K

	float II[STATEMENT_NUM*STATEMENT_NUM];//I
}Kalman;

//#ifdef __cplusplus
//extern "C" {
//#endif

HV_HVID	init_Kalman(Kalman* m_kalman, float *pfX);
HV_HVID	matrix_set_identity( float* matrix, float val, HV_S32 row, HV_S32 col );
//float* KalmanPredict( Kalman* m_kalman );
//float* KalmanCorrect( Kalman* m_kalman, float* measurement );
HV_HVID KalmanPredict(Kalman* m_kalman);
HV_HVID KalmanCorrect(Kalman* m_kalman, float* measurement);
HV_HVID _matrix_inverse(float* Min, float* Mout, HV_S32 dim);

extern Kalman m_kalman[MAX_TRACE_NUM];
//#ifdef __cplusplus
//}
//#endif

#endif

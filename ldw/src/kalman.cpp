#include "kalman.h"

//#include <memory.h>
//#include "stdafx.h"
//#include "math.h"
//#include "stdlib.h"
//#include "_kalman.h"

//Kalman m_kalman[MAX_TRACE_NUM];

//a0 △a0 a1 △a1 a2 △a2
float A_base[STATEMENT_NUM*STATEMENT_NUM] = {
	1, 1, 0, 0, 0, 0,
	0, 1, 0, 0, 0, 0,
	0, 0, 1, 1, 0, 0,
	0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 1, 1,
	0, 0, 0, 0, 0, 1
};
float H_base[MEASUREMENT_NUM*STATEMENT_NUM] = {
	1, 0, 0, 0, 0, 0,
	0, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 1, 0
};

HV_HVID matrix_set_identity( float* matrix, float val, HV_S32 row, HV_S32 col )
{
	HV_S32 index = 0, i, min_val;
	memset( (HV_HVID*)matrix, 0, sizeof(float)*row*col );
	min_val = row < col ? row:col;
	for( i = 0; i < min_val; i++ )
	{
		index=i+i*col;
		matrix[index]=val;
	}
}

//B=A*X
HV_HVID _matrix_mult( float* A, HV_S32 num_row_A, HV_S32 num_col_A, float* X, HV_S32 num_row_X, HV_S32 num_col_X, float* B )
{
	HV_S32 m=num_row_A, n=num_col_A, p=num_col_X;
	HV_S32 i,j,k;
	float lsum;
	for(i=0;i<m;i++)
	{
		for(j=0;j<p;j++)
		{
			lsum=0;
			for(k=0;k<n;k++)
			{
				lsum+=A[i*n+k]*X[k*p+j];
			}
			B[i*p+j]=lsum;
		}
	}
}

HV_HVID _matrix_transpose( float* A, HV_S32 num_r, HV_S32 num_c, float* AT )
{
	HV_S32 iter_r,iter_c,indexA,indexAT;
	for(iter_r=0;iter_r<num_r;iter_r++)
	{
		for(iter_c=0;iter_c<num_c;iter_c++)
		{
			indexA= iter_c+iter_r*num_c;
			indexAT=iter_r+iter_c*num_r;
			AT[indexAT]=A[indexA];
		}
	}
}

HV_HVID _matrix_add( float* A, float* B, float* C, HV_S32 num_r, HV_S32 num_c )
{
	HV_S32 iter_r,iter_c,index;
	for(iter_r=0;iter_r<num_r;iter_r++)
	{
		for(iter_c=0;iter_c<num_c;iter_c++)
		{
			index = iter_c+iter_r*num_c;
			C[index]=A[index]+B[index];
		}
	}
}

HV_HVID _matrix_sub( float* A, float* B, float* C, HV_S32 num_r, HV_S32 num_c )
{
	HV_S32 iter_r,iter_c,index;
	for(iter_r=0;iter_r<num_r;iter_r++)
	{
		for(iter_c=0;iter_c<num_c;iter_c++)
		{
			index = iter_c+iter_r*num_c;
			C[index]=A[index]-B[index];
		}
	}
}

HV_HVID _matrix_inverse( float* Min, float* Mout, HV_S32 dim)
{
	HV_S32 i, j, k;
	float sum, x;

	/*  Copy the input matrix to output matrix */
	for(i=0; i<dim*dim; i++) { Mout[i]=Min[i]; }

	/* Add small value to diagonal if diagonal is zero */
	for(i=0; i<dim; i++) 
	{
		j=i*dim+i;
		if((Mout[j]<1e-12f)&&(Mout[j]>-1e-12f)){ Mout[j]=1e-12f; }
	}

	/* Matrix size must be larger than one */
	if (dim <= 1) return;

	for (i=1; i < dim; i++) 
	{
		Mout[i] /= Mout[0]; /* normalize row 0 Mout前面做过非0处理*/
	}

	for (i=1; i < dim; i++)  {
		for (j=i; j < dim; j++)  
		{ /* do a column of L */
			sum = 0.0;
			for (k = 0; k < i; k++) {
				sum += Mout[j*dim+k] * Mout[k*dim+i];
			}
			Mout[j*dim+i] -= sum;
		}
		if (i == dim-1) continue;
		for (j=i+1; j < dim; j++)  
		{  /* do a row of U */
			sum = 0.0;
			for (k = 0; k < i; k++) {
				sum += Mout[i*dim+k]*Mout[k*dim+j];
			}
			if (0 == Mout[i*dim + i])
			{
				memset(Mout, 0, dim*dim*sizeof(float));
				return;
			}
			Mout[i*dim+j] = (Mout[i*dim+j]-sum) / Mout[i*dim+i];
		}
	}
	for ( i = 0; i < dim; i++ )  /* invert L */ {
		for ( j = i; j < dim; j++ )  {
			x = 1.0;
			if ( i != j ) {
				x = 0.0;
				for ( k = i; k < j; k++ ) {
					x -= Mout[j*dim+k]*Mout[k*dim+i];
				}
			}
			if (0 == Mout[j*dim + j])
			{
				memset(Mout, 0, dim*dim*sizeof(float));
				return;
			}
			Mout[j*dim+i] = x / Mout[j*dim+j];
		}
	}
	for ( i = 0; i < dim; i++ ) /* invert U */ {
		for ( j = i; j < dim; j++ )  {
			if ( i == j ) continue;
			sum = 0.0;
			for ( k = i; k < j; k++ ) {
				sum += Mout[k*dim+j]*( (i==k) ? 1.0f : Mout[i*dim+k] );
			}
			Mout[i*dim+j] = -sum;
		}
	}
	for ( i = 0; i < dim; i++ ) /* final inversion */ {
		for ( j = 0; j < dim; j++ )  {
			sum = 0.0;
			for ( k = ((i>j)?i:j); k < dim; k++ ) {
				sum += ((j==k)?1.0f:Mout[j*dim+k])*Mout[k*dim+i];
			}
			Mout[j*dim+i] = sum;
		}
	}
}

HV_HVID set_II(float* matrix, float val, HV_S32 row)
{
	HV_S32 index = 0, i;
	//memset((HV_HVID*)matrix, 0, sizeof(float)*row*row);
	for (i = 0; i < row; i++)
	{
		index = i + i*row;
		matrix[index] = val;
	};
}

HV_HVID set_matrix(float* matrix, float val, HV_S32 row, HV_S32 col)
{
	HV_S32 index,i,j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
		{
			index = i + j*row;
			matrix[index] = val;
		}
	};
}

HV_HVID copy_matrix(float* matrix1, float *matrix2, HV_S32 row, HV_S32 col)
{
	HV_S32 index, i, j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
		{
			index = i + j*row;
			matrix1[index] = matrix2[index];
		}
	};
}

HV_HVID init_Kalman( Kalman* m_kalman, float *pfX)
{
	HV_S32 i;
	memset( m_kalman, 0, sizeof(Kalman) );
	m_kalman->MP = MEASUREMENT_NUM;
	m_kalman->DP = STATEMENT_NUM;

	for (i = 0; i < STATEMENT_NUM; i++)
	{
		m_kalman->state_prio[i] = pfX[i];
		m_kalman->state_post[i] = pfX[i];
	}
	//matrix_set_identity( m_kalman->II, 1, m_kalman->DP, m_kalman->MP );
	copy_matrix(m_kalman->transition_matrix, A_base, STATEMENT_NUM, STATEMENT_NUM);//A
	copy_matrix(m_kalman->measurement_matrix, H_base, MEASUREMENT_NUM, STATEMENT_NUM);//H
	//q smaller than r means more trust to predict result
	//can't just chose a2's q, similar curve may have different a0/a1/a2, just change a2 will change curve's position
	set_II(m_kalman->process_noise_cov, (float)1e-5, STATEMENT_NUM);//Q
	set_II(m_kalman->measurement_noise_cov, (float)1e-2, MEASUREMENT_NUM);//R
	set_II(m_kalman->error_cov_prio, 1, STATEMENT_NUM);//P
	set_II(m_kalman->error_cov_post, 1, STATEMENT_NUM);//P
	set_II(m_kalman->II, 1, m_kalman->DP);
}

float temp1[MAX_CALMAN_NUM*MAX_CALMAN_NUM];
float temp2[MAX_CALMAN_NUM*MAX_CALMAN_NUM];
float temp3[MAX_CALMAN_NUM*MAX_CALMAN_NUM];
float temp4[MAX_CALMAN_NUM*MAX_CALMAN_NUM];
float temp5[MAX_CALMAN_NUM*MAX_CALMAN_NUM];
float temp6[MAX_CALMAN_NUM*MAX_CALMAN_NUM];
float temp7[MAX_CALMAN_NUM*MAX_CALMAN_NUM];
float temp8[MAX_CALMAN_NUM*MAX_CALMAN_NUM];

//float* KalmanPredict( Kalman* m_kalman )
HV_HVID KalmanPredict(Kalman* m_kalman)
{
	/* x'(k) = A*x(k) */
	_matrix_mult( m_kalman->transition_matrix, m_kalman->DP, m_kalman->DP, 
		m_kalman->state_post, m_kalman->DP, 1, m_kalman->state_prio );
	/* tempAP = A*P(k) */
	_matrix_mult( m_kalman->transition_matrix, m_kalman->DP, m_kalman->DP, 
		m_kalman->error_cov_post, m_kalman->DP, m_kalman->DP, temp1 );
	//tempAt=At
	_matrix_transpose( m_kalman->transition_matrix, m_kalman->DP, m_kalman->DP, temp2 );
	//tempAPAt=tempAP*tempAt
	_matrix_mult( temp1, m_kalman->DP, m_kalman->DP, 
		temp2, m_kalman->DP, m_kalman->DP, 
		temp3 );
	//P'(k) = tempAPAt + Q
	_matrix_add( m_kalman->process_noise_cov, temp3, m_kalman->error_cov_prio, 
		m_kalman->DP, m_kalman->DP );

	//return m_kalman->state_prio;
}

//float* KalmanCorrect( Kalman* kalman, float* measurement )
HV_HVID KalmanCorrect(Kalman* kalman, float* measurement)
{
	// H'=H
	_matrix_transpose( kalman->measurement_matrix, kalman->MP, kalman->DP, temp4 );
	// PpH'=Pp*H'
	_matrix_mult( kalman->error_cov_prio, kalman->DP, kalman->DP, temp4, kalman->DP, kalman->MP, temp5 );
	// HPpH'=H*PpH'
	_matrix_mult( kalman->measurement_matrix, kalman->MP, kalman->DP, temp5, kalman->DP, kalman->MP, temp6 );
	// HPpH'+R
	_matrix_add( temp6, kalman->measurement_noise_cov, temp7, kalman->MP, kalman->MP );
	// 1/(HPpH'+R)
	_matrix_inverse( temp7, temp6, kalman->MP );
	// K = (PpH')/(HPpH'+R)
	_matrix_mult( temp5, kalman->DP, kalman->MP, temp6, kalman->MP, kalman->MP, kalman->gain_matrix );
	// H*Xp
	_matrix_mult( kalman->measurement_matrix, kalman->MP, kalman->DP, kalman->state_prio, kalman->DP, 1, temp1 );
	// Z-H*Xp
	//measurement is new detected state
	_matrix_sub( measurement, temp1, temp2, kalman->MP, 1 );
	// K*(Z-H*Xp)
	_matrix_mult( kalman->gain_matrix, kalman->DP,kalman->MP, temp2, kalman->MP, 1, temp1 );
	// Xup=Xp+K*(Z-H*Xp)
	_matrix_add( kalman->state_prio, temp1, kalman->state_post, kalman->DP, 1 );
	// K*H
	_matrix_mult( kalman->gain_matrix, kalman->DP, kalman->MP, kalman->measurement_matrix, kalman->MP, kalman->DP, temp1 );
	// (I-K*H)
	_matrix_sub( kalman->II, temp1, temp2, kalman->DP, kalman->DP );
	// Pup = (I-K*H)*Pp
	_matrix_mult( temp2, kalman->DP, kalman->DP, kalman->error_cov_prio, kalman->DP, kalman->DP, kalman->error_cov_post );
	//return kalman->state_post;
}
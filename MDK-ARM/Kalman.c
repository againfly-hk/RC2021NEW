#include "main.h"
#include "struct_typedef.h"
#include "kalman.h"

void Init_KalmanInfo(KalmanInfo* info,float Q,float R)
{
	info->A=1;
	info->H=1;
	info->P=10;
	info->Q=Q;
	info->R=R;
	info->filterValue=0;
}

float KalmanFilter(KalmanInfo* kalmanInfo,float lastMeasurement)
{
	float predictValue=kalmanInfo->A*kalmanInfo->filterValue;
	kalmanInfo->P=kalmanInfo->A*kalmanInfo->A*kalmanInfo->P+kalmanInfo->Q;
  float preValue=kalmanInfo->filterValue;
  kalmanInfo->kalmanGain=kalmanInfo->P*kalmanInfo->H/(kalmanInfo->P*kalmanInfo->H*kalmanInfo->H+kalmanInfo->R);
	kalmanInfo->filterValue=predictValue+(lastMeasurement-predictValue)*kalmanInfo->kalmanGain;
	kalmanInfo->P=(1-kalmanInfo->kalmanGain*kalmanInfo->H)*kalmanInfo->P;
	return kalmanInfo->filterValue;
}


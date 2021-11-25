#ifndef __KALMAN_H__
#define __KALMAN_H__

typedef struct{
	float filterValue;
	float kalmanGain;
	float A;
	float H;
	float Q;
	float R;
	float P;
}	KalmanInfo;

extern void Init_KalmanInfo(KalmanInfo* info,float Q,float R);
extern float KalmanFilter(KalmanInfo* kalmanInfo,float lastMeasurement);


#endif

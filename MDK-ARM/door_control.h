#ifndef __DOOR_CONTROL_H__
#define __DOOR_CONTROL_H__

#include "tim.h"

void left_door_on(void);
void left_door_off(void);//left_servo

void right_door_on(void);
void right_door_off(void);//rigtht_servo

void door_left(void);
void door_middle(void);
void door_right(void);
void door_reset(void);//pick_servo

void front_door_lift(void);
void front_door_down(void);//front_servo
	
void rockert_level(void);
void rocker_vertical(void);
void rocket_reserse_side(void);//rockrt_servo

void back_door_away(void);
void back_door_down(void);//back_servo

#endif 

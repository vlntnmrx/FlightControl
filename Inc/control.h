#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <math.h>
#include "main.h"
#include "quaternion.h"
#include "config.h"

typedef struct __pid_value {
	float p;
	float i;
	float d;
} PIDvalue;

uint16_t servo[5];
Quaternion q_soll;
Quaternion q_ist;
PIDvalue pid_r ;//= { 1, 0.1, 0 };
PIDvalue pid_y ;//= { 1, 0.1, 0 };
PIDvalue pid_p ;//= { 1, 0.1, 0 };

extern float looptime;

//Return actual amount of control
int16_t pitch(int16_t amount);
int16_t roll(int16_t amount);
int16_t yaw(int16_t amount);
int16_t throttle(int16_t amount);

//Set all servos center and motor off
void center_off();

void pid_init();
void pid_pitch();
void pid_roll();
void pid_yaw();

#endif // __CONTROL_H__
//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------

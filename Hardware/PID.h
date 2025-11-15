/*******************************************************************************
 * 文件名: PID.h
 * 描述: PID函数头文件
 ******************************************************************************/

#ifndef __PID_H
#define __PID_H

#include "my_math.h"

// PID控制器结构体
typedef struct {
    float kp;
	float kp2;
    float ki;
    float kd;
	float gkd;
    float error;
    float last_error;
	float last_gyro;
    float integral;
    float integral_max;
    float output;
} PID_Controller_t;

float pid_calculate(PID_Controller_t *controller, float setpoint, float measurement);
float ppdd_calculate(PID_Controller_t *controller, float setpoint, float measurement, float gyro_z);

#endif

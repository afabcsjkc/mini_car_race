/**
 * @file motor_control.h
 * @brief 电机控制头文件
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include <stdint.h>
#include "dodo_BMI270.h" //陿螺仪驱动
#include "multiplexer.h"//多路复用器驱动，用于读取光电管读敿
/* 外部接口函数 */
void cascade_pid_control(uint16_t mux_value, float gyro_z, 
                         float left_encoder, float right_encoder);
void tune_pid_parameters(float steer_kp, float steer_kd,
                         float gyro_kp, float gyro_ki, float gyro_kd,
                         float speed_kp, float speed_ki);
void reset_all_pid(void);

/* 辅助函数 */
float calculate_line_position(uint16_t mux_value);

#endif // MOTOR_CONTROL_H
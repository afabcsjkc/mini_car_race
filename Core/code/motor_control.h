/*******************************************************************************
 * 文件名: motor_control.h
 * 描述: 控制函数头文件
 ******************************************************************************/


#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include <stdint.h>
#include "dodo_BMI270.h" //陿螺仪驱动
#include "multiplexer.h"//多路复用器驱动，用于读取光电管读敿
#include "my_math.h"
#include "motor_control.h"
#include "AssistFunction.h"
#include "PID.h"

/* 外部接口函数 */

void motor_Init(void);
void parallel_pid_control(uint16_t mux_value, float gyro_z, float left_encoder, float right_encoder);

/* ==================== 数据结构 ==================== */

// 电机控制结构体
typedef struct {
    float target_speed;
    float current_speed;
    PID_Controller_t speed_pid;
    int16_t pwm_output;
	
	GPIO_TypeDef* Motor_GPIO;
	uint16_t Motor_PIN;
	uint8_t Motor_DIR;
	uint8_t CCR;
} Motor_t;

typedef enum{
	ZhiDao = 0,
	During_ZhiJiao = 1,
	During_WanDao = 2,
}Condition_t;

extern uint8_t flag;
extern Condition_t printdata[2];
#endif // MOTOR_CONTROL_H
						 
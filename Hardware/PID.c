/*******************************************************************************
 * 文件名: PID.h
 * 描述: PID函数逻辑实现
 ******************************************************************************/

#include "PID.h"

/* ==================== PID控制函数 ==================== */

/**
 * @brief PID控制器计算
 * @param controller PID控制器指针
 * @param setpoint 目标值
 * @param measurement 测量值
 * @return 控制输出
 */
float pid_calculate(PID_Controller_t *controller, float setpoint, float measurement) {
    // 计算误差、积分和微分
    float error = setpoint - measurement;
    controller->integral += error;
    float derivative = error - controller->last_error;
    
    // PID输出
    controller->output = controller->kp * error + 
                        controller->ki * controller->integral + 
                        controller->kd * derivative;
    
    // 更新上次误差
    controller->last_error = error;
    controller->error = error;
    
    return controller->output;
}

/**
 * @brief PPDD控制器计算（转向环）（使用模糊PID）
 * @param controller PID控制器指针
 * @param setpoint 目标值
 * @param measurement 测量值
 * @return 控制输出
 */
float ppdd_calculate(PID_Controller_t *controller, float setpoint, float measurement, float gyro_z) {
	
	//计算位置误差、微分和角速度微分
    float error = setpoint - measurement;
    float pos_derivative = error - controller->last_error;
    float gyro_derivative = gyro_z;
	
	//计算输出
	float output =  error * controller->kp + my_abs(error) * error * controller->kp2
				 + pos_derivative * controller->kd + gyro_derivative * controller->gkd;
	
	//更新pid_controller
	controller->last_error = error;
	controller->last_gyro = gyro_z;
    controller->error = error;
	
	return output;
}

/*******************************************************************************
 * 文件名: motor_control.c
 * 描述: 控制函数逻辑实现
 ******************************************************************************/

#include "motor_control.h"
/* ==================== 配置参数 ==================== */

// 电机端口定义
#define MotorGPIO_Left	GPIOB
#define MotorPIN_Left	GPIO_PIN_15
#define MotorGPIO_Right	GPIOA
#define MotorPIN_Right	GPIO_PIN_10

// 转向环PPDD参数（光电管位置 -> 目标速度差）
#define PPDD_KP1         38.0f      // 一次位置比例系数
#define PPDD_KP2         38.0f      // 二次位置比例系数
#define PPDD_KD1         -0.0f       // 位置微分系数
#define PPDD_GKD         -1.2f       // 角速度微分系数


// 速度环PI参数（目标速度 -> PWM输出）
#define L_SPEED_KP            8.0f       // 左电机速度比例系数
#define L_SPEED_KI            0.4f       // 左电机速度积分系数
#define L_SPEED_KD			  0.1		 // 左电机速度导数系数
#define R_SPEED_KP            8.0f       // 右电机速度比例系数
#define R_SPEED_KI            0.4f       // 右电机速度积分系数
#define R_SPEED_KD			  0.1		 // 右电机速度导数系数

#define SPEED_I_MAX         400.0f     // 速度积分限幅
#define GUOCHONG			4.0
// 目标速度设置
#define TARGET_SPEED        400.0;     // 基础目标速度（根据实际调整）

// PWM输出限制
#define PWM_MAX             3600       // PWM最大值
#define PWM_MIN             -3600      // PWM最小值（反转）

#define MUX_HISTORY_NUM		 4		 // 光电管历史数据的数量

Condition_t printdata[2] = {0}; 
float TargetSpeed_now = TARGET_SPEED;
uint8_t tune_flag = 0;	//编码器模式flag，0不输出，1左，2右
float last_position_error = 0;
Condition_t condition_now = ZhiDao;
uint8_t mux_array[12] = {0};
uint8_t left_group_last = 0,right_group_last = 0;
uint8_t active_count = 0;

uint8_t stop_flag = 0;
/* ==================== 全局变量 ==================== */

// PPDD控制器
PID_Controller_t steering_ppdd = {
    .kp = PPDD_KP1,
    .kp2 = PPDD_KP2,
    .kd = PPDD_KD1,
	.gkd = PPDD_GKD,
};

PID_Controller_t left_pid = {
	.kp = L_SPEED_KP,
	.ki = L_SPEED_KI,
	.kd = L_SPEED_KD,
	.integral_max = SPEED_I_MAX
};

PID_Controller_t right_pid = {
	.kp = R_SPEED_KP,
	.ki = R_SPEED_KI,
	.kd = R_SPEED_KD,
	.integral_max = SPEED_I_MAX
};
;
// 左右电机
Motor_t motor_left;
Motor_t motor_right;


/**
 * @brief motor初始化函数
 * @param 无
 * @return 无
 */
void motor_Init(void){
	motor_left.speed_pid = left_pid;
	motor_left.Motor_GPIO = MotorGPIO_Left;
	motor_left.Motor_PIN = MotorPIN_Left;
	motor_left.Motor_DIR = GPIO_PIN_SET;
	motor_left.CCR = 1;
	
	motor_right.speed_pid = right_pid;
	motor_right.Motor_GPIO = MotorGPIO_Right;
	motor_right.Motor_PIN = MotorPIN_Right;
	motor_right.Motor_DIR = GPIO_PIN_SET;
	motor_right.CCR = 2;
}

/**
 * @brief 设置PWM波
 * @param motor：想要设置的电机
 * @return 无
 */
void SetPWM(Motor_t motor){
	if(motor.pwm_output == 0 || stop_flag)return;
	uint16_t pwm_output = my_abs_int(motor.pwm_output);
	if(motor.CCR == 1){
		TIM1->CCR1 = pwm_output;
	}
	else if(motor.CCR == 2){
		TIM1->CCR2 = pwm_output;
	}
	if(motor.pwm_output > 0){
		HAL_GPIO_WritePin(motor.Motor_GPIO, motor.Motor_PIN, motor.Motor_DIR);
	}
	else{
		HAL_GPIO_WritePin(motor.Motor_GPIO, motor.Motor_PIN, !motor.Motor_DIR);
	}

}

void StopPWM(void){
	motor_left.pwm_output = 1;
	motor_right.pwm_output = 1;
	SetPWM(motor_left);
	SetPWM(motor_right);
	stop_flag = 1;
}

uint8_t ZhiJiaoCheck(uint16_t mux_value, uint8_t active_count){
	if(active_count >= 10) return 0;
	uint8_t result = 0, left_group = 0, right_group = 0;
	
	
	
	for(uint8_t i =0;i < 2; i++){
		if(mux_array[i]){left_group ++;}
		if(mux_array[11 -i]){right_group++;}
	}

	if(left_group == 2 && left_group_last == 0){
		result = 1; 
	}
	else if(right_group == 2 && right_group_last == 0)
		result = 2; 
	left_group_last = left_group;
	right_group_last = right_group;
	return result;
}

void Update_Condition(uint16_t mux_value, uint8_t active_count){
	switch(condition_now){
		case ZhiDao:{
			if(ZhiJiaoCheck(mux_value, active_count)){
				condition_now = During_ZhiJiao;
			}
		}
		case During_ZhiJiao:{
			if(ZhiJiaoCheck(mux_value, active_count) == 0){
				condition_now = ZhiDao;
			}
		}
	}
}

/**
 * @brief 计算光电管中心位置偏差
 * @param mux_value 光电管原始数据
 * @return 位置偏差 (-18 到 18，0表示居中)
 */
float calculate_line_position(uint16_t mux_value) {
    float weighted_sum = 0.0f;
	active_count = 0;
    // ---- 根据光电管读数，更新一系列数据 ----
    for (uint8_t i = 0; i < 12; i++) {
		mux_array[i]= MUX_GET_CHANNEL(mux_value, i);
		if(MUX_GET_CHANNEL(mux_value, i)){
			weighted_sum += i - 5.5;
			active_count++;
		}
    }

    // ---- 更新光电管oldestData的索引 ----
	
    if (active_count == 0) {
		if(!last_position_error) return 0;
        return last_position_error ;//> 0 ? GUOCHONG : -GUOCHONG; // 保持上次偏差
    }
    // 返回位置偏差
    return weighted_sum / active_count;	
}


/* ==================== 调试函数 ==================== */
/**
 * @brief 计算并打印调试信息的函数
 * @param left_encoder  左编码器值
 * @param right_encoder 右编码器值
 * @return 无
 */
void tune_information(float left_encoder, float right_encoder)
{
	switch(tune_flag)
	{
		case 1:
			printf("%f,%f \r\n", motor_left.current_speed, motor_left.target_speed );
			break;
			
		case 2:
			printf("%f,%f \r\n", motor_right.current_speed, motor_right.target_speed);
			break;
		
		default:
			break;
	}
}


/* ==================== 主控制函数 ==================== */

/**
 * @brief 并联PID控制主函数
 * @param mux_value 光电管数据
 * @param gyro_z Z轴角速度（度/秒）
 * @param left_encoder 左轮编码器速度
 * @param right_encoder 右轮编码器速度
 */
void parallel_pid_control(uint16_t mux_value, float gyro_z, float left_encoder, float right_encoder) {
	//计算赛道位置偏差
    float line_position = calculate_line_position(mux_value);
	last_position_error = line_position;
	
    // PD控制计算位置偏差（positional_deviation）
    float positional_deviation = ppdd_calculate(&steering_ppdd, 0.0f, line_position, gyro_z);

	// 根据位置偏差确定左右电机的差速度（这里展示用y = x映射）
	float differential_speed = positional_deviation;
	
	Update_Condition(mux_value, active_count);
	printdata[0] = condition_now;
	switch(condition_now){
		case ZhiDao: break;
		case During_ZhiJiao: differential_speed *= 2;
	}
	
	// 根据位置偏差计算左右电机的目标速度
    motor_left.target_speed = TargetSpeed_now - differential_speed;
    motor_right.target_speed = TargetSpeed_now + differential_speed;
    // 更新当前速度（从编码器读取）
    motor_left.current_speed = left_encoder;
    motor_right.current_speed = right_encoder;

    // 左右电机PID控制
    motor_left.pwm_output = pid_calculate(&motor_left.speed_pid, 
                                         motor_left.target_speed, 
                                         motor_left.current_speed); 
    motor_right.pwm_output = pid_calculate(&motor_right.speed_pid, 
                                          motor_right.target_speed, 
                                          motor_right.current_speed);
    motor_left.pwm_output = constrain(motor_left.pwm_output,PWM_MIN,PWM_MAX);
    motor_right.pwm_output = constrain(motor_right.pwm_output,PWM_MIN,PWM_MAX);
	// 设置PWM波
	SetPWM(motor_left);
	SetPWM(motor_right);
	//调试函数
	tune_information(left_encoder, right_encoder);
}

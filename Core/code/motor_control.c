/**
 * @file motor_control.c
 * @brief 小车寻迹PWM电机控制 - 串级PID控制
 * @description 转向环PD -> 角速度环PID -> 速度环PI
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "dodo_BMI270.h" //陿螺仪驱动
#include "multiplexer.h"//多路复用器驱动，用于读取光电管读敿
#include "motor_control.h"
/* ==================== 配置参数 ==================== */


// 转向环PPDD参数（光电管位置 -> 目标速度差）
#define PPDD_KP1         35.0f      // 一次位置比例系数
#define PPDD_KP2         35.0f      // 二次位置比例系数
#define PPDD_KD1         -0.0f       // 位置微分系数
#define PPDD_GKD         -1.0f       // 角速度微分系数


// 速度环PI参数（目标速度 -> PWM输出）
#define L_SPEED_KP            8.0f       // 左电机速度比例系数
#define L_SPEED_KI            0.4f       // 左电机速度积分系数
#define L_SPEED_KD			  0.1		 // 左电机速度导数系数
#define R_SPEED_KP            8.0f       // 右电机速度比例系数
#define R_SPEED_KI            0.4f       // 右电机速度积分系数
#define R_SPEED_KD			  0.1		 // 右电机速度导数系数
//右电机速度导数系数
#define SPEED_I_MAX         400.0f     // 速度积分限幅

// 目标速度设置
float TARGET_SPEED    =    300.0;     // 基础目标速度（根据实际调整）

// PWM输出限制
#define PWM_MAX             3600       // PWM最大值
#define PWM_MIN             -3600      // PWM最小值（反转）
#define PWM_DEADZONE        50         // PWM死区

// PWM前馈值
#define PWM_QianKui 0

// 过弯减速系数
#define PWM_GuoWanEnble		0			//是否开启过弯减速，为0关闭，为1开启	
#define PWM_GuoWanXiShu		0.5			//过弯减速系数
#define PWM_GuoWanYuZhi		10			//光电管过弯判断阈值



uint16_t n=0;		//目前用于计算速度平均值的数据个数
float l_about=0;	//左电机速度平均值
float r_about=0;	//右电机速度平均值
float l_variance = 0;
float r_variance = 0;

int run_times = 0; // 刹车时间
int times=0;		//目前执行 “PID控制主函数” 的次数
int tune_flag = 1;	//编码器模式flag，0不输出，1左，2右
uint8_t condition_flag = 0;	//小车运行状态机，为0：小车在直线行驶；为1：小车在转弯；为2：小车停止

uint8_t last_muxstates[12];
uint8_t muxstates[12];

float last_position_error;
/* ==================== 数据结构 ==================== */

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

// 电机控制结构体
typedef struct {
    float target_speed;
    float current_speed;
    PID_Controller_t speed_pid;
    int16_t pwm_output;
} Motor_t;


/* ==================== 全局变量 ==================== */


// 左右电机
Motor_t motor_left = {
    .speed_pid = {
        .kp = L_SPEED_KP,
        .ki = L_SPEED_KI,
        .kd = 0.0f,
        .integral_max = SPEED_I_MAX
    }
};

Motor_t motor_right = {
    .speed_pid = {
        .kp = R_SPEED_KP,
        .ki = R_SPEED_KI,
        .kd = R_SPEED_KD,
        .integral_max = SPEED_I_MAX
    }
};

// PPDD控制器
PID_Controller_t steering_ppdd = {
    .kp = PPDD_KP1,
    .kp2 = PPDD_KP2,
    .kd = PPDD_KD1,
	.gkd = PPDD_GKD,
};

/* ==================== 辅助函数 ==================== */

/**
 * @brief 限幅函数
 */
float constrain(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/**
* @brief 小车状态机判断函数
 * @param mux_value 光电管数据
 * @param gyro_z Z轴角速度（度/秒）
 * @param left_encoder 左轮编码器速度
 * @param right_encoder 右轮编码器速度
* @return 小车状态机condition_flag
 */
uint8_t condition_check(uint16_t mux_value, float gyro_z, float left_encoder, float right_encoder)
{
	static uint8_t times = 0;
	condition_flag = 0;
	for(uint8_t i = 0; i < 12; i++){
		muxstates[i] = MUX_GET_CHANNEL(mux_value, i);
	}
//	for(uint8_t i = 0; i < 12; i++){
//		printf("%d,",muxstates[i]);
//	}
//	printf("\n");
	
	uint8_t diffstate1 = 0, diffstate2 = 0;
	//判断左转直角弯
	for(uint8_t i = 0; i < 3; i++){
		if(last_muxstates[i] != muxstates[i]){
			diffstate1++;
			if(diffstate1 >= 3){
				condition_flag = 1;
			}
		}
	}
	
	//判断右转直角弯
	for(uint8_t i = 9; i < 12; i++){
		if(last_muxstates[i] != muxstates[i]){
			diffstate2++;
			if(diffstate2 >= 3){
				condition_flag = 2;
			}
		}
	}
	
	times++;
	if(times > 3){
		for(uint8_t i = 0; i < 12; i++){
			last_muxstates[i] = muxstates[i];
		}
		times = 0;
//		printf("diff1 = %d, diff2 = %d\n", diffstate1, diffstate2);
//		printf("condition = %d", condition_flag);
		
	}
	return condition_flag;
	
}

/**
 * @brief 计算光电管中心位置偏差
 * @param mux_value 光电管原始数据
 * @return 位置偏差 (-18 到 18，0表示居中)
 */
float calculate_line_position(uint16_t mux_value) {
	static uint16_t times;
	static uint16_t time_2;
    float weighted_sum = 0.0f;
    uint8_t active_count = 0;
	
    // 加权求和计算赛道中心位置
//	if(condition_flag == 1){
//		return -40;
//	}
//	else if(condition_flag == 2){
//		return 40;
//	}
    for (int i = 0; i <= 11; i++) {
//			printf("%d",MUX_GET_CHANNEL(mux_value, i));
        if (MUX_GET_CHANNEL(mux_value, i) == 1) {
			// 传感器位置权重：-5.5, -4.5, ..., 4.5, 5.5
//			if(i == 0)
//			{
//				weighted_sum -= 11;
//				time_2 = 50;
//			}
//			else if (i == 11)
//			{
//				weighted_sum += 11;
//				time_2 = 50;
//			}
			if (1)
			{
				weighted_sum += i - 5.5f;
				active_count++;
				times = 0;
			}
        }
    }
//        printf("active_count = %d\n", active_count);

    // 如果没有检测到赛道，返回上次的偏差值
//	if(time_2 > 0)
//	{
//		time_2--;
//		steering_ppdd.kp = 10,
//		steering_ppdd.kp2 = 10,
//		steering_ppdd.kd = 0,
//		steering_ppdd.gkd = 0.2,
//		TARGET_SPEED = 200;
//	}
//	else
//	{
//		steering_ppdd.kp = PPDD_KP1,
//		steering_ppdd.kp2 = PPDD_KP2,
//		steering_ppdd.kd = PPDD_KD1,
//		steering_ppdd.gkd = PPDD_GKD,
//		TARGET_SPEED = 300;
//	}
    if (active_count == 0) {
		times++;
		if(times > 300){
			TARGET_SPEED = 0;
			return 0;
		}
        return last_position_error > 0 ? 3 : -3; // 保持上次偏差
    }
    // 返回位置偏差
    return weighted_sum / active_count;	//修改了 weighted_sum / active 以增加灵敏度
}


/* ==================== PID控制函数 ==================== */

/**
 * @brief PID控制器计算（角速度环）
 * @param controller PID控制器指针
 * @param setpoint 目标值
 * @param measurement 测量值
 * @return 控制输出
 */
float pid_calculate(PID_Controller_t *controller, float setpoint, float measurement) {
    // 计算误差
    float error = setpoint - measurement;
    
    // 积分累加
    controller->integral += error;
//    controller->integral = constrain(controller->integral, 
//                                     -controller->integral_max, 
//                                     controller->integral_max);
    
    // 计算微分
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
	
	//计算位置误差
    float error = setpoint - measurement;
	
	//模糊PID
//	if(my_abs(error) <= 2){
//		return 0;
//	}
	
	//计算位置微分和角速度微分
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


/* ==================== 调试函数 ==================== */
/**
 * @brief 计算并打印调试信息的函数
 * @param left_encoder  左编码器值
 * @param right_encoder 右编码器值
 * @return 无
 */
void tune_information(float left_encoder, float right_encoder)
{
	if(times<510&&((left_encoder!=0)||(right_encoder!=0))){										 
		times++;
	}

	switch(tune_flag)
	{
		case 0:	
//			printf("%f,%f,%f\r\n", motor_left.current_speed ,
//					motor_right.current_speed, motor_left.target_speed );
			break;
		
		case 1:
			if(times>500 && (left_encoder>20||left_encoder<-20)){
				//更新左电机速度平均值
				l_about = my_about(l_about, left_encoder, n);
				//更新左电机速度方差
//				l_variance = my_variance(l_about, left_encoder, l_about, n);
				//更新n值
				n++;
			}
			printf("%f,%f,%f,%f\r\n", motor_left.current_speed ,l_about, l_variance, motor_left.target_speed );
			break;
			
		case 2:
			if(times>500 && (right_encoder>20||right_encoder<-20)){
				//更新右电机速度平均值
				r_about = my_about(r_about, right_encoder, n);	
				//更新左电机速度方差
//				r_variance = my_variance(r_about, right_encoder, r_about, n);
				//更新n值
				n++;
			}
			printf("%f,%f,%f,%f\r\n", motor_right.current_speed ,r_about, r_variance, motor_right.target_speed);
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
	uint32_t run_times = 0;
	if(run_times < 6000)
		run_times++;
	if(run_times > 5000)
	{
		
		return ;
	}
	condition_check(mux_value, gyro_z, left_encoder, right_encoder);
	//计算赛道位置偏差
    float line_position = calculate_line_position(mux_value);

    // PD控制计算位置偏差（positional_deviation）
    float positional_deviation = ppdd_calculate(&steering_ppdd, 0.0f, line_position, gyro_z);

	// 根据位置偏差确定左右电机的差速度（这里展示用y = x映射）
	float differential_speed = positional_deviation;
	// 根据位置偏差计算左右电机的目标速度
    motor_left.target_speed = TARGET_SPEED - differential_speed;
    motor_right.target_speed = TARGET_SPEED + differential_speed;

    // 更新当前速度（从编码器读取）
    motor_left.current_speed = left_encoder;
    motor_right.current_speed = right_encoder;

    // 左电机PI控制
    motor_left.pwm_output = pid_calculate(&motor_left.speed_pid, 
                                         motor_left.target_speed, 
                                         motor_left.current_speed);
    motor_left.pwm_output = constrain(motor_left.pwm_output,PWM_MIN,PWM_MAX);
    // 右电机PI控制
    motor_right.pwm_output = pid_calculate(&motor_right.speed_pid, 
                                          motor_right.target_speed, 
                                          motor_right.current_speed);
    motor_right.pwm_output = constrain(motor_right.pwm_output,PWM_MIN,PWM_MAX);
	
	//调试函数
	tune_information(left_encoder, right_encoder);
	if (motor_left.pwm_output >= 0) {
			TIM1->CCR1 = motor_left.pwm_output + PWM_QianKui, HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	} 
	else {
			TIM1->CCR1 = -motor_left.pwm_output + PWM_QianKui, HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	} 
	if (motor_right.pwm_output >= 0) {
			TIM1->CCR2 = motor_right.pwm_output + PWM_QianKui, HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	} 
	else {
			TIM1->CCR2 = -motor_right.pwm_output + PWM_QianKui, HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	} 
	last_position_error = line_position;
}

/**
 * @brief 主PID控制主函数，根据宏定义PID_Mode决定执行3PID串联还是并联PID
 * @param mux_value 光电管数据
 * @param gyro_z Z轴角速度（度/秒）
 * @param left_encoder 左轮编码器速度
 * @param right_encoder 右轮编码器速度
 */
void master_pid_control(uint16_t mux_value, float gyro_z, float left_encoder, float right_encoder) {
	  
	parallel_pid_control(mux_value, gyro_z, left_encoder, right_encoder);
}

/* ==================== PID参数调整函数 ==================== */

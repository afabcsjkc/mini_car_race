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

// 转向环PD参数（光电管位置 -> 目标角速度）
#define STEERING_KP         15.0f      // 位置比例系数
#define STEERING_KD         3.0f       // 位置微分系数

// 转向环PPDD参数（光电管位置 -> 目标速度差）
#define PPDD_KP1         5.0f      // 一次位置比例系数
#define PPDD_KP2         5.0f      // 二次位置比例系数
#define PPDD_KD1         3.0f       // 位置微分系数
#define PPDD_GKD         3.0f       // 角速度微分系数

// 角速度环PID参数（目标角速度 -> 左右轮差速）
#define GYRO_KP             10.0f       // 角速度比例系数
#define GYRO_KI             1.1f       // 角速度积分系数
#define GYRO_KD             0.0f       // 角速度微分系数
#define GYRO_I_MAX          100.0f     // 角速度积分限幅

// 速度环PI参数（目标速度 -> PWM输出）
#define L_SPEED_KP            10.5f       // 左电机速度比例系数
#define L_SPEED_KI            0.2f       // 左电机速度积分系数
#define R_SPEED_KP            10.0f       // 右电机速度比例系数
#define R_SPEED_KI            0.8f       // 右电机速度积分系数
#define R_SPEED_KD			  13		 // 右电机速度导数系数
//右电机速度导数系数
#define SPEED_I_MAX         400.0f     // 速度积分限幅

// 目标速度设置
#define TARGET_SPEED        100.0f     // 基础目标速度（根据实际调整）

// PWM输出限制
#define PWM_MAX             1000       // PWM最大值
#define PWM_MIN             -1000      // PWM最小值（反转）
#define PWM_DEADZONE        50         // PWM死区

// PWM前馈值
#define PWM_QianKui 0

// 过弯减速系数
#define PWM_GuoWanXiShu		0.5			//过弯减速系数
#define PWM_GuoWanYuZhi		10			//光电管过弯判断阈值

//PID模式选择
#define PID_Mode 1 //PID_Mode == 0 使用三PID串联； PID_Mode == 1 使用二PID * 2并联


uint16_t n=0;		//目前用于计算速度平均值的数据个数
float l_about=0;	//左电机速度平均值
float r_about=0;	//右电机速度平均值
int times=0;		//目前执行 “PID控制主函数” 的次数
int tune_flag = -1;	//编码器模式flag，0不输出，1左，2右
uint8_t condition_flag = 0;	//小车运行状态机，为0：小车在直线行驶；为1：小车在转弯；为2：小车停止

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

// 转向环PD控制器
PID_Controller_t steering_pd = {
    .kp = STEERING_KP,
    .kd = STEERING_KD,
    .ki = 0.0f,
    .integral_max = 0.0f
};

// 角速度环PID控制器
PID_Controller_t gyro_pid = {
    .kp = GYRO_KP,
    .ki = GYRO_KI,
    .kd = GYRO_KD,
    .integral_max = GYRO_I_MAX
};

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
    .integral_max = GYRO_I_MAX
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
 * @brief 计算光电管中心位置偏差
 * @param mux_value 光电管原始数据
 * @return 位置偏差 (-18 到 18，0表示居中)
 */
float calculate_line_position(uint16_t mux_value) {
    float weighted_sum = 0.0f;
    uint8_t active_count = 0;
    
    // 加权求和计算赛道中心位置
    for (int i = 0; i <= 11; i++) {
//			printf("%d",MUX_GET_CHANNEL(mux_value, i));
        if (MUX_GET_CHANNEL(mux_value, i) == 1) {
					
            // 传感器位置权重：-5.5, -4.5, ..., 4.5, 5.5
            weighted_sum += (i - 5.5f);
            active_count++;
        }
    }
    
    // 如果没有检测到赛道，返回上次的偏差值
    if (active_count == 0) {
        return steering_pd.error; // 保持上次偏差
    }
    
    // 返回位置偏差
    return weighted_sum;	//修改了 weighted_sum / active 以增加灵敏度
}


/* ==================== PID控制函数 ==================== */

/**
 * @brief PD控制器计算（转向环）
 * @param controller PD控制器指针
 * @param setpoint 目标值（0 = 赛道中心）
 * @param measurement 测量值（当前位置偏差）
 * @return 控制输出（目标角速度）
 */
float pd_calculate(PID_Controller_t *controller, float setpoint, float measurement) {
    // 计算误差
    float error = setpoint - measurement;
    
    // 计算微分
    float derivative = error - controller->last_error;
    
    // PD输出
    controller->output = controller->kp * error + controller->kd * derivative;
    
    // 更新上次误差
    controller->last_error = error;
    controller->error = error;
    
    return controller->output;
}

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
 * @brief PI控制器计算（速度环）
 * @param controller PI控制器指针
 * @param setpoint 目标速度
 * @param measurement 当前速度
 * @return PWM输出
 */
int16_t pi_calculate(PID_Controller_t *controller, float setpoint, float measurement) {
    // 计算误差
    float error = setpoint - measurement;
    
    // 积分累加
	    controller->integral += error;
//    controller->integral = constrain(controller->integral, 
//                                     -controller->integral_max, 
//                                     controller->integral_max);
    
    // PI输出
    controller->output = controller->kp * error + controller->ki * controller->integral;
    
    // 限幅并转换为PWM值
    int16_t pwm = (int16_t)constrain(controller->output, PWM_MIN, PWM_MAX);
    //printf("%d\n",pwm);
    // 死区处理
    if (pwm > 0 && pwm < PWM_DEADZONE) {
				
        pwm = 0;
    } else if (pwm < 0 && pwm > -PWM_DEADZONE) {
        pwm = 0;
    }
    
    controller->last_error = error;
    controller->error = error;
    //printf("integral = %f\n", controller->integral);
    //printf("output = %f\n", controller->integral);
    return pwm;
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
	if(my_abs(error) <= 2){
		return 0;
	}
	
	//计算位置微分和角速度微分
    float pos_derivative = error - controller->last_error;
    float gyro_derivative = gyro_z - controller->last_gyro;
	
	//计算输出
	float output = error * controller->kp + my_abs(error) * error * controller->gkd
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
			printf("%f,%f,%f\r\n", motor_left.current_speed ,
					motor_right.current_speed, motor_left.target_speed );
			break;
		
		case 1:
			if(times>500 && (left_encoder>20||left_encoder<-20)){
				//更新左电机速度平均值
				l_about = my_about(l_about, left_encoder, &n);
			}
			printf("%f,%f,%f\r\n", motor_left.current_speed ,l_about, motor_left.target_speed );
			break;
			
		case 2:
			if(times>500 && (right_encoder>20||right_encoder<-20)){
				//更新右电机速度平均值
				r_about = my_about(r_about, right_encoder, &n);		
			}
			printf("%f,%f,%f\r\n", motor_right.current_speed ,motor_right.target_speed ,r_about);
			break;
		default:
			break;
	}
}


/* ==================== 主控制函数 ==================== */

/**
 * @brief 串级PID控制主函数
 * @param mux_value 光电管数据
 * @param gyro_z Z轴角速度（度/秒）
 * @param left_encoder 左轮编码器速度
 * @param right_encoder 右轮编码器速度
 */
void cascade_pid_control(uint16_t mux_value, float gyro_z, float left_encoder, float right_encoder) {
    /* ========== 第一级：转向环PD控制 ========== */
    // 计算赛道位置偏差
    float line_position = calculate_line_position(mux_value);
    
    // PD控制计算目标角速度（位置偏差 -> 目标角速度）
    float target_angular_velocity = pd_calculate(&steering_pd, 0.0f, line_position);
    
    /* ========== 第二级：角速度环PID控制 ========== */
    // PID控制计算左右轮差速（目标角速度 -> 差速值）
    float differential_speed = pid_calculate(&gyro_pid, target_angular_velocity, gyro_z);
    
    
    /* ========== 第三级：速度环PI控制 ========== */
    // 计算左右电机目标速度
	if()
    motor_left.target_speed = TARGET_SPEED * PWM_GuoWanXiShu - differential_speed;
    motor_right.target_speed = TARGET_SPEED * PWM_GuoWanXiShu + differential_speed;
    //printf("%f,%f\r\n", motor_left.target_speed, motor_right.target_speed);
    // 更新当前速度（从编码器读取）
    motor_left.current_speed = left_encoder;
    motor_right.current_speed = right_encoder;
    
    // 左电机PI控制
    motor_left.pwm_output = pi_calculate(&motor_left.speed_pid, 
                                         motor_left.target_speed, 
                                         motor_left.current_speed);
    constrain(motor_left.pwm_output,PWM_MIN,PWM_MAX);
    // 右电机PI控制
    motor_right.pwm_output = pid_calculate(&motor_right.speed_pid, 
                                          motor_right.target_speed, 
                                          motor_right.current_speed);
    constrain(motor_right.pwm_output,PWM_MIN,PWM_MAX);

	//调试函数
	tune_information(left_encoder, right_encoder);
		//printf("%d\n",motor_left.pwm_output);
		//printf("%d\n",motor_right.pwm_output);
    /* ========== PWM输出 ========== */
//
	
		if (motor_right.pwm_output >= 0) {
				TIM1->CCR2 = motor_right.pwm_output + PWM_QianKui, HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		} 
		else {
				TIM1->CCR2 = -motor_right.pwm_output + PWM_QianKui, HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		} 
    /* ========== 调试输出（可选） ========== */
    #ifdef DEBUG_PID
    printf("LinePos:%.2f, TargetGyro:%.2f, Gyro:%.2f, Diff:%.2f, L:%d, R:%d\n",
           line_position, target_angular_velocity, gyro_z, differential_speed,
           motor_left.pwm_output, motor_right.pwm_output);
    #endif
}


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
	
    // PD控制计算位置偏差（positional_deviation）
    float positional_deviation = ppdd_calculate(&steering_pd, 0.0f, line_position, gyro_z);
	
	// 根据位置偏差确定左右电机的差速度（这里展示用y = x映射）
	float differential_speed = positional_deviation;
	// 根据位置偏差计算左右电机的目标速度
    motor_left.target_speed = TARGET_SPEED - differential_speed;
    motor_right.target_speed = TARGET_SPEED + differential_speed;

    // 更新当前速度（从编码器读取）
    motor_left.current_speed = left_encoder;
    motor_right.current_speed = right_encoder;

    // 左电机PI控制
    motor_left.pwm_output = pi_calculate(&motor_left.speed_pid, 
                                         motor_left.target_speed, 
                                         motor_left.current_speed);
    constrain(motor_left.pwm_output,PWM_MIN,PWM_MAX);
    // 右电机PI控制
    motor_right.pwm_output = pid_calculate(&motor_right.speed_pid, 
                                          motor_right.target_speed, 
                                          motor_right.current_speed);
    constrain(motor_right.pwm_output,PWM_MIN,PWM_MAX);
	
	//调试函数
	tune_information(left_encoder, right_encoder);
}

/**
 * @brief 主PID控制主函数，根据宏定义PID_Mode决定执行3PID串联还是并联PID
 * @param mux_value 光电管数据
 * @param gyro_z Z轴角速度（度/秒）
 * @param left_encoder 左轮编码器速度
 * @param right_encoder 右轮编码器速度
 */
void master_pid_control(uint16_t mux_value, float gyro_z, float left_encoder, float right_encoder) {
	  if(PID_Mode == 0){
		cascade_pid_control(mux_value, gyro_z, left_encoder, right_encoder);
	  }
	  else if(PID_Mode == 1){
		parallel_pid_control(mux_value, gyro_z, left_encoder, right_encoder);
	  }	 
}

/* ==================== PID参数调整函数 ==================== */

/**
 * @brief 运行时调整PID参数（用于调试）
 */
void tune_pid_parameters(float steer_kp, float steer_kd,
                         float gyro_kp, float gyro_ki, float gyro_kd,
                         float speed_kp, float speed_ki) {
    // 转向环PD
    steering_pd.kp = steer_kp;
    steering_pd.kd = steer_kd;
    
    // 角速度环PID
    gyro_pid.kp = gyro_kp;
    gyro_pid.ki = gyro_ki;
    gyro_pid.kd = gyro_kd;
    
    // 速度环PI
    motor_left.speed_pid.kp = speed_kp;
    motor_left.speed_pid.ki = speed_ki;
    motor_right.speed_pid.kp = speed_kp;
    motor_right.speed_pid.ki = speed_ki;
}

/**
 * @brief 重置所有PID控制器
 */
void reset_all_pid(void) {
    steering_pd.integral = 0;
    steering_pd.last_error = 0;
    
    gyro_pid.integral = 0;
    gyro_pid.last_error = 0;
    
    motor_left.speed_pid.integral = 0;
    motor_left.speed_pid.last_error = 0;
    
    motor_right.speed_pid.integral = 0;
    motor_right.speed_pid.last_error = 0;
}
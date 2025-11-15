/*******************************************************************************
 * 文件名: AssistFunction.h
 * 描述: 辅助函数逻辑实现
 ******************************************************************************/

#include "AssistFunction.h"

clock_t Assist_clock;

/**
 * @brief 限幅函数
 * @param value 待限幅数据
 * @param min_val 目标值
 * @param measurement 测量值
 * @return 控制输出
 */
float constrain(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

void clock_Init(void){
	clock(CLOCK_START);
	clock(CLOCK_STOP);
	Assist_clock.Offset = Assist_clock.temp2 - Assist_clock.temp1 + 11;	//这个11是根据Ceshi函数的情况手动调整的
	printf("Offset = %d\n", Assist_clock.Offset);
}

// @brief 计算持续运行时间函数
// CLOCKTYPE = CLOCK_START->开启计时
// CLOCKTYPE = CLOCK_START->停止计时，打印时间
void clock(clockstate_t CLOCKTYPE){	
	static clockstate_t clock_condition = 0;

	if(clock_condition ^ CLOCKTYPE)
		clock_condition = CLOCKTYPE;
	switch(clock_condition){
		case CLOCK_STOP:
			Assist_clock.temp2 = DWT->CYCCNT;
			printf("RunTime = %.2fus\n", (Assist_clock.temp2 - Assist_clock.temp1 - Assist_clock.Offset) * 1000000 / SystemCoreClock);
			break;
		case CLOCK_START:
			Assist_clock.temp1 = DWT->CYCCNT;
			break;
		default:
			break;
	}
}

// 这是提高速度的关键：将数字倒序转换后，进行一次快速反转。
void reverse_str(char str[], int length) {
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

void my_printf(UART_HandleTypeDef *huart, float data){
// 缓冲区大小，20位足够容纳 32 位长整数（10位） + 符号 + 换行符
    char buffer[20];
    int i = 0;
    long l_data;
    
    // 1. 快速提取整数部分
    l_data = (long)data; 
    
    // 2. 检查符号并处理 0
    int is_negative = 0;
    if (l_data == 0) {
        buffer[i++] = '0';
    } else if (l_data < 0) {
        is_negative = 1;
        // 对负数取绝对值进行转换
        l_data = -l_data; 
    }

    // 3. 逐位转换 (低位在前)
    while (l_data != 0) {
        // '0' + (l_data % 10) 得到对应数字的 ASCII 码
        buffer[i++] = (char)(l_data % 10 + '0');
        l_data /= 10;
    }

    // 4. 添加负号
    if (is_negative) {
        buffer[i++] = '-';
    }

    // 5. 反转字符串（得到正确顺序）
    reverse_str(buffer, i);

    // 6. 添加换行符（可选，但推荐用于终端显示）
    buffer[i++] = '\r';
    buffer[i++] = '\n';
    
    // 7. 发送
    HAL_UART_Transmit(huart, (uint8_t *)buffer, i, 0xFFFF);}

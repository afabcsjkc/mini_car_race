/*******************************************************************************
 * 文件名: AssistFunction.h
 * 描述: 辅助函数头文件
 ******************************************************************************/

#ifndef __ASSISTFUNCTION_H
#define __ASSISTFUNCTION_H

#include "main.h"
#include "stdio.h"

typedef enum{
	CLOCK_STOP = 0,
	CLOCK_START = 1,
}clockstate_t;

typedef struct{
	float temp1, temp2;
	uint32_t Offset;
}clock_t;

float constrain(float value, float min_val, float max_val);
void clock_Init(void);
void clock(clockstate_t CLOCKTYPE);
void my_printf(UART_HandleTypeDef *huart, float data);

#endif

#ifndef MY_MATH_H
#define MY_MATH_H

#include <stdint.h>


/* 外部接口函数 */
float my_about(float last_about, float data, uint16_t total);
float my_abs(float data);
float my_pow(float data, int n);
float my_variance(float last_variance, float data, float about, uint16_t total);

#endif

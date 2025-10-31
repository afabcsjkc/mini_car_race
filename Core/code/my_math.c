#include "my_math.h"


/**
 * @brief 根据旧的平均值、新的数据和当前参与计算的数据数目，计算新平均值
 * @param last_about 旧的平均值
 * @param data 		 新的数据
 * @param total 	 参与计算的数据数目
 * @return now_about 新的平均值
 */
float my_about(float last_about, float data, uint16_t total)
{
	float now_about;
	
	//计算新的平均值
	(total)++;
	now_about = last_about * (total-1) / total + data / total ; 

	return now_about;
}


/**
 * @brief 计算绝对值函数
 * @param data	将进行计算的数额
 * @return result 数据的绝对值
 */
float my_abs(float data)
{
	float result = data;
	
	//计算新的平均值
	if(data < 0)
	{
		result = -data;
	}

	return result;
}

float my_pow(float data, int n)
{
	float temp = data;
	for( ; n > 1 ; n /= 2)
	{
		data = data * data;
	}
	if(n == 1)
	{
		data *= temp;
	}
	return data;
}

float my_variance(float last_variance, float data, float about, uint16_t total )
{
	float now_variance;
	(total)++;
	now_variance = (last_variance * (total-1) + my_pow(data - about, 2)) / total; 
	return now_variance;
}

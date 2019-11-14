#ifndef __BSP_CAN
#define __BSP_CAN

#include "stdint.h"

#define CAN_3508Moto1_ID           0x201
#define CAN_3508Moto2_ID           0x202 
#define CAN_3508Moto3_ID           0x203
#define CAN_3508Moto4_ID           0x204


typedef struct{
	
	uint16_t angle;   									//编码器不经处理的原始值
	int16_t inner_rpm;								//上一次的编码器原始值
	int16_t current;                       //经过处理后连续的编码器值
//	int32_t diff;													//两次编码器之间的差值
//	int32_t temp_count;                   //计数用
//	uint8_t buf_count;								//滤波更新buf用
//	int32_t ecd_bias;											//初始编码器值	
//	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
//	int32_t rate_buf[RATE_BUF_SIZE];	//buf，for filter
//	int32_t round_cnt;										//圈数
//	int32_t filter_rate;											//速度
//	float ecd_angle;											//角度
//	union
//	{
//	 uint8_t data[8];
//	 uint16_t ActVal[4];
//	}fdb;
}Motor;

void can_filter_init(void);

void set_moto_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif

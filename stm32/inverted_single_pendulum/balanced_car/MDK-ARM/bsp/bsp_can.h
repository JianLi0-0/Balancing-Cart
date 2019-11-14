#ifndef __BSP_CAN
#define __BSP_CAN

#include "stdint.h"

#define CAN_3508Moto1_ID           0x201
#define CAN_3508Moto2_ID           0x202 
#define CAN_3508Moto3_ID           0x203
#define CAN_3508Moto4_ID           0x204


typedef struct{
	
	uint16_t angle;   									//���������������ԭʼֵ
	int16_t inner_rpm;								//��һ�εı�����ԭʼֵ
	int16_t current;                       //��������������ı�����ֵ
//	int32_t diff;													//���α�����֮��Ĳ�ֵ
//	int32_t temp_count;                   //������
//	uint8_t buf_count;								//�˲�����buf��
//	int32_t ecd_bias;											//��ʼ������ֵ	
//	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
//	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
//	int32_t round_cnt;										//Ȧ��
//	int32_t filter_rate;											//�ٶ�
//	float ecd_angle;											//�Ƕ�
//	union
//	{
//	 uint8_t data[8];
//	 uint16_t ActVal[4];
//	}fdb;
}Motor;

void can_filter_init(void);

void set_moto_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif

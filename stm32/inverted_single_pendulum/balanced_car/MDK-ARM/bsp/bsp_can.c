#include "can.h"
#include "bsp_can.h"



CAN_FilterConfTypeDef  				sFilterConfig;
static CanTxMsgTypeDef        TxMessage;
static CanRxMsgTypeDef        RxMessage;
Motor motor[4];

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	
//	printf("%d\r\n",_hcan->pRxMsg->StdId);

	switch(_hcan->pRxMsg->StdId)
	{
		case CAN_3508Moto1_ID:
		case CAN_3508Moto2_ID:
		case CAN_3508Moto3_ID:
		case CAN_3508Moto4_ID:
			{
				static uint8_t i;
				i = _hcan->pRxMsg->StdId - CAN_3508Moto1_ID;
				motor[i].angle = (uint16_t)(_hcan->pRxMsg->Data[0]<<8|_hcan->pRxMsg->Data[1]);
				motor[i].inner_rpm = (int16_t)(_hcan->pRxMsg->Data[2]<<8|_hcan->pRxMsg->Data[3]);
				motor[i].current = (int16_t)(_hcan->pRxMsg->Data[4]<<8|_hcan->pRxMsg->Data[5]);
				
			}
			break;
	}
		
		
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan, CAN_IT_FMP0);
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
}

void can_filter_init(void)
{
	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;
	
	/*##-2- Configure the CAN1 Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	
	__HAL_CAN_ENABLE_IT(&hcan, CAN_IT_FMP0);
}

void set_moto_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	hcan.pTxMsg->StdId = 0x200;
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
	hcan.pTxMsg->DLC = 0x08;
	hcan.pTxMsg->Data[0] = iq1 >> 8;
	hcan.pTxMsg->Data[1] = iq1;
	hcan.pTxMsg->Data[2] = iq2 >> 8;
	hcan.pTxMsg->Data[3] = iq2;
	hcan.pTxMsg->Data[4] = iq3 >> 8;
	hcan.pTxMsg->Data[5] = iq3;
	hcan.pTxMsg->Data[6] = iq4 >> 8;
	hcan.pTxMsg->Data[7] = iq4;

	if(HAL_CAN_Transmit(&hcan, 10) != HAL_OK)
	{
			printf("can_failed");
	}

}	
/*
	if(HAL_CAN_Transmit(&hcan, 10) != HAL_OK)
	{
			Error_Handler();
	}
*/


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define ABSOLUTE(x) ((x)>0? (x):(-(x)))
#define PI                   (3.14159265f)
uint8_t RxBuffer_uart3[1];
	
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int16_t iq1;
int16_t iq2;
int16_t iq3;
int16_t iq4;
uint32_t Timer2_Count = 0;
uint8_t Count_1ms,Count_2ms,Count_4ms,Count_5ms,Count_10ms,Count_20ms,Count_50ms,Count_200ms,Count_100ms;
int Count_1000ms;
uint8_t Bsp_Int_Ok = 0;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int16_t left_motor_current,right_motor_current;
extern struct SAngle 	stcAngle;
extern struct SGyro 	stcGyro;
extern struct SAcc 		stcAcc;
float accx,accy,accz;
float pitch, roll, yaw, last_pitch;
float pitch_vel, roll_vel, yaw_vel;
float cal_angle;
float offsect_angle=0.0;

float x_dot,theta_dot,x,theta;
float Radius = 0.095/2;

int last_pos;

extern Motor motor[4];

uint32_t cnt_1s=0;

void Task_1000HZ(void)
{
	static uint8_t a=1;
	
	pitch = (float)stcAngle.Angle[0]/32768*180 - offsect_angle;
	pitch_vel = (float)stcGyro.w[0]/32768*2000;
	int delta_theta;
	delta_theta = motor[0].angle - last_pos;
	
	if(ABSOLUTE(delta_theta)>4095)
	{
		if(delta_theta>0)	delta_theta -= 8191;
		else delta_theta += 8191;
	}
	
	last_pos = motor[0].angle;
	
	if(a==1) {delta_theta=0;a=0;}
	
	x_dot =- delta_theta/8191.0*360.0/180.0*PI*Radius/0.001; //与规定方向相反
	theta_dot = pitch_vel/180.0f*PI;
	x += x_dot*0.001f;
	theta = pitch/180.0f*PI;

}



float step = 0.03f/20.0f*16384.0f;


float gg1[1501];
float gg2[1501];
int gg_count;
int p_flag=1;
void Task_500HZ(void)
{

	
//	accx = (float)stcAcc.a[0]/32768*16;
//	accy = (float)stcAcc.a[1]/32768*16;
//	accz = (float)stcAcc.a[2]/32768*16;
//	
//	cal_angle=atan2(accy,accz)*180/PI;
	
/*
	//identify wheel's moment of inertia
	//delta_theta 8191:360°  left_motor_current:  current(A)/20.0f*16384.0f;
	if(Timer2_Count<=5002)
	{
		int delta_theta;
		Set_Motor_Speed(CAN1, left_motor_current , 0, 0, 0);
		delta_theta = MOTOR[0].angle - last_pos;
		if((MOTOR[0].angle - last_pos)<0)
		delta_theta += 8191;
		
		last_pos = MOTOR[0].angle;
		
		gg1[gg_count] = delta_theta;
		gg2[gg_count] = left_motor_current;
		gg_count++;
		
	}
	else
	{
		if(p_flag)
		{
			p_flag=0;
			gg_count=1;
			while(gg_count<2501)
			{
				printf("%d\t%d\r\n",gg1[gg_count],gg2[gg_count]);
				gg_count++;
			}
		}
		
	}
	//identify wheel's moment of inertia
	//void Task_10HZ(void)
	//{
	//	if(Timer2_Count<=3000) left_motor_current += 0.0018;
	//	else left_motor_current = 0;
	//}
	//	)
*/
	

}

void Task_250HZ(void)
{
	
}


//esc max current: 20A/2 16384/2=8192

//角速度 +-2000°/s,角度 +-180°
int16_t command;
float CURRENT;


float Kd[4]={-2.358748 , -8.102613 , -6.608290 , -1.296257 };//good parameter for angle -1.911442 , -6.852738 , -5.402396 , -1.063010



float CURRENT_LIMIT = 10.0F;
void Task_200HZ(void)
{
/*
	//identify first rod's moment of inertia
	//角速度 +-2000°/s,角度 +-180°
	if(Timer2_Count<=5005)
	{
		gg1[gg_count] = pitch_vel;
		gg2[gg_count] = pitch;
		gg_count++;
	}
	else
	{
		if(p_flag)
		{
			p_flag=0;
			gg_count=1;
			while(gg_count<1001)
			{
				printf("%f\t%f\r\n",gg1[gg_count],gg2[gg_count]);
				gg_count++;
			}
		}
		
	}
	//identify first rod's moment of inertia

*/

	

	CURRENT = -(Kd[0]*x + Kd[1]*theta + Kd[2]*x_dot + Kd[3]*theta_dot);

	if(CURRENT>CURRENT_LIMIT) CURRENT=CURRENT_LIMIT;
	if(CURRENT<-CURRENT_LIMIT) CURRENT=-CURRENT_LIMIT;
	command = CURRENT/20.0f*16384.0f;
	
	if(ABSOLUTE(theta)>70.0f/180.0f*PI || ABSOLUTE(motor[0].inner_rpm)>2500 || ABSOLUTE(motor[1].inner_rpm)>2500)
	{
		printf("stop\r\n");
		while(1)
		{
			set_moto_current( 0,  0,  0,  0);
		}
		
	}
	else
		set_moto_current(-command , command, 0, 0);

	printf("theta：%f\t theta_det:%f\t%f\r\n",theta,theta_dot,CURRENT);

}

void Task_100HZ(void)
{
//////	send_state_variable(x,theta,x_dot,theta_dot);
}

void Task_50HZ(void)
{

}

void Task_20HZ(void)
{

}

void Task_10HZ(void)
{
	/*
	//identify wheel's moment of inertia
	if(Timer2_Count<3002) left_motor_current += step;
	else left_motor_current = 0;
	*/
}

void Task_5HZ(void)
{

}

void Task_1HZ(void)
{

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	can_filter_init();
	HAL_UART_Receive_IT(&huart3,RxBuffer_uart3,1);// Enable the USART3 Interrupt
	Bsp_Int_Ok = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(Count_1ms>=1)
		{	
			Count_1ms = 0;
			Task_1000HZ();
		}
		if(Count_2ms>=2)
		{
			Count_2ms = 0;
			Task_500HZ();
		}
		if(Count_4ms>=4)
		{
			Count_4ms = 0;
			Task_250HZ();
		}
		if(Count_5ms>=5)
		{
			Count_5ms = 0;
			Task_200HZ();
		}
		if(Count_10ms>=10)
		{
			Count_10ms = 0;
			Task_100HZ();
		}
		if(Count_20ms>=20)
		{
			Count_20ms = 0;
			Task_50HZ();
		}
		if(Count_50ms>=50)
		{
			Count_50ms = 0;
			Task_20HZ();
		}
		if(Count_100ms>=100)
		{
			Count_100ms = 0;
			Task_10HZ();
		}
		if(Count_200ms>=200)
		{
			Count_200ms = 0;
			Task_5HZ();
		}
		if(Count_1000ms>=1000)
		{
			Count_1000ms = 0;
			Task_1HZ();
			cnt_1s++;
		}

//		set_moto_current( iq1,  iq2,  iq3,  iq4);
//		HAL_Delay(1000);

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;
//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//清空缓存区
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == huart3.Instance)
	{
		CopeSerial2Data(RxBuffer_uart3[0]);//处理数据
	}
	HAL_UART_Receive_IT(&huart3,RxBuffer_uart3,1);// Enable the USART3 Interrupt again
}








void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim2.Instance)
    {
			if( Bsp_Int_Ok == 0 )	return;//硬件未初始化完成，则返回
			Timer2_Count++;
			Count_1ms++;
			Count_2ms++;
			Count_4ms++;
			Count_5ms++;
			Count_10ms++;
			Count_20ms++;
			Count_50ms++;
			Count_100ms++;
			Count_200ms++;
			Count_1000ms++;
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

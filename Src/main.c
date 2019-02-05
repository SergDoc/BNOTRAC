/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO055.h"
#include <math.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float PI = 3.14159265358979323846f;
float LINACC [3];
float LINACCOLD [3];
float Euler[3];
float Quat[4];
float QuatW;
float QuatX;
float QuatY;
float QuatZ;
float test;

float sinr_cosp =0.0f;
float cosr_cosp =0.0f;
float sinp =0.0f;
float siny_cosp =0.0f;
float cosy_cosp =0.0f;

float prevVelocity[3];
float newVelocity[3];
float prevPosition[3];
float prevAccel[3];
float Position[3];
float nevalgo[3];
float Abs[3];
float AbsOld[3];
float EulerF[3];
float EulerF2[3];

float vzero[3];
float szero[3];

uint64_t lastRotationUpdate = 0.0f;
volatile uint64_t now = 0.0f;
float elapsedTime = 0.0f;


int optimer=0;
uint16_t Code;
uint8_t bufArd[30];
uint8_t cnt=0;
uint8_t status;
uint8_t test_result;
uint8_t error;

uint8_t sys=0;
uint8_t gyro=0;
uint8_t accel=0;
uint8_t mag=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t GetMicros(void)
{
	register uint64_t systick, ticks;
	do {
	ticks = (uint64_t)HAL_GetTick();

	 systick = (uint64_t)SysTick->VAL;
	}
	 while (ticks != (uint64_t)HAL_GetTick());
	return (ticks * 1000ull) + (72ul*1000ull - systick )/ 72ul;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
    LED0_OFF;
    LED1_OFF;
    LED2_OFF;
    LED0_ON;
    HAL_Delay (100);
    LED1_ON;
    HAL_Delay (100);
    LED0_OFF;
    LED2_ON;
    HAL_Delay (100);
    LED1_OFF;
    HAL_Delay (100);
    LED2_OFF;
    LED1_ON;
    HAL_Delay (100);
    LED1_OFF;
    HAL_Delay (200);
    /*
     print("Start test I2C\r\n");
    HAL_Delay (200);


      for(int i = 0; i < 128; i++)
          {
           uint8_t address = i << 1;

           if(HAL_I2C_IsDeviceReady(&hi2c2, (uint8_t*) address, 1, 1000) == HAL_OK)
           {
            char str[4];
            sprintf(str, "0x%X", address >> 1);
            print("Address:");
            HAL_Delay (500);
            print(str);
            HAL_Delay (500);
            print("\r\n");
            HAL_Delay (500);

         }
          }
          print("Test end I2C\r\n");

*/
    begin( );
    //Calibrating the IMU
    while(sys != 3)
    {
    	char str[4];
    getCalibration(&sys, &gyro, &accel, &mag);
    LED0_TOGGLE;
    LED1_TOGGLE;
    LED2_TOGGLE;
    print("CALIBRATION: Sys=");
    sprintf(str, "0x%X",sys);
    HAL_Delay (500);
    print(str);
    HAL_Delay (500);
    print(" Gyro=");
    sprintf(str, "0x%X",gyro);
    HAL_Delay (500);
    print(str);
    HAL_Delay (500);
    print(" Accel=");
    sprintf(str, "0x%X",accel);
    HAL_Delay (500);
    print(str);
    HAL_Delay (500);
    print(" Mag=");
    sprintf(str, "0x%X",mag);
    HAL_Delay (500);
    print(str);
    HAL_Delay (500);

    print("\r\n");
    }
    LED0_OFF;
    LED1_OFF;
    LED2_OFF;
    now = GetMicros();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   while (1)
  {


	    ProcessHeadTracking();
	  		cnt++;
	  		if (cnt>=100)
	  		{
	  			LED1_TOGGLE;
	  			cnt=0;
	  		}

	  		/*	 processcalcAbs();
	  // getLinear(VECTOR_LINEARACCEL, LINACC);
	  uint8_t i = 0;
	  	char str0[100];
	  	sprintf(str0,"%08d;%08d;%08d\r\n", (int16_t)nevalgo[0], (int16_t)nevalgo[1], (int16_t)nevalgo[2] );
	  		   	 CDC_Transmit_FS((unsigned char*)str0, strlen(str0),100);

	  		  cnt++;
	  		  		if (cnt>=100)
	  		  		{
	  		  			LED0_TOGGLE;
	  		  			cnt=0;
	  		  		}
*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
    Error_Handler();
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
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void floatToByteArray(float f, uint8_t *barr, uint8_t start) {

    unsigned int asInt = *((int*)&f);

    int i;
    for (i = 0; i < 4; i++) {
        barr[start+i] = (asInt >> 8 * i) & 0xFF;
    }
}

void processQuat()
{
	getQuat(VECTOR_QUAT, Quat);

	QuatW=Quat[0];
	QuatX=Quat[1];
	QuatY=Quat[2];
	QuatZ=Quat[3];
}

void processEuler()
{

	getEuler(VECTOR_EULER, Euler);

	/* sinr_cosp = 2.0f*(QuatY*QuatW-QuatX*QuatZ);
	 cosr_cosp = 1.0f-2.0f*(QuatY*QuatY+QuatZ*QuatZ);
	 sinp = 2.0f*(QuatX*QuatY + QuatZ*QuatW);
	 siny_cosp = 2.0f*(QuatX*QuatW-QuatY*QuatZ);
	 cosy_cosp = 1.f-2.0f*(QuatX*QuatX+QuatZ*QuatZ);

 Euler[0]=atan2(sinr_cosp, cosr_cosp);
 if(fabs(sinp)>=1){
 Euler[1]=copysign(PI/2,sinp);
 }
 else {
 		  Euler[1]=asin(sinp);
 	  }
 Euler[2]=atan2(siny_cosp, cosy_cosp);
 */
	  EulerF[0]=Euler[0]-180.0f;
	  EulerF[1]=Euler[1];
	  EulerF[2]=Euler[2];
}


void print(char string[])
{
	CDC_Transmit_FS( (unsigned char*) string, strlen(string), 100);
}

void processArduinoReport()
  {
	 // optimer++;
	  //if (optimer>100) {optimer=0;}

	  EulerF2[0]=EulerF2[0]*0.9f+EulerF[0]*0.1f;
	  EulerF2[1]=EulerF2[1]*0.9f+EulerF[1]*0.1f;
	  EulerF2[2]=EulerF2[2]*0.9f+EulerF[2]*0.1f;

	  AbsOld[0]=AbsOld[0]*0.9f+Abs[0]*0.1f;
	  AbsOld[1]=AbsOld[1]*0.9f+Abs[1]*0.1f;
	  AbsOld[2]=AbsOld[2]*0.9f+Abs[2]*0.1f;
	 // if (optimer%1==0)
	 // {
		  Code++;
		  if (Code>999)
			  Code=0;

			bufArd[0]=(uint8_t)(0xAA);
			bufArd[1]=(uint8_t)(0xAA);
			bufArd[2]=(uint8_t)Code;
			bufArd[3]=(uint8_t)(Code>>8);

			bufArd[4]=(uint8_t)0x00;
			bufArd[5]=(uint8_t)0x00;
			bufArd[6]=(uint8_t)0x00;
			bufArd[7]=(uint8_t)0x00;

			bufArd[8]=(uint8_t)0x00;
			bufArd[9]=(uint8_t)0x00;
			bufArd[10]=(uint8_t)0x00;
			bufArd[11]=(uint8_t)0x00;

			bufArd[12]=(uint8_t)0x00;
			bufArd[13]=(uint8_t)0x00;
			bufArd[14]=(uint8_t)0x00;
			bufArd[15]=(uint8_t)0x00;

			bufArd[16]=(uint8_t) 0x00;
			bufArd[17]=(uint8_t) 0x00;
			bufArd[18]=(uint8_t) 0x00;
			bufArd[19]=(uint8_t) 0x00;

			bufArd[20]=(uint8_t) 0x00;
			bufArd[21]=(uint8_t) 0x00;
			bufArd[22]=(uint8_t) 0x00;
			bufArd[23]=(uint8_t) 0x00;

			bufArd[24]=(uint8_t) 0x00;
			bufArd[25]=(uint8_t) 0x00;
			bufArd[26]=(uint8_t) 0x00;
			bufArd[27]=(uint8_t) 0x00;

			bufArd[28]=(uint8_t)(0x55);
			bufArd[29]=(uint8_t)(0x55);

			floatToByteArray(EulerF2[0],bufArd,4);
			floatToByteArray(EulerF2[1],bufArd,8);
			floatToByteArray(EulerF2[2],bufArd,12);
			floatToByteArray(AbsOld[0],bufArd,16);
			floatToByteArray(AbsOld[1],bufArd,20);
			floatToByteArray(AbsOld[2],bufArd,24);
			CDC_Transmit_FS(bufArd,30);
	 // }

  }

void ProcessHeadTracking()
{
	//processQuat();
	processEuler();
	processcalcAbs();
	processArduinoReport();
}





/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==GPIO_PIN_4)
	{

		ProcessHeadTracking();
		cnt++;
		if (cnt>=250)
		{
			LED0_TOGGLE;
			cnt=0;
		}
	}
	//else if (GPIO_Pin==GPIO_PIN_13)
	//{
	//	LED1_TOGGLE;
	//	getCalibration();
	//}
}*/

/* uses last loaded QuaData and LinAccData. */
void processcalcAbs()
{
	lastRotationUpdate = now;
	now = GetMicros();
	elapsedTime = (now - lastRotationUpdate) / 1000000.0f; // set integration time by time elapsed since last filter update

    getLinear(VECTOR_LINEARACCEL, LINACC);

    LINACCOLD[0] = LINACCOLD[0]*0.995f+LINACC[0]*0.005f;
    LINACCOLD[1] = LINACCOLD[1]*0.995f+LINACC[1]*0.005f;
    LINACCOLD[2] = LINACCOLD[2]*0.995f+LINACC[2]*0.005f;

    newVelocity[0] = prevVelocity[0]+ LINACCOLD[0]* elapsedTime;
    newVelocity[1] = prevVelocity[1]+ LINACCOLD[1]* elapsedTime;
    newVelocity[2] = prevVelocity[2]+ LINACCOLD[2]* elapsedTime;

    nevalgo[0]+= LINACCOLD[0];
    nevalgo[1]+= LINACCOLD[1];
    nevalgo[2]+= LINACCOLD[2];


    Position[0] += (prevVelocity[0]* elapsedTime + (LINACCOLD[0]* elapsedTime* elapsedTime)/2);
    Position[1] += (prevVelocity[1]* elapsedTime + (LINACCOLD[1]* elapsedTime* elapsedTime)/2);
    Position[2] += (prevVelocity[2]* elapsedTime + (LINACCOLD[2]* elapsedTime* elapsedTime)/2);

    Abs[0] = Abs[0]*0.98f+Position[0]*0.02f;
    Abs[1] = Abs[1]*0.98f+Position[1]*0.02f;
    Abs[2] = Abs[2]*0.98f+Position[2]*0.02f;

   // prevAccel[0] = LINACCOLD[0];
   // prevAccel[1] = LINACCOLD[1];
   // prevAccel[2] = LINACCOLD[2];
    prevVelocity[0]= newVelocity[0];
   // prevPosition[0] = Position[0];
    prevVelocity[1]= newVelocity[1];
   // prevPosition[1] = Position[1];
    prevVelocity[2]= newVelocity[2];
   // prevPosition[2] = Position[2];



}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

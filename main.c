/**
  ******************************************************************************
  * @file    FatFs/FatFs_uSD_RTOS/Src/main.c 
  * @author  MCD Application Team
  * @brief   Main program body
  *          This sample code shows how to use FatFs with uSD card drive in RTOS
  *          mode.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "queue.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Audio file size and start address are defined here since the audio file is
    stored in Flash memory as a constant table of 16-bit data */
#define AUDIO_FILE_SZE          990000
#define AUIDO_START_ADDRESS     58 /* Offset relative to audio file header size */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint16_t AUDIO_SAMPLE[];
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void StartThread(void const *argument);
static void CPU_CACHE_Enable(void);
uint16_t *ptr;
uint8_t buffer[AUDIO_OUT_BUFFER_SIZE];
QueueHandle_t xDmaAudioQueue;
osMailQId mailId;
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 216 MHz */
  SystemClock_Config();
  
  /* Configure LED1 */
  BSP_LED_Init(LED1);
  log_init();
  xDmaAudioQueue = xQueueCreate( 10, sizeof( uint8_t ) );
  /* Create the mail queue used by the two tasks to pass the struct Amail_TypeDef */
  osMailQDef(mail, 10, uint8_t); /* Define mail queue */

  mailId = osMailCreate(osMailQ(mail), NULL); /* create mail queue */

  /*##-1- Start task #########################################################*/
  osThreadDef(uSDThread, StartThread, osPriorityNormal, 0, 8 * configMINIMAL_STACK_SIZE);
  osThreadCreate(osThread(uSDThread), NULL);
  
  /*##-2- Start scheduler ####################################################*/
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}

//BUFFER_StateTypeDef buffer_state = BUFFER_OFFSET_NONE;
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
	uint8_t buffer_state = BUFFER_OFFSET_HALF;
	osMailPut(mailId, &buffer_state);
}
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t buffer_state = BUFFER_OFFSET_FULL;
	osMailPut(mailId, &buffer_state);
}
/**
  * @brief  Start task
  * @param  pvParameters not used
  * @retval None
  */
static void StartThread(void const *argument)
{
	WAVE_FormatTypeDef info;
	WAVE_FormatTypeDef WaveFormat;
	FATFS SDFatFs;  /* File system object for SD card logical drive */
	FIL MyFile;     /* File object */
	uint32_t bytesread;
	char SDPath[4]; /* SD card logical drive path */
	FATFS_LinkDriver(&SD_Driver, SDPath);
	f_mount(&SDFatFs, (TCHAR const*)SDPath, 0);
	f_open(&MyFile, (char *)"01-Het Muc.wav", FA_OPEN_EXISTING | FA_READ);
	f_read(&MyFile, &info, sizeof(WaveFormat), (void *)&bytesread);

	printf("file name %d\r\n", info.FileSize);
	printf("sample rate %d\r\n", info.SampleRate);

	BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 64, I2S_AUDIOFREQ_48K);

	  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */
	  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_BOTH, 64, info.SampleRate) != 0)
	  {
	  }
	  else
	  {
	    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
	  }

	f_read(&MyFile,
	              &buffer[0],
	              AUDIO_OUT_BUFFER_SIZE,
	              (void *)&bytesread);
	if(bytesread != 0)
	{
	  BSP_AUDIO_OUT_Play((uint16_t*)&buffer[0], AUDIO_OUT_BUFFER_SIZE);
//	  buffer_state = BUFFER_OFFSET_NONE;
	}
  for( ;; )
  {
	  uint8_t buffer_state;
	  osEvent event;
	  event = osMailGet(mailId, osWaitForever);
	  if(event.status == osEventMail)
	  {
		  buffer_state = *((uint8_t *)event.value.p);
		  if(buffer_state == BUFFER_OFFSET_HALF)
			{
			  if(f_read(&MyFile, &buffer[0], AUDIO_OUT_BUFFER_SIZE/2,
						(void *)&bytesread) != FR_OK)
			  {
				BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
			  }
			  buffer_state = BUFFER_OFFSET_NONE;
			}

			if(buffer_state == BUFFER_OFFSET_FULL)
			{
				if(f_read(&MyFile, &buffer[AUDIO_OUT_BUFFER_SIZE /2],
						AUDIO_OUT_BUFFER_SIZE/2,
						(void *)&bytesread) != FR_OK)
				{
					BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
				}

				buffer_state = BUFFER_OFFSET_NONE;
			}

	  }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED1 on */
  BSP_LED_On(LED1);
  while(1)
  {
    BSP_LED_Toggle(LED1);
    HAL_Delay(200);
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */ 

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

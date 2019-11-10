/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_scheduler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PIN_RESET(x) (x << 16U)
#define PIN_SET(x) (x)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//Encoder waveform pattern for positive and negative signals
uint32_t pattern_pos[] = { (PIN_RESET(ENC_PIN_A) | PIN_RESET(ENC_PIN_B)),
								 (PIN_SET(ENC_PIN_A) | PIN_RESET(ENC_PIN_B)),
								 (PIN_SET(ENC_PIN_A) | PIN_SET(ENC_PIN_B)),
								 (PIN_RESET(ENC_PIN_A) | PIN_SET(ENC_PIN_B))};

uint32_t pattern_neg[] = { (PIN_RESET(ENC_PIN_A) | PIN_SET(ENC_PIN_B)),
								 (PIN_SET(ENC_PIN_A) | PIN_SET(ENC_PIN_B)),
								 (PIN_SET(ENC_PIN_A) | PIN_RESET(ENC_PIN_B)),
								 (PIN_RESET(ENC_PIN_A) | PIN_RESET(ENC_PIN_B))};
const int pattern_size = 4;
int pattern_index = 0;
int direction = 1; // 1 positive, 0 negative
short int motor_voltage = 0;
uint8_t encoder_status = 0; // 1 - start, 0 - stop
uint16_t timer_ARR = 0; //ARR value for timer
int encoder_pos = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */
extern __IO uint16_t AdcRawValue[2];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	TaskTick();
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
	motor_voltage = (short int) (AdcRawValue[0] - AdcRawValue[1]);

	if(motor_voltage<0)
	{
		direction = 0; //use negative pattern
		motor_voltage *= -1; //remove sign
	} else
		direction = 1; //use positive pattern
	//encoder lower enable threshold
	if(motor_voltage <= 39)
	{
		//If timer enabled, stop it
		if(encoder_status)
		{
			HAL_TIM_Base_Stop_IT(&htim7);
			encoder_status = 0;
		}
	} else {
		//value above lower threshold, generate encoder signal

		//TODO use equation to calculate frequency
		if(motor_voltage > 39 && motor_voltage <= 54)
			htim7.Instance->ARR = 4200; //2.5kHz
		else if(motor_voltage >= 54)
			htim7.Instance->ARR = 300; //35kHz

		//If timer disabled, start it
		if(!encoder_status)
		{
			HAL_TIM_Base_Start_IT(&htim7);
			encoder_status = 1;
		}
	}

  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

  //Remove below HAL_TIM_IRQHandler(&htim6); to do faster
  /* USER CODE END TIM6_DAC_IRQn 0 */

  //HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  if(direction == 1) {
	  ENC_GPIO->BSRR = pattern_pos[pattern_index];
	  encoder_pos++;
  } else {
	  ENC_GPIO->BSRR = pattern_neg[pattern_index];
	  encoder_pos--;
  }

  pattern_index++;
  pattern_index = pattern_index % pattern_size;

  __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
  //Remove below HAL_TIM_IRQHandler(&htim6); to do faster
  /* USER CODE END TIM7_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

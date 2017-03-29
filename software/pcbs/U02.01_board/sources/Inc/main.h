/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PWR_12V_ENABLE_Pin GPIO_PIN_14
#define PWR_12V_ENABLE_GPIO_Port GPIOC
#define GAS_ENABLE_Pin GPIO_PIN_15
#define GAS_ENABLE_GPIO_Port GPIOC
#define ADC_12V_OUT_Pin GPIO_PIN_0
#define ADC_12V_OUT_GPIO_Port GPIOA
#define ADC_GAS_Pin GPIO_PIN_1
#define ADC_GAS_GPIO_Port GPIOA
#define ADC_12V_IN_Pin GPIO_PIN_2
#define ADC_12V_IN_GPIO_Port GPIOA
#define XF003_Pin GPIO_PIN_3
#define XF003_GPIO_Port GPIOA
#define ADC_WATER2_Pin GPIO_PIN_4
#define ADC_WATER2_GPIO_Port GPIOA
#define ADC_WATER1_Pin GPIO_PIN_5
#define ADC_WATER1_GPIO_Port GPIOA
#define ADC_DRY1_Pin GPIO_PIN_6
#define ADC_DRY1_GPIO_Port GPIOA
#define ADC_DRY2_Pin GPIO_PIN_7
#define ADC_DRY2_GPIO_Port GPIOA
#define ADC_FOG2_Pin GPIO_PIN_0
#define ADC_FOG2_GPIO_Port GPIOB
#define ADC_FOG1_Pin GPIO_PIN_1
#define ADC_FOG1_GPIO_Port GPIOB
#define USER_BTN_Pin GPIO_PIN_2
#define USER_BTN_GPIO_Port GPIOB
#define FOG_ENABLE_Pin GPIO_PIN_10
#define FOG_ENABLE_GPIO_Port GPIOB
#define XF002_Pin GPIO_PIN_8
#define XF002_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define USB_CONNECTED_Pin GPIO_PIN_4
#define USB_CONNECTED_GPIO_Port GPIOB
#define BOX_CLOSED_Pin GPIO_PIN_5
#define BOX_CLOSED_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_8
#define RELAY2_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_9
#define RELAY1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

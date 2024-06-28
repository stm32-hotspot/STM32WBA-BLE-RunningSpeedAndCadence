/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service2_app.h
  * @author  MCD Application Team
  * @brief   Header for service2_app.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RSCS_APP_H
#define RSCS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  RSCS_CONN_HANDLE_EVT,
  RSCS_DISCON_HANDLE_EVT,

  /* USER CODE BEGIN Service2_OpcodeNotificationEvt_t */

  /* USER CODE END Service2_OpcodeNotificationEvt_t */

  RSCS_LAST_EVT,
} RSCS_APP_OpcodeNotificationEvt_t;

typedef struct
{
  RSCS_APP_OpcodeNotificationEvt_t          EvtOpcode;
  uint16_t                                 ConnectionHandle;

  /* USER CODE BEGIN RSCS_APP_ConnHandleNotEvt_t */

  /* USER CODE END RSCS_APP_ConnHandleNotEvt_t */
} RSCS_APP_ConnHandleNotEvt_t;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void RSCS_APP_Init(void);
void RSCS_APP_EvtRx(RSCS_APP_ConnHandleNotEvt_t *p_Notification);
/* USER CODE BEGIN EFP */
void RSCS_APP_SCCPRequestHandler(uint8_t * pRequestData, uint8_t requestDataLength);
uint8_t RSCS_APP_SCCPCheckRequestValid(uint8_t * pRequestData, uint8_t requestDataLength);
void RSCS_APP_SCCPIndicationReceived(void);
void RSCS_APP_SCCPCalibrationFailed(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*RSCS_APP_H */

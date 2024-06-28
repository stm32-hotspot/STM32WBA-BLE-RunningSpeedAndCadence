/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service2.h
  * @author  MCD Application Team
  * @brief   Header for service2.c
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
#ifndef RSCS_H
#define RSCS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported defines ----------------------------------------------------------*/
/* USER CODE BEGIN ED */

/* USER CODE END ED */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  RSCS_RSCM,
  RSCS_RSCF,
  RSCS_SEL,
  RSCS_SCP,
  /* USER CODE BEGIN Service2_CharOpcode_t */

  /* USER CODE END Service2_CharOpcode_t */
  RSCS_CHAROPCODE_LAST
} RSCS_CharOpcode_t;

typedef enum
{
  RSCS_RSCM_NOTIFY_ENABLED_EVT,
  RSCS_RSCM_NOTIFY_DISABLED_EVT,
  RSCS_RSCF_READ_EVT,
  RSCS_SEL_READ_EVT,
  RSCS_SCP_WRITE_EVT,
  RSCS_SCP_INDICATE_ENABLED_EVT,
  RSCS_SCP_INDICATE_DISABLED_EVT,
  /* USER CODE BEGIN Service2_OpcodeEvt_t */

  /* USER CODE END Service2_OpcodeEvt_t */
  RSCS_BOOT_REQUEST_EVT
} RSCS_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;

  /* USER CODE BEGIN Service2_Data_t */

  /* USER CODE END Service2_Data_t */
} RSCS_Data_t;

typedef struct
{
  RSCS_OpcodeEvt_t       EvtOpcode;
  RSCS_Data_t             DataTransfered;
  uint16_t                ConnectionHandle;
  uint16_t                AttributeHandle;
  uint8_t                 ServiceInstance;
  /* USER CODE BEGIN Service2_NotificationEvt_t */

  /* USER CODE END Service2_NotificationEvt_t */
} RSCS_NotificationEvt_t;

/* USER CODE BEGIN ET */
typedef enum
{
  NO_FEATURE = 0,
  INSTANTANEOUS_STRIDE_LENGTH_MEASUREMENT_SUPORTED = (1<<0),
  TOTAL_DISTANCE_MEASUREMENT_SUPPORTED = (1<<1),
  WARNING_OR_RUNNING_STATUS_SUPPORTED = (1<<2),
  CALIBRATION_PROCEDURE_SUPPORTED = (1<<3),
  MULTIPLE_SENSOR_LOCATIONS_SUPPORTED = (1<<4)
} RSCS_Feature_t;

typedef enum
{
  NO_RSCSM_FLAG = 0,
  INSTANTANEOUS_STRIDE_LENGTH_PRESENT = (1<<0),
  TOTAL_DISTANCE_PRESENT = (1<<1),
  RUNNING_STATUS = (1<<2),
} RSCSM_Flag_t;

typedef struct
{
  uint8_t Flags;
  uint16_t InstantaneousSpeed;
  uint8_t InstantaneousCadence;
  uint16_t InstantaneousStrideLength;
  uint32_t TotalDistance;
}RSCS_Measurement_Value_t;

typedef struct
{
  uint16_t OpCode;
  uint8_t Parameter[18];
  uint8_t response;
}RSCS_SCCP_Value_t;
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
void RSCS_Init(void);
void RSCS_Notification(RSCS_NotificationEvt_t *p_Notification);
tBleStatus RSCS_UpdateValue(RSCS_CharOpcode_t CharOpcode, RSCS_Data_t *pData);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*RSCS_H */

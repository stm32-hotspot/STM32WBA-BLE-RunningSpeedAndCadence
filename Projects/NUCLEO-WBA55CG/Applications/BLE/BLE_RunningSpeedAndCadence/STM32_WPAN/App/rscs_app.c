/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service2_app.c
  * @author  MCD Application Team
  * @brief   service2_app application definition.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "app_ble.h"
#include "ll_sys_if.h"
#include "dbg_trace.h"
#include "ble.h"
#include "rscs_app.h"
#include "rscs.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef enum
{
  Rscm_NOTIFICATION_OFF,
  Rscm_NOTIFICATION_ON,
  Scp_INDICATION_OFF,
  Scp_INDICATION_ON,
  /* USER CODE BEGIN Service2_APP_SendInformation_t */

  /* USER CODE END Service2_APP_SendInformation_t */
  RSCS_APP_SENDINFORMATION_LAST
} RSCS_APP_SendInformation_t;

typedef struct
{
  RSCS_APP_SendInformation_t     Rscm_Notification_Status;
  RSCS_APP_SendInformation_t     Scp_Indication_Status;
  /* USER CODE BEGIN Service2_APP_Context_t */
  uint16_t RSCSFChar;
  RSCS_Measurement_Value_t RSCSMChar;
  uint8_t RSCSSLChar;
  RSCS_SCCP_Value_t RSCSSCPChar;
  uint8_t RSCSSCPCccd;
  uint8_t RSCSSCP_InProgress;
  uint8_t CalibrationFailed;
  UTIL_TIMER_Object_t TimerRSCSMeasurement_Id;
  UTIL_TIMER_Object_t TimerRSCSControlPoint_Id;
  /* USER CODE END Service2_APP_Context_t */
  uint16_t              ConnectionHandle;
} RSCS_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_RSC_MEASUREMENT_INTERVAL                      (1000)  /**< 1s */
#define DEFAULT_RSC_CONTROL_POINT_INTERVAL                    (2000)  /**< 2s */

/* Operational States */
#define UNDETERMINED                                                        0x0F
#define OFF                                                                 0x33
#define STANDBY                                                             0x3C
#define PREPARING                                                           0x55
#define PRIMING                                                             0x5A
#define WAITING                                                             0x66
#define READY                                                               0x96

/* Therapy Control States */
#define STOP_STATE                                                          0x33
#define PAUSE_STATE                                                         0x3C
#define RUN_STATE                                                           0x55

/* SC Control Point */
/* Op Codes */
#define SET_CUMULATIVE_VALUE                                                0x01
#define START_SENSOR_CALIBRATION                                            0x02
#define UPDATE_SENSOR_LOCATION                                              0x03
#define REQUEST_SUPPORTED_SENSOR_LOCATIONS                                  0x04
#define RESPONSE_CODE                                                       0x10
/* Response Op Code */
#define SUCCESS_RESPONSE                                                    0x01
#define OP_CODE_NOT_SUPPORTED                                               0x02
#define INVALID_OPERAND                                                     0x03
#define OPERATION_FAILED                                                    0x04
#define CCCD_IMPROPERLY_CONFIGURED                                          0x81
#define PROCEDURE_ALLREADY_IN_PROGRESS                                      0x80
/* Sensor Location */
#define TOP_OF_SHOE                                                         0x01
#define IN_SHOE                                                             0x02
#define HIP                                                                 0x03
#define FRONT_WHEEL                                                         0x04
#define LEFT_CRANK                                                          0x05
#define RIGHT_CRANK                                                         0x06
#define LEFT_PEDAL                                                          0x07
#define RIGHT_PEDAL                                                         0x08
#define FRONT_HUB                                                           0x09
#define REAR_DROPOUT                                                        0x0A
#define CHAINSTAY                                                           0x0B
#define REAR_WHEEL                                                          0x0C
#define REAR_HUB                                                            0x0D
#define CHEST                                                               0x0E
#define SPIDER                                                              0x0F
#define CHAIN_RING                                                          0x10
/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static RSCS_APP_Context_t RSCS_APP_Context;

uint8_t a_RSCS_UpdateCharData[247];

/* USER CODE BEGIN PV */
static uint8_t a_RSCS_SensorLocationList[16] =
{
  TOP_OF_SHOE, IN_SHOE, HIP, CHEST, 
  0,0,0,0,0,0,0,0,0,0,0,0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void RSCS_Rscm_SendNotification(void);
static void RSCS_Scp_SendIndication(void);

/* USER CODE BEGIN PFP */
static tBleStatus RSCS_Scp_SendResponseCode(uint8_t Opcode, uint8_t responseCode);
static void RSCS_Rscm_MeasurementTimCb(void *arg);
static uint32_t RSCS_ReadRTCSSRSS ( void );
static void RSCS_Rscm_Measurement(void);
static void RSCS_Scp_EndControlPointTimCb(void *arg);
static void RSCS_Scp_EndControlPoint(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void RSCS_Notification(RSCS_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service2_Notification_1 */

  /* USER CODE END Service2_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service2_Notification_Service2_EvtOpcode */

    /* USER CODE END Service2_Notification_Service2_EvtOpcode */

    case RSCS_RSCM_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service2Char1_NOTIFY_ENABLED_EVT */
      {
        LOG_INFO_APP("RSC Measurement Notification enabled\r\n");
        UTIL_TIMER_Stop(&RSCS_APP_Context.TimerRSCSMeasurement_Id);
        UTIL_TIMER_StartWithPeriod(&RSCS_APP_Context.TimerRSCSMeasurement_Id, DEFAULT_RSC_MEASUREMENT_INTERVAL);
      }
      /* USER CODE END Service2Char1_NOTIFY_ENABLED_EVT */
      break;

    case RSCS_RSCM_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service2Char1_NOTIFY_DISABLED_EVT */
      {
        LOG_INFO_APP("RSC Measurement Notification disabled\r\n");
        UTIL_TIMER_Stop(&RSCS_APP_Context.TimerRSCSMeasurement_Id);
      }
      /* USER CODE END Service2Char1_NOTIFY_DISABLED_EVT */
      break;

    case RSCS_RSCF_READ_EVT:
      /* USER CODE BEGIN Service2Char2_READ_EVT */

      /* USER CODE END Service2Char2_READ_EVT */
      break;

    case RSCS_SEL_READ_EVT:
      /* USER CODE BEGIN Service2Char3_READ_EVT */

      /* USER CODE END Service2Char3_READ_EVT */
      break;

    case RSCS_SCP_WRITE_EVT:
      /* USER CODE BEGIN Service2Char4_WRITE_EVT */

      /* USER CODE END Service2Char4_WRITE_EVT */
      break;

    case RSCS_SCP_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN Service2Char4_INDICATE_ENABLED_EVT */
      {
        LOG_INFO_APP("SC Control Point Indication enabled\r\n");
        RSCS_APP_Context.RSCSSCPCccd = Scp_INDICATION_ON;
      }
      /* USER CODE END Service2Char4_INDICATE_ENABLED_EVT */
      break;

    case RSCS_SCP_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN Service2Char4_INDICATE_DISABLED_EVT */
      {
        LOG_INFO_APP("SC Control Point Indication disabled\r\n");
        RSCS_APP_Context.RSCSSCPCccd = Scp_INDICATION_OFF;
      }
      /* USER CODE END Service2Char4_INDICATE_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN Service2_Notification_default */

      /* USER CODE END Service2_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service2_Notification_2 */

  /* USER CODE END Service2_Notification_2 */
  return;
}

void RSCS_APP_EvtRx(RSCS_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service2_APP_EvtRx_1 */

  /* USER CODE END Service2_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service2_APP_EvtRx_Service2_EvtOpcode */

    /* USER CODE END Service2_APP_EvtRx_Service2_EvtOpcode */
    case RSCS_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service2_APP_CONN_HANDLE_EVT */

      /* USER CODE END Service2_APP_CONN_HANDLE_EVT */
      break;

    case RSCS_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN Service2_APP_DISCON_HANDLE_EVT */
      {
        UTIL_TIMER_Stop(&RSCS_APP_Context.TimerRSCSMeasurement_Id);
        RSCS_APP_Context.RSCSSCP_InProgress = 0;
        RSCS_APP_Context.RSCSMChar.Flags = NO_RSCSM_FLAG;
        RSCS_APP_Context.RSCSMChar.Flags = INSTANTANEOUS_STRIDE_LENGTH_PRESENT |
                                           TOTAL_DISTANCE_PRESENT |
                                           RUNNING_STATUS;
        RSCS_APP_Context.RSCSMChar.InstantaneousCadence = 0;
        RSCS_APP_Context.RSCSMChar.InstantaneousSpeed = 0;
        RSCS_APP_Context.RSCSMChar.InstantaneousStrideLength = 0;
        RSCS_APP_Context.RSCSMChar.TotalDistance = 0;
      }
      /* USER CODE END Service2_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service2_APP_EvtRx_default */

      /* USER CODE END Service2_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service2_APP_EvtRx_2 */

  /* USER CODE END Service2_APP_EvtRx_2 */

  return;
}

void RSCS_APP_Init(void)
{
  UNUSED(RSCS_APP_Context);
  RSCS_Init();

  /* USER CODE BEGIN Service2_APP_Init */
  RSCS_Data_t msg_conf;
  uint8_t length = 0;
  
  /* RSCS Feature */
  RSCS_APP_Context.RSCSFChar = NO_FEATURE;
  RSCS_APP_Context.RSCSFChar = INSTANTANEOUS_STRIDE_LENGTH_MEASUREMENT_SUPORTED |
                               TOTAL_DISTANCE_MEASUREMENT_SUPPORTED |
                               WARNING_OR_RUNNING_STATUS_SUPPORTED |
                               CALIBRATION_PROCEDURE_SUPPORTED |
                               MULTIPLE_SENSOR_LOCATIONS_SUPPORTED;

  a_RSCS_UpdateCharData[length++] =  (RSCS_APP_Context.RSCSFChar) & 0xFF;
  a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSFChar) >> 8) & 0xFF;
  msg_conf.Length = length;
  msg_conf.p_Payload = a_RSCS_UpdateCharData;
  RSCS_UpdateValue(RSCS_RSCF, &msg_conf);
  
  /* RSCS Measurement */
  RSCS_APP_Context.RSCSMChar.Flags = NO_RSCSM_FLAG;
  RSCS_APP_Context.RSCSMChar.Flags = INSTANTANEOUS_STRIDE_LENGTH_PRESENT |
                                     TOTAL_DISTANCE_PRESENT |
                                     RUNNING_STATUS;
  RSCS_APP_Context.RSCSMChar.InstantaneousCadence = 0;
  RSCS_APP_Context.RSCSMChar.InstantaneousSpeed = 0;
  RSCS_APP_Context.RSCSMChar.InstantaneousStrideLength = 0;
  RSCS_APP_Context.RSCSMChar.TotalDistance = 0;
  
  length = 0;
  a_RSCS_UpdateCharData[length++] =   RSCS_APP_Context.RSCSMChar.Flags;
  a_RSCS_UpdateCharData[length++] =  (RSCS_APP_Context.RSCSMChar.InstantaneousSpeed) & 0xFF;
  a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.InstantaneousSpeed) >> 8) & 0xFF;
  a_RSCS_UpdateCharData[length++] =   RSCS_APP_Context.RSCSMChar.InstantaneousCadence;
  a_RSCS_UpdateCharData[length++] =  (RSCS_APP_Context.RSCSMChar.InstantaneousStrideLength) & 0xFF;
  a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.InstantaneousStrideLength) >> 8) & 0xFF;
  a_RSCS_UpdateCharData[length++] =  (RSCS_APP_Context.RSCSMChar.TotalDistance) & 0xFF;
  a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.TotalDistance) >> 8) & 0xFF;
  a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.TotalDistance) >> 16) & 0xFF;
  a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.TotalDistance) >> 24) & 0xFF;
  msg_conf.Length = length;
  msg_conf.p_Payload = a_RSCS_UpdateCharData;
  RSCS_UpdateValue(RSCS_RSCM, &msg_conf);
  
  /* RSCS Sensor Location */
  RSCS_APP_Context.RSCSSLChar = TOP_OF_SHOE;

  length = 0;
  a_RSCS_UpdateCharData[length++] =   RSCS_APP_Context.RSCSSLChar;
  msg_conf.Length = length;
  msg_conf.p_Payload = a_RSCS_UpdateCharData;
  RSCS_UpdateValue(RSCS_SEL, &msg_conf);

  /* RSCS Control Point Indication disabled */
  RSCS_APP_Context.RSCSSCPCccd = Scp_INDICATION_OFF;
  
  /* RSCS Control Point proccess in progress */
  RSCS_APP_Context.RSCSSCP_InProgress = 0;

  /* Sensor Calibration Success */
  RSCS_APP_Context.CalibrationFailed = 0;
  
  /**
   * Register task for RSCS Measurement
   */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_RSCS_MEAS_REQ_ID, UTIL_SEQ_RFU, RSCS_Rscm_Measurement);

  /**
   * Create timer for RSCS Measurement
   */
  UTIL_TIMER_Create(&(RSCS_APP_Context.TimerRSCSMeasurement_Id), DEFAULT_RSC_MEASUREMENT_INTERVAL, UTIL_TIMER_PERIODIC, RSCS_Rscm_MeasurementTimCb, 0);

  /**
   * Register task for End Sensor Calibration
   */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_RSCS_END_CONTROL_POINT_REQ_ID, UTIL_SEQ_RFU, RSCS_Scp_EndControlPoint);

  /**
   * Create timer for Sensor Calibration
   */
  UTIL_TIMER_Create(&(RSCS_APP_Context.TimerRSCSControlPoint_Id), DEFAULT_RSC_CONTROL_POINT_INTERVAL, UTIL_TIMER_ONESHOT, RSCS_Scp_EndControlPointTimCb, 0);

  /* USER CODE END Service2_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
/**
* @brief SCP request handler 
* @param requestData: pointer to received SCP request data
* @param requestDataLength: received SCP request length
* @retval None
*/
void RSCS_APP_SCCPRequestHandler(uint8_t * pRequestData, uint8_t requestDataLength)
{
  uint8_t OpCode = pRequestData[0];

  RSCS_APP_Context.RSCSSCPChar.OpCode = OpCode;
  
  /* Check and Process the OpCode */
  switch(OpCode)
  {
    case SET_CUMULATIVE_VALUE:
      {
        LOG_INFO_APP("SET_CUMULATIVE_VALUE\r\n");

        LOG_INFO_APP("requestDataLength\r\n", requestDataLength);
        for(uint8_t i = 0; i < requestDataLength; i++)
        {
          LOG_INFO_APP("pRequestData[%d] = 0x%x\r\n", i, pRequestData[i]);
        }
        
        /* Stop RSC Measurement */
        UTIL_TIMER_Stop(&RSCS_APP_Context.TimerRSCSMeasurement_Id);
          
        if(((RSCS_APP_Context.RSCSMChar.Flags) & TOTAL_DISTANCE_MEASUREMENT_SUPPORTED) == TOTAL_DISTANCE_MEASUREMENT_SUPPORTED )
        {
          /* Set Total Distance to cumulative value */
          RSCS_APP_Context.RSCSMChar.TotalDistance = pRequestData[1]         |
                                                     (pRequestData[2] << 8)  |
                                                     (pRequestData[3] << 16) |
                                                     (pRequestData[4] << 24);

          RSCS_APP_Context.RSCSSCPChar.response = SUCCESS_RESPONSE;
        }
        else
        {
          LOG_INFO_APP("OPERATION FAILED\r\n");
          RSCS_APP_Context.RSCSSCPChar.response = OPERATION_FAILED;
        }
        RSCS_APP_Context.RSCSSCP_InProgress = 1;
        UTIL_TIMER_StartWithPeriod(&RSCS_APP_Context.TimerRSCSControlPoint_Id, DEFAULT_RSC_CONTROL_POINT_INTERVAL);
      }
      break;
                      
    case START_SENSOR_CALIBRATION:
      {
        /* Stop RSC Measurement */
        UTIL_TIMER_Stop(&RSCS_APP_Context.TimerRSCSMeasurement_Id);
          
        if(((RSCS_APP_Context.RSCSFChar) & CALIBRATION_PROCEDURE_SUPPORTED) == CALIBRATION_PROCEDURE_SUPPORTED )
        {
          RSCS_APP_Context.RSCSSCPChar.response = SUCCESS_RESPONSE;
          LOG_INFO_APP("START_SENSOR_CALIBRATION\r\n");
          LOG_INFO_APP("SUCCESS\r\n");
        }
        else
        {
          LOG_INFO_APP("OPERATION FAILED\r\n");
          RSCS_APP_Context.RSCSSCPChar.response = OPERATION_FAILED;
        }
        RSCS_APP_Context.RSCSSCP_InProgress = 1;
        UTIL_TIMER_StartWithPeriod(&RSCS_APP_Context.TimerRSCSControlPoint_Id, DEFAULT_RSC_CONTROL_POINT_INTERVAL);
      }
      break;
                      
    case UPDATE_SENSOR_LOCATION:
      {
        /* Stop RSC Measurement */
        UTIL_TIMER_Stop(&RSCS_APP_Context.TimerRSCSMeasurement_Id);
          
        if(((RSCS_APP_Context.RSCSFChar) & MULTIPLE_SENSOR_LOCATIONS_SUPPORTED) == MULTIPLE_SENSOR_LOCATIONS_SUPPORTED )
        {
          if(pRequestData[1] <= CHAIN_RING)
          {
            RSCS_Data_t msg_conf;
            uint8_t length = 0;
            
            LOG_INFO_APP("UPDATE_SENSOR_LOCATION\r\n");
            
            RSCS_APP_Context.RSCSSLChar = pRequestData[1];
            
            while((length < 16)                           &&
                  (a_RSCS_SensorLocationList[length] > 0) &&
                  (a_RSCS_SensorLocationList[length] != RSCS_APP_Context.RSCSSLChar))
            {
              length++;
            }
            if(a_RSCS_SensorLocationList[length] == 0)
            {
              a_RSCS_SensorLocationList[length] = RSCS_APP_Context.RSCSSLChar;
            }
                   
            length = 0;
            a_RSCS_UpdateCharData[length++] = RSCS_APP_Context.RSCSSLChar;
            msg_conf.Length = length;
            msg_conf.p_Payload = a_RSCS_UpdateCharData;
            RSCS_UpdateValue(RSCS_SEL, &msg_conf);
            
            RSCS_APP_Context.RSCSSCPChar.response = SUCCESS_RESPONSE;
            LOG_INFO_APP("SUCCESS\r\n");
          }
          else
          {
            LOG_INFO_APP("INVALID PARAMETER\r\n");
            RSCS_APP_Context.RSCSSCPChar.response = INVALID_OPERAND;
          }
        }
        else
        {
          LOG_INFO_APP("OPERATION FAILED\r\n");
          RSCS_APP_Context.RSCSSCPChar.response = OPERATION_FAILED;
        }
        RSCS_APP_Context.RSCSSCP_InProgress = 1;
        UTIL_TIMER_StartWithPeriod(&RSCS_APP_Context.TimerRSCSControlPoint_Id, DEFAULT_RSC_CONTROL_POINT_INTERVAL);
      }
      break;
                      
    case REQUEST_SUPPORTED_SENSOR_LOCATIONS:
      {
        if(((RSCS_APP_Context.RSCSFChar) & MULTIPLE_SENSOR_LOCATIONS_SUPPORTED) == MULTIPLE_SENSOR_LOCATIONS_SUPPORTED )
        {
          RSCS_APP_Context.RSCSSCPChar.response = SUCCESS_RESPONSE;
        }
        else
        {
          LOG_INFO_APP("OPERATION FAILED\r\n");
          RSCS_APP_Context.RSCSSCPChar.response = OPERATION_FAILED;
        }
        RSCS_APP_Context.RSCSSCP_InProgress = 1;
        UTIL_TIMER_StartWithPeriod(&RSCS_APP_Context.TimerRSCSControlPoint_Id, DEFAULT_RSC_CONTROL_POINT_INTERVAL);
      }
      break;
                      
    default:
      {
        LOG_INFO_APP("OP CODE NOT SUPPORTED\r\n");
        RSCS_APP_Context.RSCSSCPChar.response = OP_CODE_NOT_SUPPORTED;
        RSCS_APP_Context.RSCSSCP_InProgress = 1;
        UTIL_TIMER_StartWithPeriod(&RSCS_APP_Context.TimerRSCSControlPoint_Id, DEFAULT_RSC_CONTROL_POINT_INTERVAL);
      }
      break;
  }
} /* end of IDS_SRCP_RequestHandler() */

/**
* @brief SCP request check
* @param [in] requestData: pointer to received SCP request data
* @param [in] requestDataLength: received SCP request length
* @retval 0x00 when no error, error code otherwise
*/
uint8_t RSCS_APP_SCCPCheckRequestValid(uint8_t * pRequestData, uint8_t requestDataLength)
{
  uint8_t retval = 0x00;
  
  LOG_INFO_APP("SCP Request %d, request data length: %d\r\n", pRequestData[0], requestDataLength);
  
  if (RSCS_APP_Context.RSCSSCPCccd != Scp_INDICATION_ON)
  {
    retval = CCCD_IMPROPERLY_CONFIGURED;
  }
  else if(RSCS_APP_Context.RSCSSCP_InProgress)
  {
    retval = PROCEDURE_ALLREADY_IN_PROGRESS;
  }
  
  return retval;
} /* end of CGMS_RACP_CheckRequestValid() */

void RSCS_APP_SCCPIndicationReceived(void)
{
  RSCS_APP_Context.RSCSSCP_InProgress = 0;

  /* Start RSC Measurement updates */
  UTIL_TIMER_StartWithPeriod(&RSCS_APP_Context.TimerRSCSMeasurement_Id, DEFAULT_RSC_MEASUREMENT_INTERVAL);
}

void RSCS_APP_SCCPCalibrationFailed(void)
{
  RSCS_APP_Context.CalibrationFailed = 1; /* Sensor Calibration Failed */  
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
__USED void RSCS_Rscm_SendNotification(void) /* Property Notification */
{
  RSCS_APP_SendInformation_t notification_on_off = Rscm_NOTIFICATION_OFF;
  RSCS_Data_t rscs_notification_data;

  rscs_notification_data.p_Payload = (uint8_t*)a_RSCS_UpdateCharData;
  rscs_notification_data.Length = 0;

  /* USER CODE BEGIN Service2Char1_NS_1*/

  /* USER CODE END Service2Char1_NS_1*/

  if (notification_on_off != Rscm_NOTIFICATION_OFF)
  {
    RSCS_UpdateValue(RSCS_RSCM, &rscs_notification_data);
  }

  /* USER CODE BEGIN Service2Char1_NS_Last*/

  /* USER CODE END Service2Char1_NS_Last*/

  return;
}

__USED void RSCS_Scp_SendIndication(void) /* Property Indication */
{
  RSCS_APP_SendInformation_t indication_on_off = Scp_INDICATION_OFF;
  RSCS_Data_t rscs_indication_data;

  rscs_indication_data.p_Payload = (uint8_t*)a_RSCS_UpdateCharData;
  rscs_indication_data.Length = 0;

  /* USER CODE BEGIN Service2Char4_IS_1*/

  /* USER CODE END Service2Char4_IS_1*/

  if (indication_on_off != Scp_INDICATION_OFF)
  {
    RSCS_UpdateValue(RSCS_SCP, &rscs_indication_data);
  }

  /* USER CODE BEGIN Service2Char4_IS_Last*/

  /* USER CODE END Service2Char4_IS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void RSCS_Scp_EndControlPointTimCb(void *arg)
{
  /**
   * The code shall be executed in the background as aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  UTIL_SEQ_SetTask(1<<CFG_TASK_RSCS_END_CONTROL_POINT_REQ_ID, CFG_SEQ_PRIO_0);

  return;
}

static void RSCS_Rscm_MeasurementTimCb(void *arg)
{
  /**
   * The code shall be executed in the background as aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  UTIL_SEQ_SetTask(1<<CFG_TASK_RSCS_MEAS_REQ_ID, CFG_SEQ_PRIO_0);

  return;
}

static uint32_t RSCS_ReadRTCSSRSS ( void )
{
  return ((uint32_t) (((READ_REG(RTC->SSR) & 0x0E00) >> 9) + 65));
}

/**
* @brief End of Sensor calibration
* @param None
* @retval None
*/
static void RSCS_Scp_EndControlPoint(void)
{
  RSCS_Scp_SendResponseCode(RSCS_APP_Context.RSCSSCPChar.OpCode, RSCS_APP_Context.RSCSSCPChar.response);
}

/**
* @brief Update RSC Measurement char
* @param None
* @retval None
*/
static void RSCS_Rscm_Measurement(void)
{
  RSCS_Data_t msg_conf;
  uint8_t length = 0;

  a_RSCS_UpdateCharData[length++] =   RSCS_APP_Context.RSCSMChar.Flags;
  a_RSCS_UpdateCharData[length++] =  (RSCS_APP_Context.RSCSMChar.InstantaneousSpeed) & 0xFF;
  a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.InstantaneousSpeed) >> 8) & 0xFF;
  a_RSCS_UpdateCharData[length++] =   RSCS_APP_Context.RSCSMChar.InstantaneousCadence;
  if(((RSCS_APP_Context.RSCSMChar.Flags) & INSTANTANEOUS_STRIDE_LENGTH_PRESENT) == INSTANTANEOUS_STRIDE_LENGTH_PRESENT )
  {
    a_RSCS_UpdateCharData[length++] =  (RSCS_APP_Context.RSCSMChar.InstantaneousStrideLength) & 0xFF;
    a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.InstantaneousStrideLength) >> 8) & 0xFF;
  }
  if(((RSCS_APP_Context.RSCSMChar.Flags) & TOTAL_DISTANCE_MEASUREMENT_SUPPORTED) == TOTAL_DISTANCE_MEASUREMENT_SUPPORTED )
  {
    a_RSCS_UpdateCharData[length++] =  (RSCS_APP_Context.RSCSMChar.TotalDistance) & 0xFF;
    a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.TotalDistance) >> 8) & 0xFF;
    a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.TotalDistance) >> 16) & 0xFF;
    a_RSCS_UpdateCharData[length++] = ((RSCS_APP_Context.RSCSMChar.TotalDistance) >> 24) & 0xFF;
  }
  msg_conf.Length = length;
  msg_conf.p_Payload = a_RSCS_UpdateCharData;
  RSCS_UpdateValue(RSCS_RSCM, &msg_conf);

  RSCS_APP_Context.RSCSMChar.InstantaneousCadence += 10;
  
  if(RSCS_APP_Context.RSCSMChar.InstantaneousCadence > 60)
  {
    /* Walking status */
    RSCS_APP_Context.RSCSMChar.Flags = NO_RSCSM_FLAG;
    RSCS_APP_Context.RSCSMChar.Flags = INSTANTANEOUS_STRIDE_LENGTH_PRESENT |
                                       TOTAL_DISTANCE_PRESENT;
  }
  
  RSCS_APP_Context.RSCSMChar.InstantaneousSpeed = ((RSCS_ReadRTCSSRSS()) & 0xFF) |
                                                  (((RSCS_ReadRTCSSRSS()) >> 8) & 0xFF);
  if(((RSCS_APP_Context.RSCSMChar.Flags) & INSTANTANEOUS_STRIDE_LENGTH_PRESENT) == INSTANTANEOUS_STRIDE_LENGTH_PRESENT )
  {
    RSCS_APP_Context.RSCSMChar.InstantaneousStrideLength = ((RSCS_ReadRTCSSRSS()) & 0xFF) |
                                                           (((RSCS_ReadRTCSSRSS()) >> 8) & 0xFF);
  }
  if(((RSCS_APP_Context.RSCSMChar.Flags) & TOTAL_DISTANCE_MEASUREMENT_SUPPORTED) == TOTAL_DISTANCE_MEASUREMENT_SUPPORTED )
  {
    /* Total Distance is updated only if supported and no Control Point Set Cumulative process in progress */
    RSCS_APP_Context.RSCSMChar.TotalDistance += RSCS_ReadRTCSSRSS();
  }
  
  return;
}

/**
* @brief Send a SRCP response as consequence of a SRCP request
* @param Opcode: SRCP op code
* @param responseCode: SRCP response code or number of stored records
* @retval None
*/
static tBleStatus RSCS_Scp_SendResponseCode(uint8_t Opcode, uint8_t responseCode)
{
  tBleStatus retval = BLE_STATUS_FAILED;
  RSCS_Data_t msg_conf;
  uint8_t length = 0;

  a_RSCS_UpdateCharData[length++] = RESPONSE_CODE;
  a_RSCS_UpdateCharData[length++] = Opcode;
  if((Opcode == START_SENSOR_CALIBRATION) && (RSCS_APP_Context.CalibrationFailed == 1))
  {
    a_RSCS_UpdateCharData[length++] = OPERATION_FAILED;
    RSCS_APP_Context.CalibrationFailed = 0;
  }
  else
  {
    a_RSCS_UpdateCharData[length++] = responseCode;
  }
  
  if((responseCode == SUCCESS_RESPONSE) &&
     (Opcode == REQUEST_SUPPORTED_SENSOR_LOCATIONS))
  {
    uint8_t index = 0;
    
    while(a_RSCS_SensorLocationList[index] > 0)
    {
      a_RSCS_UpdateCharData[length++] = a_RSCS_SensorLocationList[index];
      index++;
    }
  }
  msg_conf.Length = length;
  msg_conf.p_Payload = a_RSCS_UpdateCharData;
  retval = RSCS_UpdateValue(RSCS_SCP, &msg_conf);
  
  return retval;
} /* end of RSCS_Scp_SendResponseCode() */
/* USER CODE END FD_LOCAL_FUNCTIONS*/

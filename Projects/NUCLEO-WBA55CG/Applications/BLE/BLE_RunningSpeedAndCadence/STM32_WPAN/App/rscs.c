/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service2.c
  * @author  MCD Application Team
  * @brief   service2 definition.
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
#include "common_blesvc.h"
#include "rscs.h"

/* USER CODE BEGIN Includes */
#include "rscs_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef struct{
  uint16_t  RscsSvcHdle;                  /**< Rscs Service Handle */
  uint16_t  RscmCharHdle;                  /**< RSCM Characteristic Handle */
  uint16_t  RscfCharHdle;                  /**< RSCF Characteristic Handle */
  uint16_t  SelCharHdle;                  /**< SEL Characteristic Handle */
  uint16_t  ScpCharHdle;                  /**< SCP Characteristic Handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/

/* USER CODE END Context */
}RSCS_Context_t;

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static const uint16_t SizeRscm = 10;
static const uint16_t SizeRscf = 2;
static const uint16_t SizeSel = 1;
static const uint16_t SizeScp = 19;

static RSCS_Context_t RSCS_Context;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t RSCS_EventHandler(void *p_pckt);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  p_Event: Address of the buffer holding the p_Event
 * @retval Ack: Return whether the p_Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t RSCS_EventHandler(void *p_Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *p_event_pckt;
  evt_blecore_aci *p_blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *p_attribute_modified;
  aci_gatt_write_permit_req_event_rp0   *p_write_perm_req;
  RSCS_NotificationEvt_t                 notification;
  /* USER CODE BEGIN Service2_EventHandler_1 */
  uint8_t error_code;
  /* USER CODE END Service2_EventHandler_1 */

  return_value = SVCCTL_EvtNotAck;
  p_event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)p_Event)->data);

  switch(p_event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      p_blecore_evt = (evt_blecore_aci*)p_event_pckt->data;
      switch(p_blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
        {
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          p_attribute_modified = (aci_gatt_attribute_modified_event_rp0*)p_blecore_evt->data;
          notification.ConnectionHandle         = p_attribute_modified->Connection_Handle;
          notification.AttributeHandle          = p_attribute_modified->Attr_Handle;
          notification.DataTransfered.Length    = p_attribute_modified->Attr_Data_Length;
          notification.DataTransfered.p_Payload = p_attribute_modified->Attr_Data;
          if(p_attribute_modified->Attr_Handle == (RSCS_Context.RscmCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN Service2_Char_1 */

            /* USER CODE END Service2_Char_1 */
            switch(p_attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN Service2_Char_1_attribute_modified */

              /* USER CODE END Service2_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN Service2_Char_1_Disabled_BEGIN */

                /* USER CODE END Service2_Char_1_Disabled_BEGIN */
                notification.EvtOpcode = RSCS_RSCM_NOTIFY_DISABLED_EVT;
                RSCS_Notification(&notification);
                /* USER CODE BEGIN Service2_Char_1_Disabled_END */

                /* USER CODE END Service2_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN Service2_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END Service2_Char_1_COMSVC_Notification_BEGIN */
                notification.EvtOpcode = RSCS_RSCM_NOTIFY_ENABLED_EVT;
                RSCS_Notification(&notification);
                /* USER CODE BEGIN Service2_Char_1_COMSVC_Notification_END */

                /* USER CODE END Service2_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN Service2_Char_1_default */

                /* USER CODE END Service2_Char_1_default */
                break;
            }
          }  /* if(p_attribute_modified->Attr_Handle == (RSCS_Context.RscmCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if(p_attribute_modified->Attr_Handle == (RSCS_Context.ScpCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN Service2_Char_4 */

            /* USER CODE END Service2_Char_4 */

            switch(p_attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN Service2_Char_4_attribute_modified */

              /* USER CODE END Service2_Char_4_attribute_modified */

              /* Disabled Indication management */
              case (!(COMSVC_Indication)):
                /* USER CODE BEGIN Service2_Char_4_Disabled_BEGIN */

                /* USER CODE END Service2_Char_4_Disabled_BEGIN */
                notification.EvtOpcode = RSCS_SCP_INDICATE_DISABLED_EVT;
                RSCS_Notification(&notification);
                /* USER CODE BEGIN Service2_Char_4_Disabled_END */

                /* USER CODE END Service2_Char_4_Disabled_END */
                break;

              /* Enabled Indication management */
              case COMSVC_Indication:
                /* USER CODE BEGIN Service2_Char_4_COMSVC_Indication_BEGIN */

                /* USER CODE END Service2_Char_4_COMSVC_Indication_BEGIN */
                notification.EvtOpcode = RSCS_SCP_INDICATE_ENABLED_EVT;
                RSCS_Notification(&notification);
                /* USER CODE BEGIN Service2_Char_4_COMSVC_Indication_END */

                /* USER CODE END Service2_Char_4_COMSVC_Indication_END */
                break;

              default:
                /* USER CODE BEGIN Service2_Char_4_default */

                /* USER CODE END Service2_Char_4_default */
                break;
            }
          }  /* if(p_attribute_modified->Attr_Handle == (RSCS_Context.SCPHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if(p_attribute_modified->Attr_Handle == (RSCS_Context.ScpCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;

            notification.EvtOpcode = RSCS_SCP_WRITE_EVT;
            /* USER CODE BEGIN Service2_Char_4_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            /* USER CODE END Service2_Char_4_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            RSCS_Notification(&notification);
          } /* if(p_attribute_modified->Attr_Handle == (RSCS_Context.ScpCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;/* ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
        }
        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
        {
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;/* ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE */
        }
        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
        {
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          p_write_perm_req = (aci_gatt_write_permit_req_event_rp0*)p_blecore_evt->data;
          if(p_write_perm_req->Attribute_Handle == (RSCS_Context.ScpCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN Service2_Char_4_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE */
            LOG_INFO_APP("ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE SC Control Point, data length: %d\r\n", p_write_perm_req->Data_Length);
            error_code = RSCS_APP_SCCPCheckRequestValid(p_write_perm_req->Data, p_write_perm_req->Data_Length);
            if (error_code == 0x00)
            {
              aci_gatt_write_resp(p_write_perm_req->Connection_Handle,
                                  p_write_perm_req->Attribute_Handle,
                                  0x00,
                                  0x00,
                                  p_write_perm_req->Data_Length,
                                  p_write_perm_req->Data);
          
              LOG_INFO_APP("ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE C Control Point >>> PERMITTED\r\n");
              RSCS_APP_SCCPRequestHandler(p_write_perm_req->Data, p_write_perm_req->Data_Length);
            }
            else if(error_code == 0x81)
            {
              aci_gatt_write_resp(p_write_perm_req->Connection_Handle,
                                  p_write_perm_req->Attribute_Handle,
                                  0x01,
                                  error_code,
                                  p_write_perm_req->Data_Length,
                                  p_write_perm_req->Data);
              LOG_INFO_APP("ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE SC Control Point >>> NOT PERMITTED CCCD disabled\r\n");
            }
            else if(error_code == 0x80)
            {
              aci_gatt_write_resp(p_write_perm_req->Connection_Handle,
                                  p_write_perm_req->Attribute_Handle,
                                  0x01,
                                  error_code,
                                  p_write_perm_req->Data_Length,
                                  p_write_perm_req->Data);
              LOG_INFO_APP("ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE SC Control Point >>> NOT PERMITTED SCP in progress\r\n");
            }
            /*USER CODE END Service2_Char_4_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE*/
          } /*if(p_write_perm_req->Attribute_Handle == (RSCS_Context.ScpCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;/* ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE */
        }
        case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
        {
          aci_gatt_tx_pool_available_event_rp0 *p_tx_pool_available_event;
          p_tx_pool_available_event = (aci_gatt_tx_pool_available_event_rp0 *) p_blecore_evt->data;
          UNUSED(p_tx_pool_available_event);

          /* USER CODE BEGIN ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE */

          /* USER CODE END ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE */
          break;/* ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE*/
        }
        case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
        {
          aci_att_exchange_mtu_resp_event_rp0 *p_exchange_mtu;
          p_exchange_mtu = (aci_att_exchange_mtu_resp_event_rp0 *)  p_blecore_evt->data;
          UNUSED(p_exchange_mtu);

          /* USER CODE BEGIN ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE */

          /* USER CODE END ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE */
          break;/* ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE */
        }
        /* USER CODE BEGIN BLECORE_EVT */
        case ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE:
        {
          LOG_INFO_APP("ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE IDS_SRCP\r\n");
          RSCS_APP_SCCPIndicationReceived();
          
          break;/* ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE */
        }
        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Service2_EventHandler_2 */

  /* USER CODE END Service2_EventHandler_2 */

  return(return_value);
}/* end RSCS_EventHandler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void RSCS_Init(void)
{
  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t max_attr_record;

  /* USER CODE BEGIN SVCCTL_InitService2Svc_1 */

  /* USER CODE END SVCCTL_InitService2Svc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(RSCS_EventHandler);

  /**
   * RSCS
   *
   * Max_Attribute_Records = 1 + 2*4 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for RSCS +
   *                                2 for RSCM +
   *                                2 for RSCF +
   *                                2 for SEL +
   *                                2 for SCP +
   *                                1 for RSCM configuration descriptor +
   *                                1 for SCP configuration descriptor +
   *                              = 11
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 11;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  uuid.Char_UUID_16 = 0x1814;
  ret = aci_gatt_add_service(UUID_TYPE_16,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(RSCS_Context.RscsSvcHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    LOG_INFO_APP("  Fail   : aci_gatt_add_service command: RSCS, error code: 0x%x \n\r", ret);
  }
  else
  {
    LOG_INFO_APP("  Success: aci_gatt_add_service command: RSCS \n\r");
  }

  /**
   * RSCM
   */
  uuid.Char_UUID_16 = 0x2a53;
  ret = aci_gatt_add_char(RSCS_Context.RscsSvcHdle,
                          UUID_TYPE_16,
                          (Char_UUID_t *) &uuid,
                          SizeRscm,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(RSCS_Context.RscmCharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    LOG_INFO_APP("  Fail   : aci_gatt_add_char command   : RSCM, error code: 0x%2X\n", ret);
  }
  else
  {
    LOG_INFO_APP("  Success: aci_gatt_add_char command   : RSCM\n");
  }

  /* USER CODE BEGIN SVCCTL_InitService2Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_InitService2Char1 */

  /**
   * RSCF
   */
  uuid.Char_UUID_16 = 0x2a54;
  ret = aci_gatt_add_char(RSCS_Context.RscsSvcHdle,
                          UUID_TYPE_16,
                          (Char_UUID_t *) &uuid,
                          SizeRscf,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(RSCS_Context.RscfCharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    LOG_INFO_APP("  Fail   : aci_gatt_add_char command   : RSCF, error code: 0x%2X\n", ret);
  }
  else
  {
    LOG_INFO_APP("  Success: aci_gatt_add_char command   : RSCF\n");
  }

  /* USER CODE BEGIN SVCCTL_InitService2Char2 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_InitService2Char2 */

  /**
   * SEL
   */
  uuid.Char_UUID_16 = 0x2a5d;
  ret = aci_gatt_add_char(RSCS_Context.RscsSvcHdle,
                          UUID_TYPE_16,
                          (Char_UUID_t *) &uuid,
                          SizeSel,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(RSCS_Context.SelCharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    LOG_INFO_APP("  Fail   : aci_gatt_add_char command   : SEL, error code: 0x%2X\n", ret);
  }
  else
  {
    LOG_INFO_APP("  Success: aci_gatt_add_char command   : SEL\n");
  }

  /* USER CODE BEGIN SVCCTL_InitService2Char3 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_InitService2Char3 */

  /**
   * SCP
   */
  uuid.Char_UUID_16 = 0x2a55;
  ret = aci_gatt_add_char(RSCS_Context.RscsSvcHdle,
                          UUID_TYPE_16,
                          (Char_UUID_t *) &uuid,
                          SizeScp,
                          CHAR_PROP_WRITE | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(RSCS_Context.ScpCharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    LOG_INFO_APP("  Fail   : aci_gatt_add_char command   : SCP, error code: 0x%2X\n", ret);
  }
  else
  {
    LOG_INFO_APP("  Success: aci_gatt_add_char command   : SCP\n");
  }

  /* USER CODE BEGIN SVCCTL_InitService2Char4 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_InitService2Char4 */

  /* USER CODE BEGIN SVCCTL_InitService2Svc_2 */

  /* USER CODE END SVCCTL_InitService2Svc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus RSCS_UpdateValue(RSCS_CharOpcode_t CharOpcode, RSCS_Data_t *pData)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Service2_App_Update_Char_1 */

  /* USER CODE END Service2_App_Update_Char_1 */

  switch(CharOpcode)
  {
    case RSCS_RSCM:
      ret = aci_gatt_update_char_value(RSCS_Context.RscsSvcHdle,
                                       RSCS_Context.RscmCharHdle,
                                       0, /* charValOffset */
                                       pData->Length, /* charValueLen */
                                       (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        LOG_INFO_APP("  Fail   : aci_gatt_update_char_value RSCM command, error code: 0x%2X\n", ret);
      }
      else
      {
        LOG_INFO_APP("  Success: aci_gatt_update_char_value RSCM command\n");
      }
      /* USER CODE BEGIN Service2_Char_Value_1*/

      /* USER CODE END Service2_Char_Value_1*/
      break;

    case RSCS_RSCF:
      ret = aci_gatt_update_char_value(RSCS_Context.RscsSvcHdle,
                                       RSCS_Context.RscfCharHdle,
                                       0, /* charValOffset */
                                       pData->Length, /* charValueLen */
                                       (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        LOG_INFO_APP("  Fail   : aci_gatt_update_char_value RSCF command, error code: 0x%2X\n", ret);
      }
      else
      {
        LOG_INFO_APP("  Success: aci_gatt_update_char_value RSCF command\n");
      }
      /* USER CODE BEGIN Service2_Char_Value_2*/

      /* USER CODE END Service2_Char_Value_2*/
      break;

    case RSCS_SEL:
      ret = aci_gatt_update_char_value(RSCS_Context.RscsSvcHdle,
                                       RSCS_Context.SelCharHdle,
                                       0, /* charValOffset */
                                       pData->Length, /* charValueLen */
                                       (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        LOG_INFO_APP("  Fail   : aci_gatt_update_char_value SEL command, error code: 0x%2X\n", ret);
      }
      else
      {
        LOG_INFO_APP("  Success: aci_gatt_update_char_value SEL command\n");
      }
      /* USER CODE BEGIN Service2_Char_Value_3*/

      /* USER CODE END Service2_Char_Value_3*/
      break;

    case RSCS_SCP:
      ret = aci_gatt_update_char_value(RSCS_Context.RscsSvcHdle,
                                       RSCS_Context.ScpCharHdle,
                                       0, /* charValOffset */
                                       pData->Length, /* charValueLen */
                                       (uint8_t *)pData->p_Payload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        LOG_INFO_APP("  Fail   : aci_gatt_update_char_value SCP command, error code: 0x%2X\n", ret);
      }
      else
      {
        LOG_INFO_APP("  Success: aci_gatt_update_char_value SCP command\n");
      }
      /* USER CODE BEGIN Service2_Char_Value_4*/

      /* USER CODE END Service2_Char_Value_4*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Service2_App_Update_Char_2 */

  /* USER CODE END Service2_App_Update_Char_2 */

  return ret;
}

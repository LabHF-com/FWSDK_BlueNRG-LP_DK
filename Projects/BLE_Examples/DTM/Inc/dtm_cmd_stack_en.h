
/**
  ******************************************************************************
  * @file    dtm_cmd_stack_en.h
  * @author  AMS - RF Application team
  * @version V1.0.0
  * @date    19 July 2022
  * @brief   List of macros used to configure the DTM ACI/HCI command table
  *          with modularity
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT STMicroelectronics</center></h2>
  ******************************************************************************
  */

#ifndef _DTM_CMD_STACK_EN_H_
#define _DTM_CMD_STACK_EN_H_

#include "stack_user_cfg.h"

/* Command support enabling macros */
#define ACI_GAP_ADD_DEVICE_TO_PERIODIC_ADVERTISER_LIST_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_ALLOW_REBOND_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_CLEAR_ADVERTISING_SETS_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define ACI_GAP_CLEAR_PERIODIC_ADVERTISER_LIST_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_CLEAR_SECURITY_DB_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_CREATE_CONNECTION_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_DISCOVER_NAME_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_GET_BONDED_DEVICES_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_GET_OOB_DATA_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_GET_SECURITY_LEVEL_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_IS_DEVICE_BONDED_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_NUMERIC_COMPARISON_VALUE_CONFIRM_YESNO_ENABLED\
    (SECURE_CONNECTIONS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_PASS_KEY_RESP_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_PASSKEY_INPUT_ENABLED\
    (SECURE_CONNECTIONS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_PERIODIC_ADVERTISING_CREATE_SYNC_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_PERIODIC_ADVERTISING_CREATE_SYNC_CANCEL_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_PERIODIC_ADVERTISING_SET_INFO_TRANSFER_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_PERIODIC_ADVERTISING_SYNC_TRANSFER_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_PERIODIC_ADVERTISING_TERMINATE_SYNC_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_READ_PERIODIC_ADVERTISER_LIST_SIZE_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_REMOVE_ADVERTISING_SET_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define ACI_GAP_REMOVE_BONDED_DEVICE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_REMOVE_DEVICE_FROM_PERIODIC_ADVERTISING_LIST_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_SEND_PAIRING_REQ_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_SET_AUTHENTICATION_REQUIREMENT_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_SET_CONNECTION_CONFIGURATION_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_SET_DEFAULT_PERIODIC_ADVERTISING_SYNC_TRANSFER_PARAMETERS_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_SET_IO_CAPABILITY_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_SET_OOB_DATA_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_SET_PERIODIC_ADVERTISING_CONFIGURATION_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_SET_PERIODIC_ADVERTISING_DATA_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_SET_PERIODIC_ADVERTISING_ENABLE_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_SET_PERIODIC_ADVERTISING_RECEIVE_ENABLE_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define ACI_GAP_SET_PERIODIC_ADVERTISING_SYNC_TRANSFER_PARAMETERS_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_SET_SCAN_CONFIGURATION_ENABLED\
    (CONTROLLER_MASTER_ENABLED)
#define ACI_GAP_SLAVE_SECURITY_REQ_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_START_CONNECTION_UPDATE_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_GAP_START_PROCEDURE_ENABLED\
    (CONTROLLER_MASTER_ENABLED)
#define ACI_GAP_TERMINATE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GAP_TERMINATE_PROC_ENABLED\
    (CONTROLLER_MASTER_ENABLED)
#define ACI_GATT_CLT_CONFIRM_INDICATION_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_DISC_ALL_CHAR_DESC_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_DISC_ALL_CHAR_OF_SERVICE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_DISC_ALL_PRIMARY_SERVICES_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_DISC_CHAR_BY_UUID_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_DISC_PRIMARY_SERVICE_BY_UUID_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_EXCHANGE_CONFIG_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_EXECUTE_WRITE_REQ_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_FIND_INCLUDED_SERVICES_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_PREPARE_WRITE_REQ_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_READ_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_READ_LONG_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_READ_MULTIPLE_CHAR_VALUE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_READ_MULTIPLE_VAR_LEN_CHAR_VALUE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_READ_USING_CHAR_UUID_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_SIGNED_WRITE_WITHOUT_RESP_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_WRITE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_WRITE_CHAR_RELIABLE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_WRITE_LONG_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_CLT_WRITE_WITHOUT_RESP_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_EATT_CLT_CONFIRM_INDICATION_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_DISC_ALL_CHAR_DESC_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_DISC_ALL_CHAR_OF_SERVICE_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_DISC_ALL_PRIMARY_SERVICES_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_DISC_CHAR_BY_UUID_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_DISC_PRIMARY_SERVICE_BY_UUID_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_EXECUTE_WRITE_REQ_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_FIND_INCLUDED_SERVICES_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_PREPARE_WRITE_REQ_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_READ_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_READ_LONG_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_READ_MULTIPLE_CHAR_VALUE_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_READ_MULTIPLE_VAR_LEN_CHAR_VALUE_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_READ_USING_CHAR_UUID_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_WRITE_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_WRITE_CHAR_RELIABLE_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_WRITE_LONG_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_CLT_WRITE_WITHOUT_RESP_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_SRV_INIT_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_SRV_MULTI_NOTIFY_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_SRV_NOTIFY_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_EATT_SRV_RESP_ENABLED\
    (CONNECTION_ENABLED &\
     EATT_ENABLED)
#define ACI_GATT_SET_EVENT_MASK_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_ADD_CHAR_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_ADD_CHAR_DESC_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_ADD_SERVICE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_GET_CHAR_DECL_HANDLE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_GET_DESCRIPTOR_HANDLE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_GET_INCLUDE_SERVICE_HANDLE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_GET_SERVICE_HANDLE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_INCLUDE_SERVICE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_INIT_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_MULTI_NOTIFY_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_NOTIFY_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_READ_HANDLE_VALUE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_READ_MULTIPLE_INSTANCE_HANDLE_VALUE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_RESP_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_RM_CHAR_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_RM_INCLUDE_SERVICE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_RM_SERVICE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_GATT_SRV_WRITE_MULTIPLE_INSTANCE_HANDLE_VALUE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_HAL_GET_ANCHOR_POINT_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_HAL_PERIPHERAL_LATENCY_ENABLE_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_HAL_SET_LE_POWER_CONTROL_ENABLED\
    (CONTROLLER_POWER_CONTROL_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_CFC_CONNECTION_REQ_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_CFC_CONNECTION_RESP_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_REQ_ENABLED\
    (CONNECTION_ENABLED)
#define ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_RESP_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_DISCONNECT_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_ECFC_CONNECTION_REQ_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_ECFC_CONNECTION_RESP_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_ECFC_RECONFIGURE_REQ_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_ECFC_RECONFIGURE_RESP_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_EXTRACT_SDU_DATA_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_SEND_FLOW_CONTROL_CREDITS_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define ACI_L2CAP_TRANSMIT_SDU_DATA_ENABLED\
    (L2CAP_COS_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_DISCONNECT_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_LE_ACCEPT_CIS_REQUEST_ENABLED\
    (CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_ADD_DEVICE_TO_PERIODIC_ADVERTISER_LIST_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_BIG_CREATE_SYNC_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONTROLLER_BIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_BIG_TERMINATE_SYNC_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONTROLLER_BIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_CLEAR_ADVERTISING_SETS_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_CLEAR_PERIODIC_ADVERTISER_LIST_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_CLEAR_RESOLVING_LIST_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_CONNECTION_CTE_REQUEST_ENABLE_ENABLED\
    (CONTROLLER_CTE_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_CONNECTION_CTE_RESPONSE_ENABLE_ENABLED\
    (CONTROLLER_CTE_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_CONNECTION_UPDATE_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_CREATE_BIG_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONTROLLER_BIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_CREATE_BIG_TEST_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONTROLLER_BIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_CREATE_CIS_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_CREATE_CONNECTION_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_CREATE_CONNECTION_CANCEL_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_ENHANCED_READ_TRANSMIT_POWER_LEVEL_ENABLED\
    (CONTROLLER_POWER_CONTROL_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_ENHANCED_RECEIVER_TEST_ENABLED\
    (CONTROLLER_2M_CODED_PHY_ENABLED)
#define HCI_LE_ENHANCED_TRANSMITTER_TEST_ENABLED\
    (CONTROLLER_2M_CODED_PHY_ENABLED)
#define HCI_LE_EXTENDED_CREATE_CONNECTION_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_ISO_READ_TEST_COUNTERS_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_ISO_RECEIVE_TEST_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_ISO_TEST_END_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_ISO_TRANSMIT_TEST_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_LONG_TERM_KEY_REQUEST_REPLY_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_LE_PERIODIC_ADVERTISING_CREATE_SYNC_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_PERIODIC_ADVERTISING_CREATE_SYNC_CANCEL_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_PERIODIC_ADVERTISING_SET_INFO_TRANSFER_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_PERIODIC_ADVERTISING_SYNC_TRANSFER_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_PERIODIC_ADVERTISING_TERMINATE_SYNC_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_READ_ANTENNA_INFORMATION_ENABLED\
    (CONTROLLER_CTE_ENABLED)
#define HCI_LE_READ_BUFFER_SIZE_V2_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_READ_CHANNEL_MAP_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_LE_READ_ISO_LINK_QUALITY_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_READ_ISO_TX_SYNC_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_READ_MAXIMUM_DATA_LENGTH_ENABLED\
    (CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_READ_PEER_RESOLVABLE_ADDRESS_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_READ_PERIODIC_ADVERTISER_LIST_SIZE_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_READ_PHY_ENABLED\
    (CONTROLLER_2M_CODED_PHY_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_READ_REMOTE_TRANSMIT_POWER_LEVEL_ENABLED\
    (CONTROLLER_POWER_CONTROL_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_READ_REMOTE_USED_FEATURES_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_LE_READ_RESOLVING_LIST_SIZE_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH_ENABLED\
    (CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_RECEIVER_TEST_V3_ENABLED\
    (CONTROLLER_CTE_ENABLED)
#define HCI_LE_REJECT_CIS_REQUEST_ENABLED\
    (CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_REMOVE_ADVERTISING_SET_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_REMOVE_CIG_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_REMOVE_DEVICE_FROM_PERIODIC_ADVERTISING_LIST_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_REMOVE_ISO_DATA_PATH_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_REQUEST_PEER_SCA_ENABLED\
    (CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_SET_CIG_PARAMETERS_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_SET_CIG_PARAMETERS_TEST_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_SET_CONNECTION_CTE_RECEIVE_PARAMETERS_ENABLED\
    (CONTROLLER_CTE_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SET_CONNECTION_CTE_TRANSMIT_PARAMETERS_ENABLED\
    (CONTROLLER_CTE_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SET_CONNECTIONLESS_CTE_TRANSMIT_ENABLE_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONTROLLER_CTE_ENABLED)
#define HCI_LE_SET_CONNECTIONLESS_CTE_TRANSMIT_PARAMETERS_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONTROLLER_CTE_ENABLED)
#define HCI_LE_SET_CONNECTIONLESS_IQ_SAMPLING_ENABLE_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONTROLLER_CTE_ENABLED)
#define HCI_LE_SET_DATA_LENGTH_ENABLED\
    (CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SET_DATA_RELATED_ADDRESS_CHANGES_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_SET_DEFAULT_PERIODIC_ADVERTISING_SYNC_TRANSFER_PARAMETERS_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SET_DEFAULT_PHY_ENABLED\
    (CONTROLLER_2M_CODED_PHY_ENABLED)
#define HCI_LE_SET_DEFAULT_SUBRATE_ENABLED\
    (CONNECTION_ENABLED &\
     CONNECTION_SUBRATING_ENABLED)
#define HCI_LE_SET_EXTENDED_ADVERTISING_ENABLE_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_SET_EXTENDED_ADVERTISING_PARAMETERS_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_SET_EXTENDED_SCAN_ENABLE_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_SET_EXTENDED_SCAN_PARAMETERS_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION_ENABLED\
    (CONTROLLER_MASTER_ENABLED |\
     CONTROLLER_EXT_ADV_SCAN_ENABLED |\
     CONTROLLER_CHAN_CLASS_ENABLED)
#define HCI_LE_SET_HOST_FEATURE_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_LE_SET_PATH_LOSS_REPORTING_ENABLE_ENABLED\
    (CONTROLLER_POWER_CONTROL_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SET_PATH_LOSS_REPORTING_PARAMETERS_ENABLED\
    (CONTROLLER_POWER_CONTROL_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SET_PERIODIC_ADVERTISING_ENABLE_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_SET_PERIODIC_ADVERTISING_PARAMETERS_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED)
#define HCI_LE_SET_PERIODIC_ADVERTISING_RECEIVE_ENABLE_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED)
#define HCI_LE_SET_PERIODIC_ADVERTISING_SYNC_TRANSFER_PARAMETERS_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SET_PHY_ENABLED\
    (CONTROLLER_2M_CODED_PHY_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SET_PRIVACY_MODE_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT_ENABLED\
    (CONTROLLER_PRIVACY_ENABLED)
#define HCI_LE_SET_SCAN_ENABLE_ENABLED\
    (CONTROLLER_MASTER_ENABLED)
#define HCI_LE_SET_SCAN_PARAMETERS_ENABLED\
    (CONTROLLER_MASTER_ENABLED)
#define HCI_LE_SET_TRANSMIT_POWER_REPORTING_ENABLE_ENABLED\
    (CONTROLLER_POWER_CONTROL_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SETUP_ISO_DATA_PATH_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_LE_START_ENCRYPTION_ENABLED\
    (CONTROLLER_MASTER_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_LE_SUBRATE_REQUEST_ENABLED\
    (CONNECTION_ENABLED &\
     CONNECTION_SUBRATING_ENABLED)
#define HCI_LE_TERMINATE_BIG_ENABLED\
    (CONTROLLER_EXT_ADV_SCAN_ENABLED &\
     CONTROLLER_PERIODIC_ADV_ENABLED &\
     CONTROLLER_BIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_LE_TRANSMITTER_TEST_V3_ENABLED\
    (CONTROLLER_CTE_ENABLED)
#define HCI_LE_TRANSMITTER_TEST_V4_ENABLED\
    (CONTROLLER_CTE_ENABLED |\
     CONTROLLER_POWER_CONTROL_ENABLED)
#define HCI_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH_ENABLED\
    (CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED &\
     CONNECTION_ENABLED)
#define HCI_READ_AFH_CHANNEL_ASSESSMENT_MODE_ENABLED\
    (CONNECTION_ENABLED &\
     CONTROLLER_CHAN_CLASS_ENABLED)
#define HCI_READ_AUTHENTICATED_PAYLOAD_TIMEOUT_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_READ_CONNECTION_ACCEPT_TIMEOUT_ENABLED\
    (CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)
#define HCI_READ_REMOTE_VERSION_INFORMATION_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_READ_RSSI_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_READ_TRANSMIT_POWER_LEVEL_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_TX_ISO_DATA_ENABLED\
    (CONTROLLER_ISO_ENABLED)
#define HCI_WRITE_AFH_CHANNEL_ASSESSMENT_MODE_ENABLED\
    (CONNECTION_ENABLED &\
     CONTROLLER_CHAN_CLASS_ENABLED)
#define HCI_WRITE_AUTHENTICATED_PAYLOAD_TIMEOUT_ENABLED\
    (CONNECTION_ENABLED)
#define HCI_WRITE_CONNECTION_ACCEPT_TIMEOUT_ENABLED\
    (CONNECTION_ENABLED &\
     CONTROLLER_CIS_ENABLED &\
     CONTROLLER_ISO_ENABLED)

#endif /* _DTM_CMD_STACK_EN_H_ */

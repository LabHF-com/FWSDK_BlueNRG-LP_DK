/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : gatt_db.c
* Author             : AMS - VMA division
* Version            : V1.0.0
* Date               : 25-August-2016
* Description        : Functions to build GATT DB and handle GATT events.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "master_basic_profile.h"
#include "master_config.h"
#include "gatt_db.h"

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Primary Service UUID expected from Sensor demo peripheral */
uint8_t GENERIC_ACCESS_PROFILE_UUID[]={0x00, 0x18};
uint8_t GENERIC_ATTRIBUTE_PROFILE_UUID[]={0x01, 0x18};
uint8_t ACCELERATION_SERVICE_UUID[]={0x1B,0xC5,0xD5,0xA5,0x02,0x00,0xB4,0x9A,0xE1,0x11,0x3A,0xCF,0x80,0x6E,0x36,0x02};
uint8_t ENVIRONMENTAL_SERVICE_UUID[]={0x1B,0xC5,0xD5,0xA5,0x02,0x00,0xD0,0x82,0xE2,0x11,0x77,0xE4,0x40,0x1A,0x82,0x42};

/*  UUID expected from Sensor demo peripheral */
uint8_t FREE_FALL_CHAR_UUID[]={0x1B,0xC5,0xD5,0xA5,0x02,0x00,0xFC,0x8F,0xE1,0x11,0x4A,0xCF,0xA0,0x78,0x3E,0xE2};
uint8_t ACCELERATION_CHAR_UUID[]={0x1B,0xC5,0xD5,0xA5,0x02,0x00,0x36,0xAC,0xE1,0x11,0x4B,0xCF,0x80,0x1B,0x0A,0x34};
uint8_t TEMPERATURE_CHAR_UUID[]={0x1B,0xC5,0xD5,0xA5,0x02,0x00,0xE3,0xA9,0xE2,0x11,0x77,0xE4,0x20,0x55,0x2E,0xA3};

/* Sensor for BlueMS service/characteristics  UUIDs */
uint8_t HW_SENS_W2ST_SERVICE_UUID[]={0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x00};

uint8_t ENVIRONMENTAL_W2ST_CHAR_UUID[]={0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x14,0x00}; // 0x14: (0x04|0x10) [0x04: One Temperature value; 0x10: Pressure value ]

/* 0x00 c0 00 00  = 00 80 00 00 (acc) + 00 40 00 00 (gyro); [no + 00 20 00 00 (mag),  since magnetometer is not supported on BlueNRG-1/2/LP eval kits] */
uint8_t ACC_GYRO_MAG_W2ST_CHAR_UUID[]={0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0xC0,0x00};


masterRoleContextType masterContext;

static connDevType connParam; //TBR

static volatile uint8_t is_blue_ms=0; 
/*******************************************************************************
* Function Name  : deviceInit
* Description    : Device init
* Input          : None
* Return         : Status of the call
*******************************************************************************/
uint8_t deviceInit(void)
{
  uint8_t ret;
  uint8_t bdaddr[] = MASTER_PUBLIC_ADDRESS;
  uint8_t devName[18] = "SensorDemo Central";
  initDevType param;

  /* Master context init */
  masterContext.startDeviceDisc = FALSE;
  masterContext.findCharacOfService = FALSE;
  masterContext.genAttFlag = FALSE;
  masterContext.accFlag = FALSE;
  masterContext.envFlag = FALSE;
  masterContext.enableNotif = FALSE;
  masterContext.connHandle = 0xFFFF;

  /* Master device init */
  memcpy(param.public_addr, bdaddr, 6);
  param.device_name_len = 18;
  param.device_name = devName;
  param.txPowerLevel = 25; // 0 dBm
  param.Identity_Address_Type = 0; 
  param.Privacy_Type = 0; 
  ret = Master_Init(&param);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Master_Init() status: 0x%02x\r\n", ret);
  }
  return ret;
}

/*******************************************************************************
* Function Name  : deviceScanConfiguration
* Description    : Configure the scanning configuration
* Input          : None
* Return         : Status of the call
*******************************************************************************/
uint8_t deviceScanConfiguration(void)
{
  uint8_t ret;
  
  devDiscoveryType devDiscParam;
  
  devDiscParam.Scanning_PHY = LE_PHY; 
  devDiscParam.scanInterval = LIM_DISC_SCAN_INT;
  devDiscParam.scanWindow = LIM_DISC_SCAN_WIND;
  devDiscParam.Filter_Duplicates = FILTER_DUPLICATES; 
  devDiscParam.Scanning_Filter_Policy = SCAN_FILTER_POLICY;  
  devDiscParam.Scan_Type = SCAN_TYPE;  
  
  ret = Master_DeviceScanConfiguration(&devDiscParam);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Master_DeviceScanConfiguration() status: 0x%02x\r\n", ret);
  }
    else {
    PRINTF("****Master_DeviceScanConfiguration() OK\r\n");
  }
  return ret;
}

/*******************************************************************************
* Function Name  : deviceDiscovery
* Description    : Starts the device discovery proceudre
* Input          : None
* Return         : Status of the call
*******************************************************************************/
uint8_t deviceDiscovery(void)
{
  uint8_t ret;
  
  devDiscoveryType devDiscParam;
  
  devDiscParam.procedure = GENERAL_DISCOVERY_PROCEDURE;
  devDiscParam.Scanning_PHY = LE_PHY; 
 
  ret = Master_DeviceDiscovery(&devDiscParam);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Master_DeviceDiscovery() status: 0x%02x\r\n", ret);
  } else {
    PRINTF("****Start Device Discovery Procedure\r\n");
  }
  return ret;
}

/*******************************************************************************
* Function Name  : DeviceConnectionConfiguration
* Description    : Configure the connection configuration
* Input          : None
* Return         : Status of the call
*******************************************************************************/
uint8_t deviceConnectionConfiguration(void)
{
  uint8_t ret;
  
  connParam.Initiating_PHY = LE_PHY;
  connParam.procedure = DIRECT_CONNECTION_PROCEDURE;
  connParam.fastScanDuration = FAST_SCAN_DURATION;
  connParam.fastScanInterval = FAST_SCAN_INTERVAL;  
  connParam.fastScanWindow = FAST_SCAN_WINDOW;
  connParam.reducedPowerScanInterval = REDUCED_POWER_SCAN_INTERVAL;
  connParam.reducedPowerScanWindow = REDUCED_POWER_SCAN_WINDOW;
  
  connParam.own_addr_type =0;
  connParam.conn_min_interval = FAST_MIN_CONNECTION_INTERVAL;
  connParam.conn_max_interval = FAST_MAX_CONNECTION_INTERVAL;
  connParam.conn_latency = FAST_CONNECTION_LATENCY;
  connParam.supervision_timeout = SUPERVISION_TIMEOUT;
  connParam.min_conn_length = 5;
  connParam.max_conn_length = 5;
  ret = Master_DeviceConnectionConfiguration(&connParam);
  if (ret != BLE_STATUS_SUCCESS) 
    PRINTF("Master_DeviceConnectionConfiguration() failed with status: 0x%02x\r\n", ret);
  return ret;
}

/*******************************************************************************
* Function Name  : deviceConnection
* Description    : Starts the connection proceudre with the peripheral device
* Input          : None
* Return         : Status of the call
*******************************************************************************/
uint8_t deviceConnection(void)
{
  uint8_t ret;
  
  connParam.peer_addr_type = masterContext.peer_addr_type; 
  memcpy(connParam.peer_addr, masterContext.peer_addr, 6); 
 
  ret = Master_DeviceConnection(&connParam);
  if (ret != BLE_STATUS_SUCCESS) 
    PRINTF("Master_DeviceConnection() failed with status: 0x%02x\r\n", ret);
  return ret;
}

/*******************************************************************************
* Function Name  : primaryServiceFound
* Description    : Function used to print the primary service found
* Input          : 
* Return         : 
*******************************************************************************/
void primaryServiceFound(void)
{
  uint8_t i, index, sizeElement;
  uint8_t uuid[16];

  PRINTF("\r\n************ PRIMAY SERVICE ****************************\r\n");
  PRINTF("Number Primary Service found = %d\r\n", masterContext.numPrimarySer); 
  index = 0;
  for (i=0; i<masterContext.numPrimarySer; i++) { 
    if (masterContext.primarySer[index] == 1)
      sizeElement = 6;
    else
      sizeElement = 20;
    index++;
    PRINTF("%d-- ", i+1);
    memcpy (uuid, &masterContext.primarySer[index+4], (sizeElement-4));
    if (memcmp(uuid, GENERIC_ACCESS_PROFILE_UUID, (sizeElement-4)) == 0) {
      PRINTF("Generic Access Profile\r\n");
    }
    if (memcmp (uuid, GENERIC_ATTRIBUTE_PROFILE_UUID, (sizeElement-4)) == 0) {
      PRINTF("Generic Attribute Profile\r\n");
      masterContext.genAttFlag = TRUE;
      masterContext.findCharacOfService = TRUE;
    }
    if (is_blue_ms) /* Sensor Demo blueMS */
    {
      if (memcmp (uuid, HW_SENS_W2ST_SERVICE_UUID, (sizeElement-4)) == 0) {
        PRINTF("Sensor BlueMS Service\r\n");
      }
    }
    else
    {
      if (memcmp (uuid, ACCELERATION_SERVICE_UUID, (sizeElement-4)) == 0) {
        PRINTF("Acceleration Service\r\n");
      }
      if (memcmp (uuid, ENVIRONMENTAL_SERVICE_UUID, (sizeElement-4)) == 0) {
        PRINTF("Environmental Service\r\n");
      }
    }
    index += sizeElement;
  }
  PRINTF("********************************************************\r\n");
}

/*******************************************************************************
* Function Name  : findCharacOfService
* Description    : Search the characteristics of a service
* Input          : 
* Return         : 
*******************************************************************************/
void findCharcOfService(void)
{
  uint8_t ret, i, index, sizeElement, uuid[16], max_size = 0;
  uint16_t startHandle, endHandle;
  uint8_t *numCharac = NULL, *charac = NULL;

  index = 0;
  for (i=0; i<masterContext.numPrimarySer; i++) { 
    if (masterContext.primarySer[index] == 1)
      sizeElement = 6;
    else
      sizeElement = 20;
    index++;
    memcpy (uuid, &masterContext.primarySer[index+4], (sizeElement-4));
    /* Discovery all the characteristics of the Generic Attribute Profile */
    if ((memcmp(uuid, GENERIC_ATTRIBUTE_PROFILE_UUID, (sizeElement-4)) == 0) && masterContext.genAttFlag) {
      memcpy((uint8_t*)&startHandle, &masterContext.primarySer[index], 2);
      memcpy((uint8_t*)&endHandle, &masterContext.primarySer[index+2], 2);
      numCharac = &masterContext.numCharacGenAttSer;
      charac = masterContext.characGenAttSer;
      max_size = sizeof(masterContext.characGenAttSer);
    }
    
    if (is_blue_ms)
    {
      /* Discovery all the characteristics of the Senso BlueMS Service */
      if ((memcmp(uuid, HW_SENS_W2ST_SERVICE_UUID, (sizeElement-4)) == 0) && masterContext.accFlag) {
        memcpy((uint8_t*)&startHandle, &masterContext.primarySer[index], 2);
        memcpy((uint8_t*)&endHandle, &masterContext.primarySer[index+2], 2);  
        numCharac = &masterContext.numCharacAccSer;
        charac = masterContext.characAccSer;
        max_size = sizeof(masterContext.characAccSer);
      }
    }
    else
    {
      /* Discovery all the characteristics of the Acceleration Service */
      if ((memcmp(uuid, ACCELERATION_SERVICE_UUID, (sizeElement-4)) == 0) && masterContext.accFlag) {
        memcpy((uint8_t*)&startHandle, &masterContext.primarySer[index], 2);
        memcpy((uint8_t*)&endHandle, &masterContext.primarySer[index+2], 2);  
        numCharac = &masterContext.numCharacAccSer;
        charac = masterContext.characAccSer;
        max_size = sizeof(masterContext.characAccSer);
      }

      /* Discovery all the characteristics of the Environmental Service */
      if ((memcmp(uuid, ENVIRONMENTAL_SERVICE_UUID, (sizeElement-4)) == 0) && masterContext.envFlag) {
        memcpy((uint8_t*)&startHandle, &masterContext.primarySer[index], 2);
        memcpy((uint8_t*)&endHandle, &masterContext.primarySer[index+2], 2);  
        numCharac = &masterContext.numCharacEnv;
        charac = masterContext.characEnvSer;
        max_size = sizeof(masterContext.characEnvSer);
      }
    }

    index += sizeElement;
  }
  ret = Master_GetCharacOfService(masterContext.connHandle, startHandle, endHandle,
  				  numCharac, charac, max_size);

  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Master_GetCharacOfService() failed with status 0x%02x\r\n", ret);
  }
}

/*******************************************************************************
* Function Name  : extractCharacInfo
* Description    : Extracts the handle information for all the characteristics found
* Input          : 
* Return         : 
*******************************************************************************/
void extractCharacInfo(void)
{
  uint8_t i;
    
  if (is_blue_ms)
  {
     /* Store in the master context the characteristic handles for the env acceleration */
    if (masterContext.accFlag) {
      for (i=0; i<masterContext.numCharacAccSer; i++) {
        if (memcmp(&masterContext.characAccSer[6+(i*22)], ACC_GYRO_MAG_W2ST_CHAR_UUID, 16) == 0) {
          memcpy((uint8_t*)&masterContext.accHandle, &masterContext.characAccSer[1+(i*22)], 2);
          PRINTF("Sensor BlueMS Acceleration char handle: 0x%02x\r\n", masterContext.accHandle);
        }
        if (memcmp(&masterContext.characAccSer[6+(i*22)], ENVIRONMENTAL_W2ST_CHAR_UUID, 16) == 0) { 
          memcpy((uint8_t*)&masterContext.tempHandle, &masterContext.characAccSer[1+(i*22)], 2);
          PRINTF("Sensor BlueMS Env. char handle: 0x%02x\r\n", masterContext.tempHandle);
        }
      }
    }
  }
  else
  {
    /* Store in the master context the characteristic handles for the acceleration and free fall */
    if (masterContext.accFlag) {
      for (i=0; i<masterContext.numCharacAccSer; i++) {
        if (memcmp(&masterContext.characAccSer[6+(i*22)], ACCELERATION_CHAR_UUID, 16) == 0) {
          memcpy((uint8_t*)&masterContext.accHandle, &masterContext.characAccSer[1+(i*22)], 2);
        }
        if (memcmp(&masterContext.characAccSer[6+(i*22)], FREE_FALL_CHAR_UUID, 16) == 0) {
          memcpy((uint8_t*)&masterContext.freeFallHandle, &masterContext.characAccSer[1+(i*22)], 2);
        }
      }
    }
    
    /* Store in the master context the characteristic handles for the tempereture */
    if (masterContext.envFlag) {
      for (i=0; i<masterContext.numCharacEnv; i++) {
        if (memcmp(&masterContext.characEnvSer[6+(i*22)], TEMPERATURE_CHAR_UUID, 16) == 0) {
          memcpy((uint8_t*)&masterContext.tempHandle, &masterContext.characEnvSer[1+(i*22)], 2);
        }
      }
    }
  }
}

/*******************************************************************************
* Function Name  : Master_DeviceDiscovery_CB
* Description    : Device discovery callback from master basic profile library
* Input          : 
* Return         : 
*******************************************************************************/
void Master_DeviceDiscovery_CB(uint8_t *status, uint8_t *addr_type, uint8_t *addr, 
			       uint8_t *data_length, uint8_t *data,
			       int8_t *RSSI)
{
  uint8_t i;

  if (*status == DEVICE_DISCOVERED) {
    PRINTF("ADDR Type = %d\r\n", *addr_type);
    PRINTF("ADDR = 0x");
    for (i=0; i<6; i++)
      PRINTF("%02x", addr[i]);
    PRINTF("\r\n");
    PRINTF("AD DATA (Length=%d):\r\n", *data_length);
    for (i=0; i<*data_length; i++)
      PRINTF("%02x ", data[i]);
    PRINTF("\r\n");
    PRINTF("RSSI = 0x%02x\r\n", *RSSI);
    
    /* If the device found is a BlueNRG try to connect */
    if (((!memcmp(&data[8], "BlueNRG", 7)) && data[16] == 0xFF)|| ((!memcmp(&data[8], "BlueNRGLP", 9)) && data[18] == 0xFF)){ 
      is_blue_ms = 1; 
      PRINTF("Sensor BlueMS device found\r\n");
      masterContext.peer_addr_type = *addr_type;
      memcpy(masterContext.peer_addr, addr, 6);
      if (deviceConnection() != BLE_STATUS_SUCCESS)
	PRINTF("Error during the device connection procedure\r\n");
    }
    /* If the device found is a BlueNRG try to connect */
    else if (!memcmp(&data[5], "BlueNRG", 6)) { 
      is_blue_ms = 0; 
      PRINTF("Sensor device found\r\n");
      masterContext.peer_addr_type = *addr_type;
      memcpy(masterContext.peer_addr, addr, 6);
      if (deviceConnection() != BLE_STATUS_SUCCESS)
	PRINTF("Error during the device connection procedure\r\n");
    }
  }
  if (*status == DEVICE_DISCOVERY_PROCEDURE_ENDED)
    PRINTF("***Device Discovery Procedure ended from the application\r\n");

  if (*status == DEVICE_DISCOVERY_PROCEDURE_TIMEOUT) {
    PRINTF("***Device Discovery Procedure Timeout\r\n");
    if (deviceDiscovery() != BLE_STATUS_SUCCESS) {
      PRINTF("Error during the device discovery procedure\r\n");
    }
  }    
}

/*******************************************************************************
* Function Name  : Master_Connection_CB
* Description    : Connection callback from master basic profile library
* Input          : 
* Return         : 
*******************************************************************************/
void Master_Connection_CB(uint8_t *connection_evt, uint8_t *status, uint16_t *connection_handle, connUpdateParamType *param)
{
  uint8_t ret;

  switch (*connection_evt) {
  case CONNECTION_ESTABLISHED_EVT:
    PRINTF("\r\n****Connection Established (handle 0x%04x)\r\n", *connection_handle);
    /* Start the discovery all primary services procedure */
    if (*status == BLE_STATUS_SUCCESS) {
      masterContext.connHandle = *connection_handle;
      ret = Master_GetPrimaryServices(masterContext.connHandle, &masterContext.numPrimarySer,
				     masterContext.primarySer, sizeof(masterContext.primarySer));
      if (ret != BLE_STATUS_SUCCESS) 
	PRINTF("\r\n****Master_GetPrimaryService() procedure fails with status 0x%02x\r\n", ret);
    }
    break;
  case CONNECTION_FAILED_EVT:
    PRINTF("****CONNECTION FAILED EVENT\r\n");
    break;
  case DISCONNECTION_EVT:
    PRINTF("****DISCONNECTION EVENT\r\n");
    /* Restart Discovery & Connection procedure */
    deviceDiscovery();
    masterContext.connHandle = 0xFFFF;
    break;
  case STOP_CONNECTION_PROCEDURE_EVT:
    PRINTF("****STOP CONNECTION PROCEDURE EVENT\r\n");
    break;
  case CONNECTION_UPDATE_PARAMETER_REQUEST_EVT:
    PRINTF("****CONNECTION UPDATE PARAMETER REQUEST EVENT\r\n");
    break;
  }
}

/*******************************************************************************
* Function Name  : Master_ServiceCharacPeerDiscovery_CB
* Description    : Service and Characteristics discovery callback from master basic profile library
* Input          : 
* Return         : 
*******************************************************************************/
void Master_ServiceCharacPeerDiscovery_CB(uint8_t *procedure, uint8_t *status, uint16_t *connection_handle) 
{
  switch(*procedure) {
  case PRIMARY_SERVICE_DISCOVERY:
    if (*status == BLE_STATUS_SUCCESS) {
      primaryServiceFound();
    }
    break;
  case GET_CHARACTERISTICS_OF_A_SERVICE:
    {
      if (*status == BLE_STATUS_SUCCESS) {
	/* Discovery Acceleration service characteristics */
	if (masterContext.genAttFlag) {
	  masterContext.genAttFlag = FALSE;
	  masterContext.accFlag = TRUE;
          
	  masterContext.findCharacOfService = TRUE;
	} else {
	  /* Discovery Environmental Service characteristics */
	  if (masterContext.accFlag) {
	    extractCharacInfo();
	    masterContext.accFlag = FALSE;
            if (is_blue_ms) 
            {
              masterContext.envFlag = FALSE;
              masterContext.genAttFlag = TRUE;
              /* Start the enable indication procedure for the service changed charac */
	      masterContext.enableNotif = TRUE;
            }
            else 
            {
              masterContext.envFlag = TRUE;
	      masterContext.findCharacOfService = TRUE;
            }
	  } else {
	    /* Start the enable indication procedure for the service changed charac */
	    if (masterContext.envFlag) {
	      extractCharacInfo();
	      masterContext.envFlag = FALSE;
	      masterContext.genAttFlag = TRUE;
	      masterContext.enableNotif = TRUE;
	    }
	  }
	}
      }
    }
    break;
  }
}

/*******************************************************************************
* Function Name  : enableSensorNotifications
* Description    : Enables the sensor notifications and indications
* Input          : 
* Return         : 
*******************************************************************************/
void enableSensorNotifications(void)
{
  uint8_t ret;
  uint16_t handle;

  /* Enable Service changed notification */
  if (masterContext.genAttFlag) {
    memcpy ((uint8_t*)&handle, &masterContext.characGenAttSer[4], 2);
    ret = Master_NotifIndic_Status(masterContext.connHandle, (handle+1), FALSE, TRUE); 
    if (ret != BLE_STATUS_SUCCESS)
      PRINTF("Error during the Master_NotifIndic_Status(0x%02x), Status: 0x%02x\r\n", handle+1, ret);
    else
      PRINTF("Master_NotifIndic_Status(0x%02x), Status: 0x%02x\r\n", handle+1, ret);
  }

  /* Enable Acceleration and Free fall notifications */
  if (masterContext.accFlag) {
    
    if (is_blue_ms) 
    {
      
       if (--masterContext.numCharacAccSer != 0)
        handle = masterContext.tempHandle + 2; //Temp client charac conf desc handle
      else
        handle = masterContext.accHandle + 2; // Acc client charac conf desc handle
    }
    else
    {
      if (--masterContext.numCharacAccSer != 0)
        handle = masterContext.freeFallHandle + 2; //Free fall client charac conf desc handle
      else
        handle = masterContext.accHandle + 2; // Acc client charac conf desc handle
    }
    ret = Master_NotifIndic_Status(masterContext.connHandle, handle, TRUE, FALSE);
    if (ret != BLE_STATUS_SUCCESS)
      PRINTF("Error during the Master_NotifIndic_Status(), Status 0x%02x\r\n", ret);
    else
      PRINTF("---------------- Master_NotifIndic_Status(), handle: 0x%02x\r\n", handle);
  }
}

/*******************************************************************************
* Function Name  : readTemperature
* Description    : Read the temperature sensor from the peer device
* Input          : 
* Return         : 
*******************************************************************************/
void readTemperature(void)
{
  uint8_t status;
  
  status = Master_Read_Value(masterContext.connHandle, masterContext.tempHandle+1,
			     &masterContext.tempVal_length, masterContext.tempValue,
			     sizeof(masterContext.tempValue)); 
  
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the Master_Read_Value() function call returned status=0x%02x\r\n", status);
  }

  else
  {
    PRINTF("--- Master_Read_Value(0x%02x, %d)), env: OK\r\n",masterContext.tempHandle, masterContext.tempVal_length); 
  }
}

/*******************************************************************************
* Function Name  : Master_PeerDataExchange_CB
* Description    : Peer Device data exchange callback from master basic profile library
* Input          : 
* Return         : 
*******************************************************************************/
void Master_PeerDataExchange_CB(uint8_t *procedure, uint8_t *status, uint16_t *connection_handle, dataReceivedType *data)
{
  static int16_t acc_x=0, acc_y=0, acc_z=0, temp;
  uint8_t i, start_index, expected_len;

  switch(*procedure) {
  case NOTIFICATION_INDICATION_CHANGE_STATUS:
    {
      if (masterContext.genAttFlag) {
	masterContext.genAttFlag = FALSE;
	masterContext.accFlag = TRUE;
	masterContext.enableNotif = TRUE;
	PRINTF("\r\n****Service Changed indication enabled\r\n");
      } else {
	if (masterContext.accFlag) {
	  if (masterContext.numCharacAccSer == 0) {
	    masterContext.accFlag = FALSE;
            if (is_blue_ms)
              PRINTF("\r\n****Acceleration Notification enabled\r\n\n");
            else
              PRINTF("\r\n****Free Fall Notification enabled\r\n\n");
	  } else {
              masterContext.accFlag = TRUE;
             if (is_blue_ms)
              PRINTF("\r\n****Temperature Notification enabled\r\n\n");
             else
              PRINTF("\r\n****Acc Notification enabled\r\n");
	  }
	  masterContext.enableNotif = TRUE;
	}
      }      
    }
    break;
  case NOTIFICATION_DATA_RECEIVED:
    {
      if ((masterContext.freeFallHandle+1) == data->attr_handle) {
       if ((is_blue_ms == 0) && (data->data_value[0])) {
	 PRINTF("\n\n******************* FREE FALL DETECTION ***********************\r\n\n");
       }
      }
      if ((masterContext.accHandle+1) == data->attr_handle) {
	PRINTF("\r");
	for (i=0; i<70; i++) {
	  PRINTF(" ");
	}
        
        if (is_blue_ms) 
        {
          start_index = 2;
          expected_len = 14;
        }
        else
        {
          start_index = 0; 
          expected_len = 6;
        }
          
	if (data->data_length != expected_len)
	  PRINTF("Error in notification data length received expected %d instead is %d\r\n", expected_len, data->data_length); 
	memcpy((uint8_t*)&acc_x, &data->data_value[start_index], 2);
	memcpy((uint8_t*)&acc_y, &data->data_value[start_index+2], 2);
	memcpy((uint8_t*)&acc_z, &data->data_value[start_index+4], 2);
	PRINTF("\rACC-> X:%d, Y:%d, Z:%d. Temperature->%d.%d\r\n",
	       acc_x, acc_y, acc_z, temp/10, temp%10);
      }
      /* Check if the env char value has been notified when connected device is Sensor BlueMS */
      if ((is_blue_ms) && ((masterContext.tempHandle+1) == data->attr_handle)) 
      {        
        
        /* Store full env data from BlueMS: Time (2), Pressure (4), Temperature (2) */
        memcpy(&(masterContext.tempValue[0]), &(data->data_value[0]), data->data_length);
          
        /* Get Temperature value from received data (on bytes 6 and 7 */  
        temp = (int16_t) ((masterContext.tempValue[7]<<8) | masterContext.tempValue[6]);
        
        PRINTF("--> NOTIFY: Temperature->%d.%d\r\n", temp/10, temp%10);
       
       }              
    }
    break;
  case INDICATION_DATA_RECEIVED:
    {
      PRINTF("****Service Changed Indication Received\r\n");
    }
    break;
  case READ_VALUE_STATUS:
    {
      PRINTF("\r");
      for (i=0; i<70; i++) {
	PRINTF(" ");
      }
      if (is_blue_ms == 0) 
      {
        
        temp = (masterContext.tempValue[1]<<8) | masterContext.tempValue[0];
        PRINTF("\r\nREAD:, X:%d, Y:%d, Z:%d. Temperature->%d.%d\r\n", 
               acc_x, acc_y, acc_z, temp/10, temp%10); 
      }
    }
    break;
  }
}

tBleStatus aci_gap_numeric_comparison_value_confirm_yesno(uint16_t Connection_Handle,
                                                          uint8_t Confirm_Yes_No)
{
	return BLE_STATUS_SUCCESS;
}


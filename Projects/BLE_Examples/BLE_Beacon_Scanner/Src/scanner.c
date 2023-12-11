/*
  ******************************************************************************
  * @file    scanner.c 
  * @author  RF Application Team
  * @date    22 - 08 - 2022
  * @brief   Application functions
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
  * <h2><center>&copy; COPYRIGHT 2022 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
  
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */ 
 
/* Includes-----------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "gatt_profile.h"
#include "gap_profile.h"
#include "scanner.h"
#include "osal.h" 
#include "miscutil.h"

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BEACON_LEN                      30

#define SCAN_INTERVAL_MS                160
#define SCAN_WINDOW_MS                  160
#define SYNC_TIMEOUT_MS                 5000
/* Set FILTER_DEVICES to 1 to receive only advertising packets from the address specified in beacon_address (see below). */
#define FILTER_DEVICES                  0

#ifdef CTE_LOCATOR
#define SWITCHING_PATTERN               {0x00, 0x01}
#endif

#if FILTER_DEVICES || defined(CTE_LOCATOR) || defined(PERIODIC_ADV_SYNC)
#define BEACON_ADDRESS_TYPE             PUBLIC_ADDR
static uint8_t beacon_address[] = {0x66,0x77,0x88,0xE1,0x80,0x02};
#endif

/* Private typedef ------------------------------ */

typedef enum {
  START_SCANNING,
  SCANNING_SYNC, /* Scanning or sync'd with periodic adv */
}device_state_t;

/* Private variables ------------------------------ */

device_state_t device_state = START_SCANNING;

/* Private function prototypes ------------------------------ */

void print_adv_name(uint16_t adv_data_len, uint8_t * adv_data);

/* Private functions ------------------------------ */
/**
 * @brief  Init a BlueNRG device
 * @param  None.
 * @retval None.
*/
void device_initialization(void)
{
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  tBleStatus status;
  
#ifdef CTE_LOCATOR
  /* Need to call this function to set GPIOs used for antenna switching in case of AoA. */
  aci_hal_set_antenna_switch_parameters(0x06,           /* ANTENNA_ID_1 and ANTENNA_ID_2 enabled */
                                        1,              /* Left-shift ANTENNA ID signal by one bit to exclude ANTENNA_ID_0 (PB0 is used for UART). */
                                        0x00,           /* Default antenna ID */
                                        1);              /* RF_ACTIVITY signal enabled on PB7. */
#endif
  
  status = aci_gap_init(GAP_OBSERVER_ROLE, 0x00, 0x07, STATIC_RANDOM_ADDR, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_init() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_init --> SUCCESS\r\n");
  }
  
  uint8_t scan_filter = SCAN_ACCEPT_ALL;
  
  /* Unmask events */
  uint8_t le_event_mask[] = {0x00,
    HCI_LE_EVT_MASK_BYTE1_EXTENDED_ADVERTISING_REPORT | 
    HCI_LE_EVT_MASK_BYTE1_PERIODIC_ADVERTISING_SYNC_ESTABLISHED |
    HCI_LE_EVT_MASK_BYTE1_PERIODIC_ADVERTISING_REPORT |
    HCI_LE_EVT_MASK_BYTE1_PERIODIC_ADVERTISING_SYNC_LOST,
    HCI_LE_EVT_MASK_BYTE2_CONNECTIONLESS_IQ_REPORT,
    0x00,0x00,0x00,0x00,0x00};
  
  status = hci_le_set_event_mask(le_event_mask);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("hci_le_set_event_mask() failed:0x%02x\r\n", status);
  }else{
    PRINTF("hci_le_set_event_mask --> SUCCESS\r\n");
  }
  
  
#if FILTER_DEVICES
  
  List_Entry_t peer_list[1];

  peer_list[0].Peer_Address_Type = BEACON_ADDRESS_TYPE;
  Osal_MemCpy(peer_list[0].Peer_Address, beacon_address, 6);
  
  status = aci_gap_add_devices_to_white_and_resolving_list(0x01, /* White List*/
                                                           0x00, /* Do not clear list */
                                                           0x01, /* Num of list entries */
                                                           peer_list);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_add_devices_to_white_and_resolving_list() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_add_devices_to_white_and_resolving_list --> SUCCESS\r\n");
  }
  
  scan_filter = SCAN_ACCEPT_WHITE_LIST_ONLY;
  
#endif  
  
  status = aci_gap_set_scan_configuration(DUPLICATE_FILTER_DISABLED,
                                          scan_filter,
                                          LE_1M_PHY_BIT,
                                          PASSIVE_SCAN,
                                          SCAN_INTERVAL_MS*1000/625,SCAN_WINDOW_MS*1000/625);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_scan_configuration() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_set_scan_configuration --> SUCCESS\r\n");
  }  
}


/**
 * @brief  Puts the device in scannable mode
 * @param  None.
 * @retval None.
*/
void start_scanning(void)
{
  tBleStatus status;
  
  status = aci_gap_start_procedure(GAP_OBSERVATION_PROC,LE_1M_PHY_BIT,0x0000,0x0000);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_start_procedure() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_start_procedure --> SUCCESS\r\n");
  }
  
#if defined(CTE_LOCATOR) || defined(PERIODIC_ADV_SYNC) 

  status = aci_gap_periodic_advertising_create_sync(0x00, /* No options */
                                                    0x00, /* Advertising SID */
                                                    BEACON_ADDRESS_TYPE, /* Advertising address type */
                                                    beacon_address, /* Advertiser address */
                                                    0x0000, /* Skip */
                                                    SYNC_TIMEOUT_MS/10, /* Sync timeout*/
                                                    0x10 ); /* Do not sync to packets without a Constant Tone Extension */
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_periodic_advertising_create_sync() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_periodic_advertising_create_sync --> SUCCESS\r\n");
  }
#endif  
}


/**
 * @brief  Device Demo state machine
 * @param  None.
 * @retval None.
*/
void APP_Tick(void)
{
  if(device_state == START_SCANNING)
  {
    start_scanning();
    device_state = SCANNING_SYNC;
  }
}

void print_beacon_data(uint16_t adv_data_len, uint8_t * adv_data, char * head_string)
{  
  uint8_t i = 0;
  uint16_t AD_len;
  uint8_t AD_type;
  uint8_t * uuid;
  uint16_t major, minor;
  BOOL beacon_found = FALSE;
  
  const uint8_t ibeacon_company_id[2] = {0x4C, 0x00};
  const uint8_t ibeacon_beacon_type[2] = {0x02, 0x15};
  
  while(i < adv_data_len){
    AD_len = adv_data[i];
    AD_type = adv_data[i+1];
    
    /* Identify only iBeacons */
    if(AD_len == 0x1A && AD_type == AD_TYPE_MANUFACTURER_SPECIFIC_DATA &&
       memcmp(&adv_data[i+2], ibeacon_company_id, 2) ==0 && memcmp(&adv_data[i+4], ibeacon_beacon_type, 2) == 0)
    {
      uuid = &adv_data[i+6];
      major = BE_TO_HOST_16(adv_data+i+22);
      minor = BE_TO_HOST_16(adv_data+i+24);
      PRINTF("%s UUID: %08X-%04X-%04X-%04X-%08X%04X ", head_string, BE_TO_HOST_32(uuid),BE_TO_HOST_16(uuid+4),BE_TO_HOST_16(uuid+6),\
                                                   BE_TO_HOST_16(uuid+8),BE_TO_HOST_32(uuid+10),BE_TO_HOST_16(uuid+14));
      PRINTF("Major: %d  Minor: %d ", major, minor);      
      beacon_found = TRUE;
      break;
    }    
    i += AD_len+1;
  }
  
  if(beacon_found)
  {
    print_adv_name(adv_data_len, adv_data);
    PRINTF("\r\n");
  }
  
}

void print_adv_name(uint16_t adv_data_len, uint8_t * adv_data)
{
  uint8_t i = 0;
  uint16_t AD_len;
  uint8_t AD_type;
  char * name;
  
  while(i < adv_data_len){
    AD_len = adv_data[i];
    AD_type = adv_data[i+1];
    
    if(AD_type == AD_TYPE_COMPLETE_LOCAL_NAME || AD_type == AD_TYPE_SHORTENED_LOCAL_NAME)
    {
      name = (char *)&adv_data[i+2];
      for(uint16_t j = 0; j < AD_len-1; j++)
      {
        PRINTF("%c", name[j]);
      }
      break;
    }    
    i += AD_len+1;
  }  
}

/* *************** BlueNRG-LPS Stack Callbacks****************/

void hci_le_extended_advertising_report_event(uint8_t num_reports,Extended_Advertising_Report_t extended_advertising_report[])
{
  uint16_t event_type = extended_advertising_report[0].Event_Type;
  
  if(event_type == 0x0010)
  {
    print_beacon_data(extended_advertising_report[0].Data_Length, extended_advertising_report[0].Data, "Beacon");
  }
  else if(event_type == 0x0000)
  {
    print_beacon_data(extended_advertising_report[0].Data_Length, extended_advertising_report[0].Data, "ExtAdv Beacon");
  }
}

#if defined(PERIODIC_ADV_SYNC) || defined(CTE_LOCATOR)
void hci_le_periodic_advertising_report_event(uint16_t sync_handle,int8_t tx_power,int8_t rssi,uint8_t cte_type,uint8_t data_status,uint8_t data_length,uint8_t data[])
{ 
   print_beacon_data(data_length, data, "Periodic Beacon");  
}
#endif

void hci_le_periodic_advertising_sync_established_event(uint8_t status,uint16_t sync_handle,uint8_t advertising_sid,uint8_t advertiser_address_type,uint8_t advertiser_address[6],uint8_t advertiser_phy,uint16_t periodic_advertising_interval,uint8_t advertiser_clock_accuracy)
{
  PRINTF("hci_le_periodic_advertising_sync_established_event --> EVENT\r\n");
  
  /* Scanning is no more needed. Disable it. */
  status = aci_gap_terminate_proc(GAP_OBSERVATION_PROC);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_terminate_proc() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_terminate_proc --> SUCCESS\r\n");
  }
  
#ifdef CTE_LOCATOR
  
  uint8_t antenna_ids[] = SWITCHING_PATTERN;
  
  status = hci_le_set_connectionless_iq_sampling_enable(sync_handle,
                                                        ENABLE,
                                                        CTE_SLOT_1us,
                                                        0, /* Report all CTEs*/
                                                        sizeof(antenna_ids),
                                                        antenna_ids);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("hci_le_set_connectionless_iq_sampling_enable() failed:0x%02x\r\n", status);
  }else{
    PRINTF("hci_le_set_connectionless_iq_sampling_enable --> SUCCESS\r\n");
  }
#endif
}

void aci_gap_proc_complete_event(uint8_t procedure_code,uint8_t status,uint8_t data_length,uint8_t data[])
{
  PRINTF("aci_gap_proc_complete_event --> EVENT\r\n");
}

#ifdef CTE_LOCATOR 
void hci_le_connectionless_iq_report_event(uint16_t sync_handle,uint8_t channel_index,int16_t rssi,uint8_t rssi_antenna_id,uint8_t cte_type,uint8_t slot_durations,uint8_t packet_status,uint16_t periodic_event_counter,uint8_t sample_count,Samples_t samples[])
{   
  const char * cte_type_str[3] = {"AoA", "AoD 1us", "AoD 2 us"};
  PRINTF("IQ samples (%s): ", cte_type_str[cte_type]);
  for(int i = 0; i < sample_count; i++)
  {
    PRINTF("(%d,%d)",samples[i].I_Sample,samples[i].Q_Sample);
  }
  PRINTF("\n");
}
#endif

#if defined(CTE_LOCATOR) || defined(PERIODIC_ADV_SYNC)
void hci_le_periodic_advertising_sync_lost_event(uint16_t Sync_Handle)
{
  PRINTF("Sync lost.\r\n");
  
  device_state = START_SCANNING;
}
#endif

/** \endcond 
*/

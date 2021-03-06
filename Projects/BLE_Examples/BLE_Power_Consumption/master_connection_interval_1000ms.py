#This Script has been created automatically by BlueNRG GUI.
#This script has to be reviewed using the BlueNRG script utility in order to run it by Script Engine function.
Status,HCI_Version,HCI_Revision,LMP_PAL_Version,Manufacturer_Name,LMP_PAL_Subversion= HCI_READ_LOCAL_VERSION_INFORMATION()
if Status!=0x00:
    PRINT('FAILED COMMAND HCI_READ_LOCAL_VERSION_INFORMATION')

Status,DTM_version_major,DTM_version_minor,DTM_version_patch,DTM_variant,DTM_Build_Number,BTLE_Stack_version_major,BTLE_Stack_version_minor,BTLE_Stack_version_patch,BTLE_Stack_development,BTLE_Stack_variant,BTLE_Stack_Build_Number= ACI_HAL_GET_FIRMWARE_DETAILS()
if Status!=0x00:
    PRINT('FAILED COMMAND ACI_HAL_GET_FIRMWARE_DETAILS')

Status= HCI_RESET()
if Status!=0x00:
    PRINT('FAILED COMMAND HCI_RESET')

#Received event (To manage by user) -> ACI_BLUE_INITIALIZED_EVENT(Event Code=0xFF,Parameter Total Length=0x03,Ecode=0x0001,Reason_Code=0x01,)
Status= ACI_HAL_WRITE_CONFIG_DATA(Offset=0x00,Length=0x06,Value=0x0280E1005584)
if Status!=0x00:
    PRINT('FAILED COMMAND ACI_HAL_WRITE_CONFIG_DATA')

Status= ACI_HAL_SET_TX_POWER_LEVEL(En_High_Power=0x01,PA_Level=0x19)
if Status!=0x00:
    PRINT('FAILED COMMAND ACI_HAL_SET_TX_POWER_LEVEL')

Status= ACI_GATT_SRV_INIT()
if Status!=0x00:
    PRINT('FAILED COMMAND ACI_GATT_SRV_INIT')

Status,Service_Handle,Dev_Name_Char_Handle,Appearance_Char_Handle= ACI_GAP_INIT(Role=0x0F,Privacy_Type=0x00,Device_Name_Char_Len=0x07,Identity_Address_Type = 0)
if Status!=0x00:
    PRINT('FAILED COMMAND ACI_GAP_INIT')


## ACI_GAP_SET_SCAN_CONFIGURATION is needed also for direct connection
status = ACI_GAP_SET_SCAN_CONFIGURATION(Filter_Duplicates = 0x00,
                                        Scanning_Filter_Policy = 0x00, 
                                        Scanning_PHY = 0x01,             ##0x01: LE 1M PHY 
                                        Scan_Type = 0x01,                ##0x01: Active scan
                                        Scan_Interval = 0x4000, 
                                        Scan_Window = 0x4000)

if status!= 0x00:
    ERROR('ACI_GAP_SET_SCAN_CONFIGURATION FAILED')

status = ACI_GAP_SET_CONNECTION_CONFIGURATION(Initiating_PHY = 0x01,      ## 0x01: LE 1M PHY
                                              Conn_Interval_Min = 0x0320,          
                                              Conn_Interval_Max = 0x0320,     
                                              Conn_Latency = 0, 
                                              Supervision_Timeout = 0x03E8, 
                                              Minimum_CE_Length = 0x0000,
                                              Maximum_CE_Length = 0x03E8)

if status!= 0x00:
  ERROR('ACI_GAP_SET_CONNECTION_CONFIGURATION FAILED') 

status = ACI_GAP_CREATE_CONNECTION(Initiating_PHY = 0x01,      ## 0x01: LE 1M PHY
                                   Peer_Address_Type =  0x0,
                                   Peer_Address = 0x0280E100ECBC) 

if Status!=0x00:
    ERROR('FAILED COMMAND ACI_GAP_CREATE_CONNECTION')

WAIT_EVENT(HCI_VENDOR_EVENT, Ecode=ACI_GAP_PROC_COMPLETE_EVENT)
WAIT_EVENT(HCI_LE_META_EVENT, Subevent_Code= HCI_LE_CONNECTION_COMPLETE_EVENT)    ##Received event (To manage by user) -> HCI_LE_META_EVENT(Subevent_Code=0x01, Status=0x00, Connection_Handle=0x801, Role=0x00, Peer_Address_Type=0x00, Peer_Address=0x280e100ecbc, Conn_Interval=0x50, Conn_Latency=0x00, Supervision_Timeout=0x3e8, Master_Clock_Accuracy=0x00)

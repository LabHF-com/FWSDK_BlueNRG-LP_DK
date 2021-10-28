

/**
  ******************************************************************************
  * @file    bluenrg_lp_l2cap_aci.c
  * @author  AMS - RF Application team
  * @date    09 June 2020
  * @brief   Source file for external uC - BlueNRG-x in network coprocessor mode (l2cap_aci)
  *          Autogenerated files, do not edit!!
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
#include "bluenrg_lp_l2cap_aci.h"
#include "osal.h"
tBleStatus aci_l2cap_connection_parameter_update_req(uint16_t Connection_Handle,
                                                     uint16_t Conn_Interval_Min,
                                                     uint16_t Conn_Interval_Max,
                                                     uint16_t Slave_latency,
                                                     uint16_t Timeout_Multiplier)
{
  struct hci_request rq;
  uint8_t cmd_buffer[532];
  aci_l2cap_connection_parameter_update_req_cp0 *cp0 = (aci_l2cap_connection_parameter_update_req_cp0*)(cmd_buffer);
  tBleStatus status = 0;
  uint8_t index_input = 0;
  cp0->Connection_Handle = htob(Connection_Handle, 2);
  index_input += 2;
  cp0->Conn_Interval_Min = htob(Conn_Interval_Min, 2);
  index_input += 2;
  cp0->Conn_Interval_Max = htob(Conn_Interval_Max, 2);
  index_input += 2;
  cp0->Slave_latency = htob(Slave_latency, 2);
  index_input += 2;
  cp0->Timeout_Multiplier = htob(Timeout_Multiplier, 2);
  index_input += 2;
  Osal_MemSet(&rq, 0, sizeof(rq));
  rq.ext_aci = TRUE;
  rq.ogf = 0x3f;
  rq.ocf = 0x181;
  rq.event = 0x0F;
  rq.cparam = cmd_buffer;
  rq.clen = index_input;
  rq.rparam = &status;
  rq.rlen = 1;
  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;
  if (status) {
    return status;
  }
  return BLE_STATUS_SUCCESS;
}
tBleStatus aci_l2cap_connection_parameter_update_resp(uint16_t Connection_Handle,
                                                      uint16_t Conn_Interval_Min,
                                                      uint16_t Conn_Interval_Max,
                                                      uint16_t Slave_latency,
                                                      uint16_t Timeout_Multiplier,
                                                      uint16_t Minimum_CE_Length,
                                                      uint16_t Maximum_CE_Length,
                                                      uint8_t Identifier,
                                                      uint8_t Accept)
{
  struct hci_request rq;
  uint8_t cmd_buffer[532];
  aci_l2cap_connection_parameter_update_resp_cp0 *cp0 = (aci_l2cap_connection_parameter_update_resp_cp0*)(cmd_buffer);
  tBleStatus status = 0;
  uint8_t index_input = 0;
  cp0->Connection_Handle = htob(Connection_Handle, 2);
  index_input += 2;
  cp0->Conn_Interval_Min = htob(Conn_Interval_Min, 2);
  index_input += 2;
  cp0->Conn_Interval_Max = htob(Conn_Interval_Max, 2);
  index_input += 2;
  cp0->Slave_latency = htob(Slave_latency, 2);
  index_input += 2;
  cp0->Timeout_Multiplier = htob(Timeout_Multiplier, 2);
  index_input += 2;
  cp0->Minimum_CE_Length = htob(Minimum_CE_Length, 2);
  index_input += 2;
  cp0->Maximum_CE_Length = htob(Maximum_CE_Length, 2);
  index_input += 2;
  cp0->Identifier = htob(Identifier, 1);
  index_input += 1;
  cp0->Accept = htob(Accept, 1);
  index_input += 1;
  Osal_MemSet(&rq, 0, sizeof(rq));
  rq.ext_aci = TRUE;
  rq.ogf = 0x3f;
  rq.ocf = 0x182;
  rq.cparam = cmd_buffer;
  rq.clen = index_input;
  rq.rparam = &status;
  rq.rlen = 1;
  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;
  if (status) {
    return status;
  }
  return BLE_STATUS_SUCCESS;
}
tBleStatus aci_l2cap_cfc_connection_req_nwk(uint16_t Connection_Handle,
                                            uint16_t SPSM,
                                            uint16_t CID,
                                            uint16_t MTU,
                                            uint16_t MPS,
                                            uint8_t CFC_Policy)
{
  struct hci_request rq;
  uint8_t cmd_buffer[532];
  aci_l2cap_cfc_connection_req_nwk_cp0 *cp0 = (aci_l2cap_cfc_connection_req_nwk_cp0*)(cmd_buffer);
  tBleStatus status = 0;
  uint8_t index_input = 0;
  cp0->Connection_Handle = htob(Connection_Handle, 2);
  index_input += 2;
  cp0->SPSM = htob(SPSM, 2);
  index_input += 2;
  cp0->CID = htob(CID, 2);
  index_input += 2;
  cp0->MTU = htob(MTU, 2);
  index_input += 2;
  cp0->MPS = htob(MPS, 2);
  index_input += 2;
  cp0->CFC_Policy = htob(CFC_Policy, 1);
  index_input += 1;
  Osal_MemSet(&rq, 0, sizeof(rq));
  rq.ext_aci = TRUE;
  rq.ogf = 0x3f;
  rq.ocf = 0x183;
  rq.event = 0x0F;
  rq.cparam = cmd_buffer;
  rq.clen = index_input;
  rq.rparam = &status;
  rq.rlen = 1;
  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;
  if (status) {
    return status;
  }
  return BLE_STATUS_SUCCESS;
}
tBleStatus aci_l2cap_cfc_connection_resp_nwk(uint16_t Connection_Handle,
                                             uint8_t Identifier,
                                             uint16_t CID,
                                             uint16_t MTU,
                                             uint16_t MPS,
                                             uint16_t Result,
                                             uint8_t CFC_Policy)
{
  struct hci_request rq;
  uint8_t cmd_buffer[532];
  aci_l2cap_cfc_connection_resp_nwk_cp0 *cp0 = (aci_l2cap_cfc_connection_resp_nwk_cp0*)(cmd_buffer);
  tBleStatus status = 0;
  uint8_t index_input = 0;
  cp0->Connection_Handle = htob(Connection_Handle, 2);
  index_input += 2;
  cp0->Identifier = htob(Identifier, 1);
  index_input += 1;
  cp0->CID = htob(CID, 2);
  index_input += 2;
  cp0->MTU = htob(MTU, 2);
  index_input += 2;
  cp0->MPS = htob(MPS, 2);
  index_input += 2;
  cp0->Result = htob(Result, 2);
  index_input += 2;
  cp0->CFC_Policy = htob(CFC_Policy, 1);
  index_input += 1;
  Osal_MemSet(&rq, 0, sizeof(rq));
  rq.ext_aci = TRUE;
  rq.ogf = 0x3f;
  rq.ocf = 0x184;
  rq.cparam = cmd_buffer;
  rq.clen = index_input;
  rq.rparam = &status;
  rq.rlen = 1;
  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;
  if (status) {
    return status;
  }
  return BLE_STATUS_SUCCESS;
}
tBleStatus aci_l2cap_send_flow_control_credits(uint16_t Connection_Handle,
                                               uint16_t CID,
                                               uint16_t RX_Credits,
                                               uint8_t CFC_Policy,
                                               uint16_t *RX_Credit_Balance)
{
  struct hci_request rq;
  uint8_t cmd_buffer[532];
  aci_l2cap_send_flow_control_credits_cp0 *cp0 = (aci_l2cap_send_flow_control_credits_cp0*)(cmd_buffer);
  aci_l2cap_send_flow_control_credits_rp0 resp;
  Osal_MemSet(&resp, 0, sizeof(resp));
  uint8_t index_input = 0;
  cp0->Connection_Handle = htob(Connection_Handle, 2);
  index_input += 2;
  cp0->CID = htob(CID, 2);
  index_input += 2;
  cp0->RX_Credits = htob(RX_Credits, 2);
  index_input += 2;
  cp0->CFC_Policy = htob(CFC_Policy, 1);
  index_input += 1;
  Osal_MemSet(&rq, 0, sizeof(rq));
  rq.ext_aci = TRUE;
  rq.ogf = 0x3f;
  rq.ocf = 0x185;
  rq.cparam = cmd_buffer;
  rq.clen = index_input;
  rq.rparam = &resp;
  rq.rlen = sizeof(resp);
  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;
  if (resp.Status) {
    return resp.Status;
  }
  *RX_Credit_Balance = btoh(resp.RX_Credit_Balance, 2);
  return BLE_STATUS_SUCCESS;
}
tBleStatus aci_l2cap_disconnect(uint16_t Connection_Handle,
                                uint16_t CID)
{
  struct hci_request rq;
  uint8_t cmd_buffer[532];
  aci_l2cap_disconnect_cp0 *cp0 = (aci_l2cap_disconnect_cp0*)(cmd_buffer);
  tBleStatus status = 0;
  uint8_t index_input = 0;
  cp0->Connection_Handle = htob(Connection_Handle, 2);
  index_input += 2;
  cp0->CID = htob(CID, 2);
  index_input += 2;
  Osal_MemSet(&rq, 0, sizeof(rq));
  rq.ext_aci = TRUE;
  rq.ogf = 0x3f;
  rq.ocf = 0x186;
  rq.event = 0x0F;
  rq.cparam = cmd_buffer;
  rq.clen = index_input;
  rq.rparam = &status;
  rq.rlen = 1;
  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;
  if (status) {
    return status;
  }
  return BLE_STATUS_SUCCESS;
}
tBleStatus aci_l2cap_transmit_sdu_data_nwk(uint16_t Connection_Handle,
                                           uint16_t CID,
                                           uint16_t SDU_Length,
                                           uint8_t SDU_Data[])
{
  struct hci_request rq;
  uint8_t cmd_buffer[532];
  aci_l2cap_transmit_sdu_data_nwk_cp0 *cp0 = (aci_l2cap_transmit_sdu_data_nwk_cp0*)(cmd_buffer);
  tBleStatus status = 0;
  uint8_t index_input = 0;
  cp0->Connection_Handle = htob(Connection_Handle, 2);
  index_input += 2;
  cp0->CID = htob(CID, 2);
  index_input += 2;
  cp0->SDU_Length = htob(SDU_Length, 2);
  index_input += 2;
  /* var_len_data input */
  {
    Osal_MemCpy((void *) &cp0->SDU_Data, (const void *) SDU_Data, SDU_Length*sizeof(uint8_t));
    index_input += SDU_Length*sizeof(uint8_t);
  }
  Osal_MemSet(&rq, 0, sizeof(rq));
  rq.ext_aci = TRUE;
  rq.ogf = 0x3f;
  rq.ocf = 0x187;
  rq.cparam = cmd_buffer;
  rq.clen = index_input;
  rq.rparam = &status;
  rq.rlen = 1;
  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;
  if (status) {
    return status;
  }
  return BLE_STATUS_SUCCESS;
}

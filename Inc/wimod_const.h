#ifndef __WIMOD_CONST_H__
#define __WIMOD_CONST_H__

#define DEVMGMT_ID 0x01 //Device Management
#define RLT_ID 0x02 //Radio Link Test
#define RADIOLINK_ID 0x03 //Radio Link Services
#define REMOTE_CTRL_ID 0x04 //Remote Control
#define SENSOR_ID 0x05 //Sensor App

#define DEVMGMT_MSG_PING_REQ 0x01
#define DEVMGMT_MSG_PING_RSP 0x02 
#define DEVMGMT_MSG_GET_DEVICE_INFO_REQ 0x03
#define DEVMGMT_MSG_GET_DEVICE_INFO_RSP 0x04
#define DEVMGMT_MSG_GET_FW_INFO_REQ 0x05
#define DEVMGMT_MSG_GET_FW_INFO_RSP 0x06 
#define DEVMGMT_MSG_RESET_REQ 0x07 
#define DEVMGMT_MSG_RESET_RSP 0x08
#define DEVMGMT_MSG_SET_OPMODE_REQ 0x09
#define DEVMGMT_MSG_SET_OPMODE_RSP 0x0A

#define DEVMGMT_MSG_GET_OPMODE_REQ 0x0B
#define DEVMGMT_MSG_GET_OPMODE_RSP 0x0C 
#define DEVMGMT_MSG_SET_RTC_REQ 0x0D 
#define DEVMGMT_MSG_SET_RTC_RSP 0x0E 
#define DEVMGMT_MSG_GET_RTC_REQ 0x0F 
#define DEVMGMT_MSG_GET_RTC_RSP 0x10
#define DEVMGMT_MSG_SET_RADIO_CONFIG_REQ 0x11
#define DEVMGMT_MSG_SET_RADIO_CONFIG_RSP 0x12
#define DEVMGMT_MSG_GET_RADIO_CONFIG_REQ 0x13
#define DEVMGMT_MSG_GET_RADIO_CONFIG_RSP 0x14
#define DEVMGMT_MSG_RESET_RADIO_CONFIG_REQ 0x15
#define DEVMGMT_MSG_RESET_RADIO_CONFIG_RSP 0x16 
#define DEVMGMT_MSG_GET_SYSTEM_STATUS_REQ 0x17 
#define DEVMGMT_MSG_GET_SYSTEM_STATUS_RSP 0x18
#define DEVMGMT_MSG_SET_RADIO_MODE_REQ 0x19
#define DEVMGMT_MSG_SET_RADIO_MODE_RSP 0x1A 
#define DEVMGMT_MSG_POWER_UP_IND 0x20
#define DEVMGMT_MSG_SET_AES_KEY_REQ 0x21
#define DEVMGMT_MSG_SET_AES_KEY_RSP 0x22
#define DEVMGMT_MSG_GET_AES_KEY_REQ 0x23
#define DEVMGMT_MSG_GET_AES_KEY_RSP 0x24 
#define DEVMGMT_MSG_SET_RTC_ALARM_REQ 0x31 
#define DEVMGMT_MSG_SET_RTC_ALARM_RSP 0x32 
#define DEVMGMT_MSG_CLEAR_RTC_ALARM_REQ 0x33 
#define DEVMGMT_MSG_CLEAR_RTC_ALARM_RSP 0x34 
#define DEVMGMT_MSG_GET_RTC_ALARM_REQ 0x35 
#define DEVMGMT_MSG_GET_RTC_ALARM_RSP 0x36 
#define DEVMGMT_MSG_RTC_ALARM_IND 0x38
#define DEVMGMT_MSG_SET_HCI_CFG_REQ 0x41
#define DEVMGMT_MSG_SET_HCI_CFG_RSP 0x42
#define DEVMGMT_MSG_GET_HCI_CFG_REQ 0x43
#define DEVMGMT_MSG_GET_HCI_CFG_RSP 0x44 
#define DEVMGMT_MSG_INIT_BOOTLOADER_REQ 0xF6 
#define DEVMGMT_MSG_INIT_BOOTLOADER_RSP 0xF7

#define DEVMGMT_STATUS_OK 0x00 //Operation successful
#define DEVMGMT_STATUS_ERROR 0x01 //Operation failed
#define DEVMGMT_STATUS_CMD_NOT_SUPPORTED 0x02 //Command is not supported (check system operation mode)
#define DEVMGMT_STATUS_WRONG_PARAMETER 0x03 //HCI message contains wrong parameter
#define DEVMGMT_STATUS_WRONG_DEVICE_MODE 0x04 //Stack is running in a wrong mode
#define DEVMGMT_STATUS_DEVICE_BUSY 0x06 //Device is busy, command rejected

#define RADIOLINK_MSG_SEND_U_DATA_REQ 0x01
#define RADIOLINK_MSG_SEND_U_DATA_RSP 0x02
#define RADIOLINK_MSG_U_DATA_RX_IND 0x04
#define RADIOLINK_MSG_U_DATA_TX_IND 0x06 
#define RADIOLINK_MSG_RAW_DATA_RX_IND 0x08
#define RADIOLINK_MSG_SEND_C_DATA_REQ 0x09
#define RADIOLINK_MSG_SEND_C_DATA_RSP 0x0A
#define RADIOLINK_MSG_C_DATA_RX_IND 0x0C
#define RADIOLINK_MSG_C_DATA_TX_IND 0x0E
#define RADIOLINK_MSG_ACK_RX_IND 0x10
#define RADIOLINK_MSG_ACK_TIMEOUT_IND 0x12
#define RADIOLINK_MSG_ACK_TX_IND 0x14 
#define RADIOLINK_MSG_SET_ACK_DATA_REQ 0x15 
#define RADIOLINK_MSG_SET_ACK_DATA_RSP 0x16

#define RADIOLINK_STATUS_OK 0x00 //Operation successful
#define RADIOLINK_STATUS_ERROR 0x01 //Operation failed
#define RADIOLINK_STATUS_CMD_NOT_SUPPORTED 0x02 //Command is not supported (check system operation mode)
#define RADIOLINK_STATUS_WRONG_PARAMETER 0x03 //HCI message contains wrong parameter
#define RADIOLINK_STATUS_WRONG_RADIO_MOD 0x04 //Module operates in wrong radio mode
#define RADIOLINK_STATUS_MEDIA_BUSY 0x05 //Transmission not possible due to LBT result: “Media Busy”
#define RADIOLINK_STATUS_BUFFER_FULL 0x07 //No buffer for radio transmission available
#define RADIOLINK_STATUS_LENGTH_ERROR 0x08 //Radio packet length invalid

#define RLT_MSG_START_REQ 0x01
#define RLT_MSG_START_RSP 0x02
#define RLT_MSG_STOP_REQ 0x03
#define RLT_MSG_STOP_RSP 0x04
#define RLT_MSG_STATUS_IND 0x06

#define RLT_STATUS_OK 0x00 //Operation successful
#define RLT_STATUS_ERROR 0x01 //Operation failed
#define RLT_STATUS_CMD_NOT_SUPPORTED 0x02 //Command is not supported (check system operation mode)
#define RLT_STATUS_WRONG_PARAMETER 0x03 //HCI message contains wrong parameter
#define RLT_STATUS_WRONG_RADIO_MODE 0x04 //Module operates in wrong radio mode
#define RLT_STATUS_WRONG_DEVICECONFIG 0x05 //Radio Configuration invalid

#define REMOTE_CTRL_MSG_BUTTON_PRESSED_IND 0x02

#define SENSOR_MSG_SET_CONFIG_REQ 0x09
#define SENSOR_MSG_SET_CONFIG_RSP 0x0A 
#define SENSOR_MSG_GET_CONFIG_REQ 0x0B 
#define SENSOR_MSG_GET_CONFIG_RSP 0x0C
#define SENSOR_MSG_SEND_DATA_IND 0x06
#define SENSOR_MSG_ACK_IND 0x08

#define SENSOR_STATUS_OK 0x00 //Operation successful
#define SENSOR_STATUS_ERROR 0x01 //Operation failed
#define SENSOR_STATUS_WRONG_DEVICEMODE 0x04 //Module operates in wrong radio mode

#endif //END WIMOD_CONST

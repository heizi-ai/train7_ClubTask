#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>
#include "bsp_can.h"

uint16_t motor_angle = 0;
int16_t motor_speed = 0;
uint16_t motor_voltage = 0;
int16_t motor_current = 0;
uint8_t motor_temp = 0;
uint32_t last_time=0;
volatile uint32_t motor_flag=0;


void can_filter_init(void) {
    CAN_FilterTypeDef can_filter_st={0};

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

}

CAN_TxHeaderTypeDef can_tx_header={0};
CAN_RxHeaderTypeDef can_rx_header={0};

uint32_t tx_mailbox;
uint32_t rx_mailbox;

void can_tx_header_init(void) {
    can_tx_header.StdId = 0x1FF;
    can_tx_header.IDE=CAN_ID_STD;
    can_tx_header.RTR=CAN_RTR_DATA;
    can_tx_header.DLC=8;
    can_tx_header.TransmitGlobalTime = DISABLE;
}

void can_tx_send(int16_t voltage) {
    can_tx_header_init();
    uint8_t txdata[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    txdata[0]=(uint8_t)(voltage>>8);
    txdata[1]=(uint8_t)(voltage);
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, txdata, &tx_mailbox);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rxdata[8];
    last_time=HAL_GetTick();
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, rxdata);

    if (can_rx_header.StdId == 0x205&&can_rx_header.IDE == CAN_ID_STD) {
        motor_angle = rxdata[0]<<8 | rxdata[1];
        motor_speed = rxdata[2]<<8 | rxdata[3];
        motor_current = rxdata[4]<<8 | rxdata[5];
        motor_temp = rxdata[6];
    }
}


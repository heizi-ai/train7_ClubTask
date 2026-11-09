/**
 * @file    bsp_can.c
 * @brief   BSP 层 —— CAN1 收发封装（GM6020 电机）
 *
 * 分层说明：
 *   本文件只依赖 HAL，不依赖 FreeRTOS、也不包含业务逻辑。
 *   上层（Application）通过 bsp_can.h 里的接口和全局反馈量使用它。
 */

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>
#include "bsp_can.h"

/* ================= 电机反馈量 =================
 * 全部在 CAN 接收中断里刷新，任务层只读。
 * 加 volatile：防止编译器把"循环里反复读"优化成只读一次。
 */
volatile uint16_t motor_angle   = 0;   /* 机械角度，0~8191（对应 0~360°） */
volatile int16_t  motor_speed   = 0;   /* 转速，单位 rpm，正负表示方向 */
volatile uint16_t motor_voltage = 0;   /* 预留：母线电压（当前未解析） */
volatile int16_t  motor_current = 0;   /* 实际转矩电流，正负表示方向 */
volatile uint8_t  motor_temp    = 0;   /* 电机温度，单位 ℃ */
volatile uint32_t last_time     = 0;   /* 最近一次收到反馈的 HAL 时间戳（ms），用于离线判断 */
volatile uint32_t motor_flag    = 0;   /* 串口收到一帧合法 "motor±N" 指令的标志：中断置 1，任务解析后清 0 */


/**
 * @brief  CAN1 过滤器配置 + 启动 + 打开接收中断
 * @note   过滤器全 0 = 掩码全通，接收总线上所有标准帧（本项目只有一台电机，无需筛选）
 */
void can_filter_init(void) {
    CAN_FilterTypeDef can_filter_st = {0};   /* 必须清零：未用到的成员若为随机值会导致配置失败 */

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;     /* 掩码模式 */
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;    /* 32 位宽的过滤器 */
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;               /* 掩码全 0：全部放行 */
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;    /* 收到的帧进 FIFO0 */

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);                                /* 配置完必须 Start，否则不收发 */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  /* FIFO0 有帧时进中断 */
}


/* 发送帧头/接收帧头做成全局，避免每次发送都重新构造 */
CAN_TxHeaderTypeDef can_tx_header = {0};
CAN_RxHeaderTypeDef can_rx_header = {0};

uint32_t tx_mailbox;    /* HAL_CAN_AddTxMessage 会写入选中的邮箱号 */
uint32_t rx_mailbox;    /* 预留，当前未使用 */


/**
 * @brief  初始化发送帧头（所有 GM6020 控制帧格式相同，只有数据不同）
 * @note   StdId = 0x1FF：控制 ID 1~4 号电机的电压
 */
void can_tx_header_init(void) {
    can_tx_header.StdId = MOTOR_CONTROL_STDID;    /* 0x1FF */
    can_tx_header.IDE = CAN_ID_STD;               /* 标准帧 */
    can_tx_header.RTR = CAN_RTR_DATA;             /* 数据帧 */
    can_tx_header.DLC = 8;                        /* 固定 8 字节 */
    can_tx_header.TransmitGlobalTime = DISABLE;
}


/**
 * @brief  给 1 号电机发电压指令
 * @param  voltage 电压给定，单位 mV，范围 -25000 ~ 25000
 * @note   一帧 0x1FF 里塞 4 台电机（每台占 2 字节），1 号电机占 DATA[0]~DATA[1]。
 *         大端序：高字节在前。
 */
void can_set_motor1_voltage(int16_t voltage) {
    can_tx_header_init();
    uint8_t txdata[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    txdata[0] = (uint8_t) (voltage >> 8);   /* 高 8 位 */
    txdata[1] = (uint8_t) (voltage);        /* 低 8 位（负数按补码自动截取） */
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, txdata, &tx_mailbox);
}


/**
 * @brief  CAN 接收中断回调（HAL 在 FIFO0 非空时调用）
 * @note   电机反馈帧：标识符 = 0x204 + 电机ID，1 号电机即 0x205，频率 1kHz
 *         帧格式：DATA[0:1]=角度  DATA[2:3]=转速  DATA[4:5]=电流  DATA[6]=温度  DATA[7]=空
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rxdata[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, rxdata);

    /* 只处理"标准帧 + 1 号电机"的反馈；其他 ID 直接丢弃 */
    if (can_rx_header.StdId == MOTOR1_STDID && can_rx_header.IDE == CAN_ID_STD) {
        last_time = HAL_GetTick();                      /* 刷新在线时间戳（看门狗用） */
        motor_angle = rxdata[0] << 8 | rxdata[1];       /* 拼成 16 位，范围 0~8191 */
        motor_speed = rxdata[2] << 8 | rxdata[3];       /* 拼成 int16，负值表示反转 */
        motor_current = rxdata[4] << 8 | rxdata[5];
        motor_temp = rxdata[6];
    }
}

#include "can_user.h"
#include "tim.h"
#include "Motor.h"
#include "imu.h"

extern "C" volatile uint8_t bmi088_ok;

Motor motor(36.0f);
constexpr uint32_t kMotor1FeedbackId = 0x201;

extern "C" {
volatile uint32_t stop_flag = 1;
volatile uint32_t zero_request = 0;
volatile PidMonitor pid_mon = {};
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    uint8_t data[8];
    if (hcan != &hcan1) {
        return;
    }
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) {
        return;
    }

    if (rx_header.IDE == CAN_ID_STD &&
        rx_header.RTR == CAN_RTR_DATA &&
        rx_header.DLC == 8 &&
        rx_header.StdId == kMotor1FeedbackId) {
        motor.canRxMsgCallback(data);
        }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance != TIM6) {
        return;
    }

    if (bmi088_ok != 0) {
        board_imu.update();
    }

    motor.setStopFlag(stop_flag != 0);
    if (zero_request != 0) {
        motor.requestZero();
        zero_request = 0;
    }

    motor.handle();

    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        HAL_CAN_AbortTxRequest(&hcan1, mailboxes);
    }
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, motor.getTxData(), &can_tx_mailbox);
}

extern "C" void motor_set_target_angle(float angle)
{
    motor.setPosition(angle, 0.0f, 0.0f);
}

extern "C" void motor_set_stop_flag(int flag)
{
    stop_flag = (flag != 0) ? 1 : 0;
    motor.setStopFlag(stop_flag != 0);
}

extern "C" void motor_request_zero(void)
{
    motor.requestZero();
}

extern "C" void motor_set_feedforward_only(void)
{
    motor.setFeedforwardOnly();
}
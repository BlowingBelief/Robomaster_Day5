#ifndef ROBOMASTERTEST001_CAN_USER_H
#define ROBOMASTERTEST001_CAN_USER_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mailbox;
extern CAN_FilterTypeDef can_filter_config;

typedef struct {
    float angle;
    float target_angle;
    float intensity;
    float spid_integral;
    float ppid_integral;
    float ppid_last_d;
    float error;
    float speed;
} PidMonitor;

extern volatile PidMonitor pid_mon;

#ifdef __cplusplus
}
#endif

#endif
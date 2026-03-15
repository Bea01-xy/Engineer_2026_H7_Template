#ifndef CAN_TASK_H
#define CAN_TASK_H

#include <stdint.h>

/** 主控计算出的 J6 输出，由 Control_Task 写入，CAN_Task 经 CAN 发给从板 */
extern int16_t Slave_J6_Output;

#endif
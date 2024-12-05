#ifndef PID_H
#define PID_H

#include <stdint.h>

// PID控制器结构体
typedef struct {
    float Kp;                // 比例系数
    float Ki;                // 积分系数
    float Kd;                // 微分系数
    float previous_error;    // 上一次误差
    float pre_previous_error;// 保存两次前的误差
    float output;            // 输出增量
    float integral;          // 误差积分
} PID_Controller;
PID_Controller motor_pid[3];
// 初始化PID控制器
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd);

// 重置PID控制器（清除积分和误差）
void PID_Reset(PID_Controller *pid);

// 位置式PID计算函数
float PID_Compute(PID_Controller *pid, float setpoint, float measured);

#endif // PID_H
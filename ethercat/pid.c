#include "pid.h"
#define dt 0.004f

// 初始化PID控制器
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0.0f;
    pid->pre_previous_error = 0.0f; // 保存两次前的误差
    pid->output = 0.0f;             // 输出增量初始化为0
}

// 重置PID控制器（清除误差和输出）
void PID_Reset(PID_Controller *pid) {
    pid->previous_error = 0.0f;
    pid->pre_previous_error = 0.0f;
    pid->output = 0.0f;
}

// 增量式PID计算函数
float PID_Compute(PID_Controller *pid, float setpoint, float measured) {
    float error = setpoint - measured;  // 当前误差
    // 计算增量式 PID 输出
    float delta_output = 
        pid->Kp * (error - pid->previous_error) +                       // 比例增量
        pid->Ki * error * dt +                                         // 积分增量
        pid->Kd * (error - 2 * pid->previous_error + pid->pre_previous_error) / dt; // 微分增量

    pid->output += delta_output;  // 计算累计输出值

    // 更新历史误差值
    pid->pre_previous_error = pid->previous_error;
    pid->previous_error = error;

    if(error > 60 || error < -60) PID_Reset(pid); // 如果误差过大，重置PID控制器
    else return pid->output + error;  // 返回当前的累计输出（位置调整量）
    return pid->output;  // 返回当前的累计输出（位置调整量）
}
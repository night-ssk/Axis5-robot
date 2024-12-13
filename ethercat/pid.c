#include "pid.h"
#include <stdio.h>
#define dt 0.004f
#define MAX_INT 2147483647 // Maximum value for an int
#define macro()
// 初始化PID控制器
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0.0f;
    pid->pre_previous_error = 0.0f; // 保存两次前的误差
    pid->output = 0.0f;             // 输出增量初始化为0
    pid->last_measured = MAX_INT;           // 误差积分初始化为0
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

    return pid->output;  // 返回当前的累计输出（位置调整量）
}

// float OUT_Compute(PID_Controller *pid, float setpoint, float measured, float motor_encode_val) {
//     float error = setpoint - measured;  // 当前误差
//     int output = 0;
    
//     if(error > 60 || error < -60) { 
//         PID_Reset(pid); 
//         return output;  
//     } else if(error > 5 || error < -5) {
//         output = PID_Compute(pid, setpoint, measured); // 使用PID控制
//     }else { //误差小时采用
//         //提取误差符号
//         int error_step = error > 0 ? 1 : -1; 
//         pid->output += error_step;
//         output = pid->output;
//     }
//     return output;  // 返回当前的输出（位置调整量）
// }

float OUT_Compute(PID_Controller *pid, int setpoint, int measured, int motor_encode_val) {
    setpoint = setpoint * 5;
    measured = measured * 5;

    if (pid ->last_measured == MAX_INT) {
        pid->last_measured = measured;
        return setpoint;
    }

    int error = setpoint - pid->last_measured;  // 当前误差
    if(error >= 1200 || error <= -1200) {
       pid->output = setpoint;//不补偿光栅尺误差
    }
    else if((35 < error && error < 1200) || (-35 > error && error > -1200)) {
       pid->output = (setpoint - (pid->last_measured -  motor_encode_val));//补偿光栅尺误差
    }
    else  if (error != 0) {
        pid->output += error * 0.4;
    }

    pid->last_measured = measured;
    int output = pid->output;
    return output;  // 返回当前的输出（位置调整量）
}
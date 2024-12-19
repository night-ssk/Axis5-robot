// motor.c

#include "ecrt.h"
#include "igh.h"
#include "motor.h"
#include "term.h"
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include "encoder.h"
#include "pid.h"
#include "log.h"
#define  CDHDNUM 2

// 引入命令队列
extern CommandQueue commandQueue;
MotorCommandSet cmdSet = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        MODEL_DISABLE,
        MODEL_DISABLE, 
        {0, 0, 0, 0, 0}
};
pthread_mutex_t cmdSet_mutex = PTHREAD_MUTEX_INITIALIZER;
/**********************CStruct**********************************/
ec_pdo_entry_info_t slave_motor_pdo_entries[] = {
    {0x6040, 0x00, 16}, // 控制字 U16
    {0x6060, 0x00, 8},  // 操作模式 I8
    {0x607a, 0x00, 32}, // 目标位置 S32

    {0x6041, 0x00, 16}, // 状态字 U16
    {0x6061, 0x00, 8},  // 操作模式显示 I8
    {0x6064, 0x00, 32}, // 当前位置 S32
};

ec_pdo_info_t slave_motor_pdos[] = {
    {0x1600, 3, slave_motor_pdo_entries + 0},  // RXPDO
    {0x1a00, 3, slave_motor_pdo_entries + 3}, // TXPDO
};

ec_sync_info_t slave_motor_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_motor_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_motor_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
/**********************CStruct**********************************/

// 电机回零
void homeing(struct _Domain* domain, int* motorMode, int motorID, int* state) {
    int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[motorID].operation_mode_display); // 读取状态字
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);             // 读取状态字
    if (*state < 10) {
        if (*state == 0) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 重置电机
            if((status & 0xFF) == 0x31) {
                *state = 1;
            }
        } else if(*state == 1) {
            EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_HOME); // 设置操作模式
            if (operation_mode_display == MODEL_HOME) {
                *state = 2;
            }
        }  else if (*state == 2) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
            if((status & 0xFF) == 0x33) {
                *state = 3;
            }
        } else if (*state == 3) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 回零就绪
            if((status & 0xFF) == 0x37 && (status & 0xFF00) == 0x0600) {           
                *state = 4;
            }
        }
        else if (*state == 4) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x1F); // 启动回零
            if((status & 0xFF) == 0x37 && (status & 0xFF00) == 0x1600) {  //回零完成
                printf("电机%d回零完成\n", motorID);
                *motorMode = MODEL_ENABLE;
                *state = 10;
            }
        }
    }
}


// 电机位置控制CSP模式
void csp(struct _Domain* domain, int target_pos, int motorID, int* state) {
    int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[motorID].operation_mode_display);// 读取状态字
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);// 读取状态字

    if (*state < 10) {
        if (*state == 0) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 重置电机
            if((status & 0xFF) == 0x31) {
                *state = 1;
            }
        } else if(*state == 1) {
            EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_CSP); // 设置操作模式
            if (operation_mode_display == MODEL_CSP) {
                *state = 2;
            }
        }  else if (*state == 2) {
            int actual_motor = EC_READ_S32(domain->domain_pd + motor_parm[motorID].current_pos);
            int target_motor = EC_READ_S32(domain->domain_pd + motor_parm[motorID].target_pos);
            EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, actual_motor); // 设置目标位置
            int diff1 = target_motor - actual_motor;
            int diff2 = target_pos - actual_motor;
            if(-300 < diff1 && diff1 < 300 && -300 < diff2 && diff2 < 300) {
                *state = 3;
            }
        } else if (*state == 3) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
            if((status & 0xFF) == 0x33) {
                *state = 4;
            }
        } else if (*state == 4) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 使能电机
            if((status & 0xFF) == 0x37) {
                *state = 10;
            }
        }
    } else {
        EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置
        // if (motorID == 2 || motorID == 3 || motorID == 4)//直线电机
        //     EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置
        // else if (motorID == 0 || motorID == 1 )//步进电机
        // {
        //     int motor_encode_val = EC_READ_S32(domain->domain_pd + motor_parm[motorID].current_pos);
        //     int out = OUT_Compute(&motor_pid[motorID], target_pos, cmdSet.actual_pos[motorID], motor_encode_val);

        //     EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, out); // 设置目标位置
        // }
    }
}
void enable(struct _Domain* domain, int motorID, int* state) {
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);// 读取状态字

    if (*state < 10) {
        if(*state == 0) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 重置电机
            if((status & 0xFF) == 0x31) {
                *state = 1;
            }
        } else if (*state == 1) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
            if((status & 0xFF) == 0x33) {
                *state = 2;
            }
        } else if (*state == 2) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 使能电机
            if((status & 0xFF) == 0x37) {
                *state = 10;
            }
        }
    }
}
void disable(struct _Domain* domain, int motorID, int* state) {
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);// 读取状态字

    if (*state < 10) {
        if(*state == 0) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 重置电机
            if((status & 0xFF) == 0x31) {
                *state = 10;
            }
        }
    }
}
bool initCDHD(struct _Domain* domain){
    const int cdhd_id[CDHDNUM] = {3, 4};
    static int cdhd_state[CDHDNUM] = {0, 0};
    if (cdhd_state[0] == 10 && cdhd_state[1] == 10) {
        return true;
    }
    for (int i = 0; i < CDHDNUM; i++) {
        uint16_t cdhd_status = EC_READ_U16(domain->domain_pd + motor_parm[cdhd_id[i]].status_word);
                //printf("电机 %d 状态: %d, 状态字: 0x%X\n", i, cdhd_state[i], cdhd_status);
        if (cdhd_state[i] == 0) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[cdhd_id[i]].ctrl_word, 0x06); // 重置电机
            if((cdhd_status & 0xFF) == 0xb1) {
                cdhd_state[i] = 1;
            }
        }
        else if (cdhd_state[i] == 1) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[cdhd_id[i]].ctrl_word, 0x07); // 电机得电
            if((cdhd_status & 0xFF) == 0xb3) {
                cdhd_state[i] = 2;
            }
        }
        else if (cdhd_state[i] == 2) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[cdhd_id[i]].ctrl_word, 0x0f); // 使能电机
            if((cdhd_status & 0xFF) == 0x37) {
                cdhd_state[i] = 3;
            }
        }
        else if (cdhd_state[i] == 3) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[cdhd_id[i]].ctrl_word, 0x06); // 使能电机
            if((cdhd_status & 0xFF) == 0x31) {
                cdhd_state[i] = 10;
                printf("cdhd %d号电机初始化完成\n", i);
            }
        }
    }
    return false;
}
bool motordir(struct _Domain* domain, int motorID){
    static bool init_flag[3] = {false, false , false};
    static int init_state[3] = {0};
    
    static int32_t start_pos_encoder[3] = {0};
    static int32_t end_pos_encoder[3] = {0};
    static int32_t diff_pos_encoder[3] = {0};
    
    static int32_t start_pos_linear[3] = {0};
    static int32_t end_pos_linear[3] = {0};
    static int32_t diff_pos_linear[3] = {0};

    static int16_t times_wait[3] = {50, 50, 50}; // 等待时间2s

    int8_t operation_mode_display;
    uint16_t cdhd_status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);
    if (init_flag[motorID] == true) {
        return true;
    }
    if (init_state[motorID] == 0) {
        operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[motorID].operation_mode_display);
        if ( operation_mode_display != MODEL_CSP) {
            EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_CSP);
        }else{
                init_state[motorID] = 1;
        }
    } else if (init_state[motorID] == 1) {   // 电机初始位置测量
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 重置电机
        if((cdhd_status & 0xFF) == 0x31)
        {
            init_state[motorID] = 2;
        }
    } else if (init_state[motorID] == 2) {
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 重置电机
        if((cdhd_status & 0xFF) == 0x33)
        {
            init_state[motorID] = 3;
        }
    } else if (init_state[motorID] == 3) {
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 重置电机
        if((cdhd_status & 0xFF) == 0x37)
        {
            init_state[motorID] = 4;
            start_pos_encoder[motorID] = EC_READ_S32(domain->domain_pd + motor_parm[motorID].current_pos);
            start_pos_linear[motorID] = EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[motorID]);
        }
    } else if (init_state[motorID] == 4) {
        EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, 200);
        if(times_wait[motorID]-- == 0)  init_state[motorID] = 5;
    } else if (init_state[motorID] == 5) {
        end_pos_encoder[motorID] = EC_READ_S32(domain->domain_pd + motor_parm[motorID].current_pos);
        end_pos_linear[motorID] = EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[motorID]);
        diff_pos_encoder[motorID] = end_pos_encoder[motorID] - start_pos_encoder[motorID];
        diff_pos_linear[motorID] = end_pos_linear[motorID] - start_pos_linear[motorID];
        int tmp_dir = diff_pos_encoder[motorID] / diff_pos_linear[motorID];

        cmdSet.dir[motorID] = tmp_dir > 0 ? 1 : -1;

        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 重置电机
        printf("电机%d编码器位置差: %d, 直线电机位置差: %d\n", motorID, diff_pos_encoder[motorID], diff_pos_linear[motorID]);
        init_flag[motorID] = true;
    }
    return false;
}
void updatePos(struct _Domain* domain) {
    cmdSet.actual_pos[0] = EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[0]) * cmdSet.dir[0];
    cmdSet.actual_pos[1] = EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[1]) * cmdSet.dir[1];
    cmdSet.actual_pos[2] = EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[2]) * cmdSet.dir[2] * 0.625f;
    // cmdSet.actual_pos[2]= EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[2]);
    //int actual_pos2= EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[2]);
    //cmdSet.actual_pos[2] = EC_READ_S32(domain->domain_pd + motor_parm[2].current_pos);
    cmdSet.actual_pos[3] = EC_READ_S32(domain->domain_pd + motor_parm[3].current_pos);
    cmdSet.actual_pos[4] = EC_READ_S32(domain->domain_pd + motor_parm[4].current_pos);
}
// 主电机控制函数

void motor_main(struct _Domain* domain) {
    //static int32_t times = 0;
    if (initCDHD(domain) == false) {
        return;
    }
    bool dirret0 = motordir(domain, 0);
    bool dirret1 = motordir(domain, 1);
    bool dirret2 = motordir(domain, 2);
    if (dirret0 == false || dirret1 == false || dirret2 == false) return;
    // 上锁
    pthread_mutex_lock(&cmdSet_mutex);
    MotorCommandSet cmdCur;
    int ret = dequeue_non_blocking(&commandQueue, &cmdCur);
    if(ret == 1) {
        cmdSet.mode = cmdCur.mode;
        //printf("cmdCur.mode: %d\n", cmdCur.mode);
        for (int i = 0; i < MOTOR_NUM; i++) {
            cmdSet.target_pos[i] = cmdCur.target_pos[i];
        }
    }
    updateStatues(&cmdSet);//
    updatePos(domain);
    // 执行命令集
    for (int i = 0; i < MOTOR_NUM; i++) {
            if (cmdSet.mode == MODEL_HOME) {
               // homeing(domain, &cmdSet.state[i], i, &cmdSet.state[i]);
            }
            else if (cmdSet.mode == MODEL_CSP) {
                csp(domain, cmdSet.target_pos[i], i, &cmdSet.state[i]);
            }
            else if (cmdSet.mode == MODEL_DISABLE) {
                //disable(domain, i, &cmdSet.state[i]);
            }
            else if (cmdSet.mode == MODEL_ENABLE) {
                //enable(domain, i, &cmdSet.state[i]);
            }
    }
    //解锁
    pthread_mutex_unlock(&cmdSet_mutex);
}

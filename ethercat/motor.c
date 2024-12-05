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
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 电机得电
            if((status & 0xFF) == 0x31) {
                *state = 3;
            }
        } else if (*state == 3) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
            if((status & 0xFF) == 0x33) {
                *state = 4;
            }
        } else if (*state == 4) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 使能电机
            if((status & 0xFF) == 0x37 && (status & 0xFF00) == 0x0600) {
                *state = 10;
            }
        
        }
    }
    if (operation_mode_display == MODEL_HOME && (status & 0xFF) == 0x37) {
        if ((status & 0xFF00) == 0x0600) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x1F); // 启动运行
        }
        else if ((status & 0xFF00) == 0x1600) { // 到位
            printf("电机%d回零完成\n", motorID);
            *motorMode = MODEL_ENABLE;
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 重置
            return;
        }
    }
    else {
        EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_HOME); // 设置操作模式

        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 重置
        printf("电机%d开始回零运动\n", motorID);

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
            //EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置
        if (motorID == 2 || motorID == 3 || motorID == 4)//直线电机
            EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置
        else if (motorID == 0 || motorID == 1 )//步进电机
        {
            int motor_encode_val = EC_READ_S32(domain->domain_pd + motor_parm[motorID].current_pos);
            int out = target_pos + motor_encode_val - cmdSet.actual_pos[motorID];
            if(motorID == 0) {
                printf("motorid: %d : target %d  motor_encode_val: %d actual_pos: %d\n", motorID, target_pos, motor_encode_val, cmdSet.actual_pos[motorID]);
            }

            out += PID_Compute(&motor_pid[motorID], target_pos, cmdSet.actual_pos[motorID]);
            EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, out); // 设置目标位置
        }
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
bool homeCDHD(struct _Domain* domain, int* motorMode, int motorID) {
    int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[motorID].operation_mode_display); // 读取状态字
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);             // 读取状态字
    if (operation_mode_display == MODEL_HOME && (status & 0xFF) == 0x37) {
        if ((status & 0xFF00) == 0x0600) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x1F); // 启动运行
        }
        else if ((status & 0xFF00) == 0x1600) { // 到位
            printf("cdhd %d号电机回零完成\n", motorID);
            *motorMode = MODEL_ENABLE;
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 重置
            return true;
        }
    }
    else {
        EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_HOME); // 设置操作模式

        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 重置
        printf("cdhd %d号电机开始回零运动\n", motorID);

    }
    return false;
}
void updatePos(struct _Domain* domain) {
    cmdSet.actual_pos[0]= EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[0]);
    cmdSet.actual_pos[1]= EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[1]);
    cmdSet.actual_pos[2]= EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[2]);
    cmdSet.actual_pos[3] = EC_READ_S32(domain->domain_pd + motor_parm[3].current_pos);
    cmdSet.actual_pos[4] = EC_READ_S32(domain->domain_pd + motor_parm[4].current_pos);
}
// 主电机控制函数
void motor_main(struct _Domain* domain) {
    static int32_t times = 0;
    if (initCDHD(domain) == false) {
        return;
    }
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
            // if(i == 3) {
                //printf("target_pos: %d, actual_pos: %d\n", EC_READ_S32(domain->domain_pd + motor_parm[i].target_pos), EC_READ_S32(domain->domain_pd + motor_parm[i].current_pos));
            // }
            //if(times++%500 == 0) {
                //printf("status: %d %d %d %d %d %d %d\n", cmdSet.mode, cmdSet.state[0], cmdSet.state[1], cmdSet.state[2], cmdSet.state[3], cmdSet.state[4], cmdSet.state[5]);
            //}
    }
    //解锁
    pthread_mutex_unlock(&cmdSet_mutex);
}

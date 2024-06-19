#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <libgen.h>
#include <getopt.h>
#include <limits.h>
 
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include "canopen.h"
#include "term.h"
extern MotorCommand currentCommand[5];
extern pthread_mutex_t cmd_lock;

static int running = 1;
/*************共享变量****************/
CANFrame bus[NUMSID];
/*************共享变量****************/
int canInit(void)
{
    struct ifreq ifr;
    struct sockaddr_can addr;
    int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
    int socket_ret;

    if ((socket_ret = socket(family, type, proto)) < 0) {
        perror("socket err");
        return -1;
    }
    strcpy(ifr.ifr_name, "can1");
    ioctl(socket_ret, SIOCGIFINDEX, &ifr);
    addr.can_family = family;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_ret, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind err");
        return -1;
    }
    return socket_ret;
}
void* canSend(int* arg)
{   
    int socket = *arg;
    initSDO(&bus[w_ctrlword_1],Cob1,ctrlword);//u16
    initSDO(&bus[w_ctrlword_2],Cob2,ctrlword);

    initSDO(&bus[w_operation_1],Cob1,operation);//i8
    initSDO(&bus[w_operation_2],Cob2,operation);

    initSDO(&bus[w_targetPos_1],Cob1,targetPos);//i32
    initSDO(&bus[w_targetPos_2],Cob2,targetPos);

    initSDO(&bus[w_targetSpd_1],Cob1,targetSpd);//u32
    initSDO(&bus[w_targetSpd_2],Cob2,targetSpd);
    //读取
    initSDO(&bus[r_status_1],Cob1,status);//u16
    initSDO(&bus[r_status_2],Cob2,status);

    initSDO(&bus[r_actualOp_1],Cob1,actualOp);//i8
    initSDO(&bus[r_actualOp_2],Cob2,actualOp);

    initSDO(&bus[r_actualPos_1],Cob1,actualPos);//i32
    initSDO(&bus[r_actualPos_2],Cob2,actualPos);
/************************初始化电机*********************************/
    sendNMT(socket,0x80,0);
    sendNMT(socket,0x01,0);
    for(int i = 0; i < 2; i++)
    {
        //配置PP操作模式
        addData(&bus[w_operation_1 + i], I8, 0x01);
        sendAndReceiveSDO(socket,&bus[w_operation_1 + i]);

        addData(&bus[w_targetPos_1 + i], I32, 0x00);
        sendAndReceiveSDO(socket,&bus[w_targetPos_1 + i]);//设置目标位置

        addData(&bus[w_ctrlword_1 + i], U16, 0x06);
        sendAndReceiveSDO(socket,&bus[w_ctrlword_1 + i]); // 启动电压

        addData(&bus[w_ctrlword_1 + i], U16, 0x07);
        sendAndReceiveSDO(socket,&bus[w_ctrlword_1 + i]); // 开启

        addData(&bus[w_ctrlword_1 + i], U16, 0x0F);
        sendAndReceiveSDO(socket,&bus[w_ctrlword_1 + i]); // 使能

        printf("启动%d号电机成功", i + 1);
    }
/************************电机控制*********************************/
    while (running) {
        pthread_mutex_lock(&cmd_lock); // 加锁
        int cmd = currentCommand[i + 3].commandAvailable;
        int distance = currentCommand[i + 3].distance;
        pthread_mutex_unlock(&cmd_lock); // 解锁
            for (int i = 0; i < 2; i++) {
                if (cmd == 1)
                {
                    addData(&bus[w_targetSpd_1 + i], U32, 3600);
                    sendAndReceiveSDO(socket,&bus[w_targetSpd_1 + i]);//设置轮廓速度

                    addData(&bus[w_targetPos_1 + i], I32, cmd);
                    sendSDO(socket,&bus[w_targetPos_1 + i]);
                    
                    addData(&bus[w_ctrlword_1 + i], U16, 0x1F);
                    sendSDO(socket,&bus[w_ctrlword_1 + i]); // 更新目标
                    
                    printf("cmd %d : %d \n", i, cmd);
                    pthread_mutex_lock(&cmd_lock); // 加锁
                    currentCommand[i + 3].commandAvailable = 2; //运动中
                    pthread_mutex_unlock(&cmd_lock); // 解锁
                }
                else if (cmd == 2) //到位
                {
                    readSDO(socket, &bus[r_actualPos_1 + i]);
                    int pos = bus[r_actualPos_1 + i].sdo.name.data.i32;
                    if(pos == cmd)
                    {
                        printf("val %d : %d \n", status, pos);

                        addData(&bus[w_ctrlword_1 + i], U16, 0x0F);
                        sendSDO(socket,&bus[w_ctrlword_1 + i]); // 更新目标
                        pthread_mutex_lock(&cmd_lock); // 加锁
                        currentCommand[i + 3].commandAvailable = 0;
                        pthread_mutex_unlock(&cmd_lock); // 解锁
                    }
                }
                usleep(10000);
            }
    }
    return (void*)0;
}
void* canRecv(int* arg) {
    int socket = *arg;
    struct can_frame read_buf;
    while(1){
        read(socket, &read_buf, sizeof(read_buf));
        // printf("canrecv: canid %d", read_buf.can_id);
        // for(int i = 0; i < 8; i++)
        // printf("data %d: %d", i, read_buf.data[i]);
        // printf("\n");        
        if(0 < (read_buf.can_id - 0x580) && (read_buf.can_id - 0x580) < 3) { //sdo数据
            uint8_t  type = read_buf.data[0];
            uint32_t dic = read_buf.data[1] << 8 | read_buf.data[2] << 16 | read_buf.data[3] << 24;
            if(type == 0x80) //错误返回
            {
                printf("err: sdo 0x80");
                return (void*)-1;
            }
            int index = findFrame(bus, NUMSID, dic) + read_buf.can_id - 0x580 - 1;
            if(type == 0x60)//写返回
            {
                bus[index].status = false;
            }
            else if(type == 0x4F || type == 0x4B || type == 0x43) //数据请求返回
            {
                bus[index].status = false;
                memcpy(&bus[index].sdo.bytes[4], &read_buf.data[4], 4);
            }
        }
    }
}


#include "term.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
// 全局静态变量
MotorCommand currentCommand[5] = {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}};

// 互斥锁
pthread_mutex_t cmd_lock;
void addCommand(int motorID, int distance) {
    if(motorID < 0 && motorID >= 6) return;

    pthread_mutex_lock(&cmd_lock); // 加锁
    if(currentCommand[motorID].commandAvailable == 0)
    {
        currentCommand[motorID].distance += distance;
        currentCommand[motorID].commandAvailable = 1; // 标记新命令
    }
    pthread_mutex_unlock(&cmd_lock); // 解锁
}

void* readCommands(void* arg)
{
    char input[100];
    
    while (1) {
        if (fgets(input, sizeof(input), stdin) == NULL) {
            fprintf(stderr, "读取输入失败\n");
            continue;
        }

        // 去除输入中的换行符
        input[strcspn(input, "\n")] = 0;

        // 检查是否输入了退出命令
        if (strcmp(input, "q") == 0 || strcmp(input, "Q") == 0) {
            break;
        }

        int motorID, distance;
        if (sscanf(input, "%d %d", &motorID, &distance) != 2) {
            fprintf(stderr, "输入格式错误，请重新输入。\n");
            continue;
        }
        addCommand(motorID, distance);
    }
    
    return NULL;
}

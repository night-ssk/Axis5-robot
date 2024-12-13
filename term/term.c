// term.c

#include "term.h"
#include "motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h> // For sockets
#include <netinet/in.h>
#include <errno.h>

// 宏定义
#define PORT 12345 // 服务端监听的端口
#define BUFFER_SIZE 1024
#define VALID_POSITION_MIN -10000000
#define VALID_POSITION_MAX 10000000

// 全局命令队列
CommandQueue commandQueue;
// 全局变量
int client_sock = -1;
pthread_mutex_t client_sock_mutex = PTHREAD_MUTEX_INITIALIZER;
extern MotorCommandSet cmdSet;
extern pthread_mutex_t cmdSet_mutex;

// 初始化队列
void initQueue(CommandQueue* queue) {
    //清除队列
    while(queue->front != NULL) {
        CommandNode* temp = queue->front;
        queue->front = queue->front->next;
        free(temp);
    }
    queue->front = queue->rear = NULL;
    queue->size = 0;
    queue->max = 20;
    pthread_mutex_init(&queue->mutex, NULL);
}

// 入队
void enqueue(CommandQueue* queue, MotorCommandSet* commandSet) {
    while(queue->size >= queue->max) {
        usleep(1000);
    }
    CommandNode* newNode = (CommandNode*)malloc(sizeof(CommandNode));
    if (!newNode) {
        fprintf(stderr, "内存分配失败\n");
        return;
    }
    newNode->commandSet = *commandSet;
    newNode->next = NULL;

    pthread_mutex_lock(&queue->mutex);
    if (queue->rear == NULL) {
        queue->front = queue->rear = newNode;
    } else {
        queue->rear->next = newNode;
        queue->rear = newNode;
    }
    queue->size++;
    pthread_mutex_unlock(&queue->mutex);
}
 
// 非阻塞出队
int dequeue_non_blocking(CommandQueue* queue, MotorCommandSet* commandSet) {
    int ret = 0;
    pthread_mutex_lock(&queue->mutex);
    if (queue->front != NULL) {
        CommandNode* temp = queue->front;
        *commandSet = temp->commandSet;
        queue->front = queue->front->next;
        if (queue->front == NULL) {
            queue->rear = NULL;
        }
        queue->size--;
        free(temp);
        ret = 1;
    }
    pthread_mutex_unlock(&queue->mutex);
    return ret;
}

void updateStatues(MotorCommandSet *cmd) {
    if (cmd->mode != cmd->previous_mode) {
        // 根据 mode 的变化更新 statue，这里假设 mode 为1时 statue 设置为100，否则设置为0
        for(int i = 0; i < MOTOR_NUM; i++){
            cmd->state[i] = 0;
        }
        // 更新 previous_mode
        cmd->previous_mode = cmd->mode;
    }
}
void readFile(char* arg) {
    char* filename = arg;
    printf("readCommands 接收到的文件名: %s\n", filename); // 调试输出

    FILE* file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "无法打开文件: %s\n", filename);
        return ;
    }

    char line[1024];
    // 初始化队列
    initQueue(&commandQueue);

    // 按行读取文件
    while (fgets(line, sizeof(line), file) != NULL) {
        // 去除换行符
        line[strcspn(line, "\n")] = 0;

        // 处理位置命令，例如: pp 0 -14 -27 ...
        if (strncmp(line, "pos", 3) == 0) {
            MotorCommandSet cmdGet;
            memset(&cmdGet, 0, sizeof(MotorCommandSet));
            cmdGet.mode = MODEL_CSP;

            // 解析电机位置参数
            char* token = strtok(line, " ");
            token = strtok(NULL, " ");

            int motorIndex = 0;
            int valid = 1;
            while (token != NULL && motorIndex < MOTOR_NUM) {
                int pos = atoi(token);
                cmdGet.target_pos[motorIndex] = pos;
                token = strtok(NULL, " ");
                motorIndex++;
            }
            if (motorIndex != MOTOR_NUM) {
                fprintf(stderr, "接收到的电机参数数量不匹配（期望 %d，收到 %d）\n", MOTOR_NUM, motorIndex);
                valid = 0;
            }
            if (valid) {
                // 入队命令
                enqueue(&commandQueue, &cmdGet);
            } else {
                // 发送错误信息回客户端
                char error_msg[256];
                snprintf(error_msg, sizeof(error_msg), "Error: Invalid 'pos' command: %s\n", line);
                // send(sock, error_msg, strlen(error_msg), 0);
            }
        }
    }
    fclose(file);
    printf("命令文件读取完成。\n");
    return ;
}

// 发送状态更新给客户端
void* sendStatusToClient(void* arg) {
    int sock = *(int*)arg;
    while (1) {
        // 构建状态消息
        char status_msg[256];
        snprintf(status_msg, sizeof(status_msg), "status");
        //上锁
        pthread_mutex_lock(&cmdSet_mutex);
        // 添加每个电机的模式和状态
        for (int i = 0; i < MOTOR_NUM; i++) {
            char motor_info[32];
            snprintf(motor_info, sizeof(motor_info), " %d %d %d", cmdSet.mode, cmdSet.state[i], cmdSet.actual_pos[i]);
            strncat(status_msg, motor_info, sizeof(status_msg) - strlen(status_msg) - 1);
        }
        //解锁
        pthread_mutex_unlock(&cmdSet_mutex);
        strncat(status_msg, "\n", sizeof(status_msg) - strlen(status_msg) - 1);

        // 发送状态消息
        send(sock, status_msg, strlen(status_msg), 0);

        // 休眠100ms
        usleep(100000);
    }
    return NULL;
}
// 客户端处理线程
void* clientHandler(void* arg) {
    int sock = *(int*)arg;
    char buffer[BUFFER_SIZE];
    int bytes_read;

    // 启动发送状态给客户端的线程
    pthread_t status_thread;
    pthread_create(&status_thread, NULL, sendStatusToClient, (void*)&sock);

    while (1) {
        bytes_read = recv(sock, buffer, BUFFER_SIZE - 1, 0);
        if (bytes_read <= 0) {
            // 连接关闭或出错
            printf("客户端已断开\n");
            close(sock);
            pthread_mutex_lock(&client_sock_mutex);
            client_sock = -1;
            pthread_mutex_unlock(&client_sock_mutex);

            // 切换电机到disable模式
            MotorCommandSet cmdGet;
            memset(&cmdGet, 0, sizeof(MotorCommandSet));
            cmdGet.mode = MODEL_DISABLE;
            enqueue(&commandQueue, &cmdGet);

            pthread_cancel(status_thread);
            pthread_exit(NULL);
        }
        buffer[bytes_read] = '\0';

        // 处理接收到的数据
        char* line = strtok(buffer, "\n");
        while (line != NULL) {
            // 处理每一行命令
            while (*line == ' ') line++;
            char* end = line + strlen(line) - 1;
            while (end > line && (*end == ' ' || *end == '\r' || *end == '\n')) {
                *end = '\0';
                end--;
            }
            //打印命令
            if (strlen(line) > 0) {
                //printf("pos 接收到命令: %s\n", line);
                // 处理命令
                if (strncmp(line, "pos", 3) == 0) {
                    MotorCommandSet cmdGet;
                    memset(&cmdGet, 0, sizeof(MotorCommandSet));
                    cmdGet.mode = MODEL_CSP;

                    // 解析电机位置参数
                    char* token = strtok(line, " ");
                    token = strtok(NULL, " ");

                    int motorIndex = 0;
                    int valid = 1;
                    while (token != NULL && motorIndex < MOTOR_NUM) {
                        int pos = atoi(token);
                        // if (pos < VALID_POSITION_MIN || pos > VALID_POSITION_MAX) {
                        //     fprintf(stderr, "电机 %d 的位置值无效: %d\n", motorIndex + 1, pos);
                        //     valid = 0;
                        //     break;
                        // }
                        cmdGet.target_pos[motorIndex] = pos;
                        token = strtok(NULL, " ");
                        motorIndex++;
                    }
                    if (motorIndex != MOTOR_NUM) {
                        fprintf(stderr, "接收到的电机参数数量不匹配（期望 %d，收到 %d）\n", MOTOR_NUM, motorIndex);
                        valid = 0;
                    }
                    if (valid) {
                        // 入队命令
                        enqueue(&commandQueue, &cmdGet);
                    } else {
                        // 发送错误信息回客户端
                        char error_msg[256];
                        snprintf(error_msg, sizeof(error_msg), "Error: Invalid 'pos' command: %s\n", line);
                        send(sock, error_msg, strlen(error_msg), 0);
                    }
                } else if (strcmp(line, "home") == 0) {
                    printf("接收到 home 命令%s\n", line);
                    MotorCommandSet cmdGet;
                    memset(&cmdGet, 0, sizeof(MotorCommandSet));
                    cmdGet.mode = MODEL_HOME;
                    enqueue(&commandQueue, &cmdGet);
                } else if (strcmp(line, "enable") == 0) {
                    printf("接收到 enable 命令%s\n", line);
                    MotorCommandSet cmdGet;
                    memset(&cmdGet, 0, sizeof(MotorCommandSet));
                    cmdGet.mode = MODEL_ENABLE;
                    enqueue(&commandQueue, &cmdGet);
                } else if (strcmp(line, "disable") == 0) {
                    printf("接收到 disable 命令%s\n", line);
                    MotorCommandSet cmdGet;
                    memset(&cmdGet, 0, sizeof(MotorCommandSet));
                    cmdGet.mode = MODEL_DISABLE;
                    enqueue(&commandQueue, &cmdGet);
                } else if (strncmp(line, "load", 4) == 0) { //load 1.txt readFile
                    // 读取命令文件
                    char* filename = strtok(line, " ");
                    filename = strtok(NULL, " ");
                    printf("接收到 load 命令: %s\n", filename);
                    if (filename) {
                        readFile(filename);
                    }
                }
                else {
                    fprintf(stderr, "未知命令: %s\n", line);
                    // 发送错误信息回客户端
                    char error_msg[256];
                    snprintf(error_msg, sizeof(error_msg), "Error: Unknown command: %s\n", line);
                    send(sock, error_msg, strlen(error_msg), 0);
                }
            }
            // 获取下一行
            line = strtok(NULL, "\n");
        }
    }

    return NULL;
}
int term_init() {
    // 初始化命令队列
    initQueue(&commandQueue);

    // 创建套接字
    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        perror("套接字创建失败");
        exit(EXIT_FAILURE);
    }

    // 设置套接字选项，允许地址重用
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定套接字到地址和端口
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有接口
    server_addr.sin_port = htons(PORT);

    if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("绑定失败");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    // 开始监听
    if (listen(server_sock, 1) < 0) {
        perror("监听失败");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    printf("服务器正在监听端口 %d\n", PORT);

    // 接受客户端连接
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int temp_sock = accept(server_sock, (struct sockaddr*)&client_addr, &client_addr_len);
        if (temp_sock < 0) {
            perror("接受连接失败");
            continue;
        }

        printf("客户端已连接\n");

        pthread_mutex_lock(&client_sock_mutex);
        if (client_sock != -1) {
            // 已经有客户端连接，拒绝新的连接
            printf("已有客户端连接，拒绝新的连接\n");
            close(temp_sock);
        } else {
            client_sock = temp_sock;
            // 创建线程处理客户端
            pthread_t client_thread;
            pthread_create(&client_thread, NULL, clientHandler, (void*)&client_sock);
        }
        pthread_mutex_unlock(&client_sock_mutex);
    }

    // 关闭服务器套接字
    close(server_sock);

    return 0;
}


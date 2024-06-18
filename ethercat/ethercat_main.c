#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>
#include <sys/resource.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

#include "encoder.h"
#include "igh.h"
#include "motor.h"
#include "term.h"
/**************************命令行信息*****************************/
// 全局静态变量
extern MotorCommand currentCommand[5];
// 互斥锁
extern pthread_mutex_t cmd_lock;
int moveStartTime = 0;
/**************************命令行信息*****************************/

/************************设备信息*************************/
//alias position vid pid

enum device_info
{
    encoder_alias = 100, encoder_position = 0, encoder_vid = 0x00004321, encoder_pid = 0x01500033,
    motor1_alias = 10, motor1_position = 0, motor1_vid = 0x00004321, motor1_pid = 0x00000300,
    motor2_alias = 10, motor2_position = 1, motor2_vid = 0x00004321, motor2_pid = 0x00000300,
    motor3_alias = 10, motor3_position = 2, motor3_vid = 0x00004321, motor3_pid = 0x00000300,
};
const static struct _SlaveInfo slave_info[] = {
    {encoder_alias, encoder_position, encoder_vid, encoder_pid}, //编码器从站
    {motor1_alias, motor1_position, motor1_vid, motor1_pid}, //直线电机驱动器1
    {motor2_alias, motor2_position, motor2_vid, motor2_pid}, //直线电机驱动器2
    {motor3_alias, motor3_position, motor3_vid, motor3_pid}, //直线电机驱动器2
};
/************************设备信息*************************/

/*************************域空间*************************/
const static ec_pdo_entry_reg_t domain_regs[] = {
//编码器读取
    {encoder_alias, encoder_position, encoder_vid, encoder_pid,
    0x6100, 0x01, &slave_encoder_offset.encoder_val[0]},
    {encoder_alias, encoder_position, encoder_vid, encoder_pid,
    0x6100, 0x02, &slave_encoder_offset.encoder_val[1]},
    {encoder_alias, encoder_position, encoder_vid, encoder_pid,
    0x6100, 0x03, &slave_encoder_offset.encoder_val[2]},
//直线电机1
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6060,0x00,&slave_motor_offset[0].operation_mode},
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6040,0x00,&slave_motor_offset[0].ctrl_word}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6041,0x00,&slave_motor_offset[0].status_word}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x607A,0x00,&slave_motor_offset[0].target_pos}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6081,0x00,&slave_motor_offset[0].max_spd}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6064,0x00,&slave_motor_offset[0].current_pos}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6092,0x01,&slave_motor_offset[0].step_div},
//直线电机2
    {motor2_alias, motor2_position, motor2_vid, motor2_pid,
        0x6060,0x00,&slave_motor_offset[1].operation_mode}, 
    {motor2_alias, motor2_position, motor2_vid, motor2_pid,
        0x6040,0x00,&slave_motor_offset[1].ctrl_word}, 
    {motor2_alias, motor2_position, motor2_vid, motor2_pid,
        0x6041,0x00,&slave_motor_offset[1].status_word}, 
    {motor2_alias, motor2_position, motor2_vid, motor2_pid,
        0x607A,0x00,&slave_motor_offset[1].target_pos}, 
    {motor2_alias, motor2_position, motor2_vid, motor2_pid,
        0x6081,0x00,&slave_motor_offset[1].max_spd}, 
    {motor2_alias, motor2_position, motor2_vid, motor2_pid,
        0x6064,0x00,&slave_motor_offset[1].current_pos},
    {motor2_alias, motor2_position, motor2_vid, motor2_pid,
        0x6092,0x01,&slave_motor_offset[1].step_div},
//直线电机3
    {motor3_alias, motor3_position, motor3_vid, motor3_pid,
        0x6060,0x00,&slave_motor_offset[2].operation_mode}, 
    {motor3_alias, motor3_position, motor3_vid, motor3_pid,
        0x6040,0x00,&slave_motor_offset[2].ctrl_word}, 
    {motor3_alias, motor3_position, motor3_vid, motor3_pid,
        0x6041,0x00,&slave_motor_offset[2].status_word}, 
    {motor3_alias, motor3_position, motor3_vid, motor3_pid,
        0x607A,0x00,&slave_motor_offset[2].target_pos}, 
    {motor3_alias, motor3_position, motor3_vid, motor3_pid,
        0x6081,0x00,&slave_motor_offset[2].max_spd}, 
    {motor3_alias, motor3_position, motor3_vid, motor3_pid,
        0x6064,0x00,&slave_motor_offset[2].current_pos},
    {motor3_alias, motor3_position, motor3_vid, motor3_pid,
        0x6092,0x01,&slave_motor_offset[2].step_div},
    {},
};
struct _Domain _domain;
/*************************域空间*************************/


volatile bool g_quit = false;
const struct timespec cycletime = {0, PERIOD_NS};


/* EtherCAT 主站 */
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

/**
 * 循环任务
 */
void cyclic_task(struct _SlaveConfig *slave_config, struct _Domain *domain)
{
    uint16_t command[3] = {0x004F, 0x004F, 0x004F};
    uint16_t status;
    /* 用于确定状态字的值 */
    struct timespec wakeupTime;
    uint32_t count = 0;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    /* 获取当前时间 */
    clock_gettime(CLOCK_TO_USE, &wakeupTime);
    bool over_state = false;
    while (!g_quit) {
        /* 周期时间：1ms */
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif
        /* 接收过程数据 */
        ecrt_master_receive(master);
        ecrt_domain_process(domain->domain);

        /* 检查过程数据状态（可选） */
        check_domain_state(domain);

        /* 检查主站状态（可选） */
        check_master_state(master,master_state);

        /* 检查从站配置状态（可选） */
        check_slave_config_state(slave_config,1);

#ifdef MEASURE_TIMING
        /* 不要频繁打印，否则会影响实时性能 */
        count++;
        if (count == FREQUENCY) {
            // 输出计时统计信息
            // printf("period     min: %10u(ns) ... max: %10u(ns)\n",
            //         period_min_ns, period_max_ns);
            // printf("exec       min: %10u(ns) ... max: %10u(ns)\n",
            //         exec_min_ns, exec_max_ns);
            // printf("latency    min: %10u(ns) ... max: %10u(ns)\n",
            //         latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;

            count = 0;
        }
#endif
    if(over_state == false) {
        for (int i = 0; i < 3; i++) {
            /* 读取状态字 */
            status = EC_READ_U16(domain->domain_pd + slave_motor_offset[i].status_word);
            if ((status & command[i]) == 0x0040) { // 初始状态设置操作模式、加减速
                /* 设置操作模式为PP模式 */
                EC_WRITE_S8(domain->domain_pd + slave_motor_offset[i].operation_mode, MODEL_PP);
                /* 设置最大速度 */
                EC_WRITE_U32(domain->domain_pd + slave_motor_offset[i].max_spd, 10000);
                EC_WRITE_U32(domain->domain_pd + slave_motor_offset[i].step_div, 10000);//不细分，5um

                EC_WRITE_U16(domain->domain_pd + slave_motor_offset[i].ctrl_word, 0x0006); // 电机得电
                command[i] = 0x006F;
                //printf("statue 1 \n");
            } else if ((status & command[i]) == 0x0021) {
                EC_WRITE_U16(domain->domain_pd + slave_motor_offset[i].ctrl_word, 0x0007); // 电机使能
                command[i] = 0x006F;
                //printf("statue 2 \n");
            } else if ((status & command[i]) == 0x0023) {
                EC_WRITE_U16(domain->domain_pd + slave_motor_offset[i].ctrl_word, 0x000f); // 电机启动
                command[i] = 0x006F;
                //printf("statue 3 \n");
            } else if ((status & command[i]) == 0x0027) { // 初始化完成
                //printf("init motor over");
                EC_WRITE_U16(domain->domain_pd + slave_motor_offset[i].ctrl_word, 0x002f); // 电机启动
                over_state = true;
            }
        }
    }

    if(over_state) {
        pthread_mutex_lock(&cmd_lock); // 加锁
        for (int i = 0; i < 3; i++) {
            if (currentCommand[i].commandAvailable == 1)
            {
                EC_WRITE_S32(domain->domain_pd + slave_motor_offset[i].target_pos, currentCommand[i].distance);
                EC_WRITE_U16(domain->domain_pd + slave_motor_offset[i].ctrl_word, 0x003F); // 发送
                printf("cmd %d : %d \n", i, currentCommand[i].distance);
                currentCommand[i].commandAvailable = 2;//运动中
            }
            else if (currentCommand[i].commandAvailable == 2 && (EC_READ_S32(domain->domain_pd + slave_motor_offset[i].current_pos) == currentCommand[i].distance)) //到位
            {
                int val = EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[i]);
                printf("val %d : %d \n", i, val);
                EC_WRITE_U16(domain->domain_pd + slave_motor_offset[i].ctrl_word, 0x002f); // 电机启动
                currentCommand[i].commandAvailable = 0;
            }
        }
        pthread_mutex_unlock(&cmd_lock); // 解锁
    }

        ecrt_domain_queue(domain->domain);
        ecrt_master_send(master);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }
}
/**
 * 主函数
 */
void* ethercatMaster(void* arg)
{
    int status = 0, ret = -1;
    int SLAVE_NUM = sizeof(slave_info) / sizeof(struct _SlaveInfo);

    struct _SlaveConfig slave_config[SLAVE_NUM];
    memset(&slave_config, 0, sizeof(slave_config));
    memset(&_domain, 0, sizeof(_domain));

    /* 请求EtherCAT主站 */
    master = ecrt_request_master(0);
    if (!master) {
        printf("--> main: Request master failed.\n");
        return (void*)-1;
    }
    printf("--> main: Request master success.\n");


    /* 创建Domain */
    _domain.domain = ecrt_master_create_domain(master);

    if (!_domain.domain) {
        status = -1;
        printf("--> main: Create domain failed.\n");
        goto err_leave;
    }
    printf("--> main: Create domain success.\n");

    /* 获取从站配置 ，slave_info从站信息*/
    /*  alias：从站别名、position：从站的位置、vendor_id：从站的供应商ID*/
    for (int i = 0; i < SLAVE_NUM; i++) {
        slave_config[i].sc = ecrt_master_slave_config(master, slave_info[i].alias,
                    slave_info[i].position, slave_info[i].vendor_id,
                    slave_info[i].product_code);
                    
        if (!slave_config[i].sc) {
            status = -1;
            printf("--> main: Get slave configuration failed.\n");
            goto err_leave;
        }
    }
    printf("--> main: Get slave configuration success.\n");

    /* 配置PDO *///配置mapping
    for (int i = 0; i < SLAVE_NUM; i++) {
        if(slave_info[i].product_code == encoder_pid)
            ret = ecrt_slave_config_pdos(slave_config[i].sc, EC_END, slave_encoder_syncs);
        else if(slave_info[i].product_code == motor1_pid || slave_info[i].product_code == motor2_pid)
        {
            ret = ecrt_slave_config_pdos(slave_config[i].sc, EC_END, slave_motor_syncs);
        }
        else ret = -1;
        if (ret != 0) {
            status = -1;
            printf("--> main: Configuration PDO failed.\n");
            goto err_leave;
        }
    }
    /* 注册PDO条目到Domain */
    ret = ecrt_domain_reg_pdo_entry_list(_domain.domain, domain_regs);
    if (ret != 0) {
        status = -1;
        printf("--> main: Failed to register bunch of PDO entries for domain.\n");
        goto err_leave;
    }
    printf("--> main: Success to register bunch of PDO entries for domain.\n");

    /* 激活EtherCAT主站 */
    ret = ecrt_master_activate(master);
    if (ret < 0) {
        status = -1;
        printf("--> main: Activate master failed.\n");
        goto err_leave;
    }
    printf("--> main: Activate master success.\n");

    /* 获取过程数据内存的指针 */
    _domain.domain_pd = ecrt_domain_data(_domain.domain);
    if (!_domain.domain_pd) {
        status = -1;
        printf("--> main: Get pointer to the process data memory failed.\n");
        goto err_leave;
    }
    printf("--> main: Get pointer to the process data memory success.\n");

    /* 设置进程优先级 */
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19)) {
        printf("--> main: Warning: Failed to set priority: %s\n", strerror(errno));
    }
    /**/
    /* 开始循环任务 */
    printf("--> main: Enter cycle task now...\n");
    cyclic_task(slave_config, &_domain);
err_leave:
    /* 释放EtherCAT主站 */
    ecrt_release_master(master);
    printf("--> main: Release master.\n");
    return (void*)0;
}

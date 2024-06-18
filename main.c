#include <unistd.h>
#include <pthread.h>
#include <stdio.h>

#include "igh.h"
#include "term.h"
#include "can_drive.h"
int main(int argc, char **argv)
{
	//初始化can总线
	int socket = canInit();
	//创建控制命令线程
    pthread_t cmdrecv_thread;
    pthread_create(&cmdrecv_thread, NULL, readCommands,NULL);
    
	//创建ethercat线程
    pthread_t ethercat_thread;
    pthread_create(&ethercat_thread, NULL, ethercatMaster,NULL);

	//创建canopen发送线程
    pthread_t cansend_thread;
    pthread_create(&cansend_thread, NULL, canSend,&socket);

	//创建canopen接收线程
	pthread_t canrecv_thread;
    pthread_create(&canrecv_thread, NULL, canRecv,&socket);
	
	//等待线程结束
	pthread_join(cmdrecv_thread, NULL);
	pthread_join(ethercat_thread, NULL);
	pthread_join(cansend_thread, NULL);
	pthread_join(canrecv_thread, NULL);
	while(1)
	{
		sleep(1);
	}
	return 0;
}
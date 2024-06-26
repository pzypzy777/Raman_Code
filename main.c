#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#define __USE_GNU  
#include <sched.h>
#include <pthread.h>

#include "tran.h"

void receive_PC_to_PS(void);
void send_PL_to_PC(void);

int main(int argc, char *argv[])      
{                              


	pthread_t t_send;
	pthread_t t_rece;

	pthread_attr_t attr1;
	pthread_attr_t attr2;

	pthread_attr_init(&attr1);
	pthread_attr_init(&attr2);

	cpu_set_t cpu_info;
	CPU_ZERO(&cpu_info);
	CPU_SET(1, &cpu_info);
	if (0!=pthread_attr_setaffinity_np(&attr1, sizeof(cpu_set_t), &cpu_info))
	{
		printf("set affinity failed");
		return 0;
	}

	CPU_ZERO(&cpu_info);
	CPU_SET(0, &cpu_info);
	if (0!=pthread_attr_setaffinity_np(&attr2, sizeof(cpu_set_t), &cpu_info))
	{
		printf("set affinity failed");
	}

	// 创建线程A
	if (pthread_create(&t_send, &attr1, (void *)&send_PL_to_PC, NULL) == -1){
		puts("fail to create pthread t_ad");
		exit(1);
	}
	// 创建线程B
	if (pthread_create(&t_rece, &attr2, (void *)&receive_PC_to_PS, NULL) == -1){
		puts("fail to create pthread t_pc");
		exit(1);
	}


	// 等待线程结束
	void * result;
	if (pthread_join(t_send, &result) == -1){
		puts("fail to recollect t_send");
		exit(1);
	}

	if (pthread_join(t_rece, &result) == -1){
		puts("fail to recollect t_rece");
		exit(1);
	}


	return 0;       //一般用在主函数结束时，按照程序开发的一般惯例，表示成功完成本函数                   
}      


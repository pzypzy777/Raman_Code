#include "ad2pc.h"
#include "tran.h"

#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#define __USE_GNU  
#include <sched.h>
#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <limits.h>

#include <sys/stat.h>
#include <assert.h>

#include <sys/time.h> 
#include<unistd.h>
#include<signal.h>
#include<math.h>
#include<complex.h>

#include <termios.h>


#define RECV_BUFFER_SIZE     25 
#define stm32_RECV_BUFFER_SIZE     12
#define stm32_SEND_BUFFER_SIZE     16

/*gpio init*/
int valuefd1, valuefd2, valuefd3, valuefd4, exportfd, directionfd;
/*irq init*/
int irqfd;
unsigned char key_val;

/*uart irq init*/
int uart_fd;
unsigned char key_val_uart;

int uart_ready = 0;
int start_frame = 0;
unsigned char recv_buffer[RECV_BUFFER_SIZE];
unsigned char rubbish_buffer[RECV_BUFFER_SIZE];
int uart_valid = 0;
int len_u = 0;

/*stm32 uart irq init*/
int stm32_uart_fd;
unsigned char key_val_stm32_uart;

int stm32_uart_ready = 0;
int stm32_start_frame = 0;
unsigned char stm32_recv_buffer[stm32_RECV_BUFFER_SIZE];
unsigned char stm32_rubbish_buffer[stm32_RECV_BUFFER_SIZE];
int stm32_uart_valid = 0;
int stm32_len_u = 0;

/*tec irq init*/
int irqfd_tec;
unsigned char key_val_tec;

//
int pc_send_ready = 0;
int tec_ready = 0;
int stop_sig = 1;
int time_jf = 0;
int time_flag = 0;
int send_cnt_max = 10;
int ccd_ctrl = 0;
int send_cnt = 0;

int pc_send_cnt = 0;

int sockfd;
int new_tcpserversockfd;
volatile int disconnect = 0;

//PID 
float CCD_kp=1,CCD_ki=0,CCD_kd=1;
unsigned int CCD_kT = 20000,Light_kT = 20000;
float Light_kp,Light_ki,Light_kd;
int error_data = 0,error_data_1 = 0,error_data_2 = 0,error_data_x[3],dout_data=0;

//light temperature
unsigned int Light_ad_data;
unsigned int Light_state;
unsigned int Light_temp_state;//Light temperature state
unsigned char stm32_send_buffer[stm32_SEND_BUFFER_SIZE];

unsigned int work_mode;

void eth_send(unsigned char* vir_pl_addr3, unsigned long size)
{
	int eth_length, send_length;
	for (eth_length = 0; eth_length < size; eth_length = eth_length + 256)
	{
		send_length = size - eth_length;
		if (send_length >= 256)
			send(new_tcpserversockfd, vir_pl_addr3 + eth_length, 256, 0);
		else
			send(new_tcpserversockfd, vir_pl_addr3 + eth_length, send_length, 0);
		//usleep(100000);
	}
}

int uart_configure(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)  !=  0) { 
		perror("SetupSerial 1\n");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//printf("1\n");
 
	switch( nBits )
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}
 
	switch( nEvent )
	{
		case 'O':
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E': 
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N':  
			newtio.c_cflag &= ~PARENB;
			break;
	}
 
	switch( nSpeed )
	{
		case 2400:
			cfsetispeed(&newtio, B2400);
			cfsetospeed(&newtio, B2400);
			break;
		case 4800:
			cfsetispeed(&newtio, B4800);
			cfsetospeed(&newtio, B4800);
			break;
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
		case 460800:
			cfsetispeed(&newtio, B460800);
			cfsetospeed(&newtio, B460800);
			break;
		default:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
	}
 
	if( nStop == 1 )
		newtio.c_cflag &=  ~CSTOPB;
	else if ( nStop == 2 )
		newtio.c_cflag |=  CSTOPB;
 
	newtio.c_cc[VTIME]  = 50;//time 1 = 0.1s
	newtio.c_cc[VMIN] = 100;//DATA_LEN
	//printf("2\n");
	tcflush(fd,TCIFLUSH);
 
	//printf("3\n");	
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
	//printf("4\n");
	
	return 0;
}

void my_signal_fun_tec(int signum)
{

	//printf("irq app printf signum=%d!\n",signum);

	read(irqfd_tec, &key_val_tec, 1);
	//printf(" CCD temperature interrupt!\n ");
	//printf("key_val_tec= %d\n", key_val_tec);


	//if (key_val_tec == 48)
	//{
	//	key_val_tec = 0;
		tec_ready = 1;
	//}
}

void my_signal_fun_irq(int signum)
{

	//printf("irq app printf signum=%d!\n",signum);
	read(irqfd, &key_val, 1);
	//printf(" CCD interrupt!\n ");
	//printf("key_val= %d\n", key_val);

	//if (key_val == 49)
	//{
	pc_send_cnt = pc_send_cnt + 1;
	pc_send_ready = 1;
	//	key_val = 0;
	//}
}

void my_signal_fun_uart(int signum)
{
	/*
	unsigned char tmpdata;
	//printf("irq app printf signum=%d!\n",signum);
	//printf(" PC uart interrupt!\n ");
	ioctl(uart_fd,FIONREAD,&len_u);
	if(len_u > 0)
	{
		read(uart_fd, &tmpdata, 1);
		if(tmpdata == 0xAA && start_frame == 0)
			start_frame = 1;
		if(start_frame == 1)
		{
			recv_buffer[uart_ready++] = tmpdata;
		}
		if(uart_ready >= RECV_BUFFER_SIZE)
		{
			uart_ready = 0;
			start_frame = 0;
			uart_valid = 1;
		}
	}
	//read(uart_fd, &key_val_uart, 1);
	//printf("tmpdata= %x\n", tmpdata);
	//printf("len_u= %x\n", len_u);
	//printf("uart_ready= %d\n", uart_ready);
	//ioctl(uart_fd,FIONREAD,&len_u);
	//if(len_u > 0)
	//	read(uart_fd, rubbish_buffer, len_u);
	//printf("len_u= %d\n", len_u);
   */
}


void my_signal_fun_stm32_uart(int signum)
{
	unsigned char tmpdata;
	//printf(" stm32 uart interrupt!\n ");
	ioctl(stm32_uart_fd,FIONREAD,&stm32_len_u);
	if(stm32_len_u > 0)
	{
		read(stm32_uart_fd, &tmpdata, 1);
		if(tmpdata == 0x6A && stm32_start_frame == 0)
			stm32_start_frame = 1;
		if(stm32_start_frame == 1)
		{
			stm32_recv_buffer[stm32_uart_ready++] = tmpdata;
		}
		if(stm32_uart_ready >= stm32_RECV_BUFFER_SIZE)
		{
			stm32_uart_ready = 0;
			stm32_start_frame = 0;
			stm32_uart_valid = 1;
		}
	}
	//printf("stm32_uart_ready= %d\n", stm32_uart_ready);
	//ioctl(stm32_uart_fd,FIONREAD,&stm32_len_u);
	//if(stm32_len_u > 0)
	//	read(stm32_uart_fd, stm32_rubbish_buffer, stm32_len_u);
	//printf("stm32_len_u= %d\n", stm32_len_u);
}

void receive_PC_to_PS(void)
{
	//unsigned char recv_buffer[RECV_BUFFER_SIZE];
	bzero(recv_buffer, RECV_BUFFER_SIZE);
	int recv_flag = -1;


	while (1)
	{
		recv_flag = recv(new_tcpserversockfd, recv_buffer, RECV_BUFFER_SIZE, 0);
		//printf("%d\n", recv_flag);
		if (recv_flag > 0)
		{
			if (recv_buffer[0] == 0xAA &&
				recv_buffer[1] == 0xA1 &&
				//recv_buffer[7] == 0x0D &&
				recv_buffer[24] == 0x0A)//数据头AA A1//帧尾0D 0A//
			{
				//改变积分时间
				time_flag = 1;
				work_mode = recv_buffer[2];
				time_jf = recv_buffer[4] | recv_buffer[5] << 8;
				if (time_jf < 5)
					time_jf = 5;
				//send_cnt_max = recv_buffer[7];
				ccd_ctrl = recv_buffer[3];
				send_cnt = 0;

				if (recv_buffer[6] == 0x01)
				{
					stop_sig = 0;//单次
				}
				else if (recv_buffer[6] == 0x02)
				{
					stop_sig = 1;//多次（10次）
					send_cnt_max = recv_buffer[7];
				}
				else if (recv_buffer[6] == 0x03)
				{
					stop_sig = 2;//连续-需要暂停
				}
				else if (recv_buffer[6] == 0x04)
				{
					stop_sig = 3;//暂停
				}
				else if (recv_buffer[6] == 0x05)
				{
					disconnect = 1;//断开连接 串口不需要
				}

				error_data = 0;
				error_data_1 = 0;
				error_data_2 = 0;

				CCD_kp = (float)((short)(recv_buffer[8] | recv_buffer[9] << 8)) / 100.0;
				CCD_ki = (float)((short)(recv_buffer[10] | recv_buffer[11] << 8)) / 100.0;
				CCD_kd = (float)((short)(recv_buffer[12] | recv_buffer[13] << 8)) / 100.0;
				CCD_kT = (recv_buffer[14] | recv_buffer[15] << 8);

				Light_kp = (float)((short)(recv_buffer[16] | recv_buffer[17] << 8)) / 100.0;
				Light_ki = (float)((short)(recv_buffer[18] | recv_buffer[19] << 8)) / 100.0;
				Light_kd = (float)((short)(recv_buffer[20] | recv_buffer[21] << 8)) / 100.0;
				Light_kT = (recv_buffer[22] | recv_buffer[23] << 8);
				printf("CCD_kp = %f CCD_ki = %f stop_sig = %f CCD_kT = %d\n", CCD_kp, CCD_ki, CCD_kd, CCD_kT);
				printf("Light_kp = %f Light_ki = %f Light_kd = %f Light_kT = %d\n", Light_kp, Light_ki, Light_kd, Light_kT);

				stm32_send_buffer[0] = 0xA1;
				stm32_send_buffer[1] = 0xA1;
				stm32_send_buffer[2] = recv_buffer[16];
				stm32_send_buffer[3] = recv_buffer[17];
				stm32_send_buffer[4] = recv_buffer[18];
				stm32_send_buffer[5] = recv_buffer[19];
				stm32_send_buffer[6] = recv_buffer[20];
				stm32_send_buffer[7] = recv_buffer[21];
				stm32_send_buffer[8] = recv_buffer[22];
				stm32_send_buffer[9] = recv_buffer[23];
				if (stop_sig == 0 || stop_sig == 1 || stop_sig == 2)
					stm32_send_buffer[10] = 0x01;
				else
					stm32_send_buffer[10] = 0x00;
				stm32_send_buffer[11] = 0x00;
				stm32_send_buffer[12] = 0x00;
				stm32_send_buffer[13] = 0x00;
				stm32_send_buffer[14] = 0x1A;
				stm32_send_buffer[15] = 0x1A;
				write(stm32_uart_fd, stm32_send_buffer, stm32_SEND_BUFFER_SIZE);
				//disconnect = 1; 断开链接——uart不需要，网口需要
				//recv_buffer[6] == 0x00;
			}
			else
				printf(" PC uart data error!\n ");
		}
		/*
		//PC uart
		if(uart_valid == 1)
		{
			uart_valid = 0;
			write(uart_fd,recv_buffer,RECV_BUFFER_SIZE);
			//recv_flag = read(uart_fd, recv_buffer, RECV_BUFFER_SIZE);//recv(new_tcpserversockfd, recv_buffer, RECV_BUFFER_SIZE, 0);
			//printf("%d\n", recv_flag);
			//if(recv_flag > 0)
			{
				if (recv_buffer[0] == 0xAA &&
					recv_buffer[1] == 0xA1 &&
					//recv_buffer[7] == 0x0D &&
					recv_buffer[24] == 0x0A)//数据头AA A1//帧尾0D 0A//
				{
					//改变积分时间
					time_flag = 1;
					work_mode = recv_buffer[2];
					time_jf = recv_buffer[4] | recv_buffer[5]<<8;
					if(time_jf < 5)
						time_jf = 5;
					//send_cnt_max = recv_buffer[7];
					ccd_ctrl = recv_buffer[3];
					send_cnt = 0;

					if (recv_buffer[6] == 0x01)
					{
						stop_sig = 0;//单次
					}
					else if (recv_buffer[6] == 0x02)
					{
						stop_sig = 1;//多次（10次）
						send_cnt_max = recv_buffer[7];
					}
					else if(recv_buffer[6] == 0x03)
					{
						stop_sig = 2;//连续-需要暂停
					}
					else if(recv_buffer[6] == 0x04)
					{
						stop_sig = 3;//暂停
					}

					error_data = 0;
					error_data_1 = 0;
					error_data_2 = 0;

					CCD_kp = (float)((short)(recv_buffer[8] | recv_buffer[9]<<8))/100.0;
					CCD_ki = (float)((short)(recv_buffer[10] | recv_buffer[11]<<8))/100.0;
					CCD_kd = (float)((short)(recv_buffer[12] | recv_buffer[13]<<8))/100.0;
					CCD_kT = (recv_buffer[14] | recv_buffer[15]<<8);
					
					Light_kp = (float)((short)(recv_buffer[16] | recv_buffer[17]<<8))/100.0;
					Light_ki = (float)((short)(recv_buffer[18] | recv_buffer[19]<<8))/100.0;
					Light_kd = (float)((short)(recv_buffer[20] | recv_buffer[21]<<8))/100.0;
					Light_kT = (recv_buffer[22] | recv_buffer[23]<<8);
					printf("CCD_kp = %f CCD_ki = %f stop_sig = %f CCD_kT = %d\n",CCD_kp,CCD_ki,CCD_kd,CCD_kT);
					printf("Light_kp = %f Light_ki = %f Light_kd = %f Light_kT = %d\n",Light_kp,Light_ki,Light_kd,Light_kT);
					
					stm32_send_buffer[0] = 0xA1;
					stm32_send_buffer[1] = 0xA1;
					stm32_send_buffer[2] = recv_buffer[16];
					stm32_send_buffer[3] = recv_buffer[17];
					stm32_send_buffer[4] = recv_buffer[18];
					stm32_send_buffer[5] = recv_buffer[19];
					stm32_send_buffer[6] = recv_buffer[20];
					stm32_send_buffer[7] = recv_buffer[21];
					stm32_send_buffer[8] = recv_buffer[22];
					stm32_send_buffer[9] = recv_buffer[23];
					if(stop_sig == 0 || stop_sig == 1 || stop_sig == 2)
						stm32_send_buffer[10] = 0x01;
					else
						stm32_send_buffer[10] = 0x00;
					stm32_send_buffer[11] = 0x00;
					stm32_send_buffer[12] = 0x00;
					stm32_send_buffer[13] = 0x00;					
					stm32_send_buffer[14] = 0x1A;
					stm32_send_buffer[15] = 0x1A;
					write(stm32_uart_fd,stm32_send_buffer,stm32_SEND_BUFFER_SIZE);
					//disconnect = 1; 断开链接——uart不需要，网口需要
					//recv_buffer[6] == 0x00;
				}
				else
					printf(" PC uart data error!\n ");
			}
			ioctl(uart_fd,FIONREAD,&len_u);
			if(len_u > 0)
			{
				read(uart_fd, rubbish_buffer, len_u);
				uart_ready = 0;
				start_frame = 0;
			}
		}

		//stm32 uart
		if(stm32_uart_valid == 1)
		{
			stm32_uart_valid = 0;
			//write(stm32_uart_fd,stm32_recv_buffer,stm32_RECV_BUFFER_SIZE);
			if (stm32_recv_buffer[0] == 0x6A &&
				stm32_recv_buffer[1] == 0x6A &&
				stm32_recv_buffer[stm32_RECV_BUFFER_SIZE-2] == 0x6B &&
				stm32_recv_buffer[stm32_RECV_BUFFER_SIZE-1] == 0x6B)//数据头AA A1//帧尾0D 0A//
			{
				Light_ad_data = (stm32_recv_buffer[4] | stm32_recv_buffer[5]<<8);
				Light_state = stm32_recv_buffer[6];
				Light_temp_state = stm32_recv_buffer[7];
			}
			else
				printf(" PC uart data error!\n ");		
			ioctl(stm32_uart_fd,FIONREAD,&stm32_len_u);
			if(stm32_len_u > 0)
			{	
				read(stm32_uart_fd, stm32_rubbish_buffer, stm32_len_u);
				stm32_uart_ready = 0;
				stm32_start_frame = 0;
			}
			//printf("stm32_len_u= %d\n", stm32_len_u);
		}
		*/
	}
}

void send_PL_to_PC(void)
{
	//int sofd;
	sockfd = CreateTcpServerSock(8000);
	//int sockfd;
	unsigned long pl_addr1 = 0x10000000; //300M //0x16800000; //0x10000000;//0x20000000;   //devicetree，reg=<0x0 0x10000000>;
	int fd = open("/dev/mem", O_RDWR);
	if (fd == -1)
	{
		printf("open /dev/mem error.\n");
		return;
	}

	//unsigned char* vir_pl_addr1, *vir_pl_addr2, *vir_pl_addr3, *vir_pl_addr4, *vir_pl_addr5, *vir_pl_addr6; //*virtual_pladdr3;
	unsigned char* vir_pl_addr1; //*virtual_pladdr3;

	//unsigned long size = 640*480*4; 
	unsigned long size = 4268;  //314572800; //81920000;//100*1024*1024; //5a5a+4096+2048*2+5b5b = 4102// 5a5a+4096+2128*2+5b5b = 4262

	/* map first buffer of pl ddr to linux userspace */
	vir_pl_addr1 = (unsigned char*)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)pl_addr1); //linux中的off_t类型默认是32位的long int                                                                                     
	if (vir_pl_addr1 == MAP_FAILED)
	{
		perror("vir_pl_addr1 map to user space failed.\n");
		return;
	}
	else
		printf("vir_pl_addr1 map success.\n");


	//gpio init
	printf("GPIO test running...\n");
	// The GPIO has to be exported to be able to see it
	// in sysfs
	exportfd = open("/sys/class/gpio/export", O_WRONLY);
	if (exportfd < 0)
	{
		printf("Cannot open GPIO to export it\n");
		exit(1);
	}

	write(exportfd, "960", 4);
	write(exportfd, "961", 4);
	write(exportfd, "962", 4);
	write(exportfd, "963", 4);
	close(exportfd);
	printf("GPIO exported successfully\n");
	// Update the direction of the GPIO to be an output
	directionfd = open("/sys/class/gpio/gpio960/direction", O_RDWR);
	if (directionfd < 0)
	{
		printf("Cannot open GPIO direction it\n");
		exit(1);
	}

	write(directionfd, "out", 4);
	close(directionfd);

	directionfd = open("/sys/class/gpio/gpio961/direction", O_RDWR);
	if (directionfd < 0)
	{
		printf("Cannot open GPIO direction it\n");
		exit(1);
	}
	write(directionfd, "out", 4);
	close(directionfd);

	directionfd = open("/sys/class/gpio/gpio962/direction", O_RDWR);
	if (directionfd < 0)
	{
		printf("Cannot open GPIO direction it\n");
		exit(1);
	}
	write(directionfd, "out", 4);
	close(directionfd);

	directionfd = open("/sys/class/gpio/gpio963/direction", O_RDWR);
	if (directionfd < 0)
	{
		printf("Cannot open GPIO direction it\n");
		exit(1);
	}
	write(directionfd, "out", 4);
	close(directionfd);

	printf("GPIO direction set as output successfully\n");

	// Get the GPIO value ready to be toggled
	valuefd1 = open("/sys/class/gpio/gpio960/value", O_RDWR);
	if (valuefd1 < 0)
	{
		printf("Cannot open GPIO value\n");
		exit(1);
	}
	valuefd2 = open("/sys/class/gpio/gpio961/value", O_RDWR);
	if (valuefd2 < 0)
	{
		printf("Cannot open GPIO value\n");
		exit(1);
	}
	valuefd3 = open("/sys/class/gpio/gpio962/value", O_RDWR);
	if (valuefd3 < 0)
	{
		printf("Cannot open GPIO value\n");
		exit(1);
	}
	valuefd4 = open("/sys/class/gpio/gpio963/value", O_RDWR);
	if (valuefd4 < 0)
	{
		printf("Cannot open GPIO value\n");
		exit(1);
	}
	write(valuefd1, "0", 2);
	write(valuefd2, "0", 2);
	write(valuefd3, "0", 2);
	write(valuefd4, "0", 2);

	/*irq init*/
	//int ret;
	int Oflags;

	signal(SIGIO, my_signal_fun_tec);
	signal(SIGWINCH, my_signal_fun_irq);
	//signal(SIGUSR1, my_signal_fun_uart);
	signal(SIGUSR2, my_signal_fun_stm32_uart);

	irqfd = open("/dev/irq_drv", O_RDWR);
	if (irqfd < 0)
	{
		printf("can't open irq_drv!\n");
	}

	fcntl(irqfd, F_SETOWN, getpid());

	Oflags = fcntl(irqfd, F_GETFD);

	fcntl(irqfd, F_SETFL, Oflags | FASYNC);
	fcntl(irqfd, 10, SIGWINCH);

	/*irq tec init*/
	//int ret;
	//int Oflags_tec;

	//signal(SIGIO, my_signal_fun);

	irqfd_tec = open("/dev/irq_tec", O_RDWR);
	if (irqfd_tec < 0)
	{
		printf("can't open irq_tec!\n");
	}

	fcntl(irqfd_tec, F_SETOWN, getpid());

	Oflags = fcntl(irqfd_tec, F_GETFD);

	fcntl(irqfd_tec, F_SETFL, Oflags | FASYNC);

	int i;
	unsigned char* vir_pl_addr2; //*virtual_pladdr3;
	unsigned long pl_addr2 = 0x43C00000;
	char c_test[6];
	unsigned int ad_data;
	vir_pl_addr2 = (unsigned char*)mmap(NULL, 20, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)pl_addr2); //linux中的off_t类型默认是32位的long int

   /*
	// uart init
	uart_fd = open("/dev/ttyUL1", O_RDWR | O_NOCTTY);
	//char *uart_out = "start pl uart application\r\n";
	char buffer[1500];
	memset(buffer, 0, sizeof(buffer));
	//int uart_wr = 0;
	if (uart_fd < 0)
	{
		printf("open /dev/ttyUL1 error.\n");
		return;
	}

	uart_configure(uart_fd, 115200, 8, 'N', 1);
   */

	//stm32 uart init
	stm32_uart_fd = open("/dev/ttyUL2", O_RDWR | O_NOCTTY);
	if (stm32_uart_fd < 0)
	{
		printf("open /dev/ttyUL2 error.\n");
		return;
	}

	uart_configure(stm32_uart_fd, 115200, 8, 'N', 1);
	//int recv_cnt;

/*		
	//uart
	fcntl(uart_fd, F_SETOWN, getpid());

	Oflags = fcntl(uart_fd, F_GETFD);

	fcntl(uart_fd, F_SETFL, Oflags | FASYNC);
	fcntl(uart_fd, 10, SIGUSR1);
	*/

	//stm32 uart
	fcntl(stm32_uart_fd, F_SETOWN, getpid());

	Oflags = fcntl(stm32_uart_fd, F_GETFD);

	fcntl(stm32_uart_fd, F_SETFL, Oflags | FASYNC);
	fcntl(stm32_uart_fd, 10, SIGUSR2);
	
	*(volatile unsigned char *)(vir_pl_addr2) = 1;
	*(volatile unsigned char *)(vir_pl_addr2 + 1) = 0;
	*(volatile unsigned char *)(vir_pl_addr2 + 2) = 0;
	*(volatile unsigned char *)(vir_pl_addr2 + 8) = 0;
	*(c_test + 0) = *(vir_pl_addr2 + 0);
	printf("%d\n", c_test[0]);
	int ccd_ctrl_cnt = 0;
	int ccd_ctrl_state = 0;

	unsigned char* vir_pl_addr3; //*virtual_pladdr3;
	unsigned long pl_addr3 = 0x20000000;
	vir_pl_addr3 = (unsigned char*)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)pl_addr3); //linux中的off_t类型默认是32位的long int
	
	int stop_sig_temp = 3;
	unsigned int Light_jf_time = 1;

	unsigned int Light_state_cnt = 0;
	unsigned int Light_temp_state_cnt = 0;
	
	unsigned int frame_cnt = 0;

	while (1)
	{
		struct sockaddr_in client_addr;
		socklen_t client_addr_length = sizeof(client_addr);
		printf("TCP connect...\n");
		new_tcpserversockfd = accept(sockfd, (struct sockaddr*)&client_addr, &client_addr_length);
		printf("TCP connect successful!\n");
		//eth_send(vir_pl_addr3, size);
		while (1)
		{
			//tcp disconnect
			if (disconnect == 1)
			{
				disconnect = 0;
				stop_sig = 3;
				printf("3\n");
				//shutdown(new_tcpserversockfd,SHUT_RDWR);
				//printf("4\n");
				close(new_tcpserversockfd);
				printf("5\n");
				//后续添加断电
				break;
			}

			//  后续添加断电上电功能
			ccd_ctrl_state = *(vir_pl_addr2 + 1);
			if (ccd_ctrl_state == 1 && ccd_ctrl == 0)
			{
				*(volatile unsigned char *)(vir_pl_addr2 + 1) = 0;
				*(volatile unsigned char *)(vir_pl_addr2 + 2) = 0;
			}
			else if (ccd_ctrl_state == 0 && ccd_ctrl == 1)
			{
				*(volatile unsigned short *)(vir_pl_addr2 + 12) = 0;

				*(volatile unsigned char *)(vir_pl_addr2 + 1) = 1;
				usleep(100000);
				*(volatile unsigned char *)(vir_pl_addr2 + 2) = 1;

				while (1)
				{
					//printf(" restart!\n ");
					if (pc_send_ready == 1)
					{
						ccd_ctrl_cnt = ccd_ctrl_cnt + 1;
						pc_send_ready = 0;
					}
					if (ccd_ctrl_cnt >= 5)
					{
						ccd_ctrl_cnt = 0;
						break;
					}
				}

				*(volatile unsigned short *)(vir_pl_addr2 + 12) = time_jf - 5;
				*(volatile unsigned char *)(vir_pl_addr2 + 2) = 0;
				usleep(100000);
				*(volatile unsigned char *)(vir_pl_addr2 + 2) = 1;
				pc_send_ready = 0;
				while (1)
				{
					//printf(" restart!\n ");
					if (pc_send_ready == 1)
					{
						pc_send_ready = 0;
						break;
					}
				}
			}

			//tec 测温 + PID
			if (tec_ready == 1)
			{
				for (i = 0; i < 2; i++)
				{
					*(c_test + i) = *(vir_pl_addr2 + 4 + i);
					//printf("%d\n", c_test[i]);
				}
				ad_data = (c_test[1] << 8) + c_test[0];
				ad_data = ad_data * 51000 / 65535 * 2;//改三线式  后

				//P I D

				error_data = (CCD_kT - ad_data);
				error_data_x[0] = error_data - error_data_1;
				error_data_x[1] = error_data - 2 * error_data_1 + error_data_2;
				error_data_x[2] = error_data;
				error_data_2 = error_data_1;
				error_data_1 = error_data;
				//dout_float = CCD_kp * error_data_x[0]/100.0 + CCD_ki * error_data_x[1]/100.0 + CCD_kd * error_data_x[2]/100.0;
				dout_data = dout_data + CCD_kp * error_data_x[0] / 100.0 + CCD_ki * error_data_x[1] / 100.0 + CCD_kd * error_data_x[2] / 100.0;
				if (dout_data <= 0)
					dout_data = 0;
				else if (dout_data >= 100)
					dout_data = 100;

				*(volatile unsigned char *)(vir_pl_addr2 + 8) = dout_data;

				tec_ready = 0;
			}

			//stm32 uart
			if (stm32_uart_valid == 1)
			{
				stm32_uart_valid = 0;
				//write(stm32_uart_fd,stm32_recv_buffer,stm32_RECV_BUFFER_SIZE);
				if (stm32_recv_buffer[0] == 0x6A &&
					stm32_recv_buffer[1] == 0x6A &&
					stm32_recv_buffer[stm32_RECV_BUFFER_SIZE - 2] == 0x6B &&
					stm32_recv_buffer[stm32_RECV_BUFFER_SIZE - 1] == 0x6B)//数据头AA A1//帧尾0D 0A//
				{
					Light_ad_data = (stm32_recv_buffer[4] | stm32_recv_buffer[5] << 8);
					Light_state = stm32_recv_buffer[6];
					Light_temp_state = stm32_recv_buffer[7];
				}
				else
					printf(" PC uart data error!\n ");
				ioctl(stm32_uart_fd, FIONREAD, &stm32_len_u);
				if (stm32_len_u > 0)
				{
					read(stm32_uart_fd, stm32_rubbish_buffer, stm32_len_u);
					stm32_uart_ready = 0;
					stm32_start_frame = 0;
				}
				//printf("stm32_len_u= %d\n", stm32_len_u);
			}

			//积分时间
			if (time_flag == 1)
			{
				time_flag = 0;

				*(volatile unsigned short *)(vir_pl_addr2 + 12) = time_jf - 5;

			}

			//CCD-UART-LIGHT
			if (work_mode == 1 && Light_state == 0)
			{
				*(volatile unsigned short *)(vir_pl_addr2 + 12) = 10 - 5;
				Light_jf_time = 1;
				Light_temp_state_cnt = 0;
				if (Light_state_cnt == 0)
				{
					Light_state_cnt = 1;
					*(volatile unsigned char *)(vir_pl_addr3 + 4260) = ad_data % 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4261) = ad_data / 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4262) = Light_ad_data % 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4263) = Light_ad_data / 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4264) = 0;
					*(volatile unsigned char *)(vir_pl_addr3 + 4265) = ccd_ctrl_state;
					*(volatile unsigned char *)(vir_pl_addr3 + 4266) = 91;
					*(volatile unsigned char *)(vir_pl_addr3 + 4267) = 91;
					//eth_send(vir_pl_addr3, size);
					send(new_tcpserversockfd, vir_pl_addr3, size, 0);
					//write(uart_fd, vir_pl_addr3, size);
					printf("Light_state == 0\n");
				}
			}
			else if (work_mode == 1 && Light_temp_state != 1)
			{
				*(volatile unsigned short *)(vir_pl_addr2 + 12) = 10 - 5;
				Light_jf_time = 1;
				Light_state_cnt = 0;
				if (Light_temp_state_cnt == 0)
				{
					Light_temp_state_cnt = 1;
					*(volatile unsigned char *)(vir_pl_addr3 + 4260) = ad_data % 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4261) = ad_data / 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4262) = Light_ad_data % 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4263) = Light_ad_data / 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4264) = 2;
					*(volatile unsigned char *)(vir_pl_addr3 + 4265) = ccd_ctrl_state;
					*(volatile unsigned char *)(vir_pl_addr3 + 4266) = 91;
					*(volatile unsigned char *)(vir_pl_addr3 + 4267) = 91;
					//eth_send(vir_pl_addr3, size);
					send(new_tcpserversockfd, vir_pl_addr3, size, 0);
					//write(uart_fd, vir_pl_addr3, size);
					printf("Light_temp_state != 1\n");
				}
			}
			else
			{
				Light_state_cnt = 0;
				Light_temp_state_cnt = 0;
				if (work_mode == 1 && (Light_jf_time == 1))
				{
					Light_jf_time = 0;
					*(volatile unsigned short *)(vir_pl_addr2 + 12) = time_jf - 5;
					*(volatile unsigned char *)(vir_pl_addr2 + 2) = 0;
					usleep(100000);
					*(volatile unsigned char *)(vir_pl_addr2 + 2) = 1;
					pc_send_ready = 0;
					while (1)
					{
						//printf(" restart!\n ");
						if (pc_send_ready == 1)
						{
							pc_send_ready = 0;
							break;
						}
					}
					printf("up reset\n");
				}

				if (stop_sig_temp != 3 && stop_sig == 3)
				{
					stm32_send_buffer[0] = 0xA1;
					stm32_send_buffer[1] = 0xA1;
					stm32_send_buffer[2] = recv_buffer[16];
					stm32_send_buffer[3] = recv_buffer[17];
					stm32_send_buffer[4] = recv_buffer[18];
					stm32_send_buffer[5] = recv_buffer[19];
					stm32_send_buffer[6] = recv_buffer[20];
					stm32_send_buffer[7] = recv_buffer[21];
					stm32_send_buffer[8] = recv_buffer[22];
					stm32_send_buffer[9] = recv_buffer[23];
					stm32_send_buffer[10] = 0x00;
					stm32_send_buffer[11] = 0x00;
					stm32_send_buffer[12] = 0x00;
					stm32_send_buffer[13] = 0x00;
					stm32_send_buffer[14] = 0x1A;
					stm32_send_buffer[15] = 0x1A;
					write(stm32_uart_fd, stm32_send_buffer, stm32_SEND_BUFFER_SIZE);
				}
				stop_sig_temp = stop_sig;

				if (pc_send_ready == 1)
				{
					printf("send uart\n");
					frame_cnt = frame_cnt + 1;
					memcpy(vir_pl_addr3, vir_pl_addr1, size);
					//for (i = 0; i < 2; i++)
					//{
					//	*(c_test + i) = *(vir_pl_addr3 + i);
					//	printf("%d\n", c_test[i]);
					//}
					//*(c_test + 2) = *(vir_pl_addr3 + 4264);
					//printf("%d\n", c_test[2]);
					//*(c_test + 3) = *(vir_pl_addr3 + 4265);
					//printf("%d\n", c_test[3]);
					//*(c_test + 4) = *(vir_pl_addr3 + 5118);
					//printf("%d\n", c_test[4]);
					//*(c_test + 5) = *(vir_pl_addr3 + 5119);
					//printf("%d\n", c_test[5]);
					*(volatile unsigned char *)(vir_pl_addr3 + 2) = frame_cnt % 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 3) = frame_cnt / 256;

					*(volatile unsigned char *)(vir_pl_addr3 + 4260) = ad_data % 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4261) = ad_data / 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4262) = Light_ad_data % 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4263) = Light_ad_data / 256;
					*(volatile unsigned char *)(vir_pl_addr3 + 4264) = 1;
					*(volatile unsigned char *)(vir_pl_addr3 + 4265) = ccd_ctrl_state;
					*(volatile unsigned char *)(vir_pl_addr3 + 4266) = 91;
					*(volatile unsigned char *)(vir_pl_addr3 + 4267) = 91;

					if (stop_sig == 0)
					{
						//write(uart_fd, vir_pl_addr3, size);
						send(new_tcpserversockfd, vir_pl_addr3, size, 0);
						//eth_send(vir_pl_addr3, size);
						send_cnt = 0;
						stop_sig = 3;
					}
					else if (stop_sig == 1)
					{
						//write(uart_fd, vir_pl_addr3, size);
						send(new_tcpserversockfd, vir_pl_addr3, size, 0);
						//eth_send(vir_pl_addr3, size);
						send_cnt = send_cnt + 1;
						printf("send_cnt = %d send_cnt_max = %d stop_sig = %d\n", send_cnt, send_cnt_max, stop_sig);
						if (send_cnt == send_cnt_max)
							stop_sig = 3;
					}
					else if (stop_sig == 2)
					{
						//write(uart_fd, vir_pl_addr3, size);
						send(new_tcpserversockfd, vir_pl_addr3, size, 0);
						//eth_send(vir_pl_addr3, size);
						send_cnt = 0;
					}
					else if (stop_sig == 3)
					{
						send_cnt = 0;
					}

					pc_send_cnt = 0;
					pc_send_ready = 0;
				}
			}
		}
	}
	/*while (1)
	{
		struct sockaddr_in client_addr;
		socklen_t client_addr_length = sizeof(client_addr);
		printf("1\n");
		new_tcpserversockfd = accept(sockfd, (struct sockaddr*)&client_addr, &client_addr_length);
		printf("2\n");
		while (1)
		{
			if(disconnect == 1)
			{
				disconnect = 0;
				stop_sig = 1;
				printf("3\n");
				//shutdown(new_tcpserversockfd,SHUT_RDWR);
				//printf("4\n");
				close(new_tcpserversockfd);
				printf("5\n");
				break;
			}
			if (pc_send_ready == 1 && stop_sig == 0)
			{
				send(new_tcpserversockfd, vir_pl_addr1, size, 0);
				//printf(" send success\n");
				/ *for (i = 0; i < 4; i++)
				{
				*(c_test + i) = *(vir_pl_addr1 + i);
				printf("%d\n", c_test[i]);
				}
				*(c_test + 4) = *(vir_pl_addr1 + 4100);
				printf("%d\n", c_test[4]);
				*(c_test + 5) = *(vir_pl_addr1 + 4101);
				printf("%d\n", c_test[5]); * /
				pc_send_ready = 0;
			}
			if(time_flag == 1)
			{
				time_flag = 0;
				printf("%d\n",time_jf);
				if((time_jf/1)%2 == 1)
					write(valuefd1, "1", 2);
				else
					write(valuefd1, "0", 2);

				if((time_jf/2)%2 == 1)
					write(valuefd2, "1", 2);
				else
					write(valuefd2, "0", 2);

				if((time_jf/4)%2 == 1)
					write(valuefd3, "1", 2);
				else
					write(valuefd3, "0", 2);

				if((time_jf/8)%2 == 1)
					write(valuefd4, "1", 2);
				else
					write(valuefd4, "0", 2);
			}
			if (tec_ready == 1)
			{
				for (i = 0; i < 2; i++)
				{
					*(c_test + i) = *(vir_pl_addr2 + 4 + i);
					//printf("%d\n", c_test[i]);
				}
				ad_data = (c_test[1] << 8) + c_test[0];
				printf("%d\n", ad_data);
				tec_ready = 0;
			}
		}
	}*/
}

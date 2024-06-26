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
#include <sys/types.h>   
#include <arpa/inet.h>
#include "tran.h"

#define LENGTH_OF_LISTEN_QUEUE 20
int CreateTcpServerSock(int port)
{
        struct sockaddr_in server_addr;  
	bzero(&server_addr, sizeof(server_addr));  
	server_addr.sin_family = AF_INET;  
	server_addr.sin_addr.s_addr = htons(INADDR_ANY);  
	server_addr.sin_port = htons(port); 

// 创建socket，若成功，返回socket描述符  
	int server_socket_fd = socket(PF_INET, SOCK_STREAM, 0);  
	if(server_socket_fd < 0)  
	{  
		perror("Create Socket Failed:");  
		exit(1);  
	}  
	int opt = 1;  
	setsockopt(server_socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); 
        // 绑定socket和socket地址结构  
	if(-1 == (bind(server_socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr))))  
	{  
		perror("Server Bind Failed:");  
		exit(1);  
	}  

	// socket监听  
	if(-1 == (listen(server_socket_fd, LENGTH_OF_LISTEN_QUEUE)))  
	{  
		perror("Server Listen Failed:");  
		exit(1);  
	}  
        return server_socket_fd;



}
int CreateSock(char *destip, int port)//char * destport)                 
{                                                                    
        //int flags=0;                                                             
	// 声明并初始化一个客户端的socket地址结构  
    struct sockaddr_in client_addr;  
    bzero(&client_addr, sizeof(client_addr));  
    client_addr.sin_family = AF_INET;  
    client_addr.sin_addr.s_addr = htons(INADDR_ANY);  
    client_addr.sin_port = htons(0);  
    
    // 创建socket，若成功，返回socket描述符  
    int client_socket_fd = socket(AF_INET, SOCK_STREAM, 0);  
    if(client_socket_fd < 0)  
    {  
        perror("Create Socket Failed:");  
        exit(1);  
    }  
    
    // 绑定客户端的socket和客户端的socket地址结构 非必需  
    if(-1 == (bind(client_socket_fd, (struct sockaddr*)&client_addr, sizeof(client_addr))))  
    {  
        perror("Client Bind Failed:");  
        exit(1);  
    }  
  
    // 声明一个服务器端的socket地址结构，并用服务器那边的IP地址及端口对其进行初始化，用于后面的连接  
    struct sockaddr_in server_addr;  
    bzero(&server_addr, sizeof(server_addr));  
    server_addr.sin_family = AF_INET;
      
    if(inet_pton(AF_INET, destip, &server_addr.sin_addr) == 0)  
    {  
        perror("Server IP Address Error:");  
        exit(1);  
    }  
    server_addr.sin_port = htons(port);  
    socklen_t server_addr_length = sizeof(server_addr);  
    printf("%s\n",destip);
    // 向服务器发起连接，连接成功后client_socket_fd代表了客户端和服务器的一个socket连接  
    if(connect(client_socket_fd, (struct sockaddr*)&server_addr, server_addr_length) < 0)  
    {  
        perror("Can Not Connect To Server IP:");  
        exit(0);  
    }  

                                                   
	return client_socket_fd;   //connectfd;  
}

int CreateserverSock(int port)//char * destport)                 
{
   // 声明并初始化一个服务器端的socket地址结构  
    struct sockaddr_in server_addr;  
    bzero(&server_addr, sizeof(server_addr));  
    server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);  
    server_addr.sin_port = htons(port);  
 
     // 创建socket，若成功，返回socket描述符  
    int server_socket_fd = socket(PF_INET, SOCK_STREAM, 0);  
    if(server_socket_fd < 0)  
    {  
        perror("Create Socket Failed:");  
        exit(1);  
    }  
    int opt = 1;  
    setsockopt(server_socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    // 绑定socket和socket地址结构  
    if(-1 == (bind(server_socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr))))  
    {  
        perror("Server Bind Failed:");  
        exit(1);  
    }  

    // socket监听  
    if(-1 == (listen(server_socket_fd, LENGTH_OF_LISTEN_QUEUE)))  
    {  
        perror("Server Listen Failed:");  
        exit(1);  
    }  

    // 定义客户端的socket地址结构  
        struct sockaddr_in client_addr;  
        socklen_t client_addr_length = sizeof(client_addr);  
  
        // 接受连接请求，返回一个新的socket(描述符)，这个新socket用于同连接的客户端通信  
        // accept函数会把连接到的客户端信息写到client_addr中  
        int new_server_socket_fd = accept(server_socket_fd, (struct sockaddr*)&client_addr, &client_addr_length);  
        if(new_server_socket_fd < 0)  
        {  
            perror("Server Accept Failed:");  
            exit(1);  
        }  

        return  new_server_socket_fd;
       
    
          
}

struct sockaddr_in addr_p;
int createudpSock(char *destip, int port)//char * destport)                 
{                                                                    
                                                                     
        struct sockaddr_in addr;  
        int fd;  
        struct ip_mreq mreq;  
        int rcvBufSize=10*1024*1024;
        socklen_t optlen = sizeof(rcvBufSize);
 
        u_int yes=1; /*** MODIFICATION TO ORIGINAL */  
        /* create what looks like an ordinary UDP socket */  
        if ((fd=socket(AF_INET,SOCK_DGRAM,0)) < 0)   
        {  
            perror("socket");  
            exit(1);  
        }  
        /**** MODIFICATION TO ORIGINAL */  
        /* allow multiple sockets to use the same PORT number */  
        if (setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes)) < 0)   
        {  
            perror("Reusing ADDR failed");  
            exit(1);  
        }  
        if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvBufSize, optlen) < 0)
    {
        printf("setsockopt fail!!!\n");
       
    }

        /*** END OF MODIFICATION TO ORIGINAL */  
        /* set up destination address */  
        memset(&addr,0,sizeof(addr));  
        addr.sin_family=AF_INET;  
        addr.sin_addr.s_addr=htonl(INADDR_ANY); /* N.B.: differs from sender */  
        addr.sin_port=htons(port);  
        /* bind to receive address */  
        if (bind(fd,(struct sockaddr *) &addr,sizeof(addr)) < 0)  
        {  
            perror("bind");  
            exit(1);  
        }  
        /* use setsockopt() to request that the kernel join a multicast group */  
        mreq.imr_multiaddr.s_addr=inet_addr(destip);  
        mreq.imr_interface.s_addr=htonl(INADDR_ANY);  
        if (setsockopt(fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) < 0)   
        {  
            perror("setsockopt");  
            exit(1);  
        }  
        addr_p = addr;
        return fd;
        /* now just enter a read-print loop */  
}         


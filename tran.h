#ifndef _TRAN_H
#define _TRAN_H


extern struct sockaddr_in addr_p;
extern int CreateSock(char *destip, int port);
extern int createudpSock(char *destip, int port);
extern int CreateTcpServerSock(int port);
extern int CreateserverSock(int port);

#endif

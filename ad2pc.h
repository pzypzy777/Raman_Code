#ifndef _AD2PC_H
#define _AD2PC_H

void receive_PC_to_PS(void);
void my_signal_fun(int signum);
void send_PL_to_PC(void);
int uart_configure(int,int, int, char, int);
void eth_send(unsigned char* , unsigned long );

#endif

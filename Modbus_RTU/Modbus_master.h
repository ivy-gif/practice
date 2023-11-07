#ifndef _MODBUS_MASTER_H_
#define _MODBUS_MASTER_H_

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/time.h>
#include <termios.h>

#define max_bits 256  //每个主站的可访问bit数据大小,单位是byte，也就是实际是max_bits*8//
#define max_byte 2048

/*创建一个链表，管理从站待处理的指令*/
typedef struct slave_shell_struct{
	unsigned char *read_datas;
	unsigned int datas_size;
	
	struct slave_shell_struct *next;
	struct slave_shell_struct *prev;
}SLAVE_DATA_LIST;
/*主站操作内存的结构体*/
typedef struct modbus_master_struct{
	unsigned char MasterID;  //选择打开的串口序号//
	int uart_fd; //打开后生成的文件描述符//
	unsigned char *read_datas;  //每次读到的完整的一笔指令数据//
	unsigned int datas_size;
	
	unsigned char typeBit_datas[max_bits];  //主站bit类型的数据内存区//
	unsigned char typeByte_datas[max_byte];  //主站byte类型数据的内存区//
}MODBUS_MASTER_DATA;

/*主站功能的函数接口定义*/

/*========================================================================
*调用此函数监听串口，此函数会阻塞进程执行
*传入参数：
*modbus_master_data  传入包含串口信息的变量结构体，此参数不能为NULL
*timeout_us  每次监听串口阻塞时间，若超出时间会函数会返回-1并结束执行
*返回参数:
*-1 = 传入的参数无效或非法
*-2 = 监听串口超时未收到数据
*-3 = 非呼叫本主站的指令
*-4 = 数据CRC校验错误
*0 = 接收到正确的指令
==========================================================================*/
int Listen_UartPort(MODBUS_MASTER_DATA *modbus_master_data,unsigned int timeout_us);

/*=========================================================================
*调用函数，主站将自动处理从站指令，并作数据内存的更新
*传入参数：
*modbus_master_data  传入包含串口信息的变量结构体，此参数不能为NULL
*Slave_shell 接收到的数据指令链表的数据节点
*返回参数：
*0 = 一切正常
*-1 = 传入的参数无效或非法
==========================================================================*/
int DealShell_fromSlave(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);

#endif
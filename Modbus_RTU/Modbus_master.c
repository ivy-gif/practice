#include "Modbus_master.h"

/*====================================================================================================
*定义内部功能函数
=====================================================================================================*/
static unsigned short Make_CRC(unsigned char *Data_buff,int Data_length); //调用此函数生成CRC校验码,返回CRC校验码//
static int Check_CRC(unsigned char *Data_buff,int Data_length);  //检查收到指令的CRC，错误返回-4//
static int Requert_01H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //回复处理读取线圈的指令//
static int Requert_02H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //回复处理读取离散状态指令//
static int Requert_03H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //回复处理读取保持寄存器指令//
static int Requert_04H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //回复处理读取输入寄存器指令//
static int Requert_05H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //回复处理写入单个线圈指令//
static int Requert_06H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //回复处理写入单个寄存器指令//
static int Requert_0FH(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //回复处理写入多个线圈指令//
static int Requert_10H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //回复处理写入多个寄存器指令//

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
int Listen_UartPort(MODBUS_MASTER_DATA *modbus_master_data,unsigned int timeout_us)
{
	int ret_var;
	if(NULL == modbus_master_data)
	{
		ret_var = -1;
		return ret_var;
	}
	int uart_fd = modbus_master_data->uart_fd;
	
	/*开始监控串口并对接收到的数据进行检查*/
	fd_set rset;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = timeout_us;
	FD_ZERO(&rset);
	FD_SET(uart_fd,&rset);
	
	if(NULL != (modbus_master_data->read_datas))
	{
		free((modbus_master_data->read_datas));
		(modbus_master_data->read_datas) = NULL;
	}
	
	ret_var = select(uart_fd+1,&rset,NULL,NULL,&tv);  //规定时间等待数据指令//
	if(0 == ret_var)  //超时未获得任何输入指令//
		ret_var = -2;
	else if(-1 == ret_var)
		ret_var = -1;
	else if(FD_ISSET(uart_fd,&rset))  //串口接收到指令了,这里需要连续装载8个数据或者更多//
	{
		unsigned int read_num;
		unsigned int max_num = 8;
		(modbus_master_data->read_datas) = (unsigned char*)malloc(sizeof(unsigned char)*max_num);
		if(NULL == (modbus_master_data->read_datas))
			return -1;
		memset((modbus_master_data->read_datas),0,sizeof(unsigned char)*max_num);
		for(read_num=0;read_num<max_num;++read_num)
		{
			read(uart_fd,&(modbus_master_data->read_datas[read_num]),8);
			if(7 == read_num) //特别注意多寄存器写入指令的数据较长//
			{
				if((0x0f == (modbus_master_data->read_datas[1])) || (0x10 == (modbus_master_data->read_datas[1])))  //写入多线圈指令//
				{
					max_num += ((modbus_master_data->read_datas[6])+1);
					(modbus_master_data->read_datas) = (unsigned char*)realloc((modbus_master_data->read_datas),max_num);
					memset((modbus_master_data->read_datas)+read_num+1,0,sizeof(unsigned char)*(max_num-8));
				}
			}
		}
		modbus_master_data->datas_size = max_num; //记录当前接收到的指令的字节数//
		
		/*读取到的数据进行指令检查，查看指令是否是给此主站的，且指令CRC校验是否正确*/
		ret_var = Check_CRC((modbus_master_data->read_datas),max_num);
		if(0>ret_var)
			goto end;
		if((modbus_master_data->read_datas[0]) != (modbus_master_data->MasterID))
			ret_var = -3;
	}
	
end:
	FD_CLR(uart_fd,&rset);
	return ret_var;
}

/*=========================================================================
*调用函数，主站将自动处理从站指令，并作数据内存的更新
*传入参数：
*modbus_master_data  传入包含串口信息的变量结构体，此参数不能为NULL
*Slave_shell 接收到的数据指令链表的数据节点
*返回参数：
*0 = 一切正常
*-1 = 传入的参数无效或非法
==========================================================================*/
int DealShell_fromSlave(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)
{
	int ret_var = 0;
	if(NULL == modbus_master_data)
	{
		ret_var = -1;
		goto end;
	}
	if(NULL == Slave_shell)
	{
		ret_var = -2;
		goto end;
	}
	
	//这里测试一下，看看数据返回应是否可以实现//
	switch(Slave_shell->read_datas[1])
	{
		case 0x01:  //读取线圈请求//
		{
			ret_var = Requert_01H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x02: //读取状态的请求//
		{
			ret_var = Requert_02H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x03: //读取保存寄存器请求//
		{
			ret_var = Requert_03H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x04: //读取输入寄存器请求//
		{
			ret_var = Requert_04H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x05:  //写入单个线圈的请求//
		{
			ret_var = Requert_05H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x06: //写入单个寄存器的请求//
		{
			ret_var = Requert_06H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x0f: //写入多个线圈的请求//
		{
			ret_var = Requert_0FH(modbus_master_data,Slave_shell);
			break;
		}
		case 0x10: //写入多个寄存器的请求//
		{
			ret_var = Requert_10H(modbus_master_data,Slave_shell);
			break;
		}
	}
	
end:
	return ret_var;
}

/*======================================================================================
*内部功能函数的实现
=======================================================================================*/
static unsigned short Make_CRC(unsigned char *Data_buff,int Data_length) //调用此函数生成CRC校验码，函数返回CRC校验码//
{
	if(NULL == Data_buff)
		return 0xffff;
	unsigned short wcrc = 0xffff; //CRC寄存器预置//
	unsigned char temp;
	unsigned int i=0,j=0; //计数//
	
	/*循环计算每个数据*/
	for(i=0;i<Data_length;++i)
	{
		temp = *Data_buff & 0x00ff;
		Data_buff++; //next data//
		
		wcrc ^= temp;
		for(j=0;j<8;++j)  //对单笔数据进行具体计算//
		{
			if(0!= (0x0001 & wcrc))  //判断数据右移出的是否为1,如果是，则与多项式0xa001异或//
			{
				wcrc >>= 1;
				wcrc ^= 0xa001;
			}
			else
				wcrc >>= 1;
		}
	}
	
	//对计算结果进行高低字节兑换//
	unsigned char CRC_L = wcrc & 0xff;
	unsigned char CRC_H = wcrc >> 8;
	
	return ((CRC_L << 8) | CRC_H);
}
static int Check_CRC(unsigned char *Data_buff,int Data_length)  //检查收到指令的CRC，错误返回-4//
{
	if(NULL == Data_buff)
		return -1;
	unsigned short ret_crc = Make_CRC(Data_buff,(Data_length-2));
	unsigned char crc_l = (ret_crc>>8);
	unsigned char crc_h = (ret_crc&0xff);
	
	if((crc_l == Data_buff[Data_length-2]) && (crc_h == Data_buff[Data_length-1]))
		return 0;
	
	return -4;
}
static int Requert_01H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //回复读取线圈的指令//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short StartBit_addr;
	unsigned short Bitdata_length;
	unsigned short masterbyte_addr;  //对应主站内存的字节起始地址//
	unsigned short sendByte_length;  //之际上需要发送的字节数//
	
	//抓住关键信息，读取的线圈的起始地址和读取的线圈的个数//
	StartBit_addr = Slave_shell->read_datas[2];
	StartBit_addr <<= 8;
	StartBit_addr |= Slave_shell->read_datas[3];
	Bitdata_length = Slave_shell->read_datas[4];
	Bitdata_length <<= 8;
	Bitdata_length |= Slave_shell->read_datas[5];
	
	sendByte_length = (Bitdata_length>>3);
	if(0 != (Bitdata_length%8))
		sendByte_length += 1;
	
	masterbyte_addr = (StartBit_addr>>3);  //计算需要主站内存的起始地址//
	
	//这里分配内存空间来组装回复数据包//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*(5+sendByte_length));
	if(NULL == send_data)
		return -1;
	memset(send_data,0,sizeof(unsigned char)*(5+sendByte_length));
	
	//开始从内存取数据来给到从站//
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x01;
	send_data[2] = sendByte_length;
	unsigned short data_start = 3;
	unsigned short sanddata_num = 0;
	if(0 == masterbyte_addr)
		StartBit_addr = StartBit_addr%8;
	else
		StartBit_addr = StartBit_addr%(masterbyte_addr<<3);
	for(num=0;num<Bitdata_length;)  //累计装载足够位数的数据即可，字节顺序也是低地址在前//
	{
		for(;StartBit_addr<8;++StartBit_addr)
		{
			send_data[data_start] |= ((((modbus_master_data->typeBit_datas[masterbyte_addr])>>StartBit_addr)&0x01)<<sanddata_num);
			sanddata_num++;
			num ++;
			if(num >= Bitdata_length)
				goto fish;
			if(8 <= sanddata_num)
			{
				sanddata_num=0;
				data_start++;
			}
		}
		masterbyte_addr++;
		StartBit_addr = 0;
	}
fish:
	DATA_CRC = Make_CRC(send_data,sendByte_length+3);
	send_data[sendByte_length+3] = DATA_CRC>>8;
	send_data[sendByte_length+4] = DATA_CRC&0x00ff;
	
	//将组装好的数据发送出去//
	write((modbus_master_data->uart_fd),send_data,5+sendByte_length);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_02H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //回复处理读取离散状态指令//
{
	if(NULL == modbus_master_data)
		return -1;
	return 0;
}
static int Requert_03H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //回复处理读取保持寄存器指令//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short start_addr;  //读取的保持寄存器的起始地址//
	unsigned short reg_length;  //读取的保持寄存器的长度//
	
	//解析数据帧获得有效的信息//
	start_addr = Slave_shell->read_datas[2];
	start_addr <<= 8;
	start_addr |= Slave_shell->read_datas[3];
	reg_length = Slave_shell->read_datas[4];
	reg_length <<= 8;
	reg_length |= Slave_shell->read_datas[5];
	
	//起始地址和数据长度与主站内存对齐,num*2//
	start_addr <<= 1;
	reg_length <<= 1;
	if(max_byte < (reg_length+start_addr))
		reg_length = max_byte-start_addr;
	
	//开始组装回复的数据//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*(5+reg_length));
	if(NULL == send_data)
		return -1;
	memset(send_data,0,sizeof(unsigned char)*(5+reg_length));
	
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x03;
	send_data[2] = reg_length;
	unsigned short sanddata_num = 3;
	unsigned short Int_data; //一个字的数据//
	for(num=start_addr;num<(reg_length+start_addr);num += 2)
	{
		Int_data = modbus_master_data->typeByte_datas[num];
		Int_data |= ((modbus_master_data->typeByte_datas[num+1])<<8);
		
		send_data[sanddata_num] = (Int_data>>8);
		send_data[sanddata_num+1] = (Int_data&0xff);
		
		sanddata_num += 2;
	}
	DATA_CRC = Make_CRC(send_data,reg_length+3);
	send_data[reg_length+3] = DATA_CRC>>8;
	send_data[reg_length+4] = DATA_CRC&0x00ff;
	
	//将组装好的数据写回从站//
	write((modbus_master_data->uart_fd),send_data,5+reg_length);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_04H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //回复处理读取输入寄存器指令//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short start_addr;  //读取的保持寄存器的起始地址//
	unsigned short reg_length;  //读取的保持寄存器的长度//
	
	//解析数据帧获得有效的信息//
	start_addr = Slave_shell->read_datas[2];
	start_addr <<= 8;
	start_addr |= Slave_shell->read_datas[3];
	reg_length = Slave_shell->read_datas[4];
	reg_length <<= 8;
	reg_length |= Slave_shell->read_datas[5];
	
	//起始地址和数据长度与主站内存对齐,num*2//
	start_addr <<= 1;
	reg_length <<= 1;
	if(max_byte < (reg_length+start_addr))
		reg_length = max_byte-start_addr;
	
	//开始组装回复的数据//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*(5+reg_length));
	if(NULL == send_data)
		return -1;
	memset(send_data,0,sizeof(unsigned char)*(5+reg_length));
	
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x04;
	send_data[2] = reg_length;
	unsigned short sanddata_num = 3;
	unsigned short Int_data; //一个字的数据//
	for(num=start_addr;num<(reg_length+start_addr);num += 2)
	{
		Int_data = modbus_master_data->typeByte_datas[num];
		Int_data |= ((modbus_master_data->typeByte_datas[num+1])<<8);
		
		send_data[sanddata_num] = (Int_data>>8);
		send_data[sanddata_num+1] = (Int_data&0xff);
		
		sanddata_num += 2;
	}
	DATA_CRC = Make_CRC(send_data,reg_length+3);
	send_data[reg_length+3] = DATA_CRC>>8;
	send_data[reg_length+4] = DATA_CRC&0x00ff;
	
	//将组装好的数据写回从站//
	write((modbus_master_data->uart_fd),send_data,5+reg_length);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_05H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //回复处理写入单个线圈指令//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short SetBit_addr;
	unsigned short SetBit_value;
	unsigned short BitByte_addr;  //主站内存中实际对应的字节地址//
	unsigned char Bit_setaddr;  //主站内存定位具体字节的某个bit//
	
	//获取指令的有效信息//
	SetBit_addr = Slave_shell->read_datas[2];
	SetBit_addr <<= 8;
	SetBit_addr |= Slave_shell->read_datas[3];
	SetBit_value = Slave_shell->read_datas[4];
	SetBit_value <<= 8;
	SetBit_value |= Slave_shell->read_datas[5];
	
	BitByte_addr = (SetBit_addr>>3);
	Bit_setaddr = SetBit_addr%8;
	if(max_bits <= BitByte_addr)  //超出主站内存限制//
		return -1;

	//判断具体设定值，操作主站内存//
	if(0 != (SetBit_value&0xffff))  //设置对应Bit为1//
		(modbus_master_data->typeBit_datas[BitByte_addr]) |= (1 << Bit_setaddr);
	else
		(modbus_master_data->typeBit_datas[BitByte_addr]) &= (~(1 << Bit_setaddr));
		
	//组装回复数据，回复从站//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*8);
	if(NULL == send_data)
		return -2;
	memset(send_data,0,sizeof(unsigned char)*8);
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x05;
	send_data[2] = (SetBit_value>>8);
	send_data[3] = (SetBit_value&0xff);
	send_data[4] = 0x00;
	send_data[5] = 0x01;
	DATA_CRC = Make_CRC(send_data,6);
	send_data[6] = DATA_CRC>>8;
	send_data[7] = DATA_CRC&0x00ff;
	
	//将组装好的数据发送到从站//
	write((modbus_master_data->uart_fd),send_data,8);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_06H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //回复处理写入单个寄存器指令//
{
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short SetINT_addr;
	unsigned short SetINT_value;
	unsigned short StartByte_addr;
	
	//获取关键的指令信息//
	SetINT_addr = Slave_shell->read_datas[2];
	SetINT_addr <<= 8;
	SetINT_addr |= Slave_shell->read_datas[3];
	SetINT_value = Slave_shell->read_datas[4];
	SetINT_value <<= 8;
	SetINT_value |= Slave_shell->read_datas[5];
	
	StartByte_addr = (SetINT_addr<<1);  //对应于主站内存的字节地址//
	if(max_byte<=(StartByte_addr+1))  //超出主站最大内存//
		return -1;
	
	//将数据写入主站对应的寄存器中//
	(modbus_master_data->typeByte_datas[StartByte_addr]) = (SetINT_value&0xff);
	(modbus_master_data->typeByte_datas[StartByte_addr+1]) = (SetINT_value>>8);
	
	//这里开始组装回复数据帧回复从站//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*8);
	if(NULL == send_data)
		return -2;
	memset(send_data,0,sizeof(unsigned char)*8);
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x06;
	send_data[2] = (SetINT_addr>>8);
	send_data[3] = (SetINT_addr&0xff);
	send_data[4] = (SetINT_value>>8);
	send_data[5] = (SetINT_value&0xff);
	DATA_CRC = Make_CRC(send_data,6);
	send_data[6] = DATA_CRC>>8;
	send_data[7] = DATA_CRC&0x00ff;
	
	//将组装好的数据帧写到从站//
	write((modbus_master_data->uart_fd),send_data,8);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_0FH(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //回复处理写入多个线圈指令//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short SetBit_addr;  //写入线圈地址的起始地址//
	unsigned char SetBit_ByteCount; //需要写入的寄存器字节数//
	unsigned short master_StartByte; //主站内存中实际上起始的字节地址//
	unsigned short master_StartBit;  //主站内存中实际上起始的位地址//
	
	//获取指令的关键信息//
	SetBit_addr = Slave_shell->read_datas[2];
	SetBit_addr <<= 8;
	SetBit_addr |= Slave_shell->read_datas[3];
	SetBit_ByteCount = Slave_shell->read_datas[6];
	
	//将指令中的数据取出，写入主站内存的寄存器中//
	master_StartByte = SetBit_addr>>3;
	master_StartBit = SetBit_addr%8;
	unsigned char set_Bytedata;
	unsigned char ready_setData;
	int bit_num;
	for(num=SetBit_ByteCount+6;num>6;--num)
	{
		set_Bytedata = Slave_shell->read_datas[num];
		for(bit_num=0;bit_num<8;++bit_num)
		{
			ready_setData = (((set_Bytedata >> bit_num)&0x01)<<master_StartBit);
			if(0 == ready_setData)
			{
				ready_setData |= (1<<master_StartBit);
				modbus_master_data->typeBit_datas[master_StartByte] &= ~(ready_setData);
			}
			else
				modbus_master_data->typeBit_datas[master_StartByte] |= ready_setData;
			master_StartBit++;
			if(8 <= master_StartBit)
			{
				master_StartBit=0;
				master_StartByte++;
			}
		}
	}
	
	//组装回复数据包回复从站//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*8);
	if(NULL == send_data)
		return -1;
	memset(send_data,0,sizeof(unsigned char)*8);
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x0F;
	send_data[2] = (SetBit_addr>>8);
	send_data[3] = (SetBit_addr&0xff);
	send_data[4] = Slave_shell->read_datas[4];
	send_data[5] = Slave_shell->read_datas[5];
	DATA_CRC = Make_CRC(send_data,6);
	send_data[6] = DATA_CRC>>8;
	send_data[7] = DATA_CRC&0x00ff;
	
	//将组装好的数据包写到从站//
	write((modbus_master_data->uart_fd),send_data,8);
	
	free(send_data);
	send_data = NULL;
	
	return 0;
}
static int Requert_10H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //回复处理写入多个寄存器指令//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short SetStart_addr;
	unsigned char SetByte_count;
	unsigned char SetInt_count;
	unsigned short master_byteAddr;  //主站内存对应的实际起始字节地址//
	
	//获取指令的关键信息//
	SetStart_addr = Slave_shell->read_datas[2];
	SetStart_addr <<= 8;
	SetStart_addr |= Slave_shell->read_datas[3];
	SetByte_count = Slave_shell->read_datas[6];
	SetInt_count = SetByte_count>>1;  //字的个数//
	
	master_byteAddr = SetStart_addr<<1;
	if(max_byte <= (master_byteAddr+SetByte_count))  //超出主站内存的大小//
		return -1;
	
	//将数据写入主站对应的内存中//
	unsigned short Int_data;  //每个字的值，要进行倒装//
	for(num=0;num<SetInt_count;++num)
	{
		Int_data = Slave_shell->read_datas[(num<<1)+7];
		Int_data <<= 8;
		Int_data |= Slave_shell->read_datas[(num<<1)+8];

		(modbus_master_data->typeByte_datas[master_byteAddr]) = (Int_data&0xff);
		(modbus_master_data->typeByte_datas[master_byteAddr+1]) = (Int_data>>8);
		master_byteAddr += 2;
	}
	
	//组装数据包回复从站//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*8);
	if(NULL == send_data)
		return -2;
	memset(send_data,0,sizeof(unsigned char)*8);
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x10;
	send_data[2] = (SetStart_addr>>8);
	send_data[3] = (SetStart_addr&0xff);
	send_data[4] = Slave_shell->read_datas[4];
	send_data[5] = Slave_shell->read_datas[5];
	DATA_CRC = Make_CRC(send_data,6);
	send_data[6] = DATA_CRC>>8;
	send_data[7] = DATA_CRC&0x00ff;
	
	//将组装好的数据包发送出去//
	write((modbus_master_data->uart_fd),send_data,8);
	
	free(send_data);
	send_data = NULL;
	return 0;
}


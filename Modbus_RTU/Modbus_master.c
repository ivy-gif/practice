#include "Modbus_master.h"

/*====================================================================================================
*�����ڲ����ܺ���
=====================================================================================================*/
static unsigned short Make_CRC(unsigned char *Data_buff,int Data_length); //���ô˺�������CRCУ����,����CRCУ����//
static int Check_CRC(unsigned char *Data_buff,int Data_length);  //����յ�ָ���CRC�����󷵻�-4//
static int Requert_01H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //�ظ������ȡ��Ȧ��ָ��//
static int Requert_02H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //�ظ������ȡ��ɢ״ָ̬��//
static int Requert_03H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //�ظ������ȡ���ּĴ���ָ��//
static int Requert_04H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //�ظ������ȡ����Ĵ���ָ��//
static int Requert_05H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //�ظ�����д�뵥����Ȧָ��//
static int Requert_06H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //�ظ�����д�뵥���Ĵ���ָ��//
static int Requert_0FH(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //�ظ�����д������Ȧָ��//
static int Requert_10H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell);  //�ظ�����д�����Ĵ���ָ��//

/*========================================================================
*���ô˺����������ڣ��˺�������������ִ��
*���������
*modbus_master_data  �������������Ϣ�ı����ṹ�壬�˲�������ΪNULL
*timeout_us  ÿ�μ�����������ʱ�䣬������ʱ��ắ���᷵��-1������ִ��
*���ز���:
*-1 = ����Ĳ�����Ч��Ƿ�
*-2 = �������ڳ�ʱδ�յ�����
*-3 = �Ǻ��б���վ��ָ��
*-4 = ����CRCУ�����
*0 = ���յ���ȷ��ָ��
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
	
	/*��ʼ��ش��ڲ��Խ��յ������ݽ��м��*/
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
	
	ret_var = select(uart_fd+1,&rset,NULL,NULL,&tv);  //�涨ʱ��ȴ�����ָ��//
	if(0 == ret_var)  //��ʱδ����κ�����ָ��//
		ret_var = -2;
	else if(-1 == ret_var)
		ret_var = -1;
	else if(FD_ISSET(uart_fd,&rset))  //���ڽ��յ�ָ����,������Ҫ����װ��8�����ݻ��߸���//
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
			if(7 == read_num) //�ر�ע���Ĵ���д��ָ������ݽϳ�//
			{
				if((0x0f == (modbus_master_data->read_datas[1])) || (0x10 == (modbus_master_data->read_datas[1])))  //д�����Ȧָ��//
				{
					max_num += ((modbus_master_data->read_datas[6])+1);
					(modbus_master_data->read_datas) = (unsigned char*)realloc((modbus_master_data->read_datas),max_num);
					memset((modbus_master_data->read_datas)+read_num+1,0,sizeof(unsigned char)*(max_num-8));
				}
			}
		}
		modbus_master_data->datas_size = max_num; //��¼��ǰ���յ���ָ����ֽ���//
		
		/*��ȡ�������ݽ���ָ���飬�鿴ָ���Ƿ��Ǹ�����վ�ģ���ָ��CRCУ���Ƿ���ȷ*/
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
*���ú�������վ���Զ������վָ����������ڴ�ĸ���
*���������
*modbus_master_data  �������������Ϣ�ı����ṹ�壬�˲�������ΪNULL
*Slave_shell ���յ�������ָ����������ݽڵ�
*���ز�����
*0 = һ������
*-1 = ����Ĳ�����Ч��Ƿ�
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
	
	//�������һ�£��������ݷ���Ӧ�Ƿ����ʵ��//
	switch(Slave_shell->read_datas[1])
	{
		case 0x01:  //��ȡ��Ȧ����//
		{
			ret_var = Requert_01H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x02: //��ȡ״̬������//
		{
			ret_var = Requert_02H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x03: //��ȡ����Ĵ�������//
		{
			ret_var = Requert_03H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x04: //��ȡ����Ĵ�������//
		{
			ret_var = Requert_04H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x05:  //д�뵥����Ȧ������//
		{
			ret_var = Requert_05H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x06: //д�뵥���Ĵ���������//
		{
			ret_var = Requert_06H(modbus_master_data,Slave_shell);
			break;
		}
		case 0x0f: //д������Ȧ������//
		{
			ret_var = Requert_0FH(modbus_master_data,Slave_shell);
			break;
		}
		case 0x10: //д�����Ĵ���������//
		{
			ret_var = Requert_10H(modbus_master_data,Slave_shell);
			break;
		}
	}
	
end:
	return ret_var;
}

/*======================================================================================
*�ڲ����ܺ�����ʵ��
=======================================================================================*/
static unsigned short Make_CRC(unsigned char *Data_buff,int Data_length) //���ô˺�������CRCУ���룬��������CRCУ����//
{
	if(NULL == Data_buff)
		return 0xffff;
	unsigned short wcrc = 0xffff; //CRC�Ĵ���Ԥ��//
	unsigned char temp;
	unsigned int i=0,j=0; //����//
	
	/*ѭ������ÿ������*/
	for(i=0;i<Data_length;++i)
	{
		temp = *Data_buff & 0x00ff;
		Data_buff++; //next data//
		
		wcrc ^= temp;
		for(j=0;j<8;++j)  //�Ե������ݽ��о������//
		{
			if(0!= (0x0001 & wcrc))  //�ж��������Ƴ����Ƿ�Ϊ1,����ǣ��������ʽ0xa001���//
			{
				wcrc >>= 1;
				wcrc ^= 0xa001;
			}
			else
				wcrc >>= 1;
		}
	}
	
	//�Լ��������иߵ��ֽڶһ�//
	unsigned char CRC_L = wcrc & 0xff;
	unsigned char CRC_H = wcrc >> 8;
	
	return ((CRC_L << 8) | CRC_H);
}
static int Check_CRC(unsigned char *Data_buff,int Data_length)  //����յ�ָ���CRC�����󷵻�-4//
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
static int Requert_01H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //�ظ���ȡ��Ȧ��ָ��//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short StartBit_addr;
	unsigned short Bitdata_length;
	unsigned short masterbyte_addr;  //��Ӧ��վ�ڴ���ֽ���ʼ��ַ//
	unsigned short sendByte_length;  //֮������Ҫ���͵��ֽ���//
	
	//ץס�ؼ���Ϣ����ȡ����Ȧ����ʼ��ַ�Ͷ�ȡ����Ȧ�ĸ���//
	StartBit_addr = Slave_shell->read_datas[2];
	StartBit_addr <<= 8;
	StartBit_addr |= Slave_shell->read_datas[3];
	Bitdata_length = Slave_shell->read_datas[4];
	Bitdata_length <<= 8;
	Bitdata_length |= Slave_shell->read_datas[5];
	
	sendByte_length = (Bitdata_length>>3);
	if(0 != (Bitdata_length%8))
		sendByte_length += 1;
	
	masterbyte_addr = (StartBit_addr>>3);  //������Ҫ��վ�ڴ����ʼ��ַ//
	
	//��������ڴ�ռ�����װ�ظ����ݰ�//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*(5+sendByte_length));
	if(NULL == send_data)
		return -1;
	memset(send_data,0,sizeof(unsigned char)*(5+sendByte_length));
	
	//��ʼ���ڴ�ȡ������������վ//
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x01;
	send_data[2] = sendByte_length;
	unsigned short data_start = 3;
	unsigned short sanddata_num = 0;
	if(0 == masterbyte_addr)
		StartBit_addr = StartBit_addr%8;
	else
		StartBit_addr = StartBit_addr%(masterbyte_addr<<3);
	for(num=0;num<Bitdata_length;)  //�ۼ�װ���㹻λ�������ݼ��ɣ��ֽ�˳��Ҳ�ǵ͵�ַ��ǰ//
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
	
	//����װ�õ����ݷ��ͳ�ȥ//
	write((modbus_master_data->uart_fd),send_data,5+sendByte_length);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_02H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //�ظ������ȡ��ɢ״ָ̬��//
{
	if(NULL == modbus_master_data)
		return -1;
	return 0;
}
static int Requert_03H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //�ظ������ȡ���ּĴ���ָ��//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short start_addr;  //��ȡ�ı��ּĴ�������ʼ��ַ//
	unsigned short reg_length;  //��ȡ�ı��ּĴ����ĳ���//
	
	//��������֡�����Ч����Ϣ//
	start_addr = Slave_shell->read_datas[2];
	start_addr <<= 8;
	start_addr |= Slave_shell->read_datas[3];
	reg_length = Slave_shell->read_datas[4];
	reg_length <<= 8;
	reg_length |= Slave_shell->read_datas[5];
	
	//��ʼ��ַ�����ݳ�������վ�ڴ����,num*2//
	start_addr <<= 1;
	reg_length <<= 1;
	if(max_byte < (reg_length+start_addr))
		reg_length = max_byte-start_addr;
	
	//��ʼ��װ�ظ�������//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*(5+reg_length));
	if(NULL == send_data)
		return -1;
	memset(send_data,0,sizeof(unsigned char)*(5+reg_length));
	
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x03;
	send_data[2] = reg_length;
	unsigned short sanddata_num = 3;
	unsigned short Int_data; //һ���ֵ�����//
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
	
	//����װ�õ�����д�ش�վ//
	write((modbus_master_data->uart_fd),send_data,5+reg_length);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_04H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //�ظ������ȡ����Ĵ���ָ��//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short start_addr;  //��ȡ�ı��ּĴ�������ʼ��ַ//
	unsigned short reg_length;  //��ȡ�ı��ּĴ����ĳ���//
	
	//��������֡�����Ч����Ϣ//
	start_addr = Slave_shell->read_datas[2];
	start_addr <<= 8;
	start_addr |= Slave_shell->read_datas[3];
	reg_length = Slave_shell->read_datas[4];
	reg_length <<= 8;
	reg_length |= Slave_shell->read_datas[5];
	
	//��ʼ��ַ�����ݳ�������վ�ڴ����,num*2//
	start_addr <<= 1;
	reg_length <<= 1;
	if(max_byte < (reg_length+start_addr))
		reg_length = max_byte-start_addr;
	
	//��ʼ��װ�ظ�������//
	send_data = (unsigned char*)malloc(sizeof(unsigned char)*(5+reg_length));
	if(NULL == send_data)
		return -1;
	memset(send_data,0,sizeof(unsigned char)*(5+reg_length));
	
	send_data[0] = modbus_master_data->MasterID;
	send_data[1] = 0x04;
	send_data[2] = reg_length;
	unsigned short sanddata_num = 3;
	unsigned short Int_data; //һ���ֵ�����//
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
	
	//����װ�õ�����д�ش�վ//
	write((modbus_master_data->uart_fd),send_data,5+reg_length);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_05H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //�ظ�����д�뵥����Ȧָ��//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short SetBit_addr;
	unsigned short SetBit_value;
	unsigned short BitByte_addr;  //��վ�ڴ���ʵ�ʶ�Ӧ���ֽڵ�ַ//
	unsigned char Bit_setaddr;  //��վ�ڴ涨λ�����ֽڵ�ĳ��bit//
	
	//��ȡָ�����Ч��Ϣ//
	SetBit_addr = Slave_shell->read_datas[2];
	SetBit_addr <<= 8;
	SetBit_addr |= Slave_shell->read_datas[3];
	SetBit_value = Slave_shell->read_datas[4];
	SetBit_value <<= 8;
	SetBit_value |= Slave_shell->read_datas[5];
	
	BitByte_addr = (SetBit_addr>>3);
	Bit_setaddr = SetBit_addr%8;
	if(max_bits <= BitByte_addr)  //������վ�ڴ�����//
		return -1;

	//�жϾ����趨ֵ��������վ�ڴ�//
	if(0 != (SetBit_value&0xffff))  //���ö�ӦBitΪ1//
		(modbus_master_data->typeBit_datas[BitByte_addr]) |= (1 << Bit_setaddr);
	else
		(modbus_master_data->typeBit_datas[BitByte_addr]) &= (~(1 << Bit_setaddr));
		
	//��װ�ظ����ݣ��ظ���վ//
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
	
	//����װ�õ����ݷ��͵���վ//
	write((modbus_master_data->uart_fd),send_data,8);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_06H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //�ظ�����д�뵥���Ĵ���ָ��//
{
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short SetINT_addr;
	unsigned short SetINT_value;
	unsigned short StartByte_addr;
	
	//��ȡ�ؼ���ָ����Ϣ//
	SetINT_addr = Slave_shell->read_datas[2];
	SetINT_addr <<= 8;
	SetINT_addr |= Slave_shell->read_datas[3];
	SetINT_value = Slave_shell->read_datas[4];
	SetINT_value <<= 8;
	SetINT_value |= Slave_shell->read_datas[5];
	
	StartByte_addr = (SetINT_addr<<1);  //��Ӧ����վ�ڴ���ֽڵ�ַ//
	if(max_byte<=(StartByte_addr+1))  //������վ����ڴ�//
		return -1;
	
	//������д����վ��Ӧ�ļĴ�����//
	(modbus_master_data->typeByte_datas[StartByte_addr]) = (SetINT_value&0xff);
	(modbus_master_data->typeByte_datas[StartByte_addr+1]) = (SetINT_value>>8);
	
	//���￪ʼ��װ�ظ�����֡�ظ���վ//
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
	
	//����װ�õ�����֡д����վ//
	write((modbus_master_data->uart_fd),send_data,8);
	
	free(send_data);
	send_data = NULL;
	return 0;
}
static int Requert_0FH(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //�ظ�����д������Ȧָ��//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short SetBit_addr;  //д����Ȧ��ַ����ʼ��ַ//
	unsigned char SetBit_ByteCount; //��Ҫд��ļĴ����ֽ���//
	unsigned short master_StartByte; //��վ�ڴ���ʵ������ʼ���ֽڵ�ַ//
	unsigned short master_StartBit;  //��վ�ڴ���ʵ������ʼ��λ��ַ//
	
	//��ȡָ��Ĺؼ���Ϣ//
	SetBit_addr = Slave_shell->read_datas[2];
	SetBit_addr <<= 8;
	SetBit_addr |= Slave_shell->read_datas[3];
	SetBit_ByteCount = Slave_shell->read_datas[6];
	
	//��ָ���е�����ȡ����д����վ�ڴ�ļĴ�����//
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
	
	//��װ�ظ����ݰ��ظ���վ//
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
	
	//����װ�õ����ݰ�д����վ//
	write((modbus_master_data->uart_fd),send_data,8);
	
	free(send_data);
	send_data = NULL;
	
	return 0;
}
static int Requert_10H(MODBUS_MASTER_DATA *modbus_master_data,SLAVE_DATA_LIST *Slave_shell)  //�ظ�����д�����Ĵ���ָ��//
{
	unsigned int num;
	unsigned char *send_data = NULL;
	unsigned short DATA_CRC;
	unsigned short SetStart_addr;
	unsigned char SetByte_count;
	unsigned char SetInt_count;
	unsigned short master_byteAddr;  //��վ�ڴ��Ӧ��ʵ����ʼ�ֽڵ�ַ//
	
	//��ȡָ��Ĺؼ���Ϣ//
	SetStart_addr = Slave_shell->read_datas[2];
	SetStart_addr <<= 8;
	SetStart_addr |= Slave_shell->read_datas[3];
	SetByte_count = Slave_shell->read_datas[6];
	SetInt_count = SetByte_count>>1;  //�ֵĸ���//
	
	master_byteAddr = SetStart_addr<<1;
	if(max_byte <= (master_byteAddr+SetByte_count))  //������վ�ڴ�Ĵ�С//
		return -1;
	
	//������д����վ��Ӧ���ڴ���//
	unsigned short Int_data;  //ÿ���ֵ�ֵ��Ҫ���е�װ//
	for(num=0;num<SetInt_count;++num)
	{
		Int_data = Slave_shell->read_datas[(num<<1)+7];
		Int_data <<= 8;
		Int_data |= Slave_shell->read_datas[(num<<1)+8];

		(modbus_master_data->typeByte_datas[master_byteAddr]) = (Int_data&0xff);
		(modbus_master_data->typeByte_datas[master_byteAddr+1]) = (Int_data>>8);
		master_byteAddr += 2;
	}
	
	//��װ���ݰ��ظ���վ//
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
	
	//����װ�õ����ݰ����ͳ�ȥ//
	write((modbus_master_data->uart_fd),send_data,8);
	
	free(send_data);
	send_data = NULL;
	return 0;
}


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
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
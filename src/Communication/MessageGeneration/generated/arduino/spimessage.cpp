/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-11-19 13:28:18.630424***/
/***Target: Arduino ***/
int encode_TestMessageCounterSPI(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = value1;
	*p_outbuffer++ = value2;
	*p_outbuffer++ = value3;
	*p_outbuffer++ = value4;
	*p_outbuffer++ = value5;
	*p_outbuffer++ = value6;
	*p_outbuffer++ = value7;
	*p_outbuffer++ = value8;
	*p_outbuffer++ = value9;
	*p_outbuffer++ = value10;
	*p_outbuffer++ = value11;
	*p_outbuffer++ = value12;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}
int encode_Get_DIO_Port1SPI(unsigned char* outbuffer,int* length,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = Pin1_Value;
	*p_outbuffer++ = Pin2_Value;
	*p_outbuffer++ = Pin3_Value;
	*p_outbuffer++ = Pin4_Value;
	*p_outbuffer++ = Pin5_Value;
	*p_outbuffer++ = Pin6_Value;
	*p_outbuffer++ = Pin7_Value;
	*p_outbuffer++ = Pin8_Value;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}
int encode_Get_ANA_Port1SPI(unsigned char* outbuffer,int* length,unsigned int Pin1_Value,unsigned int Pin2_Value,unsigned int Pin3_Value,unsigned int Pin4_Value,unsigned int Pin5_Value,unsigned int Pin6_Value)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = Pin1_Value>>8;
	*p_outbuffer++ = Pin1_Value;
	*p_outbuffer++ = Pin2_Value>>8;
	*p_outbuffer++ = Pin2_Value;
	*p_outbuffer++ = Pin3_Value>>8;
	*p_outbuffer++ = Pin3_Value;
	*p_outbuffer++ = Pin4_Value>>8;
	*p_outbuffer++ = Pin4_Value;
	*p_outbuffer++ = Pin5_Value>>8;
	*p_outbuffer++ = Pin5_Value;
	*p_outbuffer++ = Pin6_Value>>8;
	*p_outbuffer++ = Pin6_Value;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}

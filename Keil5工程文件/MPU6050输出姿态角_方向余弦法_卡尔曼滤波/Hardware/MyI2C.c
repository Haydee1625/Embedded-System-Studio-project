#include "stm32f10x.h"                  // Device header
#include "Delay.h"            

void MyI2C_W_SCL(uint8_t BitValue)                          //SCL引脚写操作的封装改名
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction)BitValue);
	Delay_us(10);
}

void MyI2C_W_SDA(uint8_t BitValue)							//SDA引脚写操作的封装改名
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_11,(BitAction)BitValue);
	Delay_us(10);
}
uint8_t MyI2C_R_SDA(void)									//SDA引脚读操作的封装改名
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
	Delay_us(10);
	return BitValue;
}

void MyI2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructrue;
	GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructrue.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructrue);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10 | GPIO_Pin_11);
}

void MyI2C_Start(void)               //SCL高电平期间，SDA产生下降沿
{
	MyI2C_W_SDA(1);    //先释放SDA，避免终止条件提前出现
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);    
}

void MyI2C_Stop(void)               //SCL高电平期间，SDA出现上升沿
{
	MyI2C_W_SDA(0);    //时序单元开始时先拉低SDA，确保其能产生上升沿
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));    //BitAction参数具有非零即一的特性，故0x00等价于0，其余结果都是1
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	MyI2C_W_SDA(1);                        //主机释放控制权，转换为输入模式
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);
		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}    //if成立，指定位或1，即变成1，其余位不变
		MyI2C_W_SCL(0);
	}
	return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);          //主机释放控制权，防止干扰从机
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}












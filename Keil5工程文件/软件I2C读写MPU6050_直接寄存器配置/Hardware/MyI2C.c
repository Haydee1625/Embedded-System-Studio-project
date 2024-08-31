#include "stm32f10x.h"                  // Device header
#include "Delay.h"            


void MyI2C_W_SCL(uint8_t BitValue)                            //SCL引脚写操作的封装
{
	/*
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction)BitValue);
	Delay_us(10);
	*/
	if (BitValue != Bit_RESET)
	{
		GPIOB->BSRR = 0x00000400;    //GPIO位清除/设置寄存器GPIO_BSRR的GPIO_Pin_10位置置1
	}
	else
	{
		GPIOB->BRR = 0x00000400;    //GPIO位清除寄存器GPIO_BRR的GPIO_Pin_10位置置1
	}
	Delay_us(10);
}

void MyI2C_W_SDA(uint8_t BitValue)							//SDA引脚写操作的封装
{
	/*
	GPIO_WriteBit(GPIOB,GPIO_Pin_11,(BitAction)BitValue);
	Delay_us(10);
	*/
	if (BitValue != Bit_RESET)
	{
		GPIOB->BSRR = 0x00000800;    //GPIO位清除/设置寄存器GPIO_BSRR的GPIO_Pin_11位置置1
	}
	else
	{
		GPIOB->BRR = 0x00000800;    //GPIO位清除寄存器GPIO_BRR的GPIO_Pin_11位置置1
	}
	Delay_us(10);
}


uint8_t MyI2C_R_SDA(void)									//SDA引脚读操作的封装
{
	/*
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
	Delay_us(10);
	return BitValue;
	*/
	
	uint8_t bitstatus = 0x00; 
  
	if ((GPIOB->IDR & 0x00000800) != (uint32_t)Bit_RESET)  //IDR输入寄存器的GPIO_Pin_11对应位为1
	{
		bitstatus = (uint8_t)Bit_SET;
	}
	else                                                   //IDR输入寄存器的GPIO_Pin_11对应位为0
	{
		bitstatus = (uint8_t)Bit_RESET;
	}
	Delay_us(10);
	return bitstatus;
}

void MyI2C_Init(void)
{
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);    //RCC开启APB2时钟    
	RCC->APB2ENR = 0x00000008;
	
	/* GPIO输入输出模式配置
	GPIO_InitTypeDef GPIO_InitStructrue;
	GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructrue.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructrue);
	*/
	GPIOB->CRH = 0x00007700;     //Pin10 Pin11; 开漏输出 ; 50MHz
	
	//GPIO_SetBits(GPIOB,GPIO_Pin_10 | GPIO_Pin_11);    //置高电平 
	GPIOB->ODR = 0x00000C00;
}

void MyI2C_Start(void)               //SCL高电平期间，SDA产生下降沿
{
	//MyI2C_W_SDA(1);    //先释放SDA，避免终止条件提前出现
	//MyI2C_W_SCL(1);
	//MyI2C_W_SDA(0);
	//MyI2C_W_SCL(0);  
	GPIOB->BSRR = 0x00000800;
	Delay_us(10);
	GPIOB->BSRR = 0x00000400;
	Delay_us(10);
	GPIOB->BRR = 0x00000800;
	Delay_us(10);
	GPIOB->BRR = 0x00000400;
	Delay_us(10);
}

void MyI2C_Stop(void)               //SCL高电平期间，SDA出现上升沿
{
	//MyI2C_W_SDA(0);    //时序单元开始时先拉低SDA，确保其能产生上升沿
	//MyI2C_W_SCL(1);
	//MyI2C_W_SDA(1);
	GPIOB->BRR = 0x00000800;
	Delay_us(10);
	GPIOB->BSRR = 0x00000400;
	Delay_us(10);
	GPIOB->BSRR = 0x00000800;
	Delay_us(10);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		//MyI2C_W_SDA(Byte & (0x80 >> i));    //BitAction参数具有非零即一的特性，故0x00等价于0，其余结果都是1
		//MyI2C_W_SCL(1);
		//MyI2C_W_SCL(0);
		
		if ((Byte & (0x80 >> i)) != Bit_RESET)
		{
			GPIOB->BSRR = 0x00000800;    
		}
		else
		{
			GPIOB->BRR = 0x00000800;    
		}
		Delay_us(10);
		
		GPIOB->BSRR = 0x00000400;
		Delay_us(10);
		
		GPIOB->BRR = 0x00000400;
		Delay_us(10);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	//MyI2C_W_SDA(1);                        //主机释放控制权，转换为输入模式
	GPIOB->BSRR = 0x00000800;
	for (i = 0; i < 8; i ++)
	{
		//MyI2C_W_SCL(1);
		//if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}    //if成立，指定位或1，即变成1，其余位不变
		//MyI2C_W_SCL(0);
		
		GPIOB->BSRR = 0x00000400;
		Delay_us(10);
		
		if ((GPIOB->IDR & 0x00000800) != (uint32_t)Bit_RESET)
		{
			Delay_us(10);
			Byte |= (0x80 >> i);
		}
		
		GPIOB->BRR = 0x00000400;
		Delay_us(10);
	}
	return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
	//MyI2C_W_SDA(AckBit);
	//MyI2C_W_SCL(1);
	//MyI2C_W_SCL(0);
	if (AckBit != Bit_RESET)
	{
		GPIOB->BSRR = 0x00000800;    
	}
	else
	{
		GPIOB->BRR = 0x00000800;    
	}
	Delay_us(10);
	
	GPIOB->BSRR = 0x00000400;
	Delay_us(10);
	
	GPIOB->BRR = 0x00000400;
	Delay_us(10);
}

uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	//MyI2C_W_SDA(1);          //主机释放控制权，防止干扰从机
	//MyI2C_W_SCL(1);
	//AckBit = MyI2C_R_SDA();
	//MyI2C_W_SCL(0);
	
	GPIOB->BSRR = 0x00000800;
	Delay_us(10);
	
	GPIOB->BSRR = 0x00000400;
	Delay_us(10);
	
	AckBit = GPIOB->IDR & 0x00000800;
	
	GPIOB->BRR = 0x00000400;
	Delay_us(10);
	
	return AckBit;
}

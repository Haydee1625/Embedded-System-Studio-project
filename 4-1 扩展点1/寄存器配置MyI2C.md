# ��չ��1�����Ĵ�������MyI2C      

## ֪ʶ�ʼ�       
**1. ֱ�����üĴ�������ÿ⺯��**     
�⺯���ĵײ�������üĴ�����ֻ�����⺯��������ϵͳ�ķ�װ������ֱ�����üĴ���������ֱ���׶���������ͬʱʹ��Ҳ���ӷ��㡣    
�ڹ����ļ��У����ǿ��Զ��Ҽ��⺯������һ��һ��ġ���ת�����塱���Ϳ����𲽽ӽ��ײ�ԼĴ����Ĳ�����        

**2. ֱ�����üĴ����ĺô�**
- ���ӽ��ײ㣬���ڸ����׸����Ĵ����Ĺ��ܺ�����ϸ��   
- ������ԼĴ�������λ�����������ı�ĳһλ��ֵ      

**3. ���ֱ�����üĴ���**   
���üĴ�����������Ҫ����STM32�ٷ��ֲᣬ�����ж�����ļ��ͼĴ����꾡������   
1. �ȸ������Ҫ�����Ķ���Ҫ����������   
2. ���ֲ�Ŀ¼���ҵ���Ҫ��ģ�鵥Ԫ������йؼĴ���    
3. �ҵ���Ҫʵ�ֵĹ������Ӧ�ļĴ������Ķ��Ĵ�������    
4. д��Ŀ��Ĵ����ĸ�ֵ��ת��Ϊ16����   
5. �ڹ�����ֱ�Ӹ��Ĵ�����ֵ     
6. ÿ������д��ע�ͣ���Ȼ֮��ܿ��ܲ�֪���ڸ�ʲô   
> �����ֶΣ����ҵ���Ӧ�Ŀ⺯����Ȼ����ת�����⺯���ڲ��Ĵ��룬�������ľ����ھ�⺯���ĺ��Ĳ������֡�    

**4. ����дMyI2C�ļ��йصļĴ���**    

- **My_I2C��ʼ��**
    - *APB2����ʱ��ʹ�ܼĴ���*
    <img src="RCC_APB2ENR.png" style="width:100%;padding-left:0%">      
    ͼʾ��ֵ������APB2������GPIOB�����ʱ�ӡ�     

    - *�˿����ø߼Ĵ���*
    <img src="GPIOx_CRH.png" style="width:100%; padding-left:0%">  
    CNF��00-���������01-��©�����10-�������������11-���ÿ�©���   
    MODE��00-����ģʽ��01-����������10MHz��10-2MHz��11-50MHz
    ͼʾ��ֵ��������Pin10 ��Pin11 ���� ��©��� ; ����Ƶ��50MHz��    

     - *�˿��������ݼĴ���*
    <img src="GPIOx_ODR.png" style="width:100%;padding-left:0%">      
    ͼʾ��ֵ������Pin10 ��Pin11�ߵ�ƽ��       

- **MyI2C�������ʱ��**     
    - *�˿�λ����/����Ĵ���*
    <img src="GPIOx_BSRR.png" style="width:100%;padding-left:0%">  
    ��16λΪ���ã���16λ�����     
    ͼʾ��ֵ��������Pin_10Ϊ�ߵ�ƽ��     

    - *�˿�λ����Ĵ���*
    <img src="GPIOx_BRR.png" style="width:100%;padding-left:0%">      
    ͼʾ��ֵ�������Pin_10�ĸߵ�ƽ��Ҳ��������͵�ƽ��

    - *�˿��������ݼĴ���*
    <img src="GPIOx_IDR.png" style="width:100%;padding-left:0%">   
    ����I2C��SDA�����ݣ����Ը�ֵ��ȷ����

## ʵ�ֲ���      
  
�����д�����ע�͵�����ԭ�⺯�����룬�����������ڿ���ֱ�����üĴ�������ͼ�� 

**1. ��ʼ��I2C��MyI2C_Init��������**     
~~~
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);    
RCC->APB2ENR = 0x00000008;    //RCC����APB2ʱ��    
	
/* GPIO�������ģʽ����
GPIO_InitTypeDef GPIO_InitStructrue;
GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_Out_OD;
GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
GPIO_InitStructrue.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB,&GPIO_InitStructrue);
*/
GPIOB->CRH = 0x00007700;     //Pin10 Pin11 ; ��©��� ; 50MHz
	
//GPIO_SetBits(GPIOB,GPIO_Pin_10 | GPIO_Pin_11);     
GPIOB->ODR = 0x00000C00;    //Pin10 Pin11�øߵ�ƽ
~~~      

**2. I2C��ʼ�źţ�MyI2C_Start��������**
~~~
void MyI2C_Start(void)    //SCL�ߵ�ƽ�ڼ䣬SDA�����½���
{
	//MyI2C_W_SDA(1);    
	//MyI2C_W_SCL(1);
	//MyI2C_W_SDA(0);
	//MyI2C_W_SCL(0);  

	GPIOB->BSRR = 0x00000800;    //���ͷ�SDA��������ֹ������ǰ����
	Delay_us(10);
	GPIOB->BSRR = 0x00000400;    //SCL�ߵ�ƽ
	Delay_us(10);
	GPIOB->BRR = 0x00000800;    //SDA�����½���
	Delay_us(10);
	GPIOB->BRR = 0x00000400;    //SCL�͵�ƽ
	Delay_us(10);
}
~~~    

**3. I2C��ֹ�źţ�MyI2C_Stop��������**
~~~
void MyI2C_Stop(void)    //SCL�ߵ�ƽ�ڼ䣬SDA����������
{
	//MyI2C_W_SDA(0);    
	//MyI2C_W_SCL(1);
	//MyI2C_W_SDA(1);
    
	GPIOB->BRR = 0x00000800;    //��ʼʱ������SDA��ȷ�����ܲ���������
	Delay_us(10);
	GPIOB->BSRR = 0x00000400;    //SCL�ߵ�ƽ
	Delay_us(10);
	GPIOB->BSRR = 0x00000800;    //SDA����������
	Delay_us(10);
}
~~~

**4. I2C����һ���ֽڣ�MyI2C_SendByte��uint8_t Byte������**    
~~~
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		//MyI2C_W_SDA(Byte & (0x80 >> i));    
		//MyI2C_W_SCL(1);
		//MyI2C_W_SCL(0);
		
		if ((Byte & (0x80 >> i)) != Bit_RESET)    //���Byte��ӦλΪ1
		{
			GPIOB->BSRR = 0x00000800;    //SDA�øߵ�ƽ    
		}
		else
		{
			GPIOB->BRR = 0x00000800;    //����SDA�õ͵�ƽ
		}
		Delay_us(10);
		
		GPIOB->BSRR = 0x00000400;    //SCL�øߵ�ƽ
		Delay_us(10);
		
		GPIOB->BRR = 0x00000400;    //SCL�õ͵�ƽ
		Delay_us(10);
	}
}
~~~      

**5. I2C����һ���ֽڣ�MyI2C_ReceiveByte��uint8_t Byte������**   
~~~ 
uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	//MyI2C_W_SDA(1);                        
	GPIOB->BSRR = 0x00000800;    //�����ͷſ���Ȩ��ת��Ϊ����ģʽ                       
	for (i = 0; i < 8; i ++)
	{
		//MyI2C_W_SCL(1);
		//if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}    
		//MyI2C_W_SCL(0);
		
		GPIOB->BSRR = 0x00000400;    //SCL�øߵ�ƽ
		Delay_us(10);
		
		if ((GPIOB->IDR & 0x00000800) != (uint32_t)Bit_RESET)  
		{
			Delay_us(10);
			Byte |= (0x80 >> i);    //������ոߵ�ƽ�����Ӧλд1
		}
		
		GPIOB->BRR = 0x00000400;    //SCL�õ͵�ƽ
		Delay_us(10);
	}
	return Byte;    //���ؽ�������
}
~~~

**6. I2C����Ӧ��MyI2C_SendAck(uint8_t AckBit)����** 
~~~  
void MyI2C_SendAck(uint8_t AckBit)
{
	//MyI2C_W_SDA(AckBit);
	//MyI2C_W_SCL(1);
	//MyI2C_W_SCL(0);

	if (AckBit != Bit_RESET)    //���Ӧ��Ϊ1
	{
		GPIOB->BSRR = 0x00000800;    //SDA�øߵ�ƽ
	}
	else
	{
		GPIOB->BRR = 0x00000800;    //����SDA�õ͵�ƽ
	}
	Delay_us(10);
	
	GPIOB->BSRR = 0x00000400;    //SCL�øߵ�ƽ
	Delay_us(10);
	
	GPIOB->BRR = 0x00000400;    //SCL�õ͵�ƽ
	Delay_us(10);
}
~~~

**7. I2C����Ӧ��MyI2C_ReceiveAck(void)����** 
~~~
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	//MyI2C_W_SDA(1);          
	//MyI2C_W_SCL(1);
	//AckBit = MyI2C_R_SDA();
	//MyI2C_W_SCL(0);
	
	GPIOB->BSRR = 0x00000800;    //�����ͷſ���Ȩ����ֹ���Ŵӻ�
	Delay_us(10);
	
	GPIOB->BSRR = 0x00000400;    //SCL�øߵ�ƽ
	Delay_us(10);
	
	AckBit = GPIOB->IDR & 0x00000800;   //��ȡSDA��ƽ
	
	GPIOB->BRR = 0x00000400;    //SCL�õ͵�ƽ
	Delay_us(10);
	
	return AckBit;    //����SDA��ƽ�ź�
}
~~~     

**8. ����д��MyI2C��дMPU6050ԭʼ����**     

## ��ͼ     

<img src="��չ��1��ͼ.png" style="width:100%;padding-left:0%">    

## ��Ƭ    

<img src="��չ��1��Ƭ.jpg" style="width:80%;padding-left:10%">   



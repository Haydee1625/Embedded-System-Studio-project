# ��չ��5����ͨ��DMA�ӿ����ݴ���        

## ֪ʶ�ʼ�        
**1. DMA����**  
CPU��Ϊ����оƬ�ĺ��ģ�������������Э���ͽ������䴦��Ĺ������Ǻܴ�ġ���������ֻ��CPU�ܴ�Ҫ��һ�ֹ�����CPU����Ҫ�Ĺ����ǽ����������㣬�����ж��������Ӧ�ȡ�
�����ݰ���ȴ����ռ�ô󲿷ֵ�CPU��Դ����Ϊ�˽���CPU�Ĺ���Ч�ʵ���Ҫԭ��֮һ��������Ҫһ��Ӳ���ṹ�ֵ�CPU��һ���� ���� DMA��      

**2. DMAת������**       
�����ݰ��˵ĽǶȿ������Ҫ�Ѵ洢��ַA����ֵ��������һ����ַ��B�ı�����CPUʵ�ֹ���Ϊ���ȶ���A��ַ�ϵ����ݴ洢��һ���м������Ȼ����ת�͵�B��ַ�ı����ϡ�ʹ��DMA����Ҫ�м������ֱ�ӽ�A��ַ����ֵ���͵�B��ַ�ı����    

**3. DMA��Դ**    
STM32 DMA��Դ��12�����������õ�ͨ���� DMA1��7��ͨ������DMA2��5��ͨ����
STM32F103C8T6 DMA��Դ��DMA1��7��ͨ����
 
**4. DMA����**   
���ࣺ����Ĵ����ʹ洢�����洢���ʹ洢��      
�洢�����������ڴ�SRAM������洢��Flash�ͼĴ�����         
����Ĵ����ʹ洢��һ����Ӳ���������洢���ʹ洢��һ�������������     

**5. DMA�ṹ��ϵ**   
<img src="DMA�ṹͼ.png" style="width:100%; padding-left: 0%;"/>    
- ���߾�����ߣ�������Ԫ���ɶ�д�ұߵĴ洢���������Ĵ�����
    ���߾����ұߣ�������Ԫ��ֻ�ܱ���ߵ�Ԫ��д
    DMAģ����������ߣ�֤�������ж�д�洢���ͼĴ����ķ���Ȩ�ޡ� 
    DMAģ��ͬʱ�������ұߣ�Ҳ֤��������Ϊһ�����裬Ҳ����Ӧ����Ҫ����д���õļĴ�����  
- ϵͳ�ṹ����DMA�йص��������
    (1) ���ڷ��ʴ洢����DMA����      
    (2) �ڲ��ɶ���ת�˵Ķ�ͨ��        
    (3) �ٲö��ͨ�������ȼ��������ͻ
    (4) ����DMA����
    (5) Ӳ������DMA����ת��     

**5. DMA�ڲ��ṹ**     
��ť�͵Ĳ����ǿ⺯������DMAʱ���ص�
<img src="DMA�ڲ�.png" style="width:100%; padding-left: 0%;"/>    

**6. DMAת��˫�����������**   
��ʼ��ַ�����ݿ�ȣ���ַ�Ƿ�����
- ��ʼ��ַ������DMAת�˵ķ��͵غͽ��ܵ�
- ���ݿ��Ӱ�������ݽ��ܵķ�ʽ�����ֽڣ�8bit�������֣�16bit����ȫ�֣�32bit�� 
- ��ַ�Ƿ�����������1.Դ���Ƿ��ظ�����ֵ 2.���ն��Ƿ񸲸ǽ���ֵ

**7. �洢��ӳ��**      
����ͨ��ȡ��ַ�۲�洢����ӳ���ַ�����б������������ڴ�SRAM�С�
Ҫ��õ�ĳ����Ĵ����Ļ���ַ�������Ȳ�ѯ����ĳ�ʼ��ַ���ټ���ƫ�Ƽ��ɡ�

| �洢������   | ��ʼ��ַ(0x) | �洢��    |
| ----------- | ----------- |-----------|
| ROM         | 0800 0000   | ����洢��Flash|
| ROM         | 1FFF F000   | ϵͳ�洢��|
| ROM         | 1FFF F800   | ѡ���ֽ�|
| RAM         | 2000 0000   | �����ڴ�SRAM |
| RAM         | 4000 0000   | ����Ĵ��� |
| RAM         | E000 0000   | �ں�����Ĵ��� |

## ʵ�ֲ���       
DMA��������������������Դ�д����źţ���������������㣬DMAʹ�ܣ���Ӧ����23.
1. AHB����ʱ�ӣ�DMA�ǹ�����AHB�����ϵ��豸��     
~~~
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
~~~
2. ��ʼ��DMA����Ӧ�����DMA�ڲ��ṹͼ�������ʼ���ṹ�壬���ó�Ա��������GPIO��ʼ�����ƣ�     
~~~
������void MyDMA_Init(uint32_t AddrA, uint32_t AddrB, uint16_t Size)
//�����ʼ���ṹ��
DMA_InitTypeDef DMA_InitStructure;
//����վ��������������ʼ��ַ�����ݿ�ȣ���ַ�Ƿ�����
DMA_InitStructure.DMA_PeripheralBaseAddr = AddrA;
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
//�洢��վ��������������ʼ��ַ�����ݿ�ȣ���ַ�Ƿ�����
DMA_InitStructure.DMA_MemoryBaseAddr = AddrB;
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//DMA���䷽������վ��ʱԴ�˻���Ŀ�ĵأ�DST or SRC
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//�������������Χ0~65535
DMA_InitStructure.DMA_BufferSize = Size;
//�Ƿ��Զ���װ
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//ѡ�񴥷���ʽ��"memory to memory" �洢�����洢��
DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
//���ȼ�
DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
DMA_Init(DMA1_Channel1, &DMA_InitStructure);
~~~      
ע�����
- �Զ���װ�������������ͬʱʹ�ã�����DMA����ֹ����    

3. DMAʹ��       
~~~ 
DMA_Cmd(DMA1_Channel1, DISABLE);
~~~   

�����Ѿ�����ʵ�ִ�A��B��һ��ת�˹����ˣ��������B��ֵ��������A�仯����Ҫ�������²��衣

4. дDMA���亯�������´��봫�����ֵ    
~~~
void MyDMA_Transfer(void)
{
	DMA_Cmd(DMA1_Channel1, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel1, MyDMA_Size);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	//����DMA���㹤������������
	while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);  //�ȴ�ת�����
	DMA_ClearFlag(DMA1_FLAG_TC1);  //�����ɱ�־
}

~~~
- ��д���������ʱҪȷ��DMA����Ϊ�ر�״̬ ����DMAʧ��   

5. ʹ��DMA����ת�ˣ��ȳ�ʼ������������������ַ�����������ֵ�������������Ҫ�ĵط�ʹ��MyDMA_Transfer()���ɣ���һ��ת��һ�Ρ�  
~~~
//����
MyDMA_Init((uint32_t)DataA, (uint32_t)DataB, 4);
������
void MyDMA_Transfer()��
������
~~~   
6. ʵ��Ӧ�ã�     
���ݹ�ģҪ��������ֳ�DMA�����ơ�      
��MPU6050��̬�ǽ�����龰�£�Ҫ�õ�����ת�˵Ŀ����У�     
    - MPU6050��ȡ��ԭʼ����AX��ת�˵���ת���ı���AX_Convert�� 
    - ��̬��pitc��roll��yawת�˵�����Ƕȱ���Angle��   
    ������       
���϶��ǴӴ洢�����洢����ת�˹���

## ��ͼ     
       
<img src="��չ��5��ͼ.png" style="width:80%; padding-left: 10%;"/>      
      
## ��Ƭ    
�ԣ����������MPU6050�����̬�ǣ��������ת��û���κ�����  


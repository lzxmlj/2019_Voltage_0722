#include "uart.h"
#include <stdarg.h>
#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
/* Private functions ---------------------------------------------------------*/

void USART_Configuration(void)//���ڳ�ʼ������
  {  

        GPIO_InitTypeDef  GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
                
        RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE );
        RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOF, ENABLE);
        GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_7);
        GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_7);        
        /*
        *  USART1_TX -> PA9 , USART1_RX ->        PA10
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
        GPIO_Init(GPIOF, &GPIO_InitStructure);        

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;                 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
        GPIO_Init(GPIOC, &GPIO_InitStructure);        
       
        USART_InitStructure.USART_BaudRate = 38400;//���ô��ڲ�����
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��������λ
        USART_InitStructure.USART_StopBits = USART_StopBits_1;//����ֹͣλ
        USART_InitStructure.USART_Parity = USART_Parity_No;//����Ч��λ
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����������
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//���ù���ģʽ
        USART_Init(USART3, &USART_InitStructure); //������ṹ��
        USART_Cmd(USART3, ENABLE);//ʹ�ܴ���1

		}			

void UART_send_byte(uint8_t byte) //����1�ֽ�����
{
 while(!((USART3->ISR)&(1<<7)));
 USART3->TDR=byte;	
}		

void UART_Send(uint8_t *Buffer, uint32_t Length)
{
	while(Length != 0)
	{
		while(!((USART3->ISR)&(1<<7)));//�ȴ�������
		USART3->TDR= *Buffer;
		Buffer++;
		Length--;
	}
}

uint8_t UART_Recive(void)
{	
	while(!(USART3->ISR & (1<<5)));//�ȴ����յ�����
	return(USART3->RDR);			 //��������
}


PUTCHAR_PROTOTYPE 
{
/* ��Printf���ݷ������� */
  USART_SendData(USART3,(uint8_t)  ch);
  while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
	{}
 
  return (ch);
}



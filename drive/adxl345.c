/****************************************************************************
 * �ļ���  ��i2c_ee.c
 * ����    ��i2c EEPROM(AT24C02)Ӧ�ú�����         
 * ʵ��ƽ̨��
 * Ӳ�����ӣ�-----------------
 *          |                 |
 *          |  PB8-I2C2_SCL		|
 *          |  PB9-I2C2_SDA   |
 *          |                 |
 *           -----------------
 * ��汾  ��ST3.0.0

**********************************************************************************/
#include "stm32f37x.h"
#include "Stm32f37x_conf.h"



static void ASXL345_Init(void)
{
	
		GPIO_InitTypeDef GPIO_InitStructure;
	
		/* ʹ���� I2C1 �йص�ʱ�� */
	 RCC_APB2PeriphClockCmd  (RCC_APB2Periph_GPIOB, ENABLE ); 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);  

	 /* PB8-I2C2_SCL��PB9-I2C2_SDA*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	I2C_InitTypeDef I2C_InitStructure; 
	 
	  /* I2C ���� */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C ; 
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; 
	I2C_InitStructure.I2C_OwnAddress1 = MMA_ADRESS; 
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; 
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 
	I2C_InitStructure.I2C_ClockSpeed = I2C_Speed; 
	
	/* ʹ�� I2C1 */
	I2C_Cmd  (I2C1,ENABLE); 

	/* I2C1 ��ʼ�� */
	I2C_Init(I2C1, &I2C_InitStructure);	   

	/*����Ӧ��ģʽ*/
	I2C_AcknowledgeConfig(I2C1, ENABLE);   	
	
	}
	
	
	
	
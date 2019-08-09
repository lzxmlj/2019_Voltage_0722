#include "stm32f37x_conf.h"
#include "stdint.h"
#include "stm32f37x_spi.h"
#include "io_can_config.h"
   void SPI1_Configuration(void)
   {
      SPI_InitTypeDef SPI_InitStructure;
      GPIO_InitTypeDef GPIO_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;
   
      RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC, ENABLE);
     /* Connect CAN pins to AF7 */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
   
      SPI_I2S_DeInit(SPI1);
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //??????
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8  | GPIO_Pin_7 |GPIO_Pin_9;
      GPIO_Init(GPIOC, &GPIO_InitStructure);
      //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
      //GPIO_Init(GPIOA, &GPIO_InitStructure);
      
      //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
      //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //??????
      //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
     // GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
     // GPIO_Init(GPIOF, &GPIO_InitStructure);   
      //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
      //GPIO_Init(GPIOF, &GPIO_InitStructure);   
      
      GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_5);  
      GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_5); 
      GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_5); 
      //GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_5); 
   
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1
      //SPI_I2S_DeInit(SPI2);
      //SPI_Cmd(SPI2, DISABLE);
      //--------------------- SPI1 configuration ------------------
      SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
      SPI_InitStructure.SPI_Mode   = SPI_Mode_Master;
      SPI_InitStructure.SPI_DataSize  = SPI_DataSize_8b;
      SPI_InitStructure.SPI_CPOL   = SPI_CPOL_Low;
      SPI_InitStructure.SPI_CPHA   = SPI_CPHA_2Edge;
      SPI_InitStructure.SPI_NSS   = SPI_NSS_Soft;
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
      SPI_InitStructure.SPI_FirstBit    = SPI_FirstBit_MSB;
      SPI_InitStructure.SPI_CRCPolynomial  = 7;
      SPI_Init(SPI1, &SPI_InitStructure);
      //SPI_SSOutputCmd(SPI2, ENABLE);
      //SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
      //--------- SPI2 configuration ------------------
      //SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
      //SPI_Init(SPI2, &SPI_InitStructure);
      //--------- Enable SPI1 TXE interrupt ------------
      //SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
      //--------- Enable SPI2 RXNE interrupt -------------
   //   SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE | SPI_I2S_IT_OVR, ENABLE);
      //----- Enable SPI2 ------
       SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
      //SPI_SSOutputCmd(SPI2, ENABLE);
#if 1
      SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
       NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
  #endif
      SPI_Cmd(SPI1, ENABLE);
      GPIO_WriteBit(GPIOC, GPIO_Pin_0,0);
      GPIO_WriteBit(GPIOB, GPIO_Pin_0,1);
      //----- Enable SPI1 ----
      //SPI_Cmd(SPI1, ENABLE);
   }
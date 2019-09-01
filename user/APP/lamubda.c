#include "stm32f37x_spi.h"
#include "stm32f37x_gpio.h"
#include "uart.h"
#include "systick.h"
#include <stdio.h>
#include "stm32f37x.h"
#include "reuse.h"
#define APP 1
//#define CAL_APP 1
extern uint16_t temp_readback;
float KO2_V_LamCalLoThres, KO2_V_LamCalHiThres, KO2_t_LamReCal, KO2_V_HtrEffDewPt, KO2_V_HtrEffHiLim;
float KO2_T_HtrEffDewPtTiming, KO2_V_HtrEffInit,Ua0,Ur0;
extern float g_cala[14],g_calb[14];
float  Vfsd1_9, Vfsd1_7, LAMOFST, histroy_cal, RI,Rpmes,Vfair_fuel;
extern float data1, data2, vad2, a1,b1,c1,a2,b2,c2,ref,VfIpmeas, Vua, Vur, Vfvalue_air;
extern uint8_t* ADC_float, outputdata[56];
extern uint16_t UR, UA, AD11, AD12;
extern uint16_t SPI_feedback;

extern u16 SPIByte(u16 word);
void lamuda_loop_app(void);
static void delay (int cnt) 
{
  while (cnt--);
}
void lamubda_initialize(void)
{
#if APP
   GPIO_WriteBit(GPIOB,GPIO_Pin_0, 1);
   temp_readback = SPIByte(0x5688);//5689=17,5688=8
   delay(200);
#endif
#if CAL_APP
   GPIO_WriteBit(GPIOB,GPIO_Pin_0, 1);
   temp_readback = SPIByte(0x569D);
   delay(200);
#endif
   GPIO_WriteBit(GPIOC, GPIO_Pin_6,1);
   temp_readback = SPIByte(0x6C00);
   delay(200);
   GPIO_WriteBit(GPIOC, GPIO_Pin_6,1);
   temp_readback = SPIByte(0x5A02);
   delay(200);
   GPIO_WriteBit(GPIOC, GPIO_Pin_6,1);
   temp_readback = SPIByte(0x7E00);

}
void lamuda_loop_app(void)
{
    uint8_t j;

    float lambdaMax=32.77f;
    float heat_pwm=1;
    KO2_V_LamCalLoThres = g_cala[1];//1
    KO2_V_LamCalHiThres = g_cala[2];//1
    KO2_t_LamReCal = g_cala[3];//1
    KO2_V_HtrEffDewPt =  g_cala[4];//4
    KO2_V_HtrEffHiLim =  g_cala[5];//12
    KO2_T_HtrEffDewPtTiming = g_cala[6];//1	 15s
    KO2_V_HtrEffInit = g_cala[7];//1	8.5V
    LAMOFST = g_cala[8];//1.5
	  j = 0;
#if APP
    Rpmes = (float)((u16)(UA + 32768)* 2.5/ 65535);//UA v
    Rpmes=g_cala[11]*Rpmes+g_calb[11];//ax+b
    //Rpmes=0.1*Rpmes+0.9*Ua0;
    //Ua0=Rpmes;
    
    Vfvalue_air = (float)((u16)(UR + 32768) * 5.0 / 65535 );//UR v
    Vfvalue_air=0.1*Vfvalue_air+0.9*Ur0;
    Ur0=Vfvalue_air;
        
    VfIpmeas = (float)( Rpmes - 1.5) /8.0/61.9*1000.0;//1052.3=17,495.2=8  IP ma

    if(VfIpmeas<-0.007)
        Vfair_fuel=1.0/(-0.26769*VfIpmeas+1.00148);//lambda
    else
        Vfair_fuel=-0.195+1.0/(-0.32827*VfIpmeas+0.83538);//lambda
        
    RI = (float)((float)(Vfvalue_air - 0.2941 ) / (float)(2449) * 1000000);//RI 
    //control heat
    if(RI>300)
        heat_pwm+=0.005;
    else if(RI<220)
        heat_pwm-=0.5;
    else
        heat_pwm-=0.01;
    if(heat_pwm<0)
        heat_pwm=0;
    if(heat_pwm>60)
        heat_pwm=60;
								
/*	if(VfIpmeas > KfIpmeas[8])
		 {
			Vfair_fuel = Kfvalue_air[8];
		 }
		 else if(VfIpmeas < KfIpmeas[0])
		 {
			Vfair_fuel = Kfvalue_air[0];
		 }
		 else
		 {
		   for(i =0; i < 7; i++)
		   {
			  if(VfIpmeas == KfIpmeas[i])
			  {
				 Vfair_fuel = Kfvalue_air[i];
				 i = 99;
			  }
			  else if((VfIpmeas > KfIpmeas[i]) & (VfIpmeas < KfIpmeas[i + 1]))
			  {
		 
				 Vfair_fuel = (Kfvalue_air[i+1] -Kfvalue_air[i]) / (KfIpmeas[i+1] -KfIpmeas[i]) * VfIpmeas + Kfvalue_air[i];
				 i = 99;
			  }
		   }
		 }*/
        if(RI<350 && Vfair_fuel<4.2f)
            ADC_float = (u8 *)(&Vfair_fuel);//lambda
        else
            ADC_float= (u8 *)(&lambdaMax);
         outputdata[j] = ADC_float[0];
         outputdata[j+1] = ADC_float[1];
         outputdata[j+2] = ADC_float[2];
         outputdata[j+3] = ADC_float[3];
		 j = j + 4;
         ADC_float = (u8 *)(&heat_pwm);//VfIpmeas);//IP ma
         outputdata[j] = ADC_float[0];
         outputdata[j+1] = ADC_float[1];
         outputdata[j+2] = ADC_float[2];
         outputdata[j+3] = ADC_float[3];
         CAN1_Send_Msg(1944,&outputdata[0]);//1944
		 j = j + 4;
         ADC_float = (u8 *)(&RI);//RI
         outputdata[j] = ADC_float[0];
         outputdata[j+1] = ADC_float[1];
         outputdata[j+2] = ADC_float[2];
         outputdata[j+3] = ADC_float[3];
         CAN1_Send_Msg(1945,&outputdata[8]);//1945
		 j = j + 4;
         ADC_float = (u8 *)(&Rpmes);//UA V
         outputdata[j] = ADC_float[0];
         outputdata[j+1] = ADC_float[1];
         outputdata[j+2] = ADC_float[2];
         outputdata[j+3] = ADC_float[3];
         //CAN1_Send_Msg(0x66,&outputdata[12]);
		 j = j + 4;
         ADC_float = (u8 *)(&Vfvalue_air);//UR V
         outputdata[j] = ADC_float[0];
         outputdata[j+1] = ADC_float[1];
         outputdata[j+2] = ADC_float[2];
         outputdata[j+3] = ADC_float[3];
         CAN1_Send_Msg(1946,&outputdata[12]);//6]);//1946
		 j = j + 4;
       outputdata[j] = SPI_feedback / 0x100;
       outputdata[j+1] = SPI_feedback & 0xFF;
		 j = j + 2;
       CAN1_Send_Msg(1947,&outputdata[20]);//6]);//1946

#endif
#ifdef CAL_APP
         Vfvalue_air = (float)((u16)(UR + 32768) * 2.52 / 65535 );
		 RI = (float)((float)(Vfvalue_air - 0.2941 ) / (float)(15.5 * 158) * 1000000);
         ADC_float = (u8 *)(&RI);
         outputdata[j] = ADC_float[0];
         outputdata[j+1] = ADC_float[1];
         outputdata[j+2] = ADC_float[2];
         outputdata[j+3] = ADC_float[3];
         Rpmes = (float)((u16)(UA + 32768)* 2.52/ 65535);
		 VfIpmeas = ( (Rpmes - 1.5) / (61.9 * 17) );
         ADC_float = (u8 *)(&VfIpmeas);
		 j = 4;
         outputdata[j] = ADC_float[0];
         outputdata[j+1] = ADC_float[1];
         outputdata[j+2] = ADC_float[2];
         outputdata[j+3] = ADC_float[3];
         //CAN1_Send_Msg(0x63,testTemp[i]);
         CAN1_Send_Msg(0x64,&outputdata[0]);
#endif

}
/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *           
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  SBUS 测试例程代码
  * 
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>   

#include "ax_sys.h"    //系统设置
#include "ax_delay.h"  //软件延时
#include "ax_led.h"    //LED灯控制
#include "ax_beep.h"   //蜂鸣器控制
#include "ax_uart1.h"  //调试串口

#include "ax_sbus.h" //编码器控制


int main(void)
{	
	uint16_t sbusdata[16];

	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
		
	//软件延时初始化
	AX_DELAY_Init(); 	
    AX_LED_Init();  
	
	//调试串口初始化
	AX_UART1_Init(230400); //调试串口
	printf("  \r\n");      //输出空格，CPUBUG	
	
	AX_SBUS_Init();

	while (1)
	{			
		AX_Delayms(20);
		AX_LED_Green_Toggle();
		
		//获取SBUS解码数据
		if(AX_SBUS_GetRxData(sbusdata))
		{
			//AX_SBUS_Unpack(sbusdata);
				
			printf("c1=%04d c2=%04d c3=%04d ch4=%04d ch5=%04d ch6=%04d ch7=%04d ch8=%04d ch9=%04d ch10=%04d \r\n",
			sbusdata[0],sbusdata[1],sbusdata[2],sbusdata[3],sbusdata[4],sbusdata[5],sbusdata[6],sbusdata[7],sbusdata[8],sbusdata[9]);
		}
	}
	
}

/******************* (C) 版权 2022 XTARK *******************************/

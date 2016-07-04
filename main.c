/* --------------------------Includes ----------------------------------------*/
#include "board.h"
#include "stdio.h"
#include "sensor_init.h"
#include "string.h"

/*------------------------variables -------------------------------------------*/
extern uint8_t timecount ;

enum KEY_STATE{KEY4=1,KEY3=2,KEY2=3,KEY1=4};
struct KEY_ACTION {void(*Action)(void);};
uint8_t set_key_state(enum KEY_STATE sta,struct KEY_ACTION *act);
uint8_t key_poc(struct KEY_ACTION key_act);

void workloop(void);
void refresh_powermode(void);
void Lora_Rec(void);
void lora_test(void);

/*-----------------------------main()----------------------------------------*/
int main(void)
{ 
	uint8_t buf[]="Lora Test";
	
	Board_init();
  OLED_Display_On();
	OLED_ShowString(2,0,buf);
	Lora_Init();
	workloop();
}
uint8_t Tips_Loop(uint8_t num)
{
	switch(num)
	{
		case 1: OLED_ShowString(0,6,"Key1:lora send");
			break;
		case 2: OLED_ShowString(0,6,"Key2:OLED shut");
			break;
		case 3: OLED_ShowString(0,6,"Key3:OLED ON   ");
			break;
		case 4: OLED_ShowString(0,6,"Key4:lora echo ");
			break;
		default: return FALSE;
	}
	return TRUE;
}

void workloop(void)
{
	uint8_t tip_loop_num=0;
	struct KEY_ACTION key_action={0};	
	timecount = 0;
	
	while(1) 
	{
		IWDG_Feed();
		if(timecount>100)
		{
			timecount = 0;
			
			if(tip_loop_num<4)
			{
				tip_loop_num++;
				Tips_Loop(tip_loop_num);
				if(tip_loop_num>3) 
					tip_loop_num = 0;
			}
			refresh_powermode();
		}
		key_poc(key_action);	
	}	
}
void lora_test(void)
{
	uint8_t  databuff[] = {"123456"};
	
	printf("Lora Send Test\n\r");
	OLED_ShowString(0,4,"Lora Send       ");
	Lora_Send(databuff,sizeof(databuff));
	
}
void Lora_Rec(void)
{
	uint8_t Loradata[64];
	uint8_t len;
	uint8_t i;	
	
	printf("Lora TRx Test\n\r");
  len = Lora_GetNumofRecData();	
	if(len)
	{
		Lora_RecDataGet(Loradata,len);
		printf("lora_rec data : ");
		for(i=0;i<len;i++)
		{
			printf("%c ",Loradata[i]);			
		}
		OLED_ShowString(0,4,"Lora Echo OK    ");
		printf("\n\r");
		memset(Loradata,0,len);
	}	
}
void clear_oled(void)
{
	OLED_Clear();
}
void shutdown_oled(void)
{
	OLED_Display_Off();
}
void enable_oled(void)
{
	OLED_Display_On();
}
uint8_t set_key_state(enum KEY_STATE sta,struct KEY_ACTION *act)
{
		switch (sta)
		{
			case KEY1: act->Action=lora_test;return 1;
			case KEY2: act->Action=shutdown_oled;break;
			case KEY3: act->Action=enable_oled;break;
			case KEY4: act->Action=Lora_Rec;break;
			default: return FALSE;
		}
		return TRUE;
}	
uint8_t key_poc(struct KEY_ACTION key_act)
{
	uint8_t key_value;
	key_value = Key_Scan();
		if(key_value)
		{
			set_key_state(key_value,&key_act);
			key_act.Action();
			key_value = 0;
			return TRUE;
		}
		return FALSE;
}
/*
---电池电压显示更加采集的电压分5个等级
小于 3.9V
3.9~4.0V
4.0~4.1V
4.1~4.19V
>4.19V
---供电方式显示
USB插入     闪电显示 + BAT电压
无USB插入   不显示闪电 只显示BAT电压
*/
void refresh_powermode(void)
{
	uint16_t AD_Val[10];

	float Bat_Val,USB_Val;
	memset(AD_Val,0,10);
	
	if(ADC_GetAverage(AD_Val))
	{
		Bat_Val = (float)AD_Val[1]*3.3/4096*6;  // 电池电压
		USB_Val = (float)AD_Val[2]*3.3/4096*6;  // USB供电电压
		
		// USB 供电方式 insert 否检测
		if(USB_Val>4.6)
		{
			OLED_Draw_USB(108,0,124,2,0);
		}
		else 
		{
			// 电池供电图标电压显示更新
			if(Bat_Val<3.9)
			{
				OLED_Draw_Bat(108,0,124,2,4);
			}
			else if(Bat_Val>3.9&&Bat_Val<4.0)
			{
				OLED_Draw_Bat(108,0,124,2,3);
			}
			else if(Bat_Val>4.0&&Bat_Val<4.1)
			{
				OLED_Draw_Bat(108,0,124,2,2);
			}
			else if(Bat_Val>4.1&&Bat_Val<4.19)
			{
				OLED_Draw_Bat(108,0,124,2,1);
			}
			else 
			{
				OLED_Draw_Bat(108,0,124,2,0);
			}
				//OLED_Draw_USB(86,0,102,2,1);
		}
	}
}

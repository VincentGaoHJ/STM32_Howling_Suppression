#include "led.h"
#include "adc.h"
#include "lcd.h"
#include "dac.h"
#include "timer.h"
#include "usart.h"
#include "delay.h"
#include "arm_math.h"
#include "bsp_FFT.h"
#include "stm32_dsp.h"

#define MAX(A,B) ((A > B) ? (A) : (B))

float filter_list[23] = {90,110,134, 164, 201, 245, 300, 367, 448, 548, 669, 818, 1000, 1222, 1494, 1826, 2232, 2728, 3334, 4075, 4980, 6087, 7440 };

uint16_t ReadValue1,ReadValue2,ReadValue3,ReadValue4,ReadValue5; 
int32_t lBufInArray[NPT];
int32_t lBufOutArray[NPT];
float lBufMagArray[NPT];
	
int temppp;
int ReadValue_last = 0;

#define numStages  2               /* 2阶IIR滤波的个数 */
#define TEST_LENGTH_SAMPLES  1    /* 滤波器每次采样点数 */


arm_biquad_casd_df1_inst_f32 S;
static float testInput_f32[TEST_LENGTH_SAMPLES];            /* 采样点 */
static float testOutput[TEST_LENGTH_SAMPLES];               /* 滤波后的输出 */

	
extern int ReadValue;
extern const float IIRCoeffs32LP[5*numStages*23] ;
extern float IIRStateF32[4*numStages];  

//const float ScaleValue[1] = {0.7552019639411458f};


	
//const float IIRCoeffs32LP[5*numStages] = {
//1.0f,  -1.8693825150758503f,  1.0f,    1.8218452247513464f,  -0.98685832391078687f,
//1.0f,  -1.9390053706493142f,  1.0f,    1.9453160862541781f,  -0.99282938459074133f,
//1.0f,  -1.8849194398603f,     1.0f,    1.7070852391494f ,    -0.90723103295422347f,
//1.0f,  -1.9306401708916825f,  1.0f,    1.9222360460931638f,  -0.95887136900486325f,
//1.0f,  -1.9105923088862271f,  1.0f,    1.3852641979374472f,  -0.45008874106164687f};


	
	
/*放缩系数 */
const float ScaleValue[23] = { 0.7862952804592391, 0.7845493366578619, 0.7823979906764283, 0.7797050938211607, 0.7763334615396952, 0.7722799375283812, 0.7674056992162275, 0.7612991551933054, 0.7540900908864602, 0.7449554528683267, 0.7338887548914163, 0.720205620198646, 0.7036429796423648, 0.6834153758205624, 0.6587873658944501, 0.6288446323758432, 0.59296444014834, 0.5501092776258948, 0.4992652223501446, 0.44012881607550725, 0.3723433863725368, 0.2964349473236297, 0.3840946885888625};



//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!


void InitBufInArray()
{
    unsigned short i;
    float fx;
    for(i=0; i<NPT; i++)
    {
        fx = 1.50 * sin(PI2 * i * 350.0 / Fs) +
             2.70 * sin(PI2 * i * 8400.0 / Fs) +
             4.00 * sin(PI2 * i * 18725.0 / Fs);
        lBufInArray[i] = ((signed short)(fx*1000)) << 16;
    }
}


void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
							 
}

void TIM5_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM5, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM5, ENABLE);  //使能TIMx外设
							 
}

void LCD_display(u16 x,u16 y,float num)
{
	  u16 temp;
		temp=num;
		LCD_ShowxNum(x,y,temp,1,16,0);//显示电压值
		num-=temp;
		num*=1000;
		LCD_ShowxNum(x+16,y,num,3,16,0X80);
}

void TIM3_IRQHandler(void)   //TIM3中断
{
	float temp;
	static int i = 0;
	int k = 0 , j = 0, m = 0;
	u16 adczu[NPT];
	float fft_num=0;
	float temp_max = 0.0;
	float fft_max=0.0;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
		LED1=!LED1;
		adczu[i]=Get_Adc(ADC_Channel_1);
		//LCD_display(156,100,temp);

		if (i == 1023)
			{
				i = -1;
				TIM_Cmd(TIM3, DISABLE);
				
				for(k=1;k<NPT;k++)
				{
					temp=(((float)adczu[k]*(3.3/4096)-1.5)*10/3);
					lBufInArray[k] = ((signed short)(temp*1000)) << 16;
				}

				//InitBufInArray();
				cr4_fft_1024_stm32(lBufOutArray,lBufInArray,NPT);    
				GetPowerMag();
				
				
				fft_max=lBufMagArray[1];
				
				POINT_COLOR=RED;
				
				
				for(j=1;j<NPT/2;j++)
				{
					temp_max=lBufInArray[j];
					if(lBufMagArray[j]>=fft_max)
					{
						
						fft_max=lBufMagArray[j];
						fft_num=j*39.0625;
						LCD_Fill(20,16,280,208,WHITE);
						LCD_ShowxNum(156,100,fft_max,5,16,0);
						LCD_ShowxNum(156,130,temp_max,5,16,0);
						LCD_ShowxNum(156,190,fft_num,5,16,0);
						fft_num=j;
					}
				}
				
				for(m=1;m<NPT/4;m++)
				{
					POINT_COLOR=RED;
					LCD_DrawLine(m - 1 + 20, (int)(208-0.038*MAX(lBufMagArray[2*m-1],lBufMagArray[2*m])), m -1 + 20, 208);
				}	
			}
		i += 1;
		}
}





void TIM5_IRQHandler(void)   //TIM3中断
{
	float temp;
	static int i = 0;
	int temp2;
	u16 adczu[NPT];
	float filter=0;
	int filter_num=0;
	float temp_max = 0.0;
	float fft_num = 0;
	float fft_max2 = 0.0;
	float fft_max=0.0;
	uint16_t ReadValue; 
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
		ReadValue1 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9);
		//printf("%d\t",ReadValue1);
		ReadValue2 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);
		//printf("%d\t",ReadValue2);
		ReadValue3 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
		//printf("%d\t",ReadValue3);
		ReadValue4 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
		//printf("%d\t",ReadValue4);
		ReadValue5 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13);
		//printf("%d\t",ReadValue5);
		ReadValue = ReadValue1*0 + ReadValue2*2 + ReadValue3*4 + ReadValue4*8 + ReadValue5*16;
		LED1=!LED1;
		//temp2 = Get_Adc(ADC_Channel_1);
		//Dac1_Set_Vol(temp2);
			
		//printf("%d\r\n",ReadValue);
			
		if (ReadValue == 0)
			{
				temppp = Get_Adc(ADC_Channel_1);
				Dac1_Set_Vol(temppp);
			}
			else if(ReadValue_last == ReadValue)
			{
				Dac1_Set_Vol(testOutput[0]);
	
				testInput_f32[0] = Get_Adc(ADC_Channel_1);
				
				//Dac1_Set_Vol(testInput_f32[0]);
				
				//printf("%f\r\n",testInput_f32[0]);
					
				/* IIR滤波 */
				arm_biquad_cascade_df1_f32(&S, testInput_f32, testOutput, TEST_LENGTH_SAMPLES);
				
				//Dac1_Set_Vol(testOutput[0]);
					
				//printf("%f\r\n",testOutput[0]);
				testOutput[0] *= ScaleValue[(ReadValue-1)];
				//IIRStateF32[2] = testOutput[0];
				//printf("11111");
				//printf("%f\t%f\r\n",testInput_f32[0],testOutput[0]);
			}
			else
			{
				arm_biquad_cascade_df1_init_f32(&S, numStages, (float *)&IIRCoeffs32LP[(ReadValue-1)*10], (float *)&IIRStateF32[0]);
				
				Dac1_Set_Vol(testOutput[0]);
	
				testInput_f32[0] = Get_Adc(ADC_Channel_1);
					
				/* IIR滤波 */
				arm_biquad_cascade_df1_f32(&S, testInput_f32, testOutput, TEST_LENGTH_SAMPLES);
				
				testOutput[0] *= ScaleValue[(ReadValue-1)];

				ReadValue_last = ReadValue;
			}
		}
}




#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_adc.h>
#include <stm32f0xx_usart.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_iwdg.h>
#include "delay.h"
#include "stm32f0xx.h"
#include "math.h"


//---Bit_Variables---------------------------------------
#define cmp_bit             0
#define mute_bit            1
#define defrost_bit         2
#define lock_bit            3
#define temp_bit            4
#define vacation_bit        5
#define protect1_bit        6
#define protect2_bit        7

#define door_status_bit     0
#define test_bit            1
#define led_fac_test_bit    2
#define led_test_bit        3
#define led_test_flag       4
#define led_dance_bit       5
#define fac_test_bit        6
#define error_bit           7

#define cmp_status_bit      0
#define show_text_bit       1
#define show_term_eva_bit   2
#define show_set_bit        3
#define dance_flag          4
#define equal_flag1			5
#define equal_flag2			6
#define show_end_flag       7

#define lock_blink_flag     0
#define lock_blink_flag2    1
#define term_send_flag      2

unsigned char BitChar[4]={0,0,0,0};

//****************************************************************

#define Cmp_bit	               (BitChar[0] >> cmp_bit)%2
#define Cmp_bit_set	           BitChar[0] |= 1<< cmp_bit;
#define Cmp_bit_reset	       BitChar[0] &= ~(1<< cmp_bit);

#define Mute_bit	           (BitChar[0] >> mute_bit)%2
#define Mute_bit_set	       BitChar[0] |= 1<< mute_bit;
#define Mute_bit_reset	       BitChar[0] &= ~(1<< mute_bit);

#define Defrost_bit	           (BitChar[0] >> defrost_bit)%2
#define Defrost_bit_set	       BitChar[0] |= 1<< defrost_bit;
#define Defrost_bit_reset	   BitChar[0] &= ~(1<< defrost_bit);

#define Lock_bit	           (BitChar[0] >> lock_bit)%2
#define Lock_bit_set	       BitChar[0] |= 1<< lock_bit;
#define Lock_bit_reset	       BitChar[0] &= ~(1<< lock_bit);

#define Temp_bit	           (BitChar[0] >> temp_bit)%2
#define Temp_bit_set	       BitChar[0] |= 1<< temp_bit;
#define Temp_bit_reset	       BitChar[0] &= ~(1<< temp_bit);

#define Vacation_bit	       (BitChar[0] >> vacation_bit)%2
#define Vacation_bit_set	   BitChar[0] |= 1<< vacation_bit;
#define Vacation_bit_reset	   BitChar[0] &= ~(1<< vacation_bit);

#define Protect1_bit	       (BitChar[0] >> protect1_bit)%2
#define Protect1_bit_set	   BitChar[0] |= 1<< protect1_bit;
#define Protect1_bit_reset	   BitChar[0] &= ~(1<< protect1_bit);

#define Protect2_bit	       (BitChar[0] >> protect2_bit)%2
#define Protect2_bit_set	   BitChar[0] |= 1<< protect2_bit;
#define Protect2_bit_reset	   BitChar[0] &= ~(1<< protect2_bit);
//****************************************************************

#define Door_status_bit	       (BitChar[1] >> door_status_bit)%2
#define Door_status_bit_set    BitChar[1] |= 1<< door_status_bit;
#define Door_status_bit_reset  BitChar[1] &= ~(1<< door_status_bit);

#define Test_bit	           (BitChar[1] >> test_bit)%2
#define Test_bit_set           BitChar[1] |= 1<< test_bit;
#define Test_bit_reset         BitChar[1] &= ~(1<< test_bit);

#define Led_fac_test_bit	   (BitChar[1] >> led_fac_test_bit)%2
#define Led_fac_test_bit_set   BitChar[1] |= 1<< led_fac_test_bit;
#define Led_fac_test_bit_reset BitChar[1] &= ~(1<< led_fac_test_bit);

#define Led_test_bit	       (BitChar[1] >> led_test_bit)%2
#define Led_test_bit_set       BitChar[1] |= 1<< led_test_bit;
#define Led_test_bit_reset     BitChar[1] &= ~(1<< led_test_bit);

#define Led_test_flag	       (BitChar[1] >> led_test_flag)%2
#define Led_test_flag_set      BitChar[1] |= 1<< led_test_flag;
#define Led_test_flag_reset    BitChar[1] &= ~(1<< led_test_flag);

#define Led_dance_bit	       (BitChar[1] >> led_dance_bit)%2
#define Led_dance_bit_set      BitChar[1] |= 1<< led_dance_bit;
#define Led_dance_bit_reset    BitChar[1] &= ~(1<< led_dance_bit);

#define Fac_test_bit	       (BitChar[1] >> fac_test_bit)%2
#define Fac_test_bit_set       BitChar[1] |= 1<< fac_test_bit;
#define Fac_test_bit_reset     BitChar[1] &= ~(1<< fac_test_bit);

#define Error_bit	           (BitChar[1] >> error_bit)%2
#define Error_bit_set          BitChar[1] |= 1<< error_bit;
#define Error_bit_reset        BitChar[1] &= ~(1<< error_bit);

//****************************************************************
#define Cmp_status_bit	        (BitChar[2] >> cmp_status_bit)%2
#define Cmp_status_bit_set      BitChar[2] |= 1<< cmp_status_bit;
#define Cmp_status_bit_reset    BitChar[2] &= ~(1<< cmp_status_bit);

#define Show_text_bit	        (BitChar[2] >> show_text_bit)%2
#define Show_text_bit_set       BitChar[2] |= 1<< show_text_bit;
#define Show_text_bit_reset     BitChar[2] &= ~(1<< show_text_bit);

#define Show_term_eva_bit	    (BitChar[2] >> show_term_eva_bit)%2
#define Show_term_eva_bit_set   BitChar[2] |= 1<< show_term_eva_bit;
#define Show_term_eva_bit_reset BitChar[2] &= ~(1<< show_term_eva_bit);

#define Show_set_bit	        (BitChar[2] >> show_set_bit)%2
#define Show_set_bit_set        BitChar[2] |= 1<< show_set_bit;
#define Show_set_bit_reset      BitChar[2] &= ~(1<< show_set_bit);

#define Dance_flag	            (BitChar[2] >> dance_flag)%2
#define Dance_flag_set          BitChar[2] |= 1<< dance_flag;
#define Dance_flag_reset        BitChar[2] &= ~(1<< dance_flag);

#define Equal_flag1	            (BitChar[2] >> equal_flag1)%2
#define Equal_flag1_set         BitChar[2] |= 1<< equal_flag1;
#define Equal_flag1_reset       BitChar[2] &= ~(1<< equal_flag1);

#define Equal_flag2	            (BitChar[2] >> equal_flag2)%2
#define Equal_flag2_set         BitChar[2] |= 1<< equal_flag2;
#define Equal_flag2_reset       BitChar[2] &= ~(1<< equal_flag2);

#define Show_end_flag	        (BitChar[2] >> show_end_flag)%2
#define Show_end_flag_set       BitChar[2] |= 1<< show_end_flag;
#define Show_end_flag_reset     BitChar[2] &= ~(1<< show_end_flag);
//****************************************************************
#define Lock_blink_flag	        (BitChar[3] >> lock_blink_flag)%2
#define Lock_blink_flag_set     BitChar[3] |= 1<< lock_blink_flag;
#define Lock_blink_flag_reset   BitChar[3] &= ~(1<< lock_blink_flag);

#define Lock_blink_flag2	    (BitChar[3] >> lock_blink_flag2)%2
#define Lock_blink_flag2_set    BitChar[3] |= 1<< lock_blink_flag2;
#define Lock_blink_flag2_reset  BitChar[3] &= ~(1<< lock_blink_flag2);

#define Term_send_flag	        (BitChar[3] >> term_send_flag)%2
#define Term_send_flag_set      BitChar[3] |= 1<< term_send_flag;
#define Term_send_flag_reset    BitChar[3] &= ~(1<< term_send_flag);
//****************************************************************
#define ref        0
#define freez      1
#define eva        2
#define term       3

#define key_set       1
#define key_lock      2
#define key_alarm     3
#define key_protect   4
#define key_vacation  5

#define vacation_term_degree  3

#define Seg_0_bit_set          GPIOB->ODR |= 1<<3;  
#define Seg_1_bit_set          GPIOA->ODR |= 1<<15;  
#define Seg_2_bit_set          GPIOB->ODR |= 1<<5;  
#define Seg_3_bit_set          GPIOB->ODR |= 1<<6;  
#define Disp_led_1_bit_set     GPIOB->ODR |= 1<<4; 
#define Disp_led_2_bit_set     GPIOB->ODR |= 1<<7; 

uint16_t Led_disp1,Led_disp2,Led_disp1_new,Led_disp2_new;

#define led_centig1_on         Led_disp2_new |= 0x4000; //GPIOA.14
#define led_centig1_off        Led_disp2_new &= 0xBFFF;

#define led_mines1_on          Led_disp2_new |= 0x2000; //GPIOA.13
#define led_mines1_off         Led_disp2_new &= 0xDFFF;

#define led_centig2_on         Led_disp2_new |= 0x0400; //GPIOA.10
#define led_centig2_off        Led_disp2_new &= 0xFBFF;

#define led_mines2_on          Led_disp2_new |= 0x0200; //GPIOA.09
#define led_mines2_off         Led_disp2_new &= 0xFDFF;

#define led_cmp_on             Led_disp1_new |= 0x4000; //GPIOA.14
#define led_cmp_off            Led_disp1_new &= 0xBFFF;

#define led_defrost_on         Led_disp2_new |= 0x0800; //GPIOA.10  
#define led_defrost_off        Led_disp2_new &= 0xF7FF; 

#define led_set_on             Led_disp1_new |= 0x0800; //GPIOA.11
#define led_set_off            Led_disp1_new &= 0xF7FF;

#define led_lock_on            Led_disp1_new |= 0x0400; //GPIOA.10  
#define led_lock_off           Led_disp1_new &= 0xFBFF; 

#define led_alarm_on           Led_disp1_new |= 0x0200; //GPIOA.09
#define led_alarm_off          Led_disp1_new &= 0xFDFF;

#define led_protect_green_on   Led_disp1_new |= 0x2000; //GPIOA.13
#define led_protect_green_off  Led_disp1_new &= 0xDFFF;

#define led_protect_red_on     Led_disp1_new |= 0x0100; //GPIOA.08
#define led_protect_red_off    Led_disp1_new &= 0xFEFF;

#define led_vacation_on        Led_disp1_new |= 0x1000; //GPIOA.12
#define led_vacation_off       Led_disp1_new &= 0xEFFF;


GPIO_InitTypeDef          GPIO_InitStructure;
USART_InitTypeDef         USART_InitStructure;
NVIC_InitTypeDef          NVIC_InitStructure;
TIM_TimeBaseInitTypeDef   TTB;
TIM_OCInitTypeDef         TOC;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0;
ADC_InitTypeDef           ADC_InitStructure;

unsigned char k;
unsigned char Touch_status[6];
unsigned char ADC_num, Touch_status_counter1[6], Touch_status_counter2[6], Threshold_sum_counter, Equaling_counter1, Equaling_counter2;
unsigned int  Threshold[6], Touch_threshold_1[6], Touch_threshold_2[6], Touch_ADC_data[6];
unsigned char Key=1; 
unsigned char ADC_data[6], Term_degree=3, Term_degree_last=3; 
    
unsigned char Seg_0[25]= {0x6F,0x09,0x57,0x5D,0x39,0x7C,0x7E,0x49,0x7F,0x7D,   0x7B,0x26,0x1E,0x1A,0x72,0x66,0x3B,   0x22,0x62,0x1F,0x00,0x76,0x73,0x12,0x4D};
unsigned char Seg_1[25]= {0x6F,0x22,0x7C,0x76,0x33,0x57,0x5F,0x62,0x7F,0x77,   0x7B,0x0D,0x1E,0x1A,0x59,0x4D,0x3B,   0x09,0x49,0x3E,0x00,0x5D,0x79,0x18,0x66};
unsigned char Seg_2[25]= {0x6F,0x09,0x7C,0x5D,0x1B,0x57,0x77,0x0D,0x7F,0x5F,   0x3F,0x62,0x71,0x31,0x36,0x66,0x3B,   0x22,0x26,0x79,0x00,0x76,0x3E,0x30,0x4D};
unsigned char Seg_3[25]= {0x6F,0x22,0x57,0x76,0x3A,0x7C,0x7D,0x26,0x7F,0x7E,   0x3F,0x49,0x71,0x31,0x1D,0x4D,0x3B,   0x09,0x0D,0x73,0x00,0x5D,0x1F,0x11,0x66};
	
unsigned char Seg_4[2][41]= {{0x00,0x40,0x40,0x40,0x40,   0x20,0x20,0x20,0x20,   0x01,0x01,0x01,0x01,   0x10,0x10,0x10,0x10,   0x02,0x02,0x02,0x02,   0x08,0x08,0x08,0x08,   0x04,0x04,0x04,0x04,   0x40,0x20,0x04,0x02,   0x40,0x08,0x08,0x04,0x02,   0x20,0x01,0x10},
                             {0x00,0x00,0x01,0x03,0x02,   0x00,0x01,0x03,0x02,   0x00,0x01,0x03,0x02,   0x00,0x01,0x03,0x02,   0x00,0x01,0x03,0x02,   0x00,0x01,0x03,0x02,   0x00,0x01,0x03,0x02,   0x05,0x05,0x05,0x05,   0x04,0x05,0x04,0x04,0x04,   0x04,0x04,0x04}};

char RX_data;
unsigned char  Adc_data_recieve[4], Seg_data[4], Seg_data_new[4];
int Select_data;
static unsigned char M1_p,M2_p,Count_d,Show_text_count,Led_test_count,Term_send_count;
static char i1,i2,i3,i4,i5;	
//Variables to store current states
unsigned char CChan = 1;

/* Private function -----------------------------------------------*/
//---GPIO_Config()---------------------------------------
void GPIO_Config(void)
{
    /* GPIOA, GPIOB and GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    /* GPIOA Configuration: Channel 1, 2, 3, 4 and Channel 1N as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//---Timer_Init (void)---------------------------------------
void Timer_Init (void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    //---TIM3 Configuration for Time
    /* Time Base configuration */
    TTB.TIM_ClockDivision = TIM_CKD_DIV1;
    TTB.TIM_CounterMode = TIM_CounterMode_Up;
    TTB.TIM_Period = 0x0010;
    TTB.TIM_Prescaler = 4000;
    TTB.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TTB);
    TIM_ITConfig(TIM3,0x0001,ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(TIM3_IRQn);
    
    //---TIM16 Configuration for Time
    /* Time Base configuration */
    TTB.TIM_ClockDivision = TIM_CKD_DIV1;
    TTB.TIM_CounterMode = TIM_CounterMode_Up;
    TTB.TIM_Period = 30;
    TTB.TIM_Prescaler = 64999;
    TTB.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM16, &TTB);
    TIM_ITConfig(TIM16,0x0001,ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(TIM16_IRQn);
    //---TIM1 Configuration for Touch PWM Generation 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
    TimerPeriod = (SystemCoreClock / (50*220000)) - 1;
    /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
    Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
    
    TTB.TIM_Prescaler = 0x031;
    TTB.TIM_CounterMode = TIM_CounterMode_Up;
    TTB.TIM_Period = TimerPeriod;
    TTB.TIM_ClockDivision = 0;
    TTB.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TTB);
    TIM_ITConfig(TIM1,0x0001,ENABLE);
    
    //NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
    
    //NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    
    TOC.TIM_OCMode = TIM_OCMode_PWM2;
    TOC.TIM_OutputState = TIM_OutputState_Disable;
    TOC.TIM_OutputNState = TIM_OutputNState_Enable;
    TOC.TIM_Pulse = Channel1Pulse;
    TOC.TIM_OCPolarity = TIM_OCPolarity_Low;
    TOC.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TOC.TIM_OCIdleState = TIM_OCIdleState_Set;
    TOC.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC3Init(TIM1, &TOC);
    
    /* TIM3 counter enable */
    TIM_Cmd(TIM3, ENABLE);
    /* TIM16 counter enable */
    TIM_Cmd(TIM16, ENABLE);
    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    
    //Touch PWM
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
}

//---Usart_Init (void)---------------------------------------
void Usart_Init (void)
{
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    
    USART_InitStructure.USART_BaudRate = 300;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    USART_Cmd(USART1,ENABLE);
    //Usart1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
}

//---Usart_Send(unsigned char data)--------------------------
void Usart_Send(unsigned char data)
{
    USART_SendData(USART1,data);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
}

//---ADC1_Init (void)-----------------------------------------
void ADC1_Init (void)
{
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //---Configure ADC in 8 bit mode with upward scanning for Sensors
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
    ADC_Init(ADC1, &ADC_InitStructure);

    //Enable end of conversion interrupt
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    //Enable ADC1 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
   
//    NVIC_EnableIRQ(ADC1_IRQn);
    //Configure the channels to be converted, in this case C0, C1 and
    //C2, corresponding to PA0, PA1 and PA2 respectively
    ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_6, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_7, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_8, ADC_SampleTime_239_5Cycles);
    
    ADC_Cmd(ADC1, ENABLE);
    //Wait for the ADC to be ready!
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
    //Start conversion
    ADC_StartOfConversion(ADC1);
}

//---IWD_Init(void)------------------------------------------
void IWD_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_32);
    /* Set counter reload value to obtain 250ms IWDG TimeOut.
     Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (LsiFreq/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
   */
    IWDG_SetReload(8000000/128);

    /* Reload IWDG counter */
    IWDG_ReloadCounter();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();
}

//---Adc_To_Temp (unsigned char in)---------------------------
int  Adc_To_Temp (unsigned char in)
{
	int Temp;
	double R_therm,V_meashurd;
	
	V_meashurd=(in/51.0);
	R_therm=(18000*V_meashurd)/(5-V_meashurd);
	
	Temp=(double)(3380/log(R_therm/(5000*exp(-3380/298.15))))-273.15;
	
	return Temp;
}

//---Send_Some_Data(void)-------------------------------------
void Send_Some_Data(void)
{
    Usart_Send('q');
    
    
    if(Term_send_flag)
    {
        Term_send_count++;
        if(Term_send_count>250)
        {
            Term_send_count=0;
            Term_send_flag_reset
            Usart_Send(Term_degree);
        }
    }
}

//---Show_On_7seg(void)---------------------------------------
void Show_On_7seg(void)
{
	static int Temp1, Temp2;
    static unsigned char j, k, n, m;
	
    n++;
	if(n>=100) 
	{
		n=0;
		if(Test_bit) 
        {
            Test_bit_reset
			Error_bit_reset
        }
		else 
			Error_bit_set 
	}
	if(Led_fac_test_bit==0)	
    {
        if(Protect1_bit==0)
        {
            if(Error_bit==0)
            {
                if(Vacation_bit==0)
                {
                    led_centig1_on
                    led_centig2_on
                    
                    if(Cmp_status_bit)
                    {
                        led_defrost_off
                        led_cmp_on
                    }
                    else
                    {
                        led_cmp_off
                    } 
                    
                    if(Defrost_bit)
                    {   
                        led_cmp_off
                        led_defrost_on
                    }
                    else
                    {   
                        led_defrost_off
                    }
                    
                    if(Show_set_bit)
                        led_set_on
                    else
                        led_set_off
                    
                    if(Lock_bit)
                    {   
                        led_lock_on
                        if(Lock_blink_flag)
                        {
                            m++;
                            if(m<6 || (m>10 && m<16))
                                led_lock_off
                            else if((m>=6 && m<=10) || m>=16)
                            {
                                led_lock_on
                                if(m>=20)
                                {
                                    m=0;
                                    Lock_blink_flag_reset
                                }
                            } 
                        }
                    }
                    
                    else
                        led_lock_off
                    
                    if(Door_status_bit==0)
                    {
                        if(Mute_bit)
                            led_alarm_off
                        else
                            led_alarm_on
                    }
                    
                    
                    if(Protect1_bit==0 && Protect2_bit==0)
                    {    
                        led_protect_green_on
                        led_protect_red_off
                    }
                    
                    //if(Vacation_bit)
                    //    led_vacation_on
                    //else
                        led_vacation_on
                    
                    if(Show_text_bit)
                    {
                        Show_text_count++;
                        
                        if(Show_term_eva_bit == 0 || (Adc_data_recieve[term]>245)||(Adc_data_recieve[term]<10))
                            led_centig1_off
                           
                        if(Show_term_eva_bit == 0 || (Adc_data_recieve[term]>245)||(Adc_data_recieve[term]<10) || (Adc_To_Temp(Adc_data_recieve[term])>=0))
                            led_mines1_off
                        
                        if(Show_term_eva_bit == 0 || (Adc_data_recieve[eva]>245)||(Adc_data_recieve[eva]<10))
                            led_centig2_off
                           
                        if(Show_term_eva_bit == 0 || (Adc_data_recieve[eva]>245)||(Adc_data_recieve[eva]<10) || (Adc_To_Temp(Adc_data_recieve[eva])>=0))
                            led_mines2_off
                        if(Show_set_bit)
                        {
                            if(Show_text_count==1)
                                j=0;
                            j++;
                            if(j<10) 
                            {
                                Seg_data_new[1]=Term_degree;
                            }
                            else
                            {
                                Seg_data_new[1]=20;
                                if(j>17) {j=0;}
                            }
                            
                        }
                        else if(Show_term_eva_bit)
                        {
                            
                            if((Adc_data_recieve[term]>245)||(Adc_data_recieve[term]<10))            //up display 
                            {
                                k=j;
                                j++;
                                if(j<10) 
                                {
                                    Seg_data_new[0]=21;
                                    Seg_data_new[1]=3;
                                }
                                else
                                {
                                    Seg_data_new[0]=20;
                                    Seg_data_new[1]=20;
                                    if(j>17) {j=0;}
                                }
                            }
                            else
                            {
                                Temp1= Adc_To_Temp(Adc_data_recieve[term]);
                                led_centig1_on
                                if(Temp1<0)
                                {
                                    Temp1=-Temp1;
                                    led_mines1_on
                                }

                                if(Temp1/10==0)
                                    Seg_data_new[0]=20;
                                else 
                                    Seg_data_new[0]=Temp1/10;
                                Seg_data_new[1]=Temp1%10;
                            }
                            
                            if((Adc_data_recieve[eva]>245)||(Adc_data_recieve[eva]<10))            //up display 
                            {
                                k++;
                                if(k<10) 
                                {
                                    Seg_data_new[2]=21;
                                    Seg_data_new[3]=4;
                                }
                                else
                                {
                                    Seg_data_new[2]=20;
                                    Seg_data_new[3]=20;
                                    if(k>17) {k=0;}
                                }
                            }
                            else
                            {
                                Temp2= Adc_To_Temp(Adc_data_recieve[eva]);
                                led_centig2_on
                                if(Temp2<0)
                                {
                                    Temp2=-Temp2;
                                    led_mines2_on
                                }

                                if(Temp2/10==0)
                                    Seg_data_new[2]=20;
                                else 
                                    Seg_data_new[2]=Temp2/10;
                                Seg_data_new[3]=Temp2%10;
                            }
                        }
                        if(Show_set_bit)
                        {
                            if(Show_text_count>=240)
                            {
                                Show_text_bit_reset
                                Show_text_count=0;
                                Show_set_bit_reset
                            }
                        }
                        else
                        {
                            if(Show_text_count>=60)
                            {
                                Show_text_bit_reset
                                Show_text_count=0;
                                if(Show_term_eva_bit)
                                    Show_term_eva_bit_reset
                            }
                        }    
                    } 
                    else
                    {    
                        if(j<18 && k<18)
                            k=j;
                        
                        if((Adc_data_recieve[ref]>245)||(Adc_data_recieve[ref]<10))                 //down display
                        {
                            j++;
                            led_mines1_off
                            led_centig1_off
                            if(j<10)
                            {
                                Seg_data_new[0]=21;
                                Seg_data_new[1]=1;
                            }
                            else if(j<=18)
                            {
                                Seg_data_new[0]=20;
                                Seg_data_new[1]=20;
                                if(j>17)
                                    if((Adc_data_recieve[term]<=245) && (Adc_data_recieve[term]>=10))
                                        j=0;
                            }
                            else if((j<28) && ((Adc_data_recieve[term]>245)||(Adc_data_recieve[term]<10)))
                            {
                                Seg_data_new[0]=21;
                                Seg_data_new[1]=3;
                            }
                            else if((j<=36) && ((Adc_data_recieve[term]>245)||(Adc_data_recieve[term]<10)))
                            {
                                Seg_data_new[0]=20;
                                Seg_data_new[1]=20;
                                if(j>35) {j=0;}
                            }
                            else if(j>=18)
                                if((Adc_data_recieve[term]<=245)&&(Adc_data_recieve[term]>=10))
                                    {j=0;}
                        }
                        else 
                        {
                            j++;
                            if(j<=18)
                            {
                                Temp2= Adc_To_Temp(Adc_data_recieve[ref]);
                                led_centig1_on
                                
                                //if(Temp2>9) Temp2=9;
                                
                                if(Temp2<0)
                                {
                                    Temp2=-Temp2;
                                    led_mines1_on
                                }
                                else 
                                    led_mines1_off
                                    
                                if(Temp2/10==0)
                                    Seg_data_new[0]=20;
                                else 
                                    Seg_data_new[0]=Temp2/10;
                                Seg_data_new[1]=Temp2%10;
                                if(j>=18)
                                    if((Adc_data_recieve[term]<=245)&&(Adc_data_recieve[term]>=10))
                                        {j=0;}
                            }
                            else if((j<28) && ((Adc_data_recieve[term]>245)||(Adc_data_recieve[term]<10)))
                            {
                                led_mines1_off
                                led_centig1_off
                                
                                Seg_data_new[0]=21;
                                Seg_data_new[1]=3;
                            }
                            else if((j<=36) && ((Adc_data_recieve[term]>245)||(Adc_data_recieve[term]<10)))
                            {
                                led_mines1_off
                                led_centig1_off
                                Seg_data_new[0]=20;
                                Seg_data_new[1]=20;
                                if(j>35) 
                                    {j=0;}
                            }
                            else if(j>=18)
                                if((Adc_data_recieve[term]<=245)&&(Adc_data_recieve[term]>=10))
                                    {j=0;}
                        }
                        
                        if((Adc_data_recieve[freez]>245)||(Adc_data_recieve[freez]<10))                 //down display
                        {
                            k++;
                            led_mines2_off
                            led_centig2_off
                            if(k<10)
                            {
                                Seg_data_new[2]=21;
                                Seg_data_new[3]=2;
                            }
                            else if(k<=18)
                            {
                                Seg_data_new[2]=20;
                                Seg_data_new[3]=20;
                                if(k>17)
                                    if((Adc_data_recieve[eva]<=245) && (Adc_data_recieve[eva]>=10))
                                        k=0;
                            }
                            else if((k<28) && ((Adc_data_recieve[eva]>245)||(Adc_data_recieve[eva]<10)))
                            {
                                Seg_data_new[2]=21;
                                Seg_data_new[3]=4;
                            }
                            else if((k<=36) && ((Adc_data_recieve[eva]>245)||(Adc_data_recieve[eva]<10)))
                            {
                                Seg_data_new[2]=20;
                                Seg_data_new[3]=20;
                                if(k>35) {k=0;}
                            }
                            else if(k>=18)
                                if((Adc_data_recieve[eva]<=245)&&(Adc_data_recieve[eva]>=10))
                                    {k=0;}
                        }
                        else 
                        {
                            k++;
                            if(k<=18)
                            {
                                Temp2= Adc_To_Temp(Adc_data_recieve[freez]);
                                led_centig2_on
                                
                                //if(Temp2>(-16)) Temp2= -16;
                                
                                if(Temp2<0)
                                {
                                    Temp2=-Temp2;
                                    led_mines2_on
                                }
                                else 
                                    led_mines2_off
                                    
                                if(Temp2/10==0)
                                    Seg_data_new[2]=20;
                                else 
                                    Seg_data_new[2]=Temp2/10;
                                Seg_data_new[3]=Temp2%10;
                                if(k>=18)
                                    if((Adc_data_recieve[eva]<=245)&&(Adc_data_recieve[eva]>=10))
                                        {k=0;}
                            }
                            else if((k<28) && ((Adc_data_recieve[eva]>245)||(Adc_data_recieve[eva]<10)))
                            {
                                led_mines2_off
                                led_centig2_off
                                
                                Seg_data_new[2]=21;
                                Seg_data_new[3]=4;
                            }
                            else if((k<=36) && ((Adc_data_recieve[eva]>245)||(Adc_data_recieve[eva]<10)))
                            {
                                led_mines2_off
                                led_centig2_off
                                Seg_data_new[2]=20;
                                Seg_data_new[3]=20;
                                if(k>35) {k=0;}
                            }
                            else if(k>=18)
                                if((Adc_data_recieve[eva]<=245)&&(Adc_data_recieve[eva]>=10))
                                    {k=0;}
                        }
                        
                        if(Door_status_bit)
                        {
                            Count_d++;
                            if(Count_d<10)
                                led_alarm_on
                            else
                            {
                                led_alarm_off
                                if(Count_d>=18) Count_d=0;
                            }
                        }
                        
                        if(Protect2_bit)
                        {
                            M2_p++;
                            led_protect_green_off
                            if(M2_p<19)
                                led_protect_red_on
                            else
                            {
                                led_protect_red_off
                                if(M2_p>=54) M2_p=0;
                            }
                        }
                        
                    }
                }
                else
                {
                    Seg_data_new[0]=20;
					Seg_data_new[1]=20;
					Seg_data_new[2]=20;
					Seg_data_new[3]=20;
					led_centig1_off
					led_centig2_off
					led_mines1_off
					led_mines2_off
					led_cmp_off
                    led_defrost_off
                    led_set_off
                    led_lock_off
					led_alarm_off
					led_protect_green_off
                    led_protect_red_off
                    led_vacation_on
                }
            }
            else
            {
                Seg_data_new[0]=21;
                Seg_data_new[1]=23;
                Seg_data_new[2]=21;
                Seg_data_new[3]=23;
                led_centig1_off
                led_centig2_off
                led_mines1_off
                led_mines2_off
            }
        }
        else
        {
            M1_p++;
            Seg_data_new[0]=20;
            Seg_data_new[1]=20;
            Seg_data_new[2]=20;
            Seg_data_new[3]=20;
            led_centig1_off
            led_centig2_off
            led_mines1_off
            led_mines2_off
            led_cmp_off
            led_defrost_off
            led_set_off
            led_lock_off
            led_alarm_off
            led_protect_green_off
            led_vacation_off
            if(M1_p<6 || (M1_p>=11 && M1_p<16)) 
                led_protect_red_on 
            else
            {
                led_protect_red_off
                if(M1_p>=30) M1_p=0;
            }
        } 
    }
    else
    {
        Seg_data_new[0]=8;
        Seg_data_new[1]=8;
        Seg_data_new[2]=8;
        Seg_data_new[3]=8;
        led_centig1_on
        led_centig2_on
        led_mines1_on
        led_mines2_on
        led_cmp_on
        led_defrost_on
        led_set_on
        led_lock_on
        led_alarm_on
        led_protect_green_on
        led_protect_red_on
        led_vacation_on
        
        Led_test_count++;
        if(Led_test_count==240)
        {
            Led_test_count=0;
            Led_fac_test_bit_reset
        }  
    }
    Show_end_flag_set
}

//---Defrost_Operand(int status)----------------------------
void Defrost_Operand()
{          
    if(Defrost_bit==0)
    {
        Show_text_bit_set
        Show_term_eva_bit_reset
        Show_set_bit_reset
        Show_text_count=0;
        Seg_data_new[0]=19;
        Seg_data_new[1]=14;
        Seg_data_new[2]=12;
        //Defrost_bit_set
        //Usart_Send('G');
        Seg_data_new[3]=13;
        Usart_Send('H');
    }
    /*else
    {
        Show_text_bit_set
        Show_term_eva_bit_reset
        Show_set_bit_reset
        Show_text_count=0;
        Seg_data_new[0]=19;
        Seg_data_new[1]=14;
        Seg_data_new[2]=12;
        Defrost_bit_reset
        Usart_Send('g');
        Seg_data_new[3]=14;
        Usart_Send('h');
    }*/
}

//---Set_Operand(unsigned char status)----------------------
void Set_Operand(unsigned char status)
{
	if(status) 
	{
        while(Touch_ADC_data[1] > ADC_data[1])
        {
            delay_ms(50);
            i1++;
            if(Show_set_bit)
            {
                i1=0;
                Touch_ADC_data[1]=0;
                Usart_Send('G');
                Show_text_bit_set
                Show_term_eva_bit_reset
                Show_text_count=0;
                Term_send_count=0;
                Term_send_flag_set
                Term_degree++;
                //Usart_Send(Term_degree);
                if(Term_degree >= 7)
                    Term_degree=1; 
            }
            else if((i1>20) && (Touch_ADC_data[1] > ADC_data[1]))
            {
                i1=0;
                Touch_ADC_data[1]=0;
                Usart_Send('G');
                Show_text_bit_set
                Show_term_eva_bit_reset
                Show_set_bit_set
                Show_text_count=0;
                Seg_data_new[0]=20;
				Seg_data_new[2]=20;
				Seg_data_new[3]=20;
                
                Seg_data_new[1]=Term_degree;
            }
        }
        i1=0;
	}
    
}

//---Lock_Operand(int status)--------------------------------
void Lock_Operand(unsigned char status)
{
	if(status) 
	{
		/*delay_ms(300);
		if(Touch_status[5] == 1 && Vacation_bit==1 && Lock_bit==0)
		{
			while(Touch_ADC_data[2] > ADC_data[2])
			{
				delay_ms(50);
				i2++;
				if(i2>44 && Touch_status[5]==1 && Vacation_bit==1 && Lock_bit==0)
				{
					i2=0;
					Touch_ADC_data[2]=0;
                    Usart_Send('G');
                    Led_test_count=0;
                    Led_fac_test_bit_set
                }
            }
            i2=0;
		}
		else
		{*/
			while(Touch_ADC_data[2] > ADC_data[2])
			{
				delay_ms(50);
				i2++;
				if((i2>20) && (Touch_ADC_data[2] > ADC_data[2]))
				{	
					//Touch_ADC_data[2]=0;
					i2=0;
                    //Equal_flag2_reset
                    Show_text_bit_set
                    Show_term_eva_bit_reset
                    Show_set_bit_reset
                    Show_text_count=0;
                    Seg_data_new[0]=15;
                    Seg_data_new[1]=16;
                    Seg_data_new[2]=12;
                    
					if(Lock_bit)
					{
                        //Usart_Send('j');
						Lock_bit_reset
						Seg_data_new[3]=14;
						Usart_Send('l');
					}
					else
					{
						//Usart_Send('J');
						Lock_bit_set
						Seg_data_new[3]=13;
						Usart_Send('L');
					}
					
					while(Touch_ADC_data[2] > ADC_data[2])
					{
						delay_ms(50);
						//Show_On_7seg();
						i2++;
						if(i2>180 && Touch_status[2] == 1)
						{
							i2=0;
							Touch_ADC_data[2]=0;
							Usart_Send('V');
							Show_text_bit_set
							Show_term_eva_bit_reset
							Show_set_bit_reset
							Show_text_count=0;
							Seg_data_new[0]=0;
							Seg_data_new[1]=0;
							Seg_data_new[2]=0;
							Seg_data_new[3]=0;
						}
					}
				}
			}
			i2=0;
		//}
	}
}

//---Alarm_Operand(unsigned char status)--------------------
void Alarm_Operand(unsigned char status)
{
	if(status)
	{
		if(Touch_ADC_data[3] > ADC_data[3])
		{
			delay_ms(200);
			if(Touch_status[5] == 1 && Vacation_bit==0 && Lock_bit==0)
			{
				while(Touch_ADC_data[3] > ADC_data[3])
				{
					delay_ms(50);
					i3++;
                    if(i3>46 && Touch_status[5]==1 && Vacation_bit==0 && Lock_bit==0)
                    {
                        i3=0;
						Touch_ADC_data[3]=0;
                        Usart_Send('G');
                        Defrost_Operand();
                    }
                }
                i3=0;
			}
			else
			{
				Touch_ADC_data[3]=0;
                if(Door_status_bit==0 && Lock_bit==0)
				{
					//Equal_flag1_reset
                    Show_text_bit_set
                    Show_term_eva_bit_reset
                    Show_set_bit_reset
                    Show_text_count=0;
                    Seg_data_new[0]=10;
                    Seg_data_new[1]=11;
                    Seg_data_new[2]=12;
                    
                    if(Mute_bit)
					{
                        Mute_bit_reset
                        //Usart_Send('B');
                        Seg_data_new[3]=13;
                        Usart_Send('m');
					}
					else
					{
                        Mute_bit_set
						//Usart_Send('b');
						Seg_data_new[3]=14;
						Usart_Send('M');
					}
				}
				else
				{
					Usart_Send('A');
				}
			}
		}
	}

}

//---Eva_Temp_Operand(int status)----------------------------
void Term_Eva_Temp_Operand(unsigned char status)
{
	if(status) 
	{
        while(Touch_ADC_data[4] > ADC_data[4])
        {
            delay_ms(50);
            i4++;
            if((i4==20) && (Touch_ADC_data[4] > ADC_data[4]))
            {
                
                Usart_Send('G');
                Show_text_bit_set
                Show_term_eva_bit_set
                Show_set_bit_reset
                Show_text_count=0;
            } 
            
            if((i4>100) && (Touch_ADC_data[4] > ADC_data[4]))
            {
                i4=0;
                Touch_ADC_data[4]=0;
                Usart_Send('G') ;
                Led_test_count=0;
                Led_fac_test_bit_set
            }
        }
        i4=0;
	}
}

//---Light_Operand(int status)---------------------------------
void Vacation_Operand(unsigned char status)
{
	if(status)
	{
		if(Touch_ADC_data[5] > ADC_data[5])
		{
            
            delay_ms(200);
            if((Touch_status[3] == 1) && Vacation_bit==0 && Lock_bit==0)
            {
                while(Touch_ADC_data[5] > ADC_data[5])
				{
                    delay_ms(50);
                    i5++;
                    if(i5>46 &&(/*Touch_status[2]==1 || */Touch_status[3]==1) && Vacation_bit==0 && Lock_bit==0)
                    {
                        i5=0;
						Touch_ADC_data[5]=0;
                        /*if(Touch_status[2]==1)
                        {
                            Usart_Send('G') ;
                            Led_test_count=0;
                            Led_fac_test_bit_set
                        }
                        else*/ if(Touch_status[3]==1)
                        {
                            Usart_Send('G') ;
                            Defrost_Operand();
                        }
                    }
                }
                i5=0;
			}
			else
			{
				Touch_ADC_data[5]=0;
				if(Protect1_bit==0)
				{
					if(Vacation_bit)
					{
						Vacation_bit_reset
						Usart_Send('p');
						Term_degree=Term_degree_last;
						Term_send_count=0;
						Term_send_flag_set
						Led_dance_bit_set
					}
					else
					{
						Vacation_bit_set
						Usart_Send('P');
						Term_degree_last=Term_degree;
						Term_degree=vacation_term_degree;
						Term_send_count=0;
						Term_send_flag_set
					}
					Dance_flag_set
				}
// 				while(Touch_ADC_data[5] > ADC_data[5])
//                 {
//                     delay_ms(50);
//                     i5++;
//                     if((i5>20) && (Touch_ADC_data[5] > ADC_data[5]))
//                     {	
//                         Touch_ADC_data[5]=0;
//                         i5=0;
//                         if(Protect1_bit==0)
//                         {
//                             if(Vacation_bit)
//                             {
//                                 Vacation_bit_reset
//                                 Usart_Send('p');
//                                 Term_degree=Term_degree_last;
//                                 Term_send_count=0;
//                                 Term_send_flag_set
//                                 Led_dance_bit_set
//                             }
//                             else
//                             {
//                                 Vacation_bit_set
//                                 Usart_Send('P');
//                                 Term_degree_last=Term_degree;
//                                 Term_degree=vacation_term_degree;
//                                 Term_send_count=0;
//                                 Term_send_flag_set
//                             }
//                             Dance_flag_set
//                         }
//                     }
//                 }
			}
		}
	}
}

/* Interrupt Service Routins -----------------------------------------------*/
void ADC1_IRQHandler(void){
	//Check for end of conversion
	unsigned char j;
	unsigned long long x;
	static unsigned long int Count=0, t=0;
	static unsigned int k=0;
	
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC))
	{
		//Clear interrupt bit
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		//Switch statement dependent on current channel. First
		//channel is initialised as zero.
		if(CChan==1)
        {
            ADC_data[1] = ADC_GetConversionValue(ADC1);
			ADC_num = 1;
            CChan = 2;
        }
        else if(CChan==2)
        {
            ADC_data[2] = ADC_GetConversionValue(ADC1);
            ADC_num = 2;
            CChan = 3;
        }
        else if(CChan==3)
        {
            ADC_data[3] = ADC_GetConversionValue(ADC1);
            ADC_num = 3;
            CChan = 4;
        }
        else if(CChan==4)
        {
            ADC_data[4] = ADC_GetConversionValue(ADC1);
            ADC_num = 4;
            CChan = 5;
        }
        else if(CChan==5)
        {
            ADC_data[5] = ADC_GetConversionValue(ADC1);
            ADC_num = 5;
            ADC_StopOfConversion(ADC1);
            CChan = 1;
        }
       
        
		if (ADC_data[ADC_num] < Touch_threshold_1[ADC_num])
        {
            Touch_status_counter1[ADC_num]++;
            Touch_status_counter2[ADC_num] = 0;
            if((Touch_status[1]==0 && Touch_status[2]==0 && Touch_status[3]==0 && Touch_status[4]==0) || (Touch_status[1]==0 && Touch_status[2]==0 && Touch_status[4]==0 && Touch_status[5]==0))  
            {
                if(Touch_status_counter1[ADC_num]>15 )
                {
                    if((ADC_num==1 && Touch_status[1]==0) || (ADC_num==2 && Touch_status[2]==0) || (ADC_num==3 && Touch_status[3]==0) || (ADC_num==4 && Touch_status[4]==0) || (ADC_num==5 && Touch_status[5]==0)) 
                    {
                        Touch_status[ADC_num] = 1;
                        Touch_ADC_data[ADC_num] = ADC_data[ADC_num];
                        Touch_status_counter1[ADC_num] = 0;
                    } 
                }
            }
        }
        else if (ADC_data[ADC_num] > Touch_threshold_2[ADC_num])
        {
            Touch_status_counter2[ADC_num]++;
            Touch_status_counter1[ADC_num] = 0;
            if(Touch_status_counter2[ADC_num]>5 )
            {
                Touch_status_counter2[ADC_num] = 0;
                Touch_status[ADC_num] = 0;
            }
        }
        
		//hang solution
		if(Touch_status[1]==1 || Touch_status[2]==1 || Touch_status[3]==1 || Touch_status[4]==1 || Touch_status[5]==1)
		{
			t++;
			if(t>=1000000) // 50000 per secound
			{
				t=0;
				k=0;
				Count=0;
				
				for(j=1;j<6;j++)
				{
					Threshold[j] = 0;
					Touch_threshold_1[j] = 0;
					Touch_threshold_2[j] = 0;
					Touch_status[j] = 0;
				}
				Threshold_sum_counter=0;
				//Usart_Send('p');
			}
		}
		else
		{
			t=0;
		}
		
        Count++;
        if(k<=4000)
        {	
            x=5;
            k++;
        }
        else
            x=32000;	  //32000 per minute
                    
        if(Count >= x)
        {	
            
            //if(Touch_status[1]==0 && Touch_status[2]==0 && Touch_status[3]==0 && Touch_status[4]==0 && Touch_status[5]==0)
            //{        
                /*if(k<=4000)*/Count = 0;
                for(j=1;j<6;j++)
                {
                    Threshold[j] += ADC_data[j];
                }
                Threshold_sum_counter ++;
                if (Threshold_sum_counter >=100)
                {
                    //Count = 0;
                    for(j=1;j<6;j++)
                    {
                        if(k<=4000)
                        {
                                //Usart_Send('G') ;
                                Touch_threshold_1[j] = (Threshold[j]/100)-5;
                                Touch_threshold_2[j] = (Threshold[j]/100)-3;
                        }
                        else
                        {
                                //Usart_Send('G') ;
                                if(Touch_threshold_1[j] < (Threshold[j]/100)-5)
                                Touch_threshold_1[j] +=1;
                                else if(Touch_threshold_1[j] > (Threshold[j]/100)-5)
                                Touch_threshold_1[j] -=1;
                                     
                                if(Touch_threshold_2[j] < (Threshold[j]/100)-3)
                                Touch_threshold_2[j] +=1;
                                else if(Touch_threshold_2[j] > (Threshold[j]/100)-3)
                                Touch_threshold_2[j] -=1;
                        }
                            
                        Threshold[j] = 0;
                    }
                    Threshold_sum_counter = 0;
                   
                }
            //}
        }
        ADC_StartOfConversion(ADC1);
	}
}

//---TIM3_IRQHandler (void) ---------------------------------------
void TIM3_IRQHandler (void) 
{
	static unsigned char Count=0,Count_2=0,Led_count=0;
	
    if ((TIM3->SR & 0x0001) != 0)            // check interrupt source
    {                 
        GPIOA->ODR &=0x00FF;
        GPIOB->ODR &=0xFF07;
        
        //if(Vacation_bit)
            //goto End;
        if(Dance_flag)
        {
            Count_2=0;
            Led_count=0;
            Dance_flag_reset
        }
        
        if(Led_dance_bit && Led_test_bit==0)
        {
            GPIOA->ODR |= Seg_4[0][Led_count]<<8;
            Count = Seg_4[1][Led_count];
            
            switch(Count)
            {
                case 0 : 
                    Seg_0_bit_set
                    break;
                case 1 : 
                    Seg_1_bit_set
                    break;
                case 2 : 
                    Seg_2_bit_set
                    break;
                case 3 : 
                    Seg_3_bit_set
                    break;
                case 4 :
                    Disp_led_1_bit_set
                    break;
                case 5 :
                    Disp_led_2_bit_set
                    break;
                default:
                    break;
            }
            Count_2++;
            if(Count_2>30)
            {    
                Count_2=0;
                Led_count++;
            }
            if(Led_count>=42)
            {
                Led_count=0;
                Led_dance_bit_reset
            }
        }
        else if(Led_test_bit)
        {
            if(Led_test_flag)
            {
                GPIOA->ODR |= Seg_4[0][Led_count];
                Count = Seg_4[1][Led_count];
                
                switch(Count)
                {
                    case 0 : 
                        Seg_0_bit_set
                        break;
                    case 1 : 
                        Seg_1_bit_set
                        break;
                    case 2 : 
                        Seg_2_bit_set
                        break;
                    case 3 : 
                        Seg_3_bit_set
                        break;
                    case 4 :
                        Disp_led_1_bit_set
                        break;
                    case 5 :
                        Disp_led_2_bit_set
                        break;
                    default:
                        break;
                }
            
                Led_test_flag_reset
                Led_count++;
                if(Led_count==41)
                {
                    Led_count=0;
                    Led_test_bit_reset
                }
            }
        }
        else
        {
            if(Show_end_flag)
            {
                Show_end_flag_reset
                Seg_data[0] = Seg_data_new[0];
                Seg_data[1] = Seg_data_new[1];
                Seg_data[2] = Seg_data_new[2];
                Seg_data[3] = Seg_data_new[3];
                Led_disp1 = Led_disp1_new;
                Led_disp2 = Led_disp2_new;
            }
            
            if(Count == 0)
                GPIOA->ODR |= Seg_0[Seg_data[Count]]<<8;
            else if(Count == 1)
                GPIOA->ODR |= Seg_1[Seg_data[Count]]<<8;
            else if(Count == 2)
                GPIOA->ODR |= Seg_2[Seg_data[Count]]<<8;
            else if(Count == 3)
                GPIOA->ODR |= Seg_3[Seg_data[Count]]<<8;
            else if(Count==4)
                GPIOA->ODR |= Led_disp1;
            else if(Count==5)
                GPIOA->ODR |= Led_disp2;
            
            switch(Count)
            {
                case 0 : 
                    Seg_0_bit_set
                    break;
                case 1 : 
                    Seg_1_bit_set
                    break;
                case 2 : 
                    Seg_2_bit_set
                    break;
                case 3 : 
                    Seg_3_bit_set
                    break;
                case 4 :
                    Disp_led_1_bit_set
                    break;
                case 5 :
                    Disp_led_2_bit_set
                    break;
                default:
                    break;
            }
            Count++;
            if(Count==6)
                Count=0;
        }
        //End:
        TIM3->SR &= ~(1<<0);         // clear UIF flag
    }
    IWDG_ReloadCounter();
}

//---TIM16_IRQHandler (void)---------------------------------------
void TIM16_IRQHandler (void) 
{	
    if ((TIM16->SR & 0x0001) != 0)            // check interrupt source
    {   
        if(Led_dance_bit == 0)
        {
            Show_On_7seg();
            Send_Some_Data();
            
            /*if(Equal_flag1 == 0)
            {
                Equaling_counter1++;
                if(Equaling_counter1>3)
                {
                    Equaling_counter1=0;
                    Equal_flag1_set
                }
            }
            
            if(Equal_flag2 == 0)
            {
                Equaling_counter2++;
                if(Equaling_counter2>3)
                {
                    Equaling_counter2=0;
                    Equal_flag2_set
                }
            }*/
        }
        TIM16->SR &= ~(1<<0);         // clear UIF flag
    }
    IWDG_ReloadCounter();
}

//---USART1_IRQHandler(void)---------------------------------------
void USART1_IRQHandler(void)
{
	static char Select_data_bit=0;
    
	while (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // Received characters modify string
	{
		RX_data = USART_ReceiveData(USART1);
	}
    
    if(Select_data_bit)
	{
		Select_data_bit=0; 
		switch(Select_data)
		{
			case ref :
						Adc_data_recieve[ref]=RX_data;
						break;
			case freez :
						Adc_data_recieve[freez]=RX_data;
						break; 
			case eva :
						Adc_data_recieve[eva]=RX_data;
						break; 
            case term :
						Adc_data_recieve[term]=RX_data;
						break; 
		}
		return;
	}
	switch (RX_data)
	{
        case 'R' :
                    Select_data_bit=1;
                    Select_data=ref;
                    break; 
        case 'F' :
                    Select_data_bit=1;
                    Select_data=freez;
                    break;
        case 'E' :
                    Select_data_bit=1;
                    Select_data=eva;
                    break;
        case 'T' :
                    Select_data_bit=1;
                    Select_data=term;
                    break;
        case 'C' :
                    Cmp_status_bit_set
                    break;
        case 'c' :
                    Cmp_status_bit_reset
                    break; 
        case 'D' :
                    Door_status_bit_set
                    break;
        case 'd' :
                    Door_status_bit_reset
                    break;                           
        case 'L' :
                    //if(Equal_flag2)
                        Lock_bit_set
                    break;
        case 'l' :
                    //if(Equal_flag2)
                        Lock_bit_reset
                    break;
        case 'M' :
                    //if(Equal_flag1)
                        Mute_bit_set
                    break;
        case 'm' :
                    //if(Equal_flag1)
                        Mute_bit_reset
                    break;
        case 'h' :
                    Defrost_bit_reset
                    break;
        case 'H' :
                    Defrost_bit_set
                    break;
        case 'Z' :
                    Protect1_bit_set
                    Protect2_bit_reset
                    M2_p=0;
                    break;
        case 'z' :
                    Protect1_bit_reset
                    Protect2_bit_set
                    M1_p=0;
                    break;
        case 't' :
                    Protect1_bit_reset
                    Protect2_bit_reset
                    break;
        case 'q' :
                    Test_bit_set
                    Error_bit_reset
                    break;
        case 'x' :
                    Led_test_bit_set
                    Led_test_flag_set
                    break;
	    default:
                    break;      
	}
}

/***********************************************************************
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	unsigned long i;
    Delay_Init(48);
    delay_ms(100);
    GPIO_Config();
    Timer_Init ();
    Usart_Init ();    
    IWD_Init();
    
    Led_dance_bit_set
	Vacation_bit_reset
    
    delay_ms(100); 
    ADC1_Init ();
    /* Infinite loop */
    while (1)
    {
		if(Protect1_bit==0)
		{	
			switch (Key)
			{
				case key_set:     
                                    Set_Operand(Touch_status[Key]);
							        break;
                
                case key_lock:   
                                    Lock_Operand(Touch_status[Key]);
							        break;
                
				case key_alarm:      
                                    Alarm_Operand(Touch_status[Key]); 
							        break;
                
				case key_protect:       
                                    Term_Eva_Temp_Operand(Touch_status[Key]);
							        break;
                
                case key_vacation:     
                                    Vacation_Operand(Touch_status[Key]);						
								    break;
                
				default:
                                    break;
			}
       
			Key++;
			if(Key>=6)
				Key=1;
		
			if(Lock_bit)
			{
				i++;
				if(i==1 /*&& Door_status_bit==1*/)
				{
					Touch_status[Key]=0;
					Key=key_alarm;
				}
				else
				{
					Key=key_lock;
					i=0;
				}
                
                if(Touch_status[1] == 1 || Touch_status[3] == 1 || Touch_status[4] == 1 || Touch_status[5] == 1)
                    if(Lock_blink_flag2)
                    {
                        Lock_blink_flag_set
                        Lock_blink_flag2_reset
                    }
                
                if(Touch_status[1] == 0 && Touch_status[2] == 0 && Touch_status[3] == 0 && Touch_status[4] == 0 && Touch_status[5] == 0)
                    Lock_blink_flag2_set
            }
			else if(Vacation_bit)
			{
                Key=key_vacation;
            }
            
            if(Door_status_bit && Vacation_bit)
            {
                Vacation_bit_reset
                Term_degree = Term_degree_last;
                Term_send_count=0;
                Term_send_flag_set
            }
		}
    }
}



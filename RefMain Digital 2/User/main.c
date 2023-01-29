#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_adc.h>
#include <stm32f0xx_usart.h>
#include <stm32f0xx_i2c.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_iwdg.h>
#include "delay.h"
#include "stm32f0xx.h"
#include "math.h"

#include <stdint.h>

//---AT24C01  &  AT24C02  EEPROMs------------------------
#define IC_Address1   0xA0
#define IC_Address2   0xA2
#define IC_Address3   0xA4
#define IC_Address4   0xA6
#define IC_Address5   0xA8
#define IC_Address6   0xAA
#define IC_Address7   0xAC
#define IC_Address8   0xAE
//---EEPROM PAGES ADDRESES-------------------------------
#define fac_test_address          0x00
#define lock_address              0x01
#define mute_address              0x02
#define term_degree_address       0x03

#define cmp_min_low_address       0x06
#define cmp_min_hi_address        0x07

//---Bit_Variables---------------------------------------
#define alarm_bit      0
#define lock_bit       1
#define second_bit     2
#define mute_bit       3
#define defrast_bit    4
#define cmp_led        5
#define test_bit       6
#define send_door_bit  7

#define protect_bit        0
#define hi_volt_bit        1
#define low_volt_bit       2
#define door_status_bit    3
#define done_startup_bit   4
#define vac_eco_bit        5

#define key_sound            0 
#define key_sound_alarm      1 
#define key_sound_def        2
#define key_sound_lock       3
#define key_sound_temp       4
#define key_sound_power      5

#define led_alarm_status       0 
#define led_def_status         1
#define led_lock_status        2
#define led_temp_status        3
#define led_power_status       4
#define protect_hi_unorm_flag  5
#define protect_low_unorm_flag 6
#define protect_second_bit     7
unsigned char BitChar[4]={0,0,0,0};

//****************************************************************

#define Alarm_bit	           (BitChar[0] >> alarm_bit)%2
#define Alarm_bit_set	       BitChar[0] |= 1<< alarm_bit;
#define Alarm_bit_reset	       BitChar[0] &= ~(1<< alarm_bit);

#define Lock_bit	           (BitChar[0] >> lock_bit)%2
#define Lock_bit_set	       BitChar[0] |= 1<< lock_bit;
#define Lock_bit_reset	       BitChar[0] &= ~(1<< lock_bit);

#define Second_bit	           (BitChar[0] >> second_bit)%2
#define Second_bit_set	       BitChar[0] |= 1<< second_bit;
#define Second_bit_reset	   BitChar[0] &= ~(1<< second_bit);

#define Mute_bit	           (BitChar[0] >> mute_bit)%2
#define Mute_bit_set	       BitChar[0] |= 1<< mute_bit;
#define Mute_bit_reset	       BitChar[0] &= ~(1<< mute_bit);

#define Defrast_bit	           (BitChar[0] >> defrast_bit)%2
#define Defrast_bit_set	       BitChar[0] |= 1<< defrast_bit;
#define Defrast_bit_reset	   BitChar[0] &= ~(1<< defrast_bit);

#define Cmp_led	               (BitChar[0] >> cmp_led)%2
#define Cmp_led_set	           BitChar[0] |= 1<< cmp_led;
#define Cmp_led_reset	       BitChar[0] &= ~(1<< cmp_led);

#define Test_bit	           (BitChar[0] >> test_bit)%2
#define Test_bit_set	       BitChar[0] |= 1<< test_bit;
#define Test_bit_reset	       BitChar[0] &= ~(1<< test_bit);

#define Send_door_bit	       (BitChar[0] >> send_door_bit)%2
#define Send_door_bit_set	   BitChar[0] |= 1<< send_door_bit;
#define Send_door_bit_reset	   BitChar[0] &= ~(1<< send_door_bit);

//****************************************************************

#define Protect_bit	           (BitChar[1] >> protect_bit)%2
#define Protect_bit_set	       BitChar[1] |= 1<< protect_bit;
#define Protect_bit_reset	   BitChar[1] &= ~(1<< protect_bit);

#define Hi_volt_bit	           (BitChar[1] >> hi_volt_bit)%2
#define Hi_volt_bit_set	       BitChar[1] |= 1<< hi_volt_bit;
#define Hi_volt_bit_reset	   BitChar[1] &= ~(1<< hi_volt_bit);

#define Low_volt_bit	       (BitChar[1] >> low_volt_bit)%2
#define Low_volt_bit_set	   BitChar[1] |= 1<< low_volt_bit;
#define Low_volt_bit_reset	   BitChar[1] &= ~(1<< low_volt_bit);

#define Door_status_bit	       (BitChar[1] >> door_status_bit)%2
#define Door_status_bit_set    BitChar[1] |= 1<< door_status_bit;
#define Door_status_bit_reset  BitChar[1] &= ~(1<< door_status_bit);

#define Done_startup_bit	   (BitChar[1] >> done_startup_bit)%2
#define Done_startup_bit_set   BitChar[1] |= 1<< done_startup_bit;
#define Done_startup_bit_reset BitChar[1] &= ~(1<< done_startup_bit);

#define Vac_Eco_bit	           (BitChar[1] >> vac_eco_bit)%2
#define Vac_Eco_bit_set        BitChar[1] |= 1<< vac_eco_bit;
#define Vac_Eco_bit_reset      BitChar[1] &= ~(1<< vac_eco_bit);

//****************************************************************

#define Key_sound	              (BitChar[2] >> key_sound)%2
#define Key_sound_on	          BitChar[2] |= 1<< key_sound;
#define Key_sound_off	          BitChar[2] &= ~(1<< key_sound);

#define Key_sound_alarm	          (BitChar[2] >> key_sound_alarm)%2
#define Key_sound_alarm_on	      BitChar[2] |= 1<< key_sound_alarm;
#define Key_sound_alarm_off	      BitChar[2] &= ~(1<< key_sound_alarm);

#define Key_sound_def	          (BitChar[2] >> key_sound_def)%2
#define Key_sound_def_on	      BitChar[2] |= 1<< key_sound_def;
#define Key_sound_def_off	      BitChar[2] &= ~(1<< key_sound_def);

#define Key_sound_lock	          (BitChar[2] >> key_sound_lock)%2
#define Key_sound_lock_on	      BitChar[2] |= 1<< key_sound_lock;
#define Key_sound_lock_off	      BitChar[2] &= ~(1<< key_sound_lock);

#define Key_sound_temp	          (BitChar[2] >> key_sound_temp)%2
#define Key_sound_temp_on	      BitChar[2] |= 1<< key_sound_temp;
#define Key_sound_temp_off	      BitChar[2] &= ~(1<< key_sound_temp);

#define Key_sound_power	          (BitChar[2] >> key_sound_power)%2
#define Key_sound_power_on	      BitChar[2] |= 1<< key_sound_power;
#define Key_sound_power_off	      BitChar[2] &= ~(1<< key_sound_power);

//****************************************************************

#define Led_alarm_status	      (BitChar[3] >> led_alarm_status)%2
#define Led_alarm_status_set	  BitChar[3] |= 1<< led_alarm_status;
#define Led_alarm_status_reset	  BitChar[3] &= ~(1<< led_alarm_status);

#define Led_def_status	          (BitChar[3] >> led_def_status)%2
#define Led_def_status_set	      BitChar[3] |= 1<< led_def_status;
#define Led_def_status_reset      BitChar[3] &= ~(1<< led_def_status);

#define Led_lock_status	          (BitChar[3] >> led_lock_status)%2
#define Led_lock_status_set	      BitChar[3] |= 1<< led_lock_status;
#define Led_lock_status_reset	  BitChar[3] &= ~(1<< led_lock_status);

#define Led_temp_status	          (BitChar[3] >> led_temp_status)%2
#define Led_temp_status_set       BitChar[3] |= 1<< led_temp_status;
#define Led_temp_status_reset	  BitChar[3] &= ~(1<< led_temp_status);

#define Led_power_status	      (BitChar[3] >> led_power_status)%2
#define Led_power_status_set	  BitChar[3] |= 1<< led_power_status;
#define Led_power_status_reset	  BitChar[3] &= ~(1<< led_power_status);

#define Protect_hi_unorm_flag	      (BitChar[3] >> protect_hi_unorm_flag)%2
#define Protect_hi_unorm_flag_set	  BitChar[3] |= 1<< protect_hi_unorm_flag;
#define Protect_hi_unorm_flag_reset	  BitChar[3] &= ~(1<< protect_hi_unorm_flag);

#define Protect_low_unorm_flag	      (BitChar[3] >> protect_low_unorm_flag)%2
#define Protect_low_unorm_flag_set	  BitChar[3] |= 1<< protect_low_unorm_flag;
#define Protect_low_unorm_flag_reset  BitChar[3] &= ~(1<< protect_low_unorm_flag);

#define Protect_second_bit	      (BitChar[3] >> protect_second_bit)%2
#define Protect_second_bit_set	  BitChar[3] |= 1<< protect_second_bit;
#define Protect_second_bit_reset  BitChar[3] &= ~(1<< protect_second_bit);
//****************************************************************
GPIO_InitTypeDef          GPIO_InitStructure;
USART_InitTypeDef         USART_InitStructure;
NVIC_InitTypeDef          NVIC_InitStructure;
TIM_TimeBaseInitTypeDef   TTB;
TIM_OCInitTypeDef         TOC;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0;
ADC_InitTypeDef           ADC_InitStructure;
I2C_InitTypeDef           I2C_InitStructure;

char Cmp;
#define cmp_on_time 720 //cmp on time in minute 16h=960min 12h=720 

#define Cmp_on       {GPIOA->ODR |= 1<<15;     Cmp=1; }
#define Cmp_off      {GPIOA->ODR &= ~(1<<15);  Cmp=0; }

#define Heater_on    GPIOB->ODR |= 1<<3; 
#define Heater_off   GPIOB->ODR &= ~(1<<3); 

#define Buzz_on      //GPIOB->ODR |= 1<<0;
#define Buzz_off     //GPIOB->ODR &= ~(1<<0);

#define door         !((GPIOB->IDR >>4) %2)
#define fan          !((GPIOB->IDR >>5) %2) 

#define eva           0
#define freez         1
#define ref           2
#define term          3
#define protect       4

#define normal        0
#define economy       1
#define vacation      2

#define down          0
#define up            1

#define cmp_mode           0
#define defrost_mode       1
//-----------------------------------------------------------------

char led_alarm, led_def, led_lock, led_temp, led_power;
char RX_Data;
unsigned char Fac_test, Protect_flag, led_blink_flag, fac_test_repeat=1, flag=1, test_flag, Degree_over=0;
unsigned int dr_time, mode_work, mode_start, time_sec, time_min, cmp_time_sec, cmp_time_min, Term_error_sec=59, Term_error_min, def_time_min, status, Protect_count, Protect_delay_sec, Protect_sec, def_time_min2=0; 
static unsigned long long m; 
unsigned char Term_degree=3, Term_degree_recived=3, Cmp_off_delay_sec, Cmp_off_delay_min;
	
//Variables to store current states
uint8_t   CChan;
//Variable to store current conversions
uint32_t  ADC_Data[5];
uint8_t   ADC_Data_Sensor[4],Adc_Data_Send[4];
uint16_t  ADC_Data_Protect;
uint8_t   V_IN=220;
//---Private function -----------------------------------
//---GPIO_Config()---------------------------------------
void GPIO_Config(void)
{
    /* GPIOA, GPIOB and GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    /* GPIOA Configuration: Channel 1, 2, 3, 4 and Channel 1N as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

//---Timer_Init (void)---------------------------------------
void Timer_Init (void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    //---TIM3 Configuration for Time
    /* Time Base configuration */
    TTB.TIM_ClockDivision = TIM_CKD_DIV1;
    TTB.TIM_CounterMode = TIM_CounterMode_Up;
    TTB.TIM_Period = 0x7C;
    TTB.TIM_Prescaler = 38349;
    TTB.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TTB);
    TIM_ITConfig(TIM3,0x0001,ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(TIM3_IRQn);
    
    //---TIM1 Configuration for Buzzer PWM Generation 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
    TimerPeriod = (SystemCoreClock / (100*3500)) - 1;
    /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
    Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
    
    TTB.TIM_Prescaler = 0x0063;
    TTB.TIM_CounterMode = TIM_CounterMode_Up;
    TTB.TIM_Period = TimerPeriod;
    TTB.TIM_ClockDivision = 0;
    TTB.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TTB);
    TIM_ITConfig(TIM1,0x0001,ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    
    TOC.TIM_OCMode = TIM_OCMode_PWM2;
    TOC.TIM_OutputState = TIM_OutputState_Enable;
    TOC.TIM_OutputNState = TIM_OutputNState_Enable;
    TOC.TIM_Pulse = Channel1Pulse;
    TOC.TIM_OCPolarity = TIM_OCPolarity_Low;
    TOC.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TOC.TIM_OCIdleState = TIM_OCIdleState_Set;
    TOC.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &TOC);
    /* TIM3 counter enable */
    TIM_Cmd(TIM3, ENABLE);
    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    /* TIM1 Main Output Enable */
    //TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    
    //Buzzer PWM
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
}

//---PWM_Set_Puls(uint16_t Frequency, uint8_t Duty_Cycle)----
void PWM_Set_Puls(uint16_t Frequency, uint8_t Duty_Cycle)
{
    /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
    TimerPeriod = ((SystemCoreClock / (100*Frequency)) - 1);
    /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
    Channel1Pulse = (uint16_t) (((uint32_t) Duty_Cycle * (TimerPeriod - 1)) / 100);
    
    TTB.TIM_Period = TimerPeriod;
    TIM_TimeBaseInit(TIM1, &TTB);
    
    TOC.TIM_Pulse = Channel1Pulse;
    TIM_OC1Init(TIM1, &TOC);
}

//---PWM_Enable(void)----------------------------------------
void PWM_Enable(void)
{
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

//---PWM_Disable(void)---------------------------------------
void PWM_Disable(void)
{
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
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

//---I2C1_Init (void)----for EEPROM--------------------------
void I2C1_Init (void)
{
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00; 
    I2C_InitStructure.I2C_Timing = 0x00201D2B;

    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
    //I2C
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);
}

//---I2C_Write(uint8_t address, uint8_t Data)-----------------
void I2C_Write(uint8_t IC_Address,uint8_t Page, uint8_t Data){

	//Wait until I2C isn't busy
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	//"Handle" a transfer - The STM32F0 series has a shocking
	//I2C interface... Regardless! Send the address of the HMC
	//sensor down the I2C Bus and generate a start saying we're
	//going to write one byte. I'll be completely honest,
	//the I2C peripheral doesn't make too much sense to me
	//and a lot of the code is from the Std peripheral library
	I2C_TransferHandling(I2C1, IC_Address, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	//Ensure the transmit interrupted flag is set
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register we wish to write to
	I2C_SendData(I2C1, Page);

	//Ensure that the transfer complete reload flag is
	//set, essentially a standard TC flag
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TCR) == RESET);

	//Now that the HMC5883L knows which register
	//we want to write to, send the address again
	//and ensure the I2C peripheral doesn't add
	//any start or stop conditions
	I2C_TransferHandling(I2C1, IC_Address, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

	//Again, wait until the transmit interrupted flag is set
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the value you wish you write to the register
	I2C_SendData(I2C1, Data);
	//Wait for the stop flag to be set indicating
	//a stop condition has been sent
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);

	//Clear the stop flag for the next potential transfer
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
    delay_ms(2);
}

//---I2C_Read(uint8_t address, uint8_t Data)-------------------
uint8_t I2C_Read(uint8_t IC_Address, uint8_t Page){

    uint8_t ReadByte;
    
    I2C_TransferHandling(I2C1,IC_Address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET); //the program stucks here
    
    I2C_SendData(I2C1, Page);
    
    while(I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET);
    
    I2C_TransferHandling(I2C1,IC_Address, 1, I2C_AutoEnd_Mode,I2C_Generate_Start_Read);
    
    while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET); 
    ReadByte = I2C_ReceiveData(I2C1);
    
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
    I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
    
    return ReadByte;
}

//---ADC1_Init (void)-----------------------------------------
void ADC1_Init (void)
{
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //---Configure ADC in 8 bit mode with upward scanning for Sensors
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1, &ADC_InitStructure);

    //Enable end of conversion interrupt
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    //Enable ADC1 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
   
    
    //Configure the channels to be converted, in this case C0, C1 and
    //C2, corresponding to PA0, PA1 and PA2 respectively
    ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_9, ADC_SampleTime_239_5Cycles);
    
    ADC_Cmd(ADC1, ENABLE);
    //Wait for the ADC to be ready!
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
    //Start conversion
    ADC_StartOfConversion(ADC1);
}
//---EEPROM_All_Data_Read(void)-------------------------------
void EEPROM_All_Data_Read(void)
{
    /*if(I2C_Read(IC_Address1, mute_address)==1)
        Mute_bit_set
    else
        Mute_bit_reset
    
    if(I2C_Read(IC_Address1, lock_address)==1)
        Lock_bit_set
    else
        Lock_bit_reset*/
    
    Fac_test = I2C_Read(IC_Address1, fac_test_address);
    
    if(Fac_test !=0 && Fac_test !=1 && Fac_test !=2)
        I2C_Write(IC_Address1,fac_test_address,0);
    
    if(Fac_test == 2)
    {
        cmp_time_min=I2C_Read(IC_Address1, cmp_min_low_address);
        cmp_time_min +=I2C_Read(IC_Address1, cmp_min_hi_address)*256;
        if(cmp_time_min >= 630)
            cmp_time_min=0;
    }
    
    /*Term_degree = I2C_Read(IC_Address1, term_degree_address);
    if(Term_degree != 1 && Term_degree != 2 && Term_degree != 3 && Term_degree != 4 && Term_degree != 5 && Term_degree != 6)
    {
        I2C_Write(IC_Address1,term_degree_address,3);
        Term_degree = 3;
    }*/
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

//---Send_Sensor_Data()---------------------------------------
void Send_Sensor_Data(void)
{
    if(Test_bit)
    {
        Test_bit_reset
        Usart_Send('q');
    }

    if(Adc_Data_Send[ref] != ADC_Data_Sensor[ref])
    {
        Adc_Data_Send[ref]= ADC_Data_Sensor[ref];
        Usart_Send('R');
        Usart_Send(Adc_Data_Send[ref]); 
    }

    if(Adc_Data_Send[freez] != ADC_Data_Sensor[freez])
    {
        Adc_Data_Send[freez]= ADC_Data_Sensor[freez];
        Usart_Send('F'); 
        Usart_Send(Adc_Data_Send[freez]); 
    }

    if(Adc_Data_Send[eva] != ADC_Data_Sensor[eva])
    {
        Adc_Data_Send[eva]= ADC_Data_Sensor[eva];
        Usart_Send('E');
        Usart_Send(Adc_Data_Send[eva]); 
    }
    
    if(Adc_Data_Send[term] != ADC_Data_Sensor[term])
    {
        Adc_Data_Send[term]= ADC_Data_Sensor[term];
        Usart_Send('T');
        Usart_Send(Adc_Data_Send[term]); 
    }

    if(Cmp_led != fan)  //if(Cmp_led != Cmp)
    {
        if(fan)  //if(Cmp==1)
        {	
            Cmp_led_set
            Usart_Send('C');
        }
        else  
        {	
            Cmp_led_reset
            Usart_Send('c');
        }
    }
}

//---Send_All_Data()---------------------------------------
void Send_All_Data(void)
{
    if(Test_bit)
    {
        Test_bit_reset
        Usart_Send('q');
    }
    
	Usart_Send('R');
	Usart_Send(ADC_Data_Sensor[ref]);
	
	Usart_Send('F');
	Usart_Send(ADC_Data_Sensor[freez]); 
	
	Usart_Send('E');
	Usart_Send(ADC_Data_Sensor[eva]); 
	
    Usart_Send('T');
	Usart_Send(ADC_Data_Sensor[term]); 
    
	if(Protect_bit)
		Usart_Send('Z');
	else if(Protect_bit==0 && led_blink_flag==1) 
        Usart_Send('z'); 
	else if(Protect_bit==0 && Protect_flag==0) 
 		Usart_Send('t');
    
	if(fan)    //if(Cmp==1)
        Usart_Send('C');
	else  
        Usart_Send('c'); 
    
	if((GPIOB->IDR >>3) %2 == 1) 
        Usart_Send('H'); 
	else 
        Usart_Send('h');
    
    /*if(Lock_bit)
        Usart_Send('L');  
    else 
        Usart_Send('l');
    
    if(Mute_bit)
        Usart_Send('M');  
    else 
        Usart_Send('m');*/
}

//---Door_Operand()---------------------------------------
void Door_Operand(void)
{
    if(door!=Door_status_bit) 
    {
        if(door)
            Door_status_bit_set
        else
            Door_status_bit_reset

        Send_door_bit_set
    }
    
    if(door)
    {
        if(Send_door_bit)
        {
            Send_door_bit_reset
            Usart_Send('D');
        }
        dr_time++;
        if (dr_time>=30)
        { 
            dr_time=0;
            Alarm_bit_set
            m=0;
        }
    }
    else 
    {
        if(Send_door_bit)
        {
            Send_door_bit_reset
            Usart_Send('d');
        }
        dr_time=0;
        Alarm_bit_reset
    }
}

//---Cmp_Status(unsigned char low_degree, unsigned char hi_degree)
void Cmp_Status(unsigned char low_degree, unsigned char hi_degree)
{
    static unsigned char Degree_direction=down;
    
    if(ADC_Data_Sensor[term]<=hi_degree)
    {    
        Cmp_on
        Degree_direction = down;
    }
    else if(ADC_Data_Sensor[term] < low_degree && ADC_Data_Sensor[term] > hi_degree)
    {    
        if(Degree_direction == up)
        {
            if(fan)
            {
                Cmp_off_delay_sec=0;
                Cmp_off_delay_min=0;
            }
            Cmp_off
        }
        else if(Degree_direction == down)
            Cmp_on
    }
    else if(ADC_Data_Sensor[term] >= low_degree)
    {    
        if(fan)
        {
            Cmp_off_delay_sec=0;
            Cmp_off_delay_min=0;
        }
        Cmp_off
        Degree_direction = up;
    }
}
//---Term_Status(unsigned char degree)---------------------
void Term_Status(unsigned char degree)
{
    
    switch(degree)
	{
        case 1:
            Cmp_Status(163, 105); // -16, 2.5
            break;
        
        case 2:
            Cmp_Status(169, 105); // -18, 2.5
            break;
        
        case 3:
            Cmp_Status(175, 105); // -20, 2.5
            break;
        
        case 4:
            Cmp_Status(180, 105); // -22, 2.5
            break;
        
        case 5:
            Cmp_Status(185, 105); // -24, 2.5
            break;
        
        case 6:
            Cmp_Status(190, 105); // -26, 2.5
            break;
        
        default:
            break;
    }    
}

//---Startup()---------------------------------------------
void Startup (void)
{
    Cmp_on
    Heater_off
    status= cmp_mode;
    while(1)
    { 
        if(Protect_bit)
        {
                Cmp_off
                Heater_off 
                Usart_Send('Z');
                Protect_flag=1; 
                Protect_delay_sec=0;
        }
        else if(Second_bit)
        {
            Door_Operand();
            Second_bit_reset
            if(Protect_flag==1)
            { 
               Protect_delay_sec++;
               if(Protect_delay_sec%2==0)
                    Usart_Send('z'); 
               
               if(Protect_delay_sec>=240) 
               { 
                    Protect_flag=0; 
                    Protect_delay_sec=0; 
                    //led_blink_flag=0;
               }
               
            }
            else 
            {     
                time_sec++; 
                switch(status)
                { 
                    case cmp_mode :
                            if (Done_startup_bit) {Usart_Send('h'); Heater_off goto end_start_up; }               
                            Cmp_on
                            if(time_sec>59) 
                            {    
                                time_sec=0;
                                time_min++;
                                if(time_min>=2 && time_min<=14) 
                                {    
                                   if(Fac_test==0)
                                       I2C_Write(IC_Address1,fac_test_address,1);//fac_test=1
                                   else if(Fac_test==1)
                                       I2C_Write(IC_Address1,fac_test_address,2);//fac_test=2
                                }
                                if(time_min>=15){Cmp_off status=defrost_mode; time_min=0; time_sec=59;} 
                            }
                            break; 
                            
                    case defrost_mode :
                            Cmp_off
                            Done_startup_bit_set              
                            if(time_sec>59) 
                            {
                                time_sec=0;
                                time_min++; 
                                if(time_min>=1 && time_min<=5){Heater_on Usart_Send('H');}   
                                if(time_min>=6 && time_min<=9){Heater_off Usart_Send('h');}
                                if(time_min>=10){status=cmp_mode; time_min=0; goto end_start_up;} 
                            }
                            break;                                       
                }
            }
            
            Send_All_Data();
        }
        //IWDG_ReloadCounter();
    } 
    end_start_up: 
        return;
}

/* Interrupt Service Routins -----------------------------------------------*/
//---ADC1_IRQHandler()---------------------------------------------
void ADC1_IRQHandler(void)
{
    unsigned char i;
	static unsigned long Count=0 ,Count_2=0;
    static unsigned  char Sensor_eva_flag=0, Sensor_freez_flag=0, Sensor_ref_flag=0, Sensor_term_flag=0;
    //Check for end of conversion
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC))
	{
        //Clear interrupt bit
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		//Switch statement dependent on current channel.
		switch(CChan)
		{
            case eva:
                ADC_Data[eva] += ADC_GetConversionValue(ADC1)>>2;
                CChan = 1;
                break;
            case freez:
                ADC_Data[freez] += ADC_GetConversionValue(ADC1)>>2;
                CChan = 2;
                break;
           
            case ref:
                ADC_Data[ref] += ADC_GetConversionValue(ADC1)>>2;
                CChan = 3;
                break;
            
            case term:
                ADC_Data[term] += ADC_GetConversionValue(ADC1)>>2;
                CChan = 4;
                break;
            
            case protect:
                ADC_Data[protect] += ADC_GetConversionValue(ADC1);
                CChan = 0;
                ADC_StopOfConversion(ADC1); 
                break;
            
            default:
                break;
		}
                   
		Count++;
        if(Count%1250==0)
		{
            Count_2++;
            if(ADC_Data[eva]/(250*Count_2)>245 || ADC_Data[eva]/(250*Count_2)<10)
            {    
                ADC_Data_Sensor[eva]=ADC_Data[eva]/(250*Count_2);
                Sensor_eva_flag=0;
            }
            if(ADC_Data[freez]/(250*Count_2)>245 || ADC_Data[freez]/(250*Count_2)<10)
            {
                ADC_Data_Sensor[freez]=ADC_Data[freez]/(250*Count_2);
                Sensor_freez_flag=0;
            }
            if(ADC_Data[ref]/(250*Count_2)>245 || ADC_Data[ref]/(250*Count_2)<10)
            {
                ADC_Data_Sensor[ref]=ADC_Data[ref]/(250*Count_2);
                Sensor_ref_flag=0;
            }
            if(ADC_Data[term]/(250*Count_2)>245 || ADC_Data[term]/(250*Count_2)<10)
            {
                ADC_Data_Sensor[term]=ADC_Data[term]/(250*Count_2);
                Sensor_term_flag=0;
            }
			ADC_Data_Protect = ADC_Data[protect]/250;
            
            V_IN=ADC_Data_Protect * 0.3265; //0.353;
            /*if(V_IN >245)
            {
                Hi_volt_bit_set
                Protect_bit_set
                Protect_hi_unorm_flag_set
            }
            else if(Protect_hi_unorm_flag)
            {
                if(V_IN >240)
                {
                    Hi_volt_bit_set
                    Protect_bit_set
                    Protect_sec = 0;
                }
                else
                { 
                    if(Protect_second_bit)
                    {
                        Protect_second_bit_reset
						Protect_sec++;
                        if(Protect_sec==5)
                        {
                            Protect_sec = 0;
                            Protect_hi_unorm_flag_reset
                        }  
                    }
                }                    
            }
            else*/ if(V_IN <170)
            {
                Low_volt_bit_set
                Protect_bit_set
                Protect_low_unorm_flag_set
            }
            else if(Protect_low_unorm_flag)
            {
                if(V_IN <175)
                {
                    Low_volt_bit_set
                    Protect_bit_set
                    Protect_sec = 0;
                }
                else
                { 
                    if(Protect_second_bit)
                    {
                        Protect_second_bit_reset
						Protect_sec++;
                        if(Protect_sec==5)
                        {
                            Protect_sec = 0;
                            Protect_low_unorm_flag_reset
                        }  
                    }
                }                    
            }
            else
            {
                Hi_volt_bit_reset 
                Low_volt_bit_reset
                Protect_bit_reset 
            }
            
            ADC_Data[protect]=0;
            
        }
        
        if(Count>=125000)
		{    
            if(Sensor_eva_flag<2)
                Sensor_eva_flag++;
            if(Sensor_eva_flag==2)
                ADC_Data_Sensor[eva]=ADC_Data[eva]/25000;
            
            if(Sensor_freez_flag<2)
                Sensor_freez_flag++;
            if(Sensor_freez_flag==2)
                ADC_Data_Sensor[freez]=ADC_Data[freez]/25000;
            
            if(Sensor_ref_flag<2)
                Sensor_ref_flag++;
            if(Sensor_ref_flag==2)
                ADC_Data_Sensor[ref]=ADC_Data[ref]/25000;
            
            if(Sensor_term_flag<2)
                Sensor_term_flag++;
            if(Sensor_term_flag==2)
                ADC_Data_Sensor[term]=ADC_Data[term]/25000;
            
            Count=0;
			Count_2=0;
            for(i=0;i<4;i++)
            {
                ADC_Data[i]=0;
            }
		}
        ADC_StartOfConversion(ADC1); 
		
	}
}

//---TIM3_IRQHandler()---------------------------------------------
void TIM3_IRQHandler (void) 
{
	static unsigned char Time_Count=0;
    if ((TIM3->SR & 0x0001) != 0)            // check interrupt source
    {                 
        Time_Count++;
        if(Time_Count>=10)
        {
            Time_Count=0;
            Second_bit_set
			Protect_second_bit_set
        }
        TIM3->SR &= ~(1<<0);                          // clear UIF flag
    }
    IWDG_ReloadCounter();
}

//---Sound Setting--------------------------------------------------
//---TIM16_IRQHandler()---------------------------------------------
void TIM1_BRK_UP_TRG_COM_IRQHandler (void) 
{
    
    
    if ((TIM1->SR & 0x0001) != 0)    // check interrupt source
    {                 
        if (Key_sound==0 & Alarm_bit==0) 
        {
            Buzz_off
            PWM_Disable(); 
            m=0;
        }
        else
        {
            m++;
        
            if(Alarm_bit && Mute_bit==0)
            { 
                if(m==10)             {PWM_Set_Puls(2350,10); PWM_Enable(); Buzz_on}
                else if (m==150)      {PWM_Disable();} 
                else if (m==401)      {PWM_Set_Puls(3800,10); PWM_Enable();}
                else if (m==550)      {PWM_Disable();} 
                else if (m>=5000)     {m=0; Buzz_off}        
            } 
            
            else if (Key_sound)
            {
                if (Key_sound_alarm)
                {
                    /*if(Led_alarm_status)
                    {
                        if(m==10)            {PWM_Set_Puls(2488,50); PWM_Enable();}
                        else if (m==6000)     {PWM_Disable();} 
                        //else if (m==6001)     {PWM_Set_Puls(3520,10); PWM_Enable();}
                        //else if (m==9000)     {PWM_Disable();}  
                        else if (m>=6001)     {Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }*/
                    if(Led_alarm_status)
                    {
                        if(m==10)            {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     {Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                    else
                    {
                        if(m==10)            {PWM_Set_Puls(4700,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     {Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                } 

                else if (Key_sound_def)
                {
                    if(Led_def_status)
                    {
                        if(m==10)            {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     {Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                    else
                    {
                        if(m==10)            {PWM_Set_Puls(4700,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     { Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                }
                
                else if (Key_sound_lock)
                {
                    if(Led_lock_status)
                    {
                        if(m==10)            {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     {Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                    else
                    {
                        if(m==10)            {PWM_Set_Puls(4700,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     { Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                }
                
                else if (Key_sound_temp)
                {
                    if(Led_temp_status)
                    {
                        if(m==10)            {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     {Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                    else
                    {
                        if(m==10)            {PWM_Set_Puls(4700,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     { Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                }
                
                else if (Key_sound_power)
                {
                    if(Led_power_status)
                    {
                        if(m==10)            {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     {Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                    else
                    {
                        if(m==10)            {PWM_Set_Puls(4700,10); PWM_Enable();}
                        else if (m==150)     {PWM_Disable();} 
                        else if (m==401)     {PWM_Set_Puls(3800,10); PWM_Enable();}
                        else if (m==550)     {PWM_Disable();}  
                        else if (m>=600)     { Key_sound_off  Key_sound_alarm_off; m=0; PWM_Disable();}  
                    }
                }
            }                
            
        }
        TIM1->SR &= ~(1<<0);                          // clear UIF flag
    }   
}
//---USART1_IRQHandler()--------------------------------------------
void USART1_IRQHandler(void)
{
	while (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // Received characters modify string
	{
		RX_Data = USART_ReceiveData(USART1);
	}
    
    switch (RX_Data)
    {
        case 1 :
            Term_degree_recived=1;
            break;
        case 2 :
            Term_degree_recived=2;
            break;
        case 3 :
            Term_degree_recived=3;
            break;
        case 4 :
            Term_degree_recived=4;
            break;
        case 5 :
            Term_degree_recived=5;
            break;
        case 6 :
            Term_degree_recived=6;
            break;
        
        case 'B' :
            Led_alarm_status_set
            Key_sound_alarm_on
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
            break; 

        case 'b' :
            Led_alarm_status_reset 
            Key_sound_alarm_on
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
            break;  
               
        case 'G' : 
            Led_def_status_set
            Key_sound_alarm_off
            Key_sound_def_on
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
            break;  
        
        case 'g' : 
            Led_def_status_reset
            Key_sound_alarm_off
            Key_sound_def_on
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
            break;  
                
        case 'J' : 
            Led_lock_status_set
            Key_sound_alarm_off
            Key_sound_def_off
            Key_sound_lock_on
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
            break; 
        
        case 'j' : 
            Led_lock_status_reset
            Key_sound_alarm_off
            Key_sound_def_off
            Key_sound_lock_on
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
            break; 

        case 'K' :
            Led_temp_status_set
            Key_sound_alarm_off
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_on
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
            break; 

        case 'k' :
            Led_temp_status_reset 
            Key_sound_alarm_off
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_on
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
            break; 

        case 'P' :
            Led_power_status_set
            Key_sound_alarm_off
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_on
            Key_sound_on 
            m=0;
            PWM_Disable();
            break;
        
        case 'p' :
            Led_power_status_reset
            Key_sound_alarm_off
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_on
            Key_sound_on 
            m=0;
            PWM_Disable();
            break; 
        
        case 'L' :
            Led_lock_status_set
            Key_sound_alarm_off
            Key_sound_def_off
            Key_sound_lock_on
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
        
            I2C_Write(IC_Address1,lock_address,1);//ch_lock=1
            Lock_bit_set
            break;
        
        case 'l' :
            Led_lock_status_reset
            Key_sound_alarm_off
            Key_sound_def_off
            Key_sound_lock_on
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
        
            I2C_Write(IC_Address1,lock_address,0);//ch_lock=0
            Lock_bit_reset
            break;  
        
        case 'M' :
            Led_alarm_status_reset 
            Key_sound_alarm_on
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
        
            I2C_Write(IC_Address1,mute_address,1);//mute=1
            Mute_bit_set
            break;
        
        case 'm' :
            Led_alarm_status_set
            Key_sound_alarm_on
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
        
            I2C_Write(IC_Address1,mute_address,0);//mute=0
            Mute_bit_reset
            break;
        
        case 'A' :
            Alarm_bit_reset
            dr_time=0;
            break;
        
        case 'H' : 
            Led_def_status_set
            Key_sound_alarm_off
            Key_sound_def_on
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
        
            if(status != defrost_mode)
            {
                status=defrost_mode;
                if(test_flag==1) 
                {
                    if(Fac_test==0)
                        I2C_Write(IC_Address1,fac_test_address,1);//Fac_test = 0
                    else if(Fac_test==1)
                        I2C_Write(IC_Address1,fac_test_address,2);//Fac_test = 1
                
                    time_min=0;
                    time_sec=58;
                } 
                else
                {  
                    if((ADC_Data_Sensor[eva]<10)||(ADC_Data_Sensor[eva]>245))
                        def_time_min=10; 
                    else
                        def_time_min=15;
                    
                    cmp_time_sec=58;
                }            
            }    
            break; 
            
        /*case 'h' :
            if(fac_test_repeat==0)
            {
                fac_test_repeat=1;
                if(Fac_test==0)
                    I2C_Write(IC_Address1,fac_test_address,0);//fac_test = 0
                else if(Fac_test==1)
                    I2C_Write(IC_Address1,fac_test_address,1);//fac_test = 1
            }

            def_time_min=5; 
            cmp_time_sec=59; 
            break;*/
        case 'V' : 
			Led_alarm_status_set
            Key_sound_alarm_on
            Key_sound_def_off
            Key_sound_lock_off
            Key_sound_temp_off
            Key_sound_power_off
            Key_sound_on
            m=0;
            PWM_Disable();
			
			I2C_Write(IC_Address1,fac_test_address,0);
			I2C_Write(IC_Address1,cmp_min_low_address,0);
			I2C_Write(IC_Address1,cmp_min_hi_address,0);
			break; 
		
        case 'q' :
            Test_bit_set
            break;                            
    }
}
//******************************************************************
//---main()---------------------------------------------------------
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    Delay_Init(48);
    delay_ms(1000);
    GPIO_Config();
    Timer_Init ();
    Usart_Init ();
    I2C1_Init ();
    ADC1_Init ();
    IWD_Init();
    
    //I2C_Write(IC_Address1,fac_test_address,0);
    //I2C_Write(IC_Address1,cmp_min_low_address,0);
    //I2C_Write(IC_Address1,cmp_min_hi_address,0);
    
    EEPROM_All_Data_Read();
    
    /*if(Mute_bit)
        led_alarm = 1;
    else
        led_alarm = 0;
    
    if(Lock_bit)
        led_lock = 1;
    else
        led_lock = 0;*/
    
    //delay_ms(200); 
    
    Cmp_off
    Heater_off
    Alarm_bit_reset
    Mute_bit_reset
    led_alarm = 1;
    Lock_bit_reset
    led_lock = 0;
    
    Send_All_Data();
    
    if(Fac_test == 0 || Fac_test == 1)
    {
        test_flag=1;    
        Startup();
        test_flag=0;
    }
    
    /* Infinite loop */
    while (1)
    {
      
        if(Protect_bit)
        {   
            Cmp_off
            Heater_off
            Protect_flag=1; 
            Protect_delay_sec=0;
            Usart_Send('Z');
        }
        
        else if(Second_bit)
        {
            Second_bit_reset
            Door_Operand();
            time_sec++; 
            if(time_sec>59) 
            {
                time_sec=0;
                time_min++;
                if(time_min>=16 && status==cmp_mode) // Compresor Work Time Save Every 32 Minutes
                {
                    time_min=0;
                    I2C_Write(IC_Address1,cmp_min_low_address,cmp_time_min);
                    I2C_Write(IC_Address1,cmp_min_hi_address,(cmp_time_min>>8)%256);
                }
            }
            if(Cmp_off_delay_min<4)
            {
                Cmp_off_delay_sec++;
                if(Cmp_off_delay_sec>59) 
                {
                    Cmp_off_delay_sec=0;
                    Cmp_off_delay_min++;
                }
            }            
           
            if(Protect_flag==1)
            { 
                Protect_delay_sec++;
                if(Protect_delay_sec%2==0)
                    Usart_Send('z'); 

                if(Protect_delay_sec>=240) 
                { 
                    Protect_flag=0; 
                    Protect_delay_sec=0;
                }
            }
            else if(Protect_flag==0)
            {     
                if(status==cmp_mode)
                {    
                    if(fan) 
                        cmp_time_sec++;
                }
                else if(status==defrost_mode)
                    cmp_time_sec++;
                
                if(Cmp_off_delay_min>=4)
                    Term_degree=Term_degree_recived;
                
                switch(status)
                { 
                    case cmp_mode :
                            Heater_off
                            if((ADC_Data_Sensor[term]<10)||(ADC_Data_Sensor[term]>245))
                            {
                                Term_error_sec++;
                                if(Term_error_sec>59)
                                {
                                    Term_error_sec=0;
                                    Term_error_min++;
                                    
                                    if(Term_error_min<13)
                                        Cmp_on
                                    else if(Term_error_min>=13 && Term_error_min<35)
                                        Cmp_off
                                    else if(Term_error_min>=35)
                                    {    
                                        Term_error_min=0;
                                        Term_error_sec=59;
                                        Cmp_off_delay_sec=0;
                                        Cmp_off_delay_min=0;
                                    }
                                }
                            }
                            else
                            {   
                                Term_Status(Term_degree);    //Cmp_on 
                                Term_error_sec=59;
                                Term_error_min=0; 
                            }
                            
                            if(cmp_time_sec>59) 
                            {
                                cmp_time_sec=0;
                                cmp_time_min++;

                                if(cmp_time_min>=cmp_on_time){Cmp_off status=defrost_mode; cmp_time_min=0; cmp_time_sec=59;}
                            }  
                            break; 
                        
                    case defrost_mode : 
                            Cmp_off
                            
							if(Degree_over != 1)
							{
								if (ADC_Data_Sensor[eva]<86)//10 centigrade 
								{
									Heater_off 
									Usart_Send('h');
									Degree_over=1;
									cmp_time_sec=0;
								} 
							} 
									
                            if(cmp_time_sec>59) 
                            {   
                                cmp_time_sec=0;
                                def_time_min++; 
                                if(Degree_over == 1)
								{
									def_time_min2++;
									if(def_time_min2>=1 && def_time_min2<=3)
									{ 
										Heater_off 
										Usart_Send('h');
									} 
									if(def_time_min2>=4)
									{
										 status=cmp_mode;  
										 time_min=0; 
										 def_time_min=0;
										 def_time_min2=0;
										 Degree_over=0;
										 time_sec=0; 
									}
								} 
										
								else if((ADC_Data_Sensor[eva]<10)||(ADC_Data_Sensor[eva]>245))
                                {
                                    if(def_time_min>=1 && def_time_min<=16) {Heater_on  Usart_Send('H');}
                                    if(def_time_min>=17 && def_time_min<=20) {Heater_off  Usart_Send('h');}
                                    if(def_time_min>=21) {status=cmp_mode; time_min=0; def_time_min=0; time_sec=0;} 
                                }
                                
                                else
                                {
                                    if(def_time_min>=1 && def_time_min<=20){Heater_on Usart_Send('H');}   
                                    if(def_time_min>=21 && def_time_min<=24){Heater_off Usart_Send('h');} 
                                    if(def_time_min>=25){status=cmp_mode; time_min=0; def_time_min=0; time_sec=0;}
                                }
                            }
                            break;                                      
                }
            }

            Send_All_Data();   
        }
        //IWDG_ReloadCounter();
    }
}

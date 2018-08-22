

#include "stm32f4_discovery.h"
#include <stdio.h>
#include "defines.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_usart.h"
#include "math.h"
#include "arm_math.h"
#include "intrinsics.h"
#define EEPROM_HW_ADDRESS      0xA0   /* E0 = E1 = E2 = 0 */
#define I2C_EE             I2C3//interface number
void Delay_ms(uint32_t ms);
static uint16_t sine100[3205];
static uint16_t sine200[3205];
static uint16_t sine400[3205];
static uint16_t sine800[3205];
static uint16_t sine1600[3205];
static uint16_t sine3200[3205];
static uint16_t * sine=sine3200;
static float FL=10,FG=1000,TEMP=0,freq=100;
static unsigned long sine_period=1000;
static int sin_gain=0;

static float last_FL =0 , last_FG=0;
static int FL_change=0,FG_change=0;


#include "base64.h"



/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

int TOTAL_ERROR=0;;
unsigned int d_counter=0;
int uncallibrated;
/** @addtogroup ADC_ADC3_DMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
#define ADC1_DR_ADDRESS     ((uint32_t)0x4001024C)

#define LEDON GPIO_SetBits(GPIOD,GPIO_Pin_12)
#define LEDOFF GPIO_ResetBits(GPIOD,GPIO_Pin_12)
#define LED2ON GPIO_SetBits(GPIOD,GPIO_Pin_13)
#define LED2OFF GPIO_ResetBits(GPIOD,GPIO_Pin_13)
#define LED3ON GPIO_SetBits(GPIOD,GPIO_Pin_14)
#define LED3OFF GPIO_ResetBits(GPIOD,GPIO_Pin_14)
#define LED4ON GPIO_SetBits(GPIOD,GPIO_Pin_15)
#define LED4OFF GPIO_ResetBits(GPIOD,GPIO_Pin_15)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* You can monitor the converted value by adding the variable "ADC3ConvertedValue" 
   to the debugger watch window */
__IO uint16_t ADC3ConvertedValue = 0;
__IO uint32_t ADC3ConvertedVoltage = 0;
  

#define BUFFERSIZE 800 // 1000 * 2 * 4 * 200hz 
  
uint16_t ADCConvertedValues[BUFFERSIZE];
double RAWV1=0,RAWV2=0,RAWV3=0,RAWV4=0,RAWV5=0;
double V1=0,V2=0,V3=0,V4=0,V5=0;
double period_ms=0,Landa_mm=0,peak_mm=0;
float period1_ms=0,period2_ms=0,period3_ms=0,period4_ms=0,period5_ms=0;
int peak1=0,peak2=0,peak3=0,peak4=0,peak5=0;
bool side1=false;
bool side2=false;
bool side3=false;
bool side4=false;
bool side5=false;
float period1,period2,period3,period4,period5;
int absolutetime1=0,absolutetime2=0,absolutetime3=0,absolutetime4=0,absolutetime5=0;
int delay_v5v4=0;
int delay_v5v3=0;
int delay_v5v2=0;
int delay_v5v1=0;
int delay_v4v3=0;
int delay_v4v2=0;
int delay_v4v1=0;
int delay_v3v2=0;
int delay_v3v1=0;
int delay_v2v1=0;
/* Private function prototypes -----------------------------------------------*/


static struct cal_val_t{
	float offset[5];
	float 	mul[5];
	unsigned long cs;
}CAL;


static struct WG_PAKAGE{
	short	guages[5];
	float 	freg[5];
	unsigned long cs;
}OUT_DATA;



unsigned int cs_cal()
{
	unsigned int *p=(unsigned int *) &CAL;
	
	unsigned int sum=0;
	for(int i=((sizeof(CAL))/4)-1;i--;)
	{
		sum+=*p++;
	}
	return sum;
}
char load_cal();

void I2C_Configuration(void)
{

           I2C_InitTypeDef  I2C_InitStructure;
           GPIO_InitTypeDef  GPIO_InitStructure;

           RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,ENABLE);
          // RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE);//

           /* Configure I2C1 pins: PA8->SCL and PC9->SDA */
           GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 ;
           GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
           GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
           GPIO_Init(GPIOA, &GPIO_InitStructure);
           GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 ;
           GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
           GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
           GPIO_Init(GPIOC, &GPIO_InitStructure);
		   
		   GPIO_PinAFConfig(GPIOA,8,GPIO_AF_I2C3);
		   GPIO_PinAFConfig(GPIOC,9,GPIO_AF_I2C3); 
		   
		   
           I2C_DeInit(I2C_EE);
           I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
           I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
           I2C_InitStructure.I2C_OwnAddress1 = 1;
           I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
           I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
           I2C_InitStructure.I2C_ClockSpeed = 100000;  /* 100kHz */

           I2C_Cmd(I2C_EE, ENABLE);
           I2C_Init(I2C_EE, &I2C_InitStructure);
           I2C_AcknowledgeConfig(I2C_EE, ENABLE);

}


void I2C_EE_ByteWrite(uint8_t val, uint16_t WriteAddr)
{


    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_HW_ADDRESS, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


    /* Send the EEPROM's internal address to write to : MSB of the address first */
    I2C_SendData(I2C_EE, (uint8_t)((WriteAddr & 0xFF00) >> 8));

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));



    /* Send the EEPROM's internal address to write to : LSB of the address */
    I2C_SendData(I2C_EE, (uint8_t)(WriteAddr & 0x00FF));

    /* Test on EV8 and clear it */
    while(! I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));


     I2C_SendData(I2C_EE, val);

        /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);

    //delay between write and read...not less 4ms
    Delay_ms(5);
}
//*********************************************************************************
uint8_t I2C_EE_ByteRead( uint16_t ReadAddr)
{
    uint8_t tmp;

        /* While the bus is busy */
    while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_HW_ADDRESS, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


    /* Send the EEPROM's internal address to read from: MSB of the address first */
    I2C_SendData(I2C_EE, (uint8_t)((ReadAddr & 0xFF00) >> 8));

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the EEPROM's internal address to read from: LSB of the address */
    I2C_SendData(I2C_EE, (uint8_t)(ReadAddr & 0x00FF));

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));


    /* Send STRAT condition a second time */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for read */
    I2C_Send7bitAddress(I2C_EE, EEPROM_HW_ADDRESS, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE,I2C_EVENT_MASTER_BYTE_RECEIVED));//I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    tmp=I2C_ReceiveData(I2C_EE);


    I2C_AcknowledgeConfig(I2C_EE, DISABLE);

    /* Send STOP Condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);

    return tmp;
    }
//*******************************************************************************
void Delay_ms(uint32_t ms)
{
        volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq (&RCC_Clocks);

        nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
        for (; nCount!=0; nCount--);
}
//*****************************************************************************














void reset_cal()
{
	/*CAL.offset[0]=2500;
	CAL.mul[0]=0.3;
	CAL.offset[1]=2500;
	CAL.mul[1]=0.3;
	CAL.offset[2]=2500;
	CAL.mul[2]=0.3;
	CAL.offset[3]=2500;
	CAL.mul[3]=0.3;
	CAL.offset[4]=2500;
	CAL.mul[4]=0.3;
	CAL.cs=cs_cal();*/
}


char load_cal()
{

	char * p=(char *)&CAL;
	for(int i=0;i<sizeof(CAL);i++)
	{	
		*p++=I2C_EE_ByteRead(i);
		
		for(int i=10000;i--;);
	}
	
	if(CAL.cs==cs_cal())
			return 1;
	
	return 0;
		
	
	
}

char save_cal()
{

		CAL.cs=cs_cal();
		char * p=(char *)&CAL;
		for(int i=0;i<sizeof(CAL);i++){
			I2C_EE_ByteWrite(*p++,i);
		}


	return 1;
}





/* Private functions ---------------------------------------------------------*/





// STM32 ADC Sample @ 1440.000 KHz (PC.1) STM32F4 Discovery - sourcer32@gmail.com
  
// Assumptions per system_stm32f4xx.c CPU @ 168 MHz, APB2 @ 84 MHz (/2), APB1 @ 42 MHz (/4)
  
#include "stm32f4_discovery.h"
  
/**************************************************************************************/
  
void RCC_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
}
  
/**************************************************************************************/
  
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  // ADC Channel 11 -> PC1 12 -> PC2  13 -> PC3
 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //DAC1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //DAC2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  

  // LED  PA11
  

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //green
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
}
  
/**************************************************************************************/
  
void ADC_Configuration(void)
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1 Channel
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 4;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* ADC1 regular channel 11 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles); // PA0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_480Cycles); // PA1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_480Cycles); // PA4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_480Cycles); // PA5
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 5, ADC_SampleTime_15Cycles); // PA7


  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
}
  
/**************************************************************************************/

  
static void DMA_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  
  #define   OUT_FREQ          5000                                 // Output waveform frequency
#define   SINE_RES          256                                  // Waveform resolution
#define   DAC_DHR12R1_ADDR  0x40007408                           // DMA writes into this reg on every request
#define   DAC_DHR12R2_ADDR  0x40007414                           // DMA writes into this reg on every request
#define   CNT_FREQ          42000000                             // TIM6 counter clock (prescaled APB1)
#define   TIM_PERIOD        ((CNT_FREQ)/((SINE_RES)*(OUT_FREQ))) // Autoreload reg value

  
  
  //ADC1
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValues[0];
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFFERSIZE; // Count of 16-bit words
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  
  /* Enable DMA Stream Half / Transfer Complete interrupt */
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);
  
  /* DMA2_Stream0 enable */
  DMA_Cmd(DMA2_Stream0, ENABLE);
  
 
  
  DAC_InitTypeDef DAC_INIT;
  DMA_InitTypeDef DMA_INIT;
  
  DAC_INIT.DAC_Trigger        = DAC_Trigger_T6_TRGO;
  DAC_INIT.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_INIT.DAC_OutputBuffer   = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_2, &DAC_INIT);

  DMA_DeInit(DMA1_Stream6);
  DMA_INIT.DMA_Channel            = DMA_Channel_7;  
  DMA_INIT.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R2_ADDR;
  DMA_INIT.DMA_Memory0BaseAddr    = (uint32_t)&sine200;
  DMA_INIT.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_INIT.DMA_BufferSize         = 3200;
  DMA_INIT.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_INIT.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_INIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_INIT.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_INIT.DMA_Mode               = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_INIT.DMA_Priority           = DMA_Priority_High;
  DMA_INIT.DMA_FIFOMode           = DMA_FIFOMode_Disable;         
  DMA_INIT.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_INIT.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_INIT.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_INIT);
  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC |DMA_IT_TE |DMA_IT_DME |DMA_IT_FE/*| DMA_IT_HT*/, ENABLE);
  
  
 
}
  
/**************************************************************************************/
  
void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = (84000000 / 100000) - 1; // 100 KHz, from 84 MHz TIM2CLK (ie APB1 = HCLK/4, TIM2CLK = HCLK/2)
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  /* TIM2 TRGO selection */
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO
  
  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);

  
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = (84000000 / 200000) - 1; // 100 KHz, from 84 MHz TIM2CLK (ie APB1 = HCLK/4, TIM2CLK = HCLK/2)
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  
  /* TIM6 TRGO selection */
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO
  
  /* TIM6 enable counter */
  TIM_Cmd(TIM6, ENABLE);
  
}
  
/**************************************************************************************/
  
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the DMA Stream IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}
  
/**************************************************************************************/












////////////////////////////////////
////////////////////////////////////
////////////////////////////////////
extern "C"{
////////////////////////////////////
////////////////////////////////////
////////////////////////////////////
	
	
	void DMA1_Stream6_IRQHandler(void) // Called at 1 KHz for 200 KHz sample rate, LED Toggles at 500 Hz
{
	
	LED2ON;
	
	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6))
	{
	/* Clear DMA Stream Transfer Complete interrupt pending bit */
		
	DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6 | DMA_IT_FEIF6 | DMA_IT_DMEIF6 | DMA_IT_TEIF6);
	}

	DMA1_Stream6->M0AR=(unsigned long) sine;
//  	DMA_Cmd(DMA1_Stream6, ENABLE);
	TIM6->ARR=sine_period;
	DMA1_Stream6->CR|=1;
	LED2OFF;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
	
	
	
	
void DMA2_Stream0_IRQHandler(void) // Called at 1 KHz for 200 KHz sample rate, LED Toggles at 500 Hz
{
	
	LEDON;
   int start_pos;
  /* Test on DMA Stream Half Transfer interrupt */
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
  {
    /* Clear DMA Stream Half Transfer interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
    start_pos=0;
  }
  
  /* Test on DMA Stream Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
  	start_pos=BUFFERSIZE/2;
  }

	LEDON;

	static float _average=2048;
	  /*static float _dc=0;
	static double _ac=0;
	static long long sum;*/
	
	uint16_t  *p;
	p=&ADCConvertedValues[start_pos];
	
	
	
	static int sign_static=0,count_static=0,avr_static=2048;
	int avr=0/*avr_static*/,sign=sign_static,count=count_static;
	
	static int j=0;
	
	
	static unsigned long long avr2_static;
	unsigned long long avr2=avr2_static;
	unsigned long RMS_SQUARE_SUM=0;
	
	int RAW1=0,RAW2=0,RAW3=0,RAW4=0,c;
	
	for(int i=10;i--;)   //1 Cylce
	{
		
		//get and average 10 * 5ch samples // assumptive 30.000 KSMP 
		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);


		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

		RAW1+=(*p++);
		RAW2+=(*p++);
		RAW3+=(*p++);
		RAW4+=(*p++);

	}
	
	
	
	

	RAWV1+=(((float)RAW1/100)-RAWV1)/4;
	RAWV2+=(((float)RAW2/100)-RAWV2)/4;
	RAWV3+=(((float)RAW3/100)-RAWV3)/4;
	RAWV4+=(((float)RAW4/100)-RAWV4)/4;
	
	
	TEMP=((RAWV2/4096)-(RAWV4/4096))*295;//2.95 V
	FL=((1-(RAWV3/4096))*100)-50;
	FG=((1-RAWV1/4096)*60)+0.1;
	float ff=fabs(TEMP-FL);
	freq=(ff*FG)+10;
	if(freq<10)freq=10;
	if(freq>9999)freq=9999;
	float f;
	if(freq<100)
	{
		//3200 step sine
		sine=sine3200;
		f=((84000000/3200)/freq);  // (84000000/12800)
		sine_period=(int)f;
	}
	else if(freq<200)
	{
		//1600 step sine
		sine=sine1600;
		f=((84000000/1600)/freq);
		sine_period=(int)f;
	}
	else if(freq<400)
	{
		//800 step sine
		sine=sine800;
		f=((84000000/800)/freq);
		sine_period=(int)f;
	}
	else if(freq<800)
	{
		//400 step sine
		sine=sine400;
		f=((84000000/400)/freq);
		sine_period=(int)f;
	}
	else if(freq<1600)
	{
		//200 step sine
		sine=sine200;
		f=((84000000/200)/freq);
		sine_period=(int)f;
	}
	else// if(freq<3200)
	{
		//100 step sine
		sine=sine100;
		f=((84000000/100)/freq);
		sine_period=(int)f;
	}	
	
	d_counter++;
	
	
	LEDOFF;
	

   
}
  

////////////////////////////////////
////////////////////////////////////
////////////////////////////////////
}//extern "C"
////////////////////////////////////
////////////////////////////////////
////////////////////////////////////
	
/**************************************************************************************/





/*
 struct TESLA_ANALOG_DAA{
	unsigned short AC;
	unsigned short DC;
	unsigned short ZERO;
	unsigned short CS;
};

static TESLA_ANALOG_DAA D;*/

unsigned short int D[10];
/*

if ( x < nmaxval )
{
	 nscaledrootmean +( ( navgcoeff * x ) nrootmean ) - ( navgcoeff * nrootmean ); 
} 
else 
{ 
	nscaledrootmean +nAvgCoeff * ( ( xnrootmean ) - nrootmean );
}
*/






void execute_command(char * str){
	///////////////////////////////////////////
	if(str[0]=='~' && str[1]=='Z' && str[2]==';' )   // zeroing
	{
	/*	CAL.offset[0]=RAWV1;
		CAL.offset[1]=RAWV2;
		CAL.offset[2]=RAWV3;
		CAL.offset[3]=RAWV4;
		CAL.offset[4]=RAWV5;*/
		
	}
	///////////////////////////////////////////
	if(str[0]=='~' && str[1]=='S' && str[2]==';' )   // saving
	{
		
		/*if(save_cal())
		{
			TM_USART_Puts(USART3, "~^d^o^n^E    ;");
		}
		else
		{
			TM_USART_Puts(USART3, "~^E^r^r^     ;");
		}*/
	}
	///////////////////////////////////////////
	if(str[0]=='~' && str[1]=='C' && str[2]==':' )   // zeroing
	{
		/*int input;
		float real_val;
		sscanf(str,"~C:%d;",&input);
		real_val=input;
		
		CAL.mul[4]=(RAWV5-CAL.offset[4])/real_val;
		CAL.mul[3]=(RAWV4-CAL.offset[3])/real_val;
		CAL.mul[2]=(RAWV3-CAL.offset[2])/real_val;
		CAL.mul[1]=(RAWV2-CAL.offset[1])/real_val;
		CAL.mul[0]=(RAWV1-CAL.offset[0])/real_val;*/
		
	}
	///////////////////////////////////////////
}







int main(void)
{
SystemInit();
  RCC_Configuration();
  GPIO_Configuration();

  for(int i=4;i--;){
	  LEDON;
	  LED4OFF;
	  Delay_ms(70);
	  LED2ON;
	  LEDOFF;
	  Delay_ms(70);
	  LED3ON;
	  LED2OFF;
	  Delay_ms(70);
	  LED4ON;
	  LED3OFF;
	  Delay_ms(70);
  }
  NVIC_Configuration();
  TIM2_Configuration();
  DMA_Configuration();
  ADC_Configuration();
  ADC_SoftwareStartConv(ADC1);
    // Initialize USART1 at 115200 baud, TX: PD8, RX: PD9 
  TM_USART_Init(USART3, TM_USART_PinsPack_3, 38400);
     // Initialize USART1 at 115200 baud, TX: PD8, RX: PD9 
 // TM_USART_Init(USART2, TM_USART_PinsPack_1, 115200);

 // TM_I2C_Init(I2C3, TM_I2C_PinsPack_1, 10000);
 // I2C_Configuration();
	double f=0;
	int k;
	
	for(f=0,k=0;f<6.2831853;f+=6.2831853/3200,k++)
	  sine100[k]=(int)((sin(f*32)+1)*2000);
	for(f=0,k=0;f<6.2831853;f+=6.2831853/3200,k++)
	  sine200[k]=(int)((sin(f*16)+1)*2000);
	for(f=0,k=0;f<6.2831853;f+=6.2831853/3200,k++)
	  sine400[k]=(int)((sin(f*8)+1)*2000);
	for(f=0,k=0;f<6.2831853;f+=6.2831853/3200,k++)
	  sine800[k]=(int)((sin(f*4)+1)*2000);
	for(f=0,k=0;f<6.2831853;f+=6.2831853/3200,k++)
	  sine1600[k]=(int)((sin(f*2)+1)*2000);
	for(f=0,k=0;f<6.2831853;f+=6.2831853/3200,k++)
	  sine3200[k]=(int)((sin(f)+1)*2000);
	
	
	
	static  char d=0, DISP_STR[40], * DS; 
	static unsigned int last_d_counter=0; 
	static char s[100];
	static char instr[100];
	char inlen=0;   
	char inchar;
  
  	DAC_Cmd(DAC_Channel_2, ENABLE);
	
	DMA_Cmd(DMA1_Stream6, ENABLE);		
	DAC_DMACmd(DAC_Channel_2, ENABLE);
 //  sprintf(s,";;~^L^o^a^d^-^-^-^-;");
 //  TM_USART_Puts(USART3, s); 
   Delay_ms(100);
   last_FL=FL;
   last_FG=FG;
   FL_change=0;
   FG_change=0;
   

	


  while(1)
  {
	

			 
	if(!uncallibrated){
	static int disp_ch=0,disp_ch_counter=0;static int disp_prescaler=10;
		if(d>0)
		{
			
			if(disp_prescaler<1)
			{	
				TM_USART_Putc(USART3, *DS++);
				d=d-1;
				disp_prescaler=2;
			}
		}
		else{
			d=0;
			disp_prescaler=0;
			if(FL_change)
			{
				sprintf(DISP_STR,"~SEt0%2.1fc ;",FL);
				d=strlen(DISP_STR);
				DS=DISP_STR;
				FL_change--;
			}
			else if(FG_change)
			{
				sprintf(DISP_STR,"~rAtE%2.2f  ;",FG);
				d=strlen(DISP_STR);
				DS=DISP_STR;
				FG_change--;
			}
			else
			{

				if(TEMP<=0)
					sprintf(DISP_STR,"~%03dc%2.1f ;",(int)TEMP,freq);
				else if(TEMP<10)
					sprintf(DISP_STR,"~%2.2fc%2.1f ;",TEMP,freq);
				else if(TEMP<100)
					sprintf(DISP_STR,"~%3.1fc%2.1f ;",TEMP,freq);
				else
					sprintf(DISP_STR,"~%3dc%2.1f ;",(unsigned int)TEMP,freq);
				d=strlen(DISP_STR);
				DS=DISP_STR;
			}
			
			if(fabs(last_FL-FL)>0.08){FL_change=30;last_FL=FL;}
			if(fabs(last_FG-FG)>0.3){FG_change=30;last_FG=FG;}
		}
		if(d_counter>=last_d_counter+1)
		{
			disp_prescaler--;
			disp_ch_counter++;
			last_d_counter=d_counter;
		}
  	}
	
	
  
  }

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 



#include "stm32f10x.h"   // Device header
#include "misc.h"
#include "string.h"
#include "stdio.h"

#define BLDC_CHOPPER_PERIOD 4500

#define BLDC_NOL 72

#define BLDC_ADC_START 200
#define BLDC_ADC_STOP 50
#define BLDC_ADC_MAX 4000

#define BLDC_STOP	0
#define BLDC_CW		1
#define BLDC_CCW	2

#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

uint8_t BLDC_MotorSpin = 0;
uint8_t BLDC_STATE[6] = {0,0,0,0,0,0};
uint8_t BLDC_STATE_PREV[6] = {0,0,0,0,0,0};

volatile uint16_t ADCBuffer[] = {0xAAAA};

static const uint8_t BLDC_BRIDGE_STATE_FORWARD[8][6] =   
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  
   { 0,1	,	0,0	,	1,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,0	,	0,1	,	1,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	1,0	,	0,0 },
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,0	,	0,0 },  
};

static const uint8_t BLDC_BRIDGE_STATE_BACKWARD[8][6] =   
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  
   { 1,0	,	0,0	,	0,1 },
   { 0,1	,	1,0	,	0,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	0,0	,	0,0 }, 
};
void BLDC_Init(void);
void BLDC_HallSensorsInit(void);
void BLDC_PWMTimerInit(void);
uint8_t BLDC_HallSensorsGetPosition(void);
void BLDC_MotorSetSpin(uint8_t spin);
uint8_t BLDC_MotorGetSpin(void);
void BLDC_MotorStop(void);
void BLDC_MotorCommutation(uint16_t hallpos);
uint16_t BLDC_ADCToPWM(uint16_t ADC_VALUE);
void BLDC_SetPWM(uint16_t PWM);
void ADC_DMA_init(void);

void EXTI9_5_IRQHandler(void) {
    if ((EXTI_GetITStatus(EXTI_Line7) | EXTI_GetITStatus(EXTI_Line8) | EXTI_GetITStatus(EXTI_Line9)) != RESET) {
    	
    	EXTI_ClearITPendingBit(EXTI_Line7);
    	EXTI_ClearITPendingBit(EXTI_Line8);
    	EXTI_ClearITPendingBit(EXTI_Line9);
        
			BLDC_MotorCommutation(BLDC_HallSensorsGetPosition());
    }
}

int main(void)
{

	ADC_DMA_init();

	BLDC_Init();

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_0);

    while(1)
    {
    	if (ADCBuffer[0] > BLDC_ADC_START) {
    		if (BLDC_MotorGetSpin() == BLDC_STOP) {
    		
    			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) != 0) {
    				
    				BLDC_MotorSetSpin(BLDC_CW);
    			}
    			else {
    			
    				BLDC_MotorSetSpin(BLDC_CCW);
    			}
    			BLDC_MotorCommutation(BLDC_HallSensorsGetPosition());
    		}
    		BLDC_SetPWM(BLDC_ADCToPWM(ADCBuffer[0]));
    	}
    	else {
    		if (BLDC_MotorGetSpin() != BLDC_STOP) {
    			if (ADCBuffer[0] < BLDC_ADC_STOP) {
    				BLDC_MotorStop();
    			}
    		}
    	}
    }
}


void BLDC_Init(void) {
	BLDC_HallSensorsInit();
	BLDC_PWMTimerInit();
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	BLDC_MotorStop();
}

void BLDC_HallSensorsInit(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

    EXTI_InitStruct.EXTI_Line = EXTI_Line7 | EXTI_Line8 | EXTI_Line9;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_Init(&EXTI_InitStruct);
}

void BLDC_PWMTimerInit(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	//initialize Tim1 PWM outputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = BLDC_CHOPPER_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2, 3 – set to PWM mode - all 6 outputs
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // initialize to zero output

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; /// !!!
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; /// !!!

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;

	TIM_BDTRInitStructure.TIM_DeadTime = BLDC_NOL;

	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

uint8_t BLDC_HallSensorsGetPosition(void) {
	return (uint8_t) ((GPIO_ReadInputData(GPIOB) & (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)) >> 7);
}

void BLDC_MotorSetSpin(uint8_t spin) {
	BLDC_MotorSpin = spin;
}
uint8_t BLDC_MotorGetSpin(void) {
	return BLDC_MotorSpin;
}

void BLDC_MotorStop(void)
{
	BLDC_SetPWM(0);

	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	BLDC_MotorSpin = 0;

		memset(BLDC_STATE_PREV, 0, sizeof(BLDC_STATE_PREV));
	
}

void BLDC_MotorCommutation(uint16_t hallpos)
{
	if (BLDC_MotorSpin == BLDC_CW) {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_FORWARD[hallpos], sizeof(BLDC_STATE));
	}
	else {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_BACKWARD[hallpos], sizeof(BLDC_STATE));
	}

	if (!BLDC_STATE[UH]) TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	if (!BLDC_STATE[UL]) TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	if (!BLDC_STATE[VH]) TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	if (!BLDC_STATE[VL]) TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	if (!BLDC_STATE[WH]) TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	if (!BLDC_STATE[WL]) TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	if (BLDC_STATE[UH] & !BLDC_STATE[UL] & !BLDC_STATE_PREV[UH]) TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	if (BLDC_STATE[UL] & !BLDC_STATE[UH] & !BLDC_STATE_PREV[UL]) TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	if (BLDC_STATE[VH] & !BLDC_STATE[VL] & !BLDC_STATE_PREV[VH]) TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	if (BLDC_STATE[VL] & !BLDC_STATE[VH] & !BLDC_STATE_PREV[VL]) TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	if (BLDC_STATE[WH] & !BLDC_STATE[WL] & !BLDC_STATE_PREV[WH]) TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	if (BLDC_STATE[WL] & !BLDC_STATE[WH] & !BLDC_STATE_PREV[WL]) TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

	memcpy(BLDC_STATE_PREV, BLDC_STATE, sizeof(BLDC_STATE));
}

uint16_t BLDC_ADCToPWM(uint16_t ADC_VALUE) {
	uint32_t tmp;

	if (ADC_VALUE < BLDC_ADC_STOP) {
		return 0;
	}

	if (ADC_VALUE > BLDC_ADC_MAX) {
		return BLDC_CHOPPER_PERIOD+1;
	}

	tmp = (uint32_t)(ADC_VALUE-BLDC_ADC_STOP) * (uint32_t)BLDC_CHOPPER_PERIOD / (uint32_t)(BLDC_ADC_MAX - BLDC_ADC_START);

	return (uint16_t) tmp;

}

void BLDC_SetPWM(uint16_t PWM)
{
	TIM1->CCR1 = PWM;
	TIM1->CCR2 = PWM;
	TIM1->CCR3 = PWM;
}

void ADC_DMA_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
    /* Enable ADC1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE );

	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCBuffer;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1 , ENABLE ) ;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5);
	ADC_Cmd(ADC1 , ENABLE ) ;
	ADC_DMACmd(ADC1 , ENABLE ) ;
	ADC_ResetCalibration(ADC1);

	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);

	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd ( ADC1 , ENABLE ) ;
}

#include "stm32f10x.h"
#include <math.h>

#define Supply_Voltage 3285
#define Resolution_Value 4096 
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

#define CATHODE1 GPIO_Pin_3
#define CATHODE2 GPIO_Pin_14
#define CATHODE3 GPIO_Pin_13
#define CATHODE4 GPIO_Pin_12
#define CATHODES_OFF (CATHODE1 | CATHODE2 | CATHODE3 | CATHODE4)

#define SWAP_BUTTON GPIO_Pin_5

#define A GPIO_Pin_4
#define B GPIO_Pin_5
#define C GPIO_Pin_8
#define D GPIO_Pin_7
#define E GPIO_Pin_6
#define F GPIO_Pin_3
#define G GPIO_Pin_2 
#define DP GPIO_Pin_9

#define D0 (A | B | C | D | E | F)
#define D1 (B | C)
#define D2 (A | B | G | E | D)
#define D3 (A | B | C | D | G)
#define D4 (F | G | B | C)
#define D5 (A | F | G | C | D)
#define D6 (A | F | G | C | E | D)
#define D7 (A | B | C)
#define D8 (A | B | C | D | E | F | G)
#define D9 (A | B | C | D | F | G)

#define COUNT_FILTER	64
#define DIVCOEF	53.3/33

typedef union
{
    unsigned int Val;
    struct
    {
        unsigned Flag:1;        // flag of filling of summ
        unsigned Index:9;       // buffer index
        unsigned Filter_sum:23; // summ of filter's value
    } Reg;
} FILTER_REG;
//*******************************************************************************
// declared variables
//*******************************************************************************
uint16_t digits[] = {D0, D1, D2, D3, D4, D5, D6, D7, D8, D9};
uint32_t AD_value;
uint16_t ADC1_voltage, ADC2_voltage, displayedValue;
uint16_t buffADC1[COUNT_FILTER], buffADC2[COUNT_FILTER];
uint8_t changeDisplayedValue = 1, buttonCurrState, buttonPrevState;
FILTER_REG filterADC1, filterADC2;
//*******************************************************************************
// function prototypes
//*******************************************************************************
void delay(void);
void SetNumber(uint32_t number);
void Initialization_ADC_DMA_GPIO(void);
void checkButtonState(void);
float roundf (float x);
uint16_t filter_average(uint16_t ADC_val, uint16_t* buf, FILTER_REG* filter_reg);
uint16_t calibrationFnx (uint16_t calibratedValue);
//*******************************************************************************
// main function
//*******************************************************************************
int main() {
	
	Initialization_ADC_DMA_GPIO();
	
	buttonCurrState = buttonPrevState = GPIO_ReadInputDataBit(GPIOB,SWAP_BUTTON);
	
	while(1) {
		
		ADC1_voltage = ((filter_average((uint16_t)(AD_value), buffADC1, &filterADC1)) * Supply_Voltage / Resolution_Value);
		ADC2_voltage = ((filter_average((uint16_t)(AD_value >> 16), buffADC2, &filterADC2)) * Supply_Voltage / Resolution_Value);
		
		if (changeDisplayedValue) {
			displayedValue = (ADC1_voltage / 2) + calibrationFnx(ADC1_voltage / 2);
		} else displayedValue = (uint16_t)(roundf(ADC2_voltage * DIVCOEF)); 

		SetNumber(displayedValue);
		
		checkButtonState();
	}
}
//*******************************************************************************
// function for calibration
// second-order polynimial approximation is used for calibrating ADC's values
//*******************************************************************************
uint16_t calibrationFnx (uint16_t calibratedValue) {
	
	float val = (float)calibratedValue / 1000;

	return (uint16_t)(((-0.003) * pow(val, 2) + 0.0282 * val + 0.0029) * 1000 + 3);
}
//*******************************************************************************
// function for changing displayed value between ADC1 value and ADC2 value
// also function implements bounce effect
//*******************************************************************************
void checkButtonState() {
	
	buttonCurrState = GPIO_ReadInputDataBit(GPIOB,SWAP_BUTTON);
		
	if (buttonCurrState != buttonPrevState) {
		if (buttonCurrState == 0) {
			if (changeDisplayedValue) {
				changeDisplayedValue = 0;
			} else changeDisplayedValue = 1;
		}
	}
		
	buttonPrevState = buttonCurrState;
}
//*******************************************************************************
// function for moving average filter
// ADC_val - value of ADC1;
// buf - buffer for different ADC values;
// filter_reg - structure for initialization MAF and saving results of filtering
//*******************************************************************************
uint16_t  filter_average(uint16_t ADC_val, uint16_t* buf, FILTER_REG* filter_reg){
	
    if (filter_reg -> Reg.Flag){
        filter_reg -> Reg.Filter_sum -= buf[filter_reg -> Reg.Index];
        filter_reg -> Reg.Filter_sum += ADC_val;
        buf[filter_reg -> Reg.Index] = ADC_val;
        if (filter_reg -> Reg.Index >= COUNT_FILTER - 1){
            filter_reg -> Reg.Index = 0;
        }
        else{
            filter_reg -> Reg.Index++;
        }
    }
    else{
        filter_reg -> Reg.Filter_sum += ADC_val;
        buf[filter_reg -> Reg.Index] = ADC_val;
        if (filter_reg -> Reg.Index >= COUNT_FILTER - 1){
            filter_reg -> Reg.Index = 0;
            filter_reg -> Reg.Flag = 1;
        }
        else{
            filter_reg -> Reg.Index++;
        }
    }
    return (filter_reg -> Reg.Filter_sum / COUNT_FILTER);
}
//*******************************************************************************
// delay for dynamic indication
// displayed number at seven-segment dislay powered for the delay period 
//*******************************************************************************
void delay() {
	__IO uint32_t i;
	for( i = 0; i < 5000; i++);
}
//*******************************************************************************
// function for setting different numbers at 4 seven-segment displays
// number - value needed to be displayed
//*******************************************************************************
void SetNumber(uint32_t number) {
		
	if (changeDisplayedValue) {
		GPIO_Write(GPIOB, CATHODE1);
		GPIO_Write(GPIOA, digits[number / 1000] | DP);
		delay();
	} else {
		GPIO_Write(GPIOB, CATHODE1);
		GPIO_Write(GPIOA, digits[number / 1000]);
		delay();
	}
		
	if (changeDisplayedValue) {
		GPIO_Write(GPIOB, CATHODE2);
		GPIO_Write(GPIOA, digits[number % 1000 / 100]);
		delay();
	} else {
		GPIO_Write(GPIOB, CATHODE2);
		GPIO_Write(GPIOA, digits[number % 1000 / 100] | DP);
		delay();
	}
	
	GPIO_Write(GPIOB, CATHODE2);
	GPIO_Write(GPIOA, digits[number % 1000 / 100]);
	delay();

	GPIO_Write(GPIOB, CATHODE3);
	GPIO_Write(GPIOA, digits[number % 100 / 10]);
	delay();

	GPIO_Write(GPIOB, CATHODE4);
	if (number >= 10) {
		GPIO_Write(GPIOA, digits[number % 10]);
	} else {
		GPIO_Write(GPIOA, digits[number]);
	}
	delay();
}
//*******************************************************************************
// Initialization of all used peripherals
//*******************************************************************************
void Initialization_ADC_DMA_GPIO(){
	
	GPIO_InitTypeDef GPIOB_InitStructure;
	GPIO_InitTypeDef GPIOA_InitStructure;
	GPIO_InitTypeDef GPIOADC_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	
  RCC_APB2PeriphClockCmd(	RCC_APB2Periph_AFIO |
													RCC_APB2Periph_ADC1 |
													RCC_APB2Periph_ADC2 |
													RCC_APB2Periph_GPIOB |
													RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	GPIOADC_InitStructure.GPIO_Pin = 		GPIO_Pin_0;
  GPIOADC_InitStructure.GPIO_Mode = 	GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIOADC_InitStructure);
	
	GPIOB_InitStructure.GPIO_Pin = 			GPIO_Pin_5;
  GPIOB_InitStructure.GPIO_Mode = 		GPIO_Mode_IPU;
	GPIOB_InitStructure.GPIO_Speed =		GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOB, &GPIOB_InitStructure);

  GPIOB_InitStructure.GPIO_Pin = 			GPIO_Pin_3 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIOB_InitStructure.GPIO_Mode = 		GPIO_Mode_Out_PP;
	GPIOB_InitStructure.GPIO_Speed =		GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOB, &GPIOB_InitStructure);	

	GPIOA_InitStructure.GPIO_Pin = 			GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIOA_InitStructure.GPIO_Mode = 		GPIO_Mode_Out_PP;
	GPIOA_InitStructure.GPIO_Speed =		GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);	

	GPIO_ResetBits(GPIOB, CATHODES_OFF);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
  /* DMA1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = 	ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = 			(uint32_t)&AD_value;
  DMA_InitStructure.DMA_DIR = 								DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 					1;
  DMA_InitStructure.DMA_PeripheralInc = 			DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = 					DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize =	DMA_PeripheralDataSize_Word;;
  DMA_InitStructure.DMA_MemoryDataSize = 			DMA_MemoryDataSize_Word;;
  DMA_InitStructure.DMA_Mode = 								DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = 						DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = 								DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_Cmd(DMA1_Channel1, ENABLE);
	
  /* ADC1 configuration */
  ADC_InitStructure.ADC_Mode = 								ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = 				DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = 	ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = 		ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = 					ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel =	 			1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	
	ADC_DMACmd(ADC1, ENABLE);
	
  /* ADC2 configuration */
	ADC_InitStructure.ADC_Mode = 								ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = 				DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = 	ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = 		ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = 					ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 				1;
  ADC_Init(ADC2, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);

  ADC_ExternalTrigConvCmd(ADC2, ENABLE); // ADC2 starts simultaniously with ADC1 right after ADC1 start
	
	/* ADC1 calibrations */
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));

	/* ADC2 calibrations */
  ADC_Cmd(ADC2, ENABLE);
  ADC_ResetCalibration(ADC2);
  while(ADC_GetResetCalibrationStatus(ADC2));
  ADC_StartCalibration(ADC2);
  while(ADC_GetCalibrationStatus(ADC2));

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
  DMA_ClearFlag(DMA1_FLAG_TC1);
}

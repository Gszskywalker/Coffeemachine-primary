
#include "stm32f4xx.h"
#include "stdbool.h"
#include "main.h"

GPIO_InitTypeDef GPIO_Initstructure;
TIM_TimeBaseInitTypeDef timer_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

bool valid_double_click = false;
bool button_clicked = false;
bool single_click = false;
bool double_click = false;
bool long_click = false;
bool button_up = true;
bool program_on = false;
bool display_default_timer = false;
bool programming_state = false;
bool idle = false;
bool blink_done = false;
fir_8 filt;
bool output_sound = false;
int timer_for_sound = 0;
bool start_sound_timer = false;

unsigned int button_hold_time = 0;
unsigned int button_release_time = 0;
unsigned int idle_time = 0;
uint16_t display_LED = 0;
int num_blink = 0;
int coffee_size = 0;
uint16_t time_small = 3;
uint16_t time_medium = 5;
uint16_t time_ex_large = 8;
uint16_t num_click = 0;

void InitLEDs(){
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_Initstructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Initstructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, & GPIO_Initstructure);
}

void LEDOn(uint16_t GPIO_Pin){
  GPIO_SetBits(GPIOD, GPIO_Pin);
}

void LEDOff(uint16_t GPIO_Pin){
  GPIO_ResetBits(GPIOD, GPIO_Pin);
}

void InitTimers(){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  timer_InitStructure.TIM_Prescaler = 232;
  timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timer_InitStructure.TIM_Period = 2999;
  timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timer_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, & timer_InitStructure);
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	timer_InitStructure.TIM_Prescaler = 2100 - 1;
	timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timer_InitStructure.TIM_Period = 10000 - 1;
	timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timer_InitStructure);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	timer_InitStructure.TIM_Prescaler = 280 - 1;
	timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timer_InitStructure.TIM_Period = 10000 - 1;
	timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timer_InitStructure);
	TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

void InitButton(){
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, & GPIO_Initstructure);
}

void EnableTimersInterrupt(){
  NVIC_InitTypeDef nvicStructure2;
  nvicStructure2.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure2.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure2.NVIC_IRQChannelSubPriority = 1;
  nvicStructure2.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( & nvicStructure2);
	
	NVIC_InitTypeDef nvicStructure3;
	nvicStructure3.NVIC_IRQChannel = TIM3_IRQn;
	nvicStructure3.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure3.NVIC_IRQChannelSubPriority = 1;
	nvicStructure3.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure3);
}

void EnableTimer4Interrupt(){
	NVIC_InitTypeDef nvicStructure4;
	nvicStructure4.NVIC_IRQChannel = TIM4_IRQn;
	nvicStructure4.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure4.NVIC_IRQChannelSubPriority = 1;
	nvicStructure4.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure4);
}

void DisableTimer4Interrupt(){
	NVIC_InitTypeDef nvicStructure4;
	nvicStructure4.NVIC_IRQChannel = TIM4_IRQn;
	nvicStructure4.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure4.NVIC_IRQChannelSubPriority = 1;
	nvicStructure4.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&nvicStructure4);
}

void EnableEXTIInterrupt() {
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( & NVIC_InitStructure);
}

void InitEXTI(){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_Init( & EXTI_InitStructure);
}

void TIM2_IRQHandler(){
	
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		button_hold_time++;
    button_release_time++;
	
    if (!button_up && button_hold_time >= 3 * 120) {
			button_hold_time = 0;
			idle_time = 0;
      long_click = true;
			idle = false;
    }
		
		if ( !idle && program_on ) {
			idle_time++;
			if ( idle_time > 5 * 120 ) {
				if ( programming_state ) {
					programming_state = false;
				} else {
					idle = true;
					blink_done = false;
				}
				idle_time = 0;
			}
		}
	
		if (start_sound_timer) {
			timer_for_sound++;
			output_sound = timer_for_sound <= 0.5 * 120;
		}
		
  }
}

void TIM3_IRQHandler(){
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		
		if ( num_blink && display_LED) {
			GPIO_ToggleBits(GPIOD, display_LED);
			num_blink--;
			idle_time = 0;
		}
		if ( !num_blink ) blink_done = true;
	}
}

void TIM4_IRQHandler(){
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		if( idle ) {
			if ( num_blink <= 0 && !blink_done ) {
				if ( coffee_size == 0 ) {
					LEDOn(GPIO_Pin_13);
					LEDOff(GPIO_Pin_14);
					LEDOff(GPIO_Pin_15);
					num_blink = time_small * 2;
				}
				else if ( coffee_size == 1 ) {
					LEDOn(GPIO_Pin_14);
					LEDOff(GPIO_Pin_13);
					LEDOff(GPIO_Pin_15);
					num_blink = time_medium * 2;
				}
				else if ( coffee_size == 2 ) {
					LEDOn(GPIO_Pin_15);
					LEDOff(GPIO_Pin_14);
					LEDOff(GPIO_Pin_13);
					num_blink = time_ex_large * 2;
				} 
				display_LED = GPIO_Pin_12;
			}else {
				if ( !num_blink ) {
					LEDOff(GPIO_Pin_14);
					LEDOff(GPIO_Pin_13);
					LEDOff(GPIO_Pin_15);
					start_sound_timer = true;
				}
			}
		}
	}
}

bool validClickState(){
	return (!long_click && program_on && !button_up && button_hold_time >= 0.05 * 120);
}

void EXTI0_IRQHandler(){

  if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
      if (button_up) {
        button_hold_time = 0;
      }
      button_up = false;
    } else {
      if (validClickState()) {
				if (!single_click && !double_click) {
					single_click = true;
				} else if (single_click && !double_click) {
					single_click = false;
					double_click = true;
				}
      }
      button_up = true;
      button_release_time = 0;
    }
		
		idle_time = 0;
		
		if ( idle ) {
			idle = false;
			coffee_size = -1;
		}
		
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

void newCoffeeTime(){
	if (coffee_size == 0)
		time_small = num_click;
	else if (coffee_size == 1) 
		time_medium = num_click;
	else if (coffee_size == 2) 
		time_ex_large = num_click;
}

void newProgrammingStatus(){
	
	if (display_default_timer && num_blink <= 0) {
		
		valid_double_click = (button_release_time <= (0.5 * 120));
		
		if (single_click && !valid_double_click) {
			num_click++;
			single_click = false;
		} else if(long_click) {
			
			if ( num_click > 0) {
				newCoffeeTime();
				num_click = 0;
				programming_state = false;
			}
			long_click = false;
			double_click = false;
			single_click = false;
			button_up = true;
		} else if(double_click)  {
			num_click+=2;
			double_click = false;
		}
	} else {
		if (single_click) single_click = false;
		if (long_click) long_click = false;
		if (double_click) double_click = false;
	}
}

void newNonProgrammingStatus(){
	valid_double_click = (button_release_time <= (0.5 * 120));
	if (single_click && !valid_double_click) {
		coffee_size = (coffee_size + 1) % 3;
		single_click = false;
	} else if (double_click) {
		num_click = 0;
		idle_time = 0;
		programming_state = true;
		display_default_timer = false;
		double_click = false;
	} else if (long_click)
		long_click = false;
}

void UpdateMachine(){
  if (program_on) {
    if (programming_state) {
			newProgrammingStatus();
    } else
			newNonProgrammingStatus();
  } else if (long_click && !program_on) {
    long_click = false;
    program_on = true;
    button_up = true;
  }
}

void DisplayLED(){
	if ( coffee_size == 0) {
		display_LED = GPIO_Pin_13;
		num_blink = time_small * 2;
	}
	else if (coffee_size == 1) {
		display_LED = GPIO_Pin_14;
		num_blink = time_medium * 2;
	}
	else if (coffee_size == 2) {
		display_LED = GPIO_Pin_15;
		num_blink = time_ex_large * 2;
	}
	display_default_timer = true;
}

void ShowProgramLED(){
	if (!display_default_timer)
		DisplayLED();
	
	
	if ( num_blink <= 0 ) { 
		LEDOn(GPIO_Pin_12);
		if ( button_up ) LEDOn(display_LED);
		else LEDOff(display_LED);
	}
}

void ShowSelectionLED(){
	LEDOn(GPIO_Pin_12);
	if (coffee_size == 0) {
		LEDOn(GPIO_Pin_13);
		LEDOff(GPIO_Pin_15);
		LEDOff(GPIO_Pin_14);
	} else if (coffee_size == 1) {
		LEDOn(GPIO_Pin_14);
		LEDOff(GPIO_Pin_13);
		LEDOff(GPIO_Pin_15);
	} else if (coffee_size == 2) {
		LEDOn(GPIO_Pin_15);
		LEDOff(GPIO_Pin_14);
		LEDOff(GPIO_Pin_13);
	}
}	

void ShowLED(){
	if (programming_state) {
		ShowProgramLED();
	} else
		ShowSelectionLED();
}

int main(){
	InitLEDs();
	InitButton();
	InitEXTI();
	InitTimers();
	EnableTimersInterrupt();
	EnableEXTIInterrupt();

	SystemInit();
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	codec_init();
	codec_ctrl_init();

	I2S_Cmd(CODEC_I2S, ENABLE);

	initFilter(&filt);
		
	while(1) {
		UpdateMachine();
		if (program_on)  {
			if ( !idle )	{
				DisableTimer4Interrupt();
				ShowLED();
				output_sound = false;
				timer_for_sound = 0;
				start_sound_timer = false;
			} else	{
				EnableTimer4Interrupt();
			}
			
			if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE) && output_sound)
    	{
    		SPI_I2S_SendData(CODEC_I2S, sample);

    		if (sampleCounter & 0x00000001)
    		{
    			sawWave += NOTEFREQUENCY;
    			if (sawWave > 1.0)
    				sawWave -= 2.0;

    			filteredSaw = updateFilter(&filt, sawWave);
    			sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
    		}
    		sampleCounter++;
    	}
		}
	}
}

float updateFilter(fir_8* filt, float val){
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter){
	uint8_t i;
	theFilter->currIndex = 0;
	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;
	theFilter->params[0] = 0.01;
	theFilter->params[1] = 0.05;
	theFilter->params[2] = 0.12;
	theFilter->params[3] = 0.32;
	theFilter->params[4] = 0.32;
	theFilter->params[5] = 0.12;
	theFilter->params[6] = 0.05;
	theFilter->params[7] = 0.01;
}

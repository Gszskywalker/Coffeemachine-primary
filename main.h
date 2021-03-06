//Includes
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "codec.h"

//Defines
#define LED_BLUE_ON   GPIOD->BSRRL = GPIO_Pin_15;
#define LED_BLUE_OFF  GPIOD->BSRRH = GPIO_Pin_15;

#define NOTEFREQUENCY 0.1
#define NOTEAMPLITUDE 100.0	


typedef struct {
	float tabs[8];
	float params[8];
	uint8_t currIndex;
} fir_8;


// struct to initialize GPIO pins
GPIO_InitTypeDef GPIO_InitStructure;

volatile uint32_t sampleCounter = 0;
volatile int16_t sample = 0;

double sawWave = 0.0;

float filteredSaw = 0.0;

float updateFilter(fir_8* theFilter, float newValue);

void initFilter(fir_8* theFilter);


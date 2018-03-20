#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_spi.h"
#include "LPC17xx.h"
#include "core_cm3.h"

#include "oled.h"
#include "rgb.h"
#include "temp.h"
#include "led7seg.h"

volatile uint32_t msTicks;
volatile uint8_t mode;

#define PRESCALE (25000-1)

void init_Timer0(void)
{
	/*Assuming that PLL0 has been setup with CCLK = 100Mhz and PCLK = 25Mhz.*/
	LPC_SC->PCONP |= (1<<1); //Power up TIM0. By default TIM0 and TIM1 are enabled.
	LPC_SC->PCLKSEL0 &= ~(0x3<<3); //Set PCLK for timer = CCLK/4 = 100/4 (default)

	LPC_TIM0->CTCR = 0x0;
	LPC_TIM0->PR = PRESCALE; //Increment TC at every 24999+1 clock cycles
	//25000 clock cycles @25Mhz = 1 mS

	LPC_TIM0->TCR = 0x02; //Reset Timer
}

static void init_GPIO(void)
{

	// Initialize button SW4 and SW3
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);

	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);

	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 6;
	PINSEL_ConfigPin(&PinCfg);
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

void SysTick_Handler(void){
	msTicks++;
}

void EINT3_IRQHandler(void){
	if((LPC_GPIOINT->IO2IntStatR>>10) & 0x1){
		//print statements in interrupts cause a lot of overhead, use flags instead to print statements
		printf("Button right pressed.\n");
		mode = 0x01;
		//clears the interrupt (have to do this or the interrupt will loop infinitely
		LPC_GPIOINT->IO2IntClr = 1<<10;
	}
}

void delayMS(unsigned int milliseconds) //Using Timer0
{
	LPC_TIM0->TCR = 0x02; //Reset Timer

	LPC_TIM0->TCR = 0x01; //Enable timer

	while(LPC_TIM0->TC < milliseconds); //wait until timer counter reaches the desired delay

	LPC_TIM0->TCR = 0x00; //Disable timer
}

uint32_t getTicks(void){
	return msTicks;
}

int main (void) {

	SysTick_Config(SystemCoreClock/1000);

    init_GPIO();
    init_i2c();
    init_ssp();
	init_Timer0();

    rgb_init();
    oled_init();
    led7seg_init();
	temp_init(getTicks);

	LPC_GPIOINT->IO2IntEnR |= 1<<10;
	NVIC_EnableIRQ(EINT3_IRQn);

	mode = 0; //init as STATIONARY MODE
	char* stationaryStrPtr;
	char* stationaryTempPtr[50]={};
	uint32_t temp_value;
	uint32_t stationary_counter = 15;
	uint8_t sseg_chars[] = {
	        /* digits 0 - 9 */
	        0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x22, 0x7C, 0x20, 0x38,
	        /* A to F */
	        0x28, 0x23, 0xA6, 0x61, 0xA2, 0xAA
	};

	//stationaryTempPtr = "Temp:";
	stationaryStrPtr = "STATIONARY";
	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(20, 20, (unsigned char*)stationaryStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(20, 28, (unsigned char*)stationaryTempPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	led7seg_setChar(sseg_chars[stationary_counter], TRUE);

    while (1)
    {
    	/* <---STATIONARY MODE---> */
    	if(mode == 0x00){
    		temp_value = temp_read();
    		sprintf(stationaryTempPtr, "Temp: %2.2f", temp_value/10.0);
    		oled_putString(20, 28, (unsigned char*)stationaryTempPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    		printf("checking temp \n");
    	}
    	/* <---STATIONARY TO LAUNCH---> */
    	else if(mode == 0x01){
    		if(stationary_counter == 0){
    			mode = 0x02;
    		} else {
    			temp_value = temp_read();
    			sprintf(stationaryTempPtr, "Temp: %2.2f", temp_value/10.0);
    			oled_putString(20, 28, (unsigned char*)stationaryTempPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    			printf("checking temp \n");

    			stationary_counter = stationary_counter - 1;
    			led7seg_setChar(sseg_chars[stationary_counter], TRUE);

    	    	delayMS(1000);
    		}
    	}
    }
}

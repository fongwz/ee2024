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
#include "acc.h"
#include "pca9532.h"

volatile uint32_t msTicks;
volatile uint32_t recordTicks;

uint8_t mode;
char* modeStrPtr;
char* stateStrPtr;
uint8_t toggle_count_flag = 0;
uint8_t sw4btn;

uint32_t tcount = 0;
uint32_t t1 = 0;
uint32_t t2 = 0;
uint32_t temp_value = 0;
char* tempStrPtr[50]={};

uint8_t temp_state;

int8_t acc_warning_flag = 0;
int8_t xoff;
int8_t yoff;
int8_t x;
int8_t y;
int8_t z;
char* accx[40];
char* accy[40];
char* accz[40];

volatile uint32_t count;

#define PRESCALE (25000-1)
#define TEMP_HIGH_THRESHOLD 39.0
#define OBSTACLE_NEAR_THRESHOLD 3000
#define ACC_THRESHOLD 0.4

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

void check_clearWarning(){
	sw4btn = (GPIO_ReadValue(1) >> 31) & 0x01;
	if(sw4btn == 0){
		if(acc_warning_flag == 1){
			acc_warning_flag = 0;
			oled_clearScreen(OLED_COLOR_BLACK);
		}
	} else {
		return;
	}
}

void blink_blue(){
    GPIO_SetValue( 0, (1<<26) );
    check_clearWarning();
    delayMS(333);
    GPIO_ClearValue( 0, (1<<26) );
    check_clearWarning();
    delayMS(333);
}

void ACCELEROMETER(){
	x=0;
	y=0;
	//reading accelerometer
	acc_read(&x, &y, &z);
	x=x+xoff;
	y=y+yoff;
	//need values in terms of g, according to acc.h, g level is set to default 2g
	//divide value read by accelerometer by 64, according to datasheet
	sprintf(accx, "X:%3.1f", x/64.0);
	sprintf(accy, "Y:%3.1f", y/64.0);
	printf("X:%2.2f  Y:%2.2f \n",x/64.0,y/64.0);
	//check if accelerometer in g exceed threshold value
	if(x/64.0 >= 0.4 || y/64.0 >= 0.4){
		printf("acc exceed\n");
		acc_warning_flag = 1;
		oled_clearScreen(OLED_COLOR_BLACK);
	} else {
		oled_putString(10, 38, (unsigned char*)accx, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(55, 38, (unsigned char*)accy, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
}

void temp_sensor(){
	if(tcount==0){
		temp_state = ((GPIO_ReadValue(0) & (1 << 2)) != 0);
		t1 = getTicks();
	}

	if(((GPIO_ReadValue(0) & (1 << 2)) != 0) == temp_state){
		while(((GPIO_ReadValue(0) & (1 << 2)) != 0) == temp_state);

	}

	tcount = tcount+1;
	temp_state = !temp_state;

	if(tcount==170){
		t2 = getTicks();
		if(t2>t1){
			t2 = (t2-t1)+1;
			//printf("difference is %d\n",t2/170);
		} else {
			t2 = (t2 = (0xFFFFFFFF - t1 + 1) + t2);
		}
		tcount = 0;
		temp_value = (((1000*t2)/(160)-2731));
	}
}

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
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, 1<<2, 0);
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
		/* if in STATIONARY mode, go to STATIONARY-TO-LAUNCH mode */
		if(mode == 0x00){
			mode = 0x01;
			LPC_GPIOINT->IO2IntClr = 1<<10;
		}

		/* if in LAUNCH mode, check number of TOGGLE_MODE(SW3) button presses */
		else if(mode == 0x02 && toggle_count_flag==0){
			toggle_count_flag = 0x01;
			LPC_GPIOINT->IO2IntClr = 1<<10;
		}
		else if(mode == 0x02 && toggle_count_flag==0x01){
			toggle_count_flag = 0x02;
			LPC_GPIOINT->IO2IntClr = 1<<10;
		}

		/* if in RETURN mode, go back to STATIONARY mode */
		else if(mode == 0x03){
			mode = 0x00;
		}
		else {
			LPC_GPIOINT->IO2IntClr = 1<<10;
		}
	}
	else if ((LPC_GPIOINT->IO0IntStatR)>>3 & 0x01){
		printf("3.1f\n", x/1.0);
		LPC_GPIOINT->IO0IntClr = 1<<3;

	}
}

int main (void) {

	SysTick_Config(SystemCoreClock/1000);

    init_GPIO();
    init_i2c();
    init_ssp();
	init_Timer0();

	pca9532_init(); //led_array
	acc_init();
    rgb_init();
    oled_init();
    led7seg_init();
	temp_init(getTicks);

	LPC_GPIOINT->IO2IntEnR |= 1<<10;
	//LPC_GPIOINT->IO0IntEnF |= 1<<2;
	LPC_GPIOINT->IO0IntEnR |= 1<<3;
	NVIC_EnableIRQ(EINT3_IRQn);

	sw4btn = 1;
	mode = 0; //init as STATIONARY MODE
	uint32_t stationary_counter = 15;
	uint8_t sseg_chars[] = {
	        /* digits 0 - 9 */
	        0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x22, 0x7C, 0x20, 0x38,
	        /* A to F */
	        0x28, 0x23, 0xA6, 0x61, 0xA2, 0xAA
	};

	oled_clearScreen(OLED_COLOR_BLACK);
	led7seg_setChar(sseg_chars[stationary_counter], TRUE);
	acc_read(&x, &y, &z);
	xoff = (0-x)/1.0;
	yoff = (0-y)/1.0;

    while (1)
    {
    	/* <---STATIONARY MODE---> */
    	if(mode == 0x00){
    		temp_value = temp_read();
    		//printf("value of t2 = %d\n",t2);
    		sprintf(tempStrPtr, "Temp: %2.2f", temp_value/10.0);
    		modeStrPtr = "STATIONARY";
    		oled_putString(20, 20, (unsigned char*)modeStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(20, 28, (unsigned char*)tempStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    		//HIGH TEMP CHECK
			/*
    		if(temp_value/10.0 > TEMP_HIGH_THRESHOLD){
    			oled_clearScreen(OLED_COLOR_BLACK);
    			mode=0x05;
    		}else{
    			oled_putString(20, 28, (unsigned char*)tempStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    		}*/
    	}

    	/* <---STATIONARY TO LAUNCH---> */
    	else if(mode == 0x01){
    		if(stationary_counter == 0){
    			mode = 0x02; //enter LAUNCH mode
    			oled_clearScreen(OLED_COLOR_BLACK);
    		} else {
    			stationary_counter = stationary_counter - 1;
    			led7seg_setChar(sseg_chars[stationary_counter], TRUE);

    			temp_value = temp_read();
    			sprintf(tempStrPtr, "Temp: %2.2f", temp_value/10.0);

    			//HIGH TEMP CHECK
    			if(temp_value/10.0 > TEMP_HIGH_THRESHOLD){
    			    oled_clearScreen(OLED_COLOR_BLACK);
    	    		mode=0x05;
    	    		stationary_counter = 15;
    	    		led7seg_setChar(sseg_chars[stationary_counter], TRUE);
    			} else {
        			oled_putString(20, 28, (unsigned char*)tempStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    			}
    	    	delayMS(1000);
    		}
    	}

    	/* <---LAUNCH MODE---> */
    	else if(mode == 0x02){
    		if(acc_warning_flag == 1){
    			stateStrPtr = "Veer off course";
    			oled_putString(0, 20, (unsigned char*)stateStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    			check_clearWarning();
    			blink_blue();
    		}else{
    			temp_value = temp_read();
    			sprintf(tempStrPtr, "Temp: %2.2f", temp_value/10.0);
    			modeStrPtr = "LAUNCH";
    			oled_putString(20,20, (unsigned char*)modeStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    			oled_putString(20, 28, (unsigned char*)tempStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    			ACCELEROMETER(); //read from accelerometer
    		}

    		if(getTicks()-recordTicks<1000){
    			//within 1 second
    			if(toggle_count_flag==0x02){
    				mode = 0x03; //go to return
    				acc_warning_flag = 0;
    				toggle_count_flag = 0;
    				oled_clearScreen(OLED_COLOR_BLACK);
    			}
    		} else {
    			//exceeded 1second
    			if(toggle_count_flag==0x01){
    				mode = 0x00;
    				toggle_count_flag = 0;
    				oled_clearScreen(OLED_COLOR_BLACK);
    			}else{
    				recordTicks=getTicks();
    			}
    		}
    	}

    	/* <---RETURN MODE---> */
    	else if(mode == 0x03){
    		modeStrPtr = "RETURN";
    		oled_putString(20,20, (unsigned char*)modeStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    		led7seg_setChar(0x24, TRUE); //set 0 on sseg
    	}

    	/* <---EXCEED TEMPERATURE MODE---> */
    	else if(mode==0x05){
    		modeStrPtr = "Temp. too high";
    		oled_putString(10,20, (unsigned char*)modeStrPtr, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
    		/*
    		if(clearWarning() == 0x01){
    			if(sseg_chars[stationary_counter] == 0x24){
    				mode = 0x02;
    			} else {
    				stationary_counter = 15;
    				mode = 0x00;
    				oled_clearScreen(OLED_COLOR_BLACK);
    			}
    		}*/
    	}


    	/* <---reset toggle_counter---> */
    	//printf("sseg value = %X\n", sseg_chars[stationary_counter]);
    	//printf("temp value = %.2f\n", temp_value/10.0);
    }
}

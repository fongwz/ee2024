#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_spi.h"
#include "lpc17xx_uart.h"
#include "LPC17xx.h"
#include "core_cm3.h"

#include "oled.h"
#include "rgb.h"
#include "temp.h"
#include "led7seg.h"
#include "acc.h"
#include "pca9532.h"

volatile uint32_t msTicks;

//MODE variables
uint8_t mode;
uint8_t mode_change_flag = 0;
uint8_t countdown_flag = 0;
char* modeStrPtr;
char* stateStrPtr;
int8_t toggle_count;
uint8_t sw4btn;

uint8_t blink_blue_flag = 0;
uint8_t blink_red_flag = 0;
uint8_t rgb_flag = 0;

//seven segment variables
uint32_t stationary_counter = 15;
uint8_t sseg_chars[] = {
		/* digits 0 - 9 */
		0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x22, 0x7C, 0x20, 0x38,
		/* A to F */
		0x28, 0x23, 0xA6, 0x61, 0xA2, 0xAA
};

//temperature sensor variables
int8_t temp_warning_flag;
int8_t temp_warning_message_flag;
uint32_t temp_value = 0;
volatile char* tempStrPtr[50]={};
volatile uint32_t period;

//accelerometer variables
int8_t acc_warning_flag;
int8_t acc_warning_message_flag;
int8_t xoff;
int8_t yoff;
int8_t x;
int8_t y;
int8_t z;
char* accx[40];
char* accy[40];
char* accz[40];

//uart variables
static char* msg = NULL;

#define PRESCALE (25000-1)
#define TEMP_HIGH_THRESHOLD 31.0
#define OBSTACLE_NEAR_THRESHOLD 3000
#define ACC_THRESHOLD 0.4

uint32_t getTicks(void){
	return msTicks;
}

void TOGGLE_MODE(){
	// if in STATIONARY mode, go to COUNTDOWN mode
	if(mode == 0x00){
		mode = 0x01;
		countdown_flag = 1;
		toggle_count = 0;
		//COUNTDOWN has same oled display as STATIONARY, no need to enable mode_change_flag
	}

	// if in LAUNCH mode, check number of TOGGLE_MODE(SW3) button presses
	else if(mode == 0x02){
		if(toggle_count >= 2){
			mode = 0x03;
			toggle_count = 0;

			//Clear all warnings
			temp_warning_flag = 0;
			acc_warning_flag = 0;
			temp_warning_message_flag = 0;
			acc_warning_message_flag = 0;
			GPIO_ClearValue( 0, (1<<26) );
			GPIO_ClearValue(2,1<<0);

			//Change viewmode
			mode_change_flag = 1;
		}
	}

	// if in RETURN mode, go back to STATIONARY mode
	else if(mode == 0x03){
		mode = 0x00;
		toggle_count = 0;

		//Change viewmode
		mode_change_flag = 1;
	}

	else {
		//do nothing
	}
}

void check_clearWarning(){
	sw4btn = (GPIO_ReadValue(1) >> 31) & 0x01;
	if(sw4btn == 0){
		if(acc_warning_flag == 1){
			//Turn off flag
			acc_warning_flag = 0;
			acc_warning_message_flag = 0;
			//Clear rgb led and oled
			blink_blue_flag = 0;
		    GPIO_ClearValue( 0, (1<<26) );
			oled_clearScreen(OLED_COLOR_BLACK);
		}

		if(temp_warning_flag == 1){
			//Turn off flag
			temp_warning_flag = 0;
			temp_warning_message_flag = 0;
			//Clear rgb led and oled
			blink_red_flag = 0;
			GPIO_ClearValue(2,1<<0);
			oled_clearScreen(OLED_COLOR_BLACK);
		}
	}
}

void ACCELEROMETER(){
	//reset x and y values to 0 first
	x=0;
	y=0;

	//reading accelerometer
	acc_read(&x, &y, &z);
	x=x+xoff;
	y=y+yoff;

	//check if accelerometer in g exceed threshold value
	if(x/64.0 >= 0.4 || y/64.0 >= 0.4){
		acc_warning_flag = 1;
		if(acc_warning_flag == 0){
			oled_clearScreen(OLED_COLOR_BLACK);
		}
	} else {
		//need values in terms of g, according to acc.h, g level is set to default 2g
		//divide value read by accelerometer by 64, according to datasheet

		if(temp_warning_flag == 0 && acc_warning_flag == 0){ //make sure no warnings are triggered
			sprintf(accx, "X:%3.1f", x/64.0);
			sprintf(accy, "Y:%3.1f", y/64.0);

			oled_putString(10, 38, (unsigned char*)accx, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(55, 38, (unsigned char*)accy, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		}
	}
}

void TEMP_SENSOR(){

	static uint32_t t1 = 0;
	static int count = 0;

	if (!count) {
		t1 = LPC_TIM2->TC;
	} else {
		period = LPC_TIM2->TC;
		if (period > t1) {
			period = period - t1;	//obtained period is in 10^-8s
		} else {
			period = (100000000 - t1 + 1) + period;
		}
	}
	count = !count;
	//using TS0/TS1 configuration of GND/GND(both jumpers attached)
	//Following datasheet formula:
	//10temp_value(deg celcius) = 10(period(us)/scalar multiplier of 10) - 2731
	//if(mode_change_flag == 0){
		temp_value = period/100 - 2731;
	//}
	if(temp_value/10.0 > TEMP_HIGH_THRESHOLD){
		temp_warning_flag = 1;
	}
}

static void init_GPIO(void)
{

	// Initialize button SW4 and SW3
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
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

	GPIO_SetDir( 0, (1<<2), 0 );	//GPIO Setting for Temp Sensor
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

void init_uart(void){

	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	//supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);
	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
}

//Timer for 333ms intervals
void init_Timer1(void){
	LPC_SC->PCONP |= (1<<2);		//Turns on timer1 (On by default)
	LPC_SC->PCLKSEL0 |= 1<<4;		//Set Timer3 CLK = CCLK(100MHZ)
	LPC_TIM1->TCR = 0x02;			//Resets TC
	LPC_TIM1->PR = 0x00;			//Prescale is 0
	LPC_TIM1->MR3 = 33000000;		//Match Count 3(333ms)
	LPC_TIM1->IR = 0xff;			//Reset Timer1 interrupts
	LPC_TIM1->MCR |= (1<<10);		//Clears TC when TC hits MR3 value of 33000000
	LPC_TIM1->MCR |= (1<<9);		//Triggers TIMER1 interrupt when TC hits MR3 value of 33000000
	LPC_TIM1->TCR = 0x01;			//Start Timer1
}

//Timer for temperature sensor
void init_Timer2(void){				//Initialization for timer2

	LPC_SC->PCONP |= (1<<22); 		//Turns on timer2 (Off by default)
	LPC_SC->PCLKSEL1 |= 1<< 12;		//Set Timer2 CLK = CCLK(100MHZ)
	LPC_TIM2->TCR = 0x02;			//Resets Timer Counter (TC)
	LPC_TIM2->PR  = 0x00;			//Clock Prescaler = 0
	//LPC_TIM2->MR0 = 33300000;		//Match Count 0 (1 Count = 10ns)
	//LPC_TIM2->MR1 = 66600000;		//Match Count 1
	//LPC_TIM2->MR2 = 99900000;		//Match Count 2
	LPC_TIM2->MR3 = 100000000;		//Match Count 3
	LPC_TIM2->IR  = 0xff;			//Resets Timer2 Interrupts
	LPC_TIM2->MCR |= (1<<10);		//Clears TC when TC hits MR3 value of 100000000 and triggers timer2 interrupt
	LPC_TIM2->MCR |= (1<<9);
	LPC_TIM2->TCR = 0x01;			//Start timer2
}

//Timer for 1 second intervals
void init_Timer3(void){				//Initialization for timer3

	LPC_SC->PCONP |= (1<<23); 		//Turns on timer3 (Off by default)
	LPC_SC->PCLKSEL1 |= 1<< 14;		//Set Timer3 CLK = CCLK(100MHZ)
	LPC_TIM3->TCR = 0x02;			//Resets Timer Counter (TC)
	LPC_TIM3->PR  = 0x00;			//Clock Prescaler = 0
	LPC_TIM3->MR3 = 100000000;		//Match Count 3 (1 second)
	LPC_TIM3->IR  = 0xff;			//Resets Timer3 Interrupts
	LPC_TIM3->MCR |= (1<<11);		//Stops timer when TC hits MR3 value of 100000000
	LPC_TIM3->MCR |= (1<<10);		//Clears TC when TC hits MR3 value of 100000000
	LPC_TIM3->MCR |= (1<<9);		//Triggers TIMER3 interrupt when TC hits MR3 value of 100000000
	//LPC_TIM3->TCR = 0x01;			//Start timer3
}

void SysTick_Handler(void){
	msTicks++;
}

//333ms Timer interrupt
void TIMER1_IRQHandler(void){
	if(rgb_flag==4){
		rgb_flag=0;
	}

	if(acc_warning_flag == 1 && temp_warning_flag == 1){
		GPIO_ClearValue(0, (1<<26));
		GPIO_ClearValue(2, (1<<0));
		if(rgb_flag==0){
			GPIO_SetValue(0, 1<<26);
			rgb_flag++;
		}
		else if(rgb_flag==1){
			GPIO_ClearValue(0, 1<<26);
			rgb_flag++;
		}
		else if(rgb_flag==2){
			GPIO_SetValue(2,1<<0);
			rgb_flag++;
		}
		else if(rgb_flag==3){
			GPIO_ClearValue(2,1<<0);
			rgb_flag++;
		}
	}

	else if(acc_warning_flag == 1){
		if(blink_blue_flag == 0){
			GPIO_SetValue( 0, (1<<26) );
			blink_blue_flag = 1;
		} else {
			GPIO_ClearValue( 0, (1<<26) );
			blink_blue_flag = 0;
		}
	}

	else if(temp_warning_flag == 1){
		if(blink_red_flag == 0){
			GPIO_SetValue( 2, (1<<0));
			blink_red_flag = 1;
		} else {
			GPIO_ClearValue( 2, 1<<0 );
			blink_red_flag = 0;
		}
	}
	LPC_TIM1->IR |= 1<<0;
}

//1 second Timer Interrupt
void TIMER3_IRQHandler(void){
	toggle_count = 0;
	if(mode == 0x01){
		countdown_flag = 1;
	}
	LPC_TIM3->IR |= 1<<0;
}

void EINT0_IRQHandler(void){
	//SW3 interrupt handler
	toggle_count++;
	TOGGLE_MODE();

	if(mode != 0x01){
		LPC_TIM3->TCR = 0x01;
	}
	LPC_SC->EXTINT |= 1<<0;
}

void EINT3_IRQHandler(void){
	if ((LPC_GPIOINT->IO0IntStatF)>>2 & 0x01){
		TEMP_SENSOR();
		LPC_GPIOINT->IO0IntClr = 1<<2;
	}
}

void STATIONARY(){
	sprintf(tempStrPtr, "Temp: %2.2f", temp_value/10.0);
	modeStrPtr = "STATIONARY";
	oled_putString(20, 20, (unsigned char*)modeStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(20, 28, (unsigned char*)tempStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void COUNTDOWN(){
	//When countdown reaches 0, move to launch mode
	if(stationary_counter == 0){
		mode = 0x02; //enter LAUNCH mode
		toggle_count = 0;
		oled_clearScreen(OLED_COLOR_BLACK);

		//reset timer 3 for launch mode
		//LPC_TIM3->TCR = 0x02;
		//LPC_TIM3->TCR = 0x01;
	} else if(countdown_flag == 1){
		stationary_counter = stationary_counter - 1;
		led7seg_setChar(sseg_chars[stationary_counter], TRUE);

		sprintf(tempStrPtr, "Temp: %2.2f", temp_value/10.0);
		oled_putString(20,28, (unsigned char*)tempStrPtr, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		countdown_flag = 0;
		//count 1sec
    	LPC_TIM3->TCR = 0x01;
	} else {
		sprintf(tempStrPtr, "Temp: %2.2f", temp_value/10.0);
		oled_putString(20,28, (unsigned char*)tempStrPtr, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	}
}

void LAUNCH(){
	sprintf(tempStrPtr, "Temp: %2.2f", temp_value/10.0);
	modeStrPtr = "LAUNCH";
	oled_putString(20,20, (unsigned char*)modeStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(20, 28, (unsigned char*)tempStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	ACCELEROMETER(); //read from accelerometer
}

void RETURN(){
	modeStrPtr = "RETURN";
	oled_putString(20,20, (unsigned char*)modeStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	//led7seg_setChar(0xAA, TRUE); //set 0 on sseg
}

void CHANGE_VIEWMODE(){
	if(mode_change_flag == 1){
		oled_clearScreen(OLED_COLOR_BLACK);
		mode_change_flag = 0;
	}
}

void SET_MODE(){
	if(acc_warning_flag == 1 || temp_warning_flag == 1){
		return;
	}

	CHANGE_VIEWMODE();

	/* <---STATIONARY MODE---> */
	if(mode == 0x00){
		STATIONARY();
	}

	/* <---COUNTDOWN---> */
	else if(mode == 0x01){
		COUNTDOWN();
	}

	/* <---LAUNCH MODE---> */
	else if(mode == 0x02){
		LAUNCH();
	}

	/* <---RETURN MODE---> */
	else if(mode == 0x03){
		RETURN();
	}
}

void SET_WARNING(){
	if(mode == 0x02){
		ACCELEROMETER();
	}

	if(temp_warning_flag == 1 && temp_warning_message_flag == 0){
		stateStrPtr = "Temp. too high";
		if(acc_warning_flag == 1){
			if(temp_warning_message_flag == 0){
				//print oled temp warning message under acc warning message
				oled_putString(0,28, (unsigned char*)stateStrPtr, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
				temp_warning_message_flag = 1;
			}
		} else {
			if(temp_warning_message_flag == 0){
				oled_clearScreen(OLED_COLOR_BLACK);
				oled_putString(0, 20, (unsigned char*)stateStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				temp_warning_message_flag = 1;
			}
		}

		//reset sseg if on countdown mode
		if(mode == 0x01){
			stationary_counter = 15;
			led7seg_setChar(sseg_chars[stationary_counter],TRUE);
			mode = 0x00;
		}

		check_clearWarning();
	}

	else if(acc_warning_flag == 1){
		stateStrPtr = "Veer off course";
		if(temp_warning_flag == 1){
			if(acc_warning_message_flag == 0){
				//display acc warning message under already displayed temp warning message
				oled_putString(0,28, (unsigned char*)stateStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				acc_warning_message_flag = 1;
			}
		} else {
			if(acc_warning_message_flag == 0){
				oled_clearScreen(OLED_COLOR_BLACK);
				oled_putString(0, 20, (unsigned char*)stateStrPtr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				acc_warning_message_flag = 1;
			}
		}
		check_clearWarning();
	}
}

int main (void) {

	SysTick_Config(SystemCoreClock/1000);

    init_GPIO();
    init_i2c();
    init_ssp();
    init_uart();
	init_Timer1();
	init_Timer2();	//init timer 2
	init_Timer3();

	pca9532_init(); //led_array
	acc_init();
    rgb_init();
    oled_init();
    led7seg_init();

    //Clears all interrupts
    NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(TIMER3_IRQn);

	//Set up EINT0 for SW3
	LPC_SC->EXTMODE |= 1<<0;
	LPC_SC->EXTPOLAR = 0;

	//Set up temp sensor interrupt
	LPC_GPIOINT->IO0IntEnF |= 1<<2;

	//Set up interrupt priorities
	NVIC_SetPriorityGrouping(5);
	NVIC_SetPriority(SysTick_IRQn,0x00);
	NVIC_SetPriority(EINT0_IRQn,0x40);
	NVIC_SetPriority(EINT3_IRQn,0x48);
	NVIC_SetPriority(TIMER1_IRQn,0x50);
	NVIC_SetPriority(TIMER3_IRQn,0x60);

	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);

	toggle_count = 0; //init number of sw3 button presses to be 0
	sw4btn = 1; //init sw4 button
	mode = 0; //init as STATIONARY MODE

	//Set initial OLED and SSEG displays
	oled_clearScreen(OLED_COLOR_BLACK);
	led7seg_setChar(sseg_chars[stationary_counter], TRUE);

	//Get initial offset for accelerometer
	acc_read(&x, &y, &z);
	xoff = (0-x)/1.0;
	yoff = (0-y)/1.0;

	//Reset flag statuses
	acc_warning_flag = 0;
	acc_warning_message_flag = 0;
	temp_warning_flag = 0;
	temp_warning_message_flag = 0;


	//uart testing
	/*
	//(uncomment this block and comment out SET_MODE and SET_WARNING in the loop to test uart connection)
	uint8_t data = 0;
	uint32_t len = 0;
	uint8_t line[64];

	//test sending message
	msg = "Welcome to EE2024 \r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
	//test receiving a letter and sending back to port
	UART_Receive(LPC_UART3, &data, 1, BLOCKING);
	UART_Send(LPC_UART3, &data, 1, BLOCKING);
	*/

    while (1)
    {
    	SET_MODE();
    	SET_WARNING();
    }
}

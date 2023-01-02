/**
 * \file
 *
 * \brief Implementasi RTOS v10.2.1
 *
 */

#include <asf.h>
#include <stdio.h>
#include <string.h>

#define RED_DURATION 10
#define YELLOW_DURATION 3
#define GREEN_DURATION 5

#define RED_STATE 0
#define YELLOW_STATE 1
#define GREEN_STATE 2
#define RED_INTERRUPT_STATE 3
#define YELLOW_TO_RED_STATE 4

#define MAX_DISTANCE 100

#define USART_SERIAL_EXAMPLE             &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE    9600
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            false

static char reads[5];

char *close = "CLOSE";

/* Define a task */
static portTASK_FUNCTION_PROTO(vTimeCount, p_);
static portTASK_FUNCTION_PROTO(vButtonInput, q_);
static portTASK_FUNCTION_PROTO(vSensor, r_);
static portTASK_FUNCTION_PROTO(vMessager, s_);

// Traffic
char* state_enum_to_string(int state);
void update_LCD(void);
void ping_sensor(void);
void setup_timer(void);
int next_state(int state);
int state_countdown(int state);
int state_output(int state);

// Serial
void setUpSerial(void);
char receiveChar(void);
void receiveString(void);

/* Define semaphore */
SemaphoreHandle_t xSemaphore;

uint16_t distance = 0;
uint16_t sensor_count = 0;
uint16_t countdown = 10;
uint16_t alarmAck = 0;
uint16_t currState = RED_STATE;


char* state_enum_to_string(int state) {
	switch(state) {
		case RED_STATE:
		case RED_INTERRUPT_STATE:
			return "MERAH ";
		case YELLOW_STATE:
		case YELLOW_TO_RED_STATE:
			return "KUNING";
		case GREEN_STATE:
			return "HIJAU ";
		default:
			return "MERAH ";
	}
}

int next_state(int state) {
	switch(state) {
		case RED_STATE:
			return YELLOW_STATE;
		case RED_INTERRUPT_STATE:
			return YELLOW_STATE;
		case YELLOW_STATE:
			return GREEN_STATE;
		case YELLOW_TO_RED_STATE:
			return RED_STATE;
		case GREEN_STATE:
			return YELLOW_TO_RED_STATE;
		default:
			return RED_STATE;
	}
}

int state_countdown(int state) {
	switch(state) {
		case RED_STATE:
		case RED_INTERRUPT_STATE:
			return RED_DURATION;
		case YELLOW_STATE:
		case YELLOW_TO_RED_STATE:
			return YELLOW_DURATION;
		case GREEN_STATE:
			return GREEN_DURATION;
		default:
			return RED_DURATION;
	}
}

int state_output(int state) {
	switch(state) {
		case RED_STATE:
		case RED_INTERRUPT_STATE:
			return 1;
		case YELLOW_STATE:
		case YELLOW_TO_RED_STATE:
			return 2;
		case GREEN_STATE:
			return 4;
		default:
			return 1;
	}
}

void update_LCD(void) {
	char buffarray[128];
	snprintf(buffarray, sizeof(buffarray), "LAMPU: %s    \nPanjang: %d cm    \nTime: %d s   \nMessage: %s ", state_enum_to_string(currState), distance, countdown, reads);
	gfx_mono_draw_string(buffarray, 0, 0, &sysfont);
}

void setUpSerial()
{
	// Baud rate selection
	// BSEL = (2000000 / (2^0 * 16*9600) -1 = 12.0208... ~ 12 -> BSCALE = 0
	// FBAUD = ( (2000000)/(2^0*16(12+1)) = 9615.384 -> mendekati lah ya
	
	USARTC0_BAUDCTRLB = 0; //memastikan BSCALE = 0
	USARTC0_BAUDCTRLA = 0x0C; // 12
	
	//USARTC0_BAUDCTRLB = 0; //Just to be sure that BSCALE is 0
	//USARTC0_BAUDCTRLA = 0xCF; // 207
	
	//Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	//8 data bits, no parity and 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	
	//Enable receive and transmit
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

char receiveChar()
{
	while( !(USARTC0_STATUS & USART_RXCIF_bm) ); //Wait until receive finish
	return USARTC0_DATA;
}

void receiveString()
{
	int i = 0;
	while(1){
		//char inp = receiveChar();
		char inp = usart_getchar(USART_SERIAL_EXAMPLE);
		if(inp=='\n') break;
		else reads[i++] = inp;
		if (i > 4) break;
	}
	if(strcmp(close,reads) == 0 && alarmAck == 0){
		currState = RED_INTERRUPT_STATE;
		alarmAck = 1;
		countdown = state_countdown(currState);
		PORTA.OUT = state_output(currState);
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	board_init();
	pmic_init();
	gfx_mono_init();
	
	PORTC_OUTSET = PIN3_bm; // PC3 as TX
	PORTC_DIRSET = PIN3_bm; //TX pin as output
	
	PORTC_OUTCLR = PIN2_bm; //PC2 as RX
	PORTC_DIRCLR = PIN2_bm; //RX pin as input
	
	setUpSerial();
	
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	usart_init_rs232(USART_SERIAL_EXAMPLE, &USART_SERIAL_OPTIONS);
	
	ioport_set_pin_dir(J2_PIN0, IOPORT_DIR_OUTPUT);
	
	PORTA.DIR = 0x07;
	
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
	gfx_mono_draw_string("RTOS v10.2.1", 0, 0, &sysfont);
	
	// Workaround for known issue: Enable RTC32 sysclk
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	while (RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm) {
		// Wait for RTC32 sysclk to become stable
	}

	/* Create the task */
	
	xTaskCreate(vMessager, "", 1000, NULL, tskIDLE_PRIORITY + 3 , NULL);
	xTaskCreate(vButtonInput, "", 1000, NULL, tskIDLE_PRIORITY + 2 , NULL);
	xTaskCreate(vSensor, "", 1000, NULL, tskIDLE_PRIORITY + 1 , NULL);
	xTaskCreate(vTimeCount, "", 1000, NULL, tskIDLE_PRIORITY, NULL);
	
	/* Semaphore */
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);
	
	/* Start the task */
	
	vTaskStartScheduler();
}

static portTASK_FUNCTION(vTimeCount, p_) {
	while(1) {
		if (countdown > 0) {
			countdown = countdown - 1;
		} else {
			if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
				currState = next_state(currState);
				countdown = state_countdown(currState);
				PORTA.OUT = state_output(currState);
				alarmAck = 0;
				update_LCD();
				xSemaphoreGive(xSemaphore);
			}
		}
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vButtonInput, q_) {
	while (1) {
		if(ioport_get_pin_level(GPIO_PUSH_BUTTON_1)==0){
			if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
				currState = RED_INTERRUPT_STATE;
				countdown = state_countdown(currState);
				PORTA.OUT = state_output(currState);
				update_LCD();
				xSemaphoreGive(xSemaphore);
			}
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vSensor, r_) {
	int distanceCount = 0;
	
	while (1) {
		if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
			PORTB.DIR = 0b11111111;
			PORTB.OUT = 0b00000000;
			PORTB.OUT = 0b11111111; // Send PING
			delay_us(5);
			PORTB.OUT = 0b00000000;
			PORTB.DIR = 0b00000000; // Set to input
			delay_us(750);
			delay_us(115); // Delay until output PIN is high
			
			while (PORTB.IN & PIN0_bm) {
				delay_us(58);
				distanceCount = distanceCount + 1;
				if (distanceCount == MAX_DISTANCE) {
					break;
				}
			}
			
			distance = distanceCount;
			distanceCount = 0;
			
			if (distance < 10 && currState == RED_STATE) {
				currState = YELLOW_STATE;
				countdown = state_countdown(currState);
				PORTA.OUT = state_output(currState);
			}
			
			update_LCD();
			xSemaphoreGive(xSemaphore);
		}		
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vMessager, s_) {
	while(1){
		if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
			receiveString();
			update_LCD();
			xSemaphoreGive(xSemaphore);
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

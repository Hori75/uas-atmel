/**
 * \file
 *
 * \brief Implementasi RTOS v10.2.1
 *
 */

#include <asf.h>
#include <stdio.h>

#define RED_DURATION 10
#define YELLOW_DURATION 3
#define GREEN_DURATION 5

#define RED_STATE 0
#define YELLOW_STATE 1
#define GREEN_STATE 2
#define RED_INTERRUPT_STATE 3

#define MAX_DISTANCE 100

/* Define a task */
static portTASK_FUNCTION_PROTO(vTimeCount, p_);
static portTASK_FUNCTION_PROTO(vManageState, q_);
static portTASK_FUNCTION_PROTO(vSensor, r_);
static portTASK_FUNCTION_PROTO(vMessager, s_);

/* Define semaphore */
SemaphoreHandle_t xSemaphore;
uint16_t counter = 0;

int distance = 0;
int sensor_count = 0;
int seconds = 0;
int currState = RED_STATE;
int nextState = GREEN_STATE;
static char buffarray[200];

char* state_enum_to_string(int state) {
	switch(currState) {
		case RED_STATE:
		return "MERAH ";
		case YELLOW_STATE:
		return "KUNING";
		case GREEN_STATE:
		return "HIJAU ";
		case RED_INTERRUPT_STATE:
		return "MERAH ";
		default:
		return "MERAH ";
	}
}

int remaining_time(int state) {
	switch(currState) {
		case YELLOW_STATE:
		return YELLOW_DURATION - seconds;
		case GREEN_STATE:
		return GREEN_DURATION - seconds;
		default:
		return RED_DURATION - seconds;
	}
}

void update_LCD(void) {
	snprintf(buffarray, sizeof(buffarray), "LAMPU: %s    \nPanjang: %d cm    \nTime: %d s   ", state_enum_to_string(currState), distance, remaining_time(currState));
	gfx_mono_draw_string(buffarray, 0, 0, &sysfont);
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	board_init();
	pmic_init();
	gfx_mono_init();
	
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
	gfx_mono_draw_string("RTOS v10.2.1", 0, 0, &sysfont);

	/* Create the task */
	
	xTaskCreate(vTimeCount, "", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vManageState, "", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(vSensor, "", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vMessager, "", 1000, NULL, tskIDLE_PRIORITY, NULL);
	
	/* Semaphore */
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);
	
	/* Start the task */
	
	vTaskStartScheduler();
}

static portTASK_FUNCTION(vTimeCount, p_) {
	while(1) {
		vTaskDelay(1);
		seconds = seconds + 1;
	}
}

static portTASK_FUNCTION(vManageState, q_) {
	while (1) {
		// State handling
		switch(currState) {
			case RED_STATE:
				PORTA.OUT = 1;
				nextState = GREEN_STATE;
				if (distance < 10 || seconds >= RED_DURATION) {
					currState = YELLOW_STATE;
					seconds = 0;
				}
				break;
			case YELLOW_STATE:
			PORTA.OUT = 2;
				if (seconds >= YELLOW_DURATION) {
					currState = nextState;
					seconds = 0;
					} else if (ioport_get_pin_level(GPIO_PUSH_BUTTON_1)==0) {
					PORTC.OUT = 1;
					currState = RED_INTERRUPT_STATE;
					seconds = 0;
				}
				break;
			case GREEN_STATE:
				PORTA.OUT = 4;
				if (seconds >= GREEN_DURATION) {
					currState = RED_STATE;
					seconds = 0;
					} else if (ioport_get_pin_level(GPIO_PUSH_BUTTON_1)==0) {
					PORTC.OUT = 1;
					currState = RED_INTERRUPT_STATE;
					seconds = 0;
				}
				break;
			case RED_INTERRUPT_STATE:
				PORTA.OUT = 1;
				nextState = GREEN_STATE;
				if (seconds >= RED_DURATION) {
					PORTC.OUT = 0;
					currState = YELLOW_STATE;
					seconds = 0;
				}
				break;
		}
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vSensor, r_) {
	int distanceCount = 0;
	
	// Workaround for known issue: Enable RTC32 sysclk
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	while (RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm) {
		// Wait for RTC32 sysclk to become stable
	}
	
	while (1) {
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
		
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vMessager, s_) {
	
}

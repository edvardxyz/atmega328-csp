/*
 * main.c
 *
 *  Created on: Feb 4, 2015
 *      Author: jcobb
 */
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <led.h>
#include <atmel_start.h>
#include <time.h>
#include <util/delay.h>

#define mainLED_TASK_PRIORITY			(tskIDLE_PRIORITY)
#define mainLED_TASK_PRIORITY 			(tskIDLE_PRIORITY+1)
// #define mainNEXT_TASK_2				(tskIDLE_PRIORITY+2)
// #define mainNEXT_TASK_3				(tskIDLE_PRIORITY+3)
//


void vLEDFlashTask(void *pvParms)
{
	vLEDInit();
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1000;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) {
		vLEDToggle();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	/*
	for(;;) {
		vLEDToggle();
		PORTB ^= _BV(PB1); // green
		PORTB ^= _BV(PB2); // yellow
		PORTB ^= _BV(PB3); // red
		_delay_ms(1000);
	}
	*/

	/* Replace with your application code */
	xTaskCreate(vLEDFlashTask, (int8_t*) "LED", configMINIMAL_STACK_SIZE, NULL, mainLED_TASK_PRIORITY, NULL);

	vTaskStartScheduler();
}

void vApplicationIdleHook( void );

void vApplicationIdleHook( void )
{
	for(;;) {
		PORTB ^= _BV(PB1); // green
		_delay_ms(1000);
	}
	//vCoRoutineSchedule();
}


/*
 * main.c
 *
 *  Created on: Feb 4, 2015
 *      Author: jcobb
 */
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <atmel_start.h>
#include <time.h>
#include <util/delay.h>

void vLEDFlashTask(void *pvParms)
{
	DDRB |= _BV(PB5);
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) {
		PORTB ^= _BV(PB5); // green
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

	static StaticTask_t led_task_tcb __attribute__((section(".noinit")));
	static StackType_t led_task_stack[64] __attribute__((section(".noinit")));
	xTaskCreateStatic(vLEDFlashTask, "LED", 64, NULL, 1, led_task_stack, &led_task_tcb);

	vTaskStartScheduler();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    for(;;);
}

/*
void vApplicationIdleHook( void );

void vApplicationIdleHook( void )
{
	for(;;) {
		PORTB ^= _BV(PB1); // green
		_delay_ms(1000);
	}
	//vCoRoutineSchedule();
}
*/

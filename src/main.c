/*
 * main.c
 *
 *  Created on: Feb 4, 2015
 *      Author: jcobb
 */
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <time.h>
#include <util/delay.h>
#include "projdefs.h"
#include <csp/csp.h>
#include <atmel_start.h>

void vLEDFlashTask(void * pvParms) {
	DDRB |= _BV(PB5);
	PORTB ^= _BV(PB5);  // green
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		PORTB ^= _BV(PB5);  // green
		// PORTB ^= _BV(PB1); // green
		// PORTB ^= _BV(PB2); // yellow
		// PORTB ^= _BV(PB3); // red
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void router_task(void * param) {
	while (1) {
		csp_route_work();
	}
}

int main(void) {

	atmel_start_init();

	csp_conf.hostname = "atmega";
	csp_conf.model = "328pb";
	csp_conf.revision = "1";
	csp_conf.conn_dfl_so = CSP_O_NONE;
	csp_conf.dedup = 0;
	csp_conf.version = 2;
	csp_init();

	static StaticTask_t led_task_tcb;
	static StackType_t led_task_stack[32];
	xTaskCreateStatic(vLEDFlashTask, "LED", 64, NULL, 1, led_task_stack, &led_task_tcb);

	static StaticTask_t router_tcb;
	static StackType_t router_stack[64];
	xTaskCreateStatic(router_task, "RTE", 64, NULL, 2, router_stack, &router_tcb);

	csp_bind_callback(csp_service_handler, CSP_ANY);

	vTaskStartScheduler();
	while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName) {
	for (;;) {
		PORTB ^= _BV(PB5);
		_delay_ms(10);
	};
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

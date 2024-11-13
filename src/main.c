/*
 * main.c
 *
 *  Created on: Feb 4, 2015
 *      Author: jcobb
 */
#include <avr/io.h>
#include <FreeRTOS.h>
#include <stdint.h>
#include <task.h>
#include <time.h>
#include <util/delay.h>
#include "FreeRTOSConfig.h"
#include "projdefs.h"
#include <csp/csp.h>
#include <atmel_start.h>
#include <csp/csp_hooks.h>
#include <semphr.h>

#include <csp/drivers/usart.h>
#include <usart_basic.h>
#include <csp/interfaces/csp_if_kiss.h>

int uart_init();

static void vLEDFlashTask(void * pvParms) {
	DDRB |= _BV(PB5);
	PORTB ^= _BV(PB5);  // green
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		PORTB ^= _BV(PB5);  // green
		// printf("hello\n");
		//  USART_0_read()
		//   PORTB ^= _BV(PB1); // green
		//   PORTB ^= _BV(PB2); // yellow
		//   PORTB ^= _BV(PB3); // red
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

static void router_task(void * param) {
	while (1) {
		csp_route_work();
	}
}

int main(void) {

	atmel_start_init();
	// printf("atmel init done\n");

	//csp_dbg_packet_print = 0;

	static StaticTask_t led_task_tcb;
	static StackType_t led_task_stack[configMINIMAL_STACK_SIZE] __attribute__((section(".noinit")));
	xTaskCreateStatic(vLEDFlashTask, "LED", configMINIMAL_STACK_SIZE, NULL, 1, led_task_stack, &led_task_tcb);

	csp_conf.hostname = "atmega";
	csp_conf.model = "328pb";
	csp_conf.revision = "1";
	csp_conf.conn_dfl_so = CSP_O_NONE;
	csp_conf.dedup = 0;
	csp_conf.version = 2;

	// printf("before uart_init\n");
	uart_init();
	// printf("after uart_init\n");
	csp_init();

	static StaticTask_t router_tcb;
	static StackType_t router_stack[300];
	xTaskCreateStatic(router_task, "RTE", 300, NULL, 2, router_stack, &router_tcb);

	csp_bind_callback(csp_service_handler, CSP_ANY);

	ENABLE_INTERRUPTS();

	vTaskStartScheduler();
	while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName) {
	// printf("stackoverflow\n");
	for (;;) {
		PORTB ^= _BV(PB5);
		_delay_ms(100);
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

void csp_clock_get_time(csp_timestamp_t * time) {
}

int csp_clock_set_time(const csp_timestamp_t * time) {
	return CSP_ERR_NONE;
}

void csp_reboot_hook(void) {
	//printf("reboot\n");
}
void csp_shutdown_hook(void) {
}

static int kiss_driver_tx(void * driver_data, const unsigned char * data, size_t data_length) {

	for (uint16_t i = 0; i < data_length; i++) {
		if(USART1_is_tx_busy()){
			_delay_us(1);
		}
		USART1_write(data[i]);
		UDR1 = data[i];
	}
	return 0;
}

csp_kiss_interface_data_t ifdata = {
	.tx_func = kiss_driver_tx,
};

csp_iface_t csp_if_kiss = {
	.name = "KISS",
	.interface_data = &ifdata,
};

void USART1_csp_rx_isr_cb(void) {

	BaseType_t xTaskWoken = pdFALSE;
	uint8_t data;

	/* Read the received data */
	data = UDR1;
	csp_kiss_rx(&csp_if_kiss, &data, 1, &xTaskWoken);
}

static StaticSemaphore_t kiss_lock_buf;
static SemaphoreHandle_t kiss_lock = NULL;

void csp_usart_lock(void * driver_data) {
	xSemaphoreTake(kiss_lock, 100);
}

void csp_usart_unlock(void * driver_data) {
	xSemaphoreGive(kiss_lock);
}

int uart_init() {

	if (kiss_lock == NULL) {
		kiss_lock = xSemaphoreCreateMutexStatic(&kiss_lock_buf);
	}

	csp_if_kiss.addr = 1;
	csp_if_kiss.netmask = 8;
	csp_if_kiss.is_default = 1;

	USART1_set_ISR_cb(USART1_csp_rx_isr_cb, RX_CB);

	csp_kiss_add_interface(&csp_if_kiss);

	return 0;
}

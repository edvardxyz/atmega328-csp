/*
 * main.c
 *
 *  Created on: Feb 4, 2015
 *      Author: jcobb
 */
#include <avr/io.h>
#include <FreeRTOS.h>
#include <stdint.h>
#include <string.h>
#include <task.h>
#include <time.h>
#include <util/delay.h>
#include "FreeRTOSConfig.h"
#include "csp/autoconfig.h"
#include "csp/csp_buffer.h"
#include "csp/csp_types.h"
#include "projdefs.h"
#include <csp/csp.h>
#include <atmel_start.h>
#include <csp/csp_hooks.h>
#include <semphr.h>

#include <csp/drivers/usart.h>
#include <usart_basic.h>
#include <csp/interfaces/csp_if_kiss.h>
#include <csp/csp_id.h>

// #include "ssd1306.h"

/*
#include <param/param_server.h>
#include <param/param.h>
#include <x86_64-linux-gnu/sys/socket.h>
*/
int csp_xbee_add_interface(csp_iface_t * iface);
void send_16bit_frame(uint16_t dest_address, void * driver_data, const uint8_t * payload, uint16_t len);
typedef void (*csp_xbee_driver_tx_t)(uint16_t dest_address, void * driver_data, const uint8_t * data, size_t len);
void xbee_driver_rx(void);
#define XBEE_FRAME_LEN 5

typedef enum {
	XBEE_MODE_NOT_STARTED,
	XBEE_MODE_LENGTH_MSB,
	XBEE_MODE_LENGTH_LSB,
	XBEE_MODE_SKIP_FRAMING,
	XBEE_MODE_RECEIVING,
} xbee_receive_state_t;

typedef struct {
	csp_xbee_driver_tx_t tx_func; /**< Tx function */
	xbee_receive_state_t rx_mode; /**< Rx mode/state. */
	unsigned int rx_length;       /**< Rx length */
	csp_packet_t * rx_packet;     /**< CSP packet for storing Rx data. */
} csp_xbee_interface_data_t;

csp_xbee_interface_data_t ifdata_xbee = {
	.tx_func = send_16bit_frame,
};

typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	int fd;
	csp_packet_t * rx_packet;
	xbee_receive_state_t xbee_mode;
	uint16_t data_index;
	uint16_t length;
	uint8_t checksum;
	bool escape_next;
	uint8_t header_sz;
} xbee_ctx_t;
xbee_ctx_t user_data = {.name = "xbee", .xbee_mode = XBEE_MODE_NOT_STARTED, .iface = {.name = "xbee", .interface_data = &ifdata_xbee, .addr = 2, .netmask = 8, .is_default = 1}};

int uart_init();

/*
char row[12];
static void vLEDFlashTask(void * pvParms) {
	DDRB |= _BV(PB5);
	PORTB ^= _BV(PB5);  // green
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();
	uint8_t row_num = 1;
	uint8_t row_idx = 0;

	for (;;) {

		char c = USART0_read();
		if (row_idx < sizeof(row) - 1 && c > 31 && c < 126) {
			row[row_idx++] = c;
			row[row_idx] = '\0';
		}
		if (c == 127) {
			row[row_idx > 0 ? --row_idx : 0] = '\0';
		}

		SSD1306_ClearPage(row_num);
		SSD1306_SetPosition(0, row_num);
		SSD1306_DrawString(row, BOLD);

		if (c == '\r' || c == '\n' || row_idx >= sizeof(row) - 1) {
			row_num = row_num == 7 ? 1 : row_num + 2;
			row_idx = 0;
			row[row_idx] = '\0';
		}

		PORTB ^= _BV(PB5);
		// green
		// printf("hello\n");
		//  USART_0_read()
		//   PORTB ^= _BV(PB1); // green
		//   PORTB ^= _BV(PB2); // yellow
		//   PORTB ^= _BV(PB3); // red
		// vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
*/

/*
static void _TaskPing(void * pvParms) {

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		//csp_ping(1, 1000, 5, CSP_O_NONE);
		PORTB ^= _BV(PB5);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

}
*/

static void router_task(void * param) {
	while (1) {
		csp_route_work();
	}
}

// int frame_in_len = 0;
// uint8_t frame_buf_in[30];

int main(void) {

	atmel_start_init();

	// csp_dbg_packet_print = 0;

	// static StaticTask_t led_task_tcb;
	// static StackType_t led_task_stack[configMINIMAL_STACK_SIZE] __attribute__((section(".noinit")));
	// xTaskCreateStatic(vLEDFlashTask, "LED", configMINIMAL_STACK_SIZE, NULL, 1, led_task_stack, &led_task_tcb);
	//
	// static StaticTask_t _task_tcb;
	// static StackType_t _task_stack[configMINIMAL_STACK_SIZE + 32] __attribute__((section(".noinit")));
	// xTaskCreateStatic(_TaskPing, "T1", configMINIMAL_STACK_SIZE + 32, NULL, 1, _task_stack, &_task_tcb);

	csp_conf.hostname = "atmega";
	csp_conf.model = "328pb";
	csp_conf.revision = "1";
	csp_conf.conn_dfl_so = CSP_O_NONE;
	csp_conf.dedup = 0;
	csp_conf.version = 2;

	// uart_init();

	csp_xbee_add_interface(&user_data.iface);

	csp_init();
	static StaticTask_t router_tcb;
	static StackType_t router_stack[300] __attribute__((section(".noinit")));
	xTaskCreateStatic(router_task, "RTE", 300, NULL, 2, router_stack, &router_tcb);

	csp_bind_callback(csp_service_handler, CSP_ANY);
	// csp_bind_callback(param_serve, PARAM_PORT_SERVER);

	ENABLE_INTERRUPTS();

	USART0_set_ISR_cb(xbee_driver_rx, RX_CB);

	/*
	while (1) {
		send_16bit_frame(1, NULL, (uint8_t *)"hello", 5);
		if (frame_in_len > 0) {
			for (int i = 0; i < frame_in_len; i++) {
				printf("%02X ", frame_buf_in[i]);
			}
			printf("\n");
			frame_in_len = 0;
		}
		_delay_ms(1000);
	}
	*/

	// SSD1306_Init(SSD1306_ADDR);
	//  SSD1306_ClearScreen();
	/*
	TOP = 0x01,
	TOPDOUBLE = 0x03,
	MIDDLE = 0x01,
	MIDDLEDOUBLE = 0x18,
	BOTTOM = 0x80,
	BOTTOMDOUBLE = 0xC0
	*/
	/*
		SSD1306_DrawLineHorizontal(0, 0, END_COLUMN_ADDR, TOPDOUBLE);
		SSD1306_DrawLineHorizontal(0, 2, END_COLUMN_ADDR, MIDDLEDOUBLE);
		SSD1306_DrawLineHorizontal(0, END_PAGE_ADDR, END_COLUMN_ADDR, BOTTOM);
		SSD1306_DrawLineVertical(0, 0, END_PAGE_ADDR);
		SSD1306_DrawLineVertical(END_COLUMN_ADDR, 0, END_PAGE_ADDR);
	*/

	// SSD1306_HorizontalScroll(SSD1306_ADDR, 0, END_PAGE_ADDR);
	vTaskStartScheduler();
	while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName) {
	printf("stackoverflow\n");
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
	// printf("reboot\n");
}
void csp_shutdown_hook(void) {
}

static int kiss_driver_tx(void * driver_data, const unsigned char * data, size_t data_length) {

	for (uint16_t i = 0; i < data_length; i++) {
		if (USART1_is_tx_busy()) {
			_delay_us(1);
		}
		USART1_write(data[i]);
		// UDR1 = data[i];
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

void escape_byte(uint8_t byte, uint8_t * output, uint16_t * index) {
	if (byte == 0x7E || byte == 0x7D || byte == 0x11 || byte == 0x13) {
		output[(*index)++] = 0x7D;         // Escape character
		output[(*index)++] = byte ^ 0x20;  // XOR with 0x20
	} else {
		output[(*index)++] = byte;
	}
}

void xbee_driver_rx(void) {

	uint8_t data = UDR0;
	BaseType_t xTaskWoken = pdFALSE;

	if (data == 0x7D) {
		user_data.escape_next = true;  // Next byte is escaped
		return;                        // Wait for next byte
	}

	if (user_data.escape_next) {
		data ^= 0x20;  // Unescape the byte
		user_data.escape_next = false;
	}

	switch (user_data.xbee_mode) {
		case XBEE_MODE_NOT_STARTED:
			if (data == 0x7E) {
				// Start delimiter detected
				if (!user_data.rx_packet) {
					user_data.rx_packet = csp_buffer_get_isr(0);
				}
				user_data.header_sz = csp_id_setup_rx(user_data.rx_packet);
				user_data.xbee_mode = XBEE_MODE_LENGTH_MSB;
				user_data.length = 0;
				user_data.data_index = 0;
				user_data.checksum = 0;
			}
			break;

		case XBEE_MODE_LENGTH_MSB:
			user_data.length = data << 8;  // Store MSB of length
			user_data.xbee_mode = XBEE_MODE_LENGTH_LSB;
			break;

		case XBEE_MODE_LENGTH_LSB:
			user_data.length |= data;  // Store LSB of length
			if (user_data.length > CSP_BUFFER_SIZE + user_data.header_sz - XBEE_FRAME_LEN) {
				user_data.iface.rx_error++;
				user_data.iface.drop++;
				user_data.xbee_mode = XBEE_MODE_NOT_STARTED;
			} else {
				user_data.xbee_mode = XBEE_MODE_SKIP_FRAMING;
			}
			break;
		case XBEE_MODE_SKIP_FRAMING:
			user_data.checksum += data;
			user_data.data_index++;

			if (user_data.data_index >= XBEE_FRAME_LEN) {
				user_data.xbee_mode = XBEE_MODE_RECEIVING;
			}
			break;

		case XBEE_MODE_RECEIVING:
			if (user_data.data_index < user_data.length) {
				user_data.checksum += data;
				if (user_data.rx_packet->frame_length >= CSP_BUFFER_SIZE + user_data.header_sz) {
					user_data.iface.rx_error++;
					user_data.iface.drop++;
					user_data.xbee_mode = XBEE_MODE_SKIP_FRAMING;
					break;
				}
				user_data.rx_packet->frame_begin[user_data.rx_packet->frame_length++] = data;  // xd
				user_data.data_index++;
			} else {
				// Last byte is the checksum
				uint8_t calculated_checksum = 0xFF - user_data.checksum;
				if (calculated_checksum == data) {

					uint16_t frame_len = user_data.rx_packet->frame_length;
					user_data.iface.rxbytes += frame_len;
					user_data.iface.rx++;

					// for (uint16_t i = 0; i < frame_len; i++) {
					// printf("%02X\n", user_data.rx_packet->frame_begin[i]);
					//}

					if (csp_id_strip(user_data.rx_packet) < 0) {
						user_data.iface.frame++;
						user_data.xbee_mode = XBEE_MODE_NOT_STARTED;
						break;
					}

					csp_qfifo_write(user_data.rx_packet, &user_data.iface, &xTaskWoken);
					user_data.rx_packet = NULL;
				} else {
					user_data.iface.rx_error++;
					user_data.iface.drop++;
					// printf("%u!%u", calculated_checksum, data);
					//  Checksum invalid, handle error
				}
				// Reset state for next frame
				user_data.xbee_mode = XBEE_MODE_NOT_STARTED;
			}
			break;

		default:
			user_data.iface.rx_error++;
			user_data.iface.drop++;
			user_data.xbee_mode = XBEE_MODE_NOT_STARTED;
			break;
	}
}

int csp_xbee_tx(csp_iface_t * iface, uint16_t via, csp_packet_t * packet, int from_me) {

	// packet->id.flags |= 0xFE;
	csp_id_prepend(packet);
	send_16bit_frame(packet->id.dst, iface->driver_data, packet->frame_begin, packet->frame_length);
	csp_buffer_free(packet);

	return 0;
}

void send_16bit_frame(uint16_t dest_address, void * driver_data, const uint8_t * payload, uint16_t len) {
	uint8_t frame[128];
	uint16_t length = 0;
	uint8_t sum = 0x01;

	// Start delimiter
	frame[length++] = 0x7E;

	// Length (MSB, LSB)
	uint16_t frame_data_length = XBEE_FRAME_LEN + len;  // 5 bytes of frame data + payload length
	escape_byte((frame_data_length >> 8) & 0xFF, frame, &length);
	escape_byte(frame_data_length & 0xFF, frame, &length);

	// Transmit Request (16-bit address) 0x01
	frame[length++] = 0x01;
	frame[length++] = 0x00;

	// Destination address (MSB, LSB)
	sum += (dest_address >> 8) & 0xFF;
	escape_byte((dest_address >> 8) & 0xFF, frame, &length);
	sum += dest_address & 0xFF;
	escape_byte(dest_address & 0xFF, frame, &length);

	// no options
	frame[length++] = 0x00;
	// add payload
	for (uint8_t i = 0; i < len; i++) {
		sum += payload[i];
		escape_byte(payload[i], frame, &length);
	}

	// Checksum after len
	escape_byte(0xFF - sum, frame, &length);

	// Send the frame over UART
	for (uint16_t i = 0; i < length; i++) {
		USART0_write(frame[i]);
	}
}

int csp_xbee_add_interface(csp_iface_t * iface) {

	if ((iface == NULL) || (iface->name == NULL) || (iface->interface_data == NULL)) {
		return CSP_ERR_INVAL;
	}

	csp_xbee_interface_data_t * ifdata = iface->interface_data;
	if (ifdata->tx_func == NULL) {
		return CSP_ERR_INVAL;
	}

	ifdata->rx_length = 0;
	ifdata->rx_mode = XBEE_MODE_NOT_STARTED;
	ifdata->rx_packet = NULL;

	iface->nexthop = csp_xbee_tx;

	csp_iflist_add(iface);

	return CSP_ERR_NONE;
}

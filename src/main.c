/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

// Register for logging
LOG_MODULE_REGISTER(fundamentals_practice, LOG_LEVEL_INFO);

// Threads for handling reading the temp, averaging, setting led, and writing to console
#define STACK_SIZE 1024

#define TEMP_READ_THREAD_PRI 2
#define TEMP_AVG_THREAD_PRI 2
#define TEMP_AVG_LED_THREAD_PRI 1
#define TEMP_AVG_CONSOLE_THREAD_PRI 1

// Thread fun forward declarations
void temp_read_thread_cb(void* p1, void* p2, void* p3);
void temp_avg_thread_cb(void* p1, void* p2, void* p3);
void temp_avg_led_thread_cb(void* p1, void* p2, void* p3);
void temp_avg_console_thread_cb(void* p1, void* p2, void* p3);

K_THREAD_DEFINE(temp_read_thread, STACK_SIZE, temp_read_thread_cb, 
					NULL, NULL, NULL, TEMP_READ_THREAD_PRI, 0, 0);
K_THREAD_DEFINE(temp_avg_thread, STACK_SIZE, temp_avg_thread_cb, 
					NULL, NULL, NULL, TEMP_AVG_THREAD_PRI, 0, 0);
K_THREAD_DEFINE(temp_avg_led_thread, STACK_SIZE, temp_avg_led_thread_cb, 
					NULL, NULL, NULL, TEMP_AVG_LED_THREAD_PRI, 0, 0);
K_THREAD_DEFINE(temp_avg_console_thread, STACK_SIZE, temp_avg_console_thread_cb, 
					NULL, NULL, NULL, TEMP_AVG_CONSOLE_THREAD_PRI, 0, 0);

// Blocking semaphores
K_SEM_DEFINE(temp_data_read_sem, 0, 1);
K_SEM_DEFINE(temp_data_write_sem, 1, 1);
K_SEM_DEFINE(temp_avg_led_block_sem, 0, 1,);
K_SEM_DEFINE(temp_avg_console_block_sem, 0, 1);

// Creating a queue for temp data. This should be treated as a circular queue... for now I'll
// put this down, but maybe I'll just make a struct to handle the head/tail later
static uint32_t temp_data_queue[10];
// This index needs to wrap around the queue. We write over old entries. 
static uint32_t temp_data_index = 0;

// Keep track of old and new temperature averages. This will simply keep track of whether there
// was a difference or not. For now. I guess
static bool old_temp_avg = false;
static bool new_temp_avg = false;

// Flag for allowing the LED to be set. Thread sets it to false until it's ready again
static bool temp_avg_led_ready = true;

// LED used as an indicator for temperature change. When the average from the old and new changes
// by some amount, then the user will be told via LED

// LED node identifier
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec temp_change_led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Button node identifier
#define BUTTON_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec pause_read_button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

// Communication_interfaces
// Interfaces for communicating over I2C
#define I2C_NODE DT_ALIAS(tempsensor)
static const struct i2c_dt_spec temp_sens_intf = I2C_DT_SPEC_GET(I2C_NODE, i2c);

// Interfaces for communicating over UART
#define UART_NODE DT_NODELABEL(uart20)
static const struct device *temp_avg_console_intf = DEVICE_DT_GET(UART_NODE);

// UART event handler for writing to the console
static void temp_avg_console_uart_cb(const struct *dev, struct uart_event *evt, void *user_data)
{
	switch(evt->type)
	{
		case UART_TX_DONE:
		{

		}
		case UART_TX_ABORTED:
		{

		}
		case UART_RX_RDY:
		{

		}
		case UART_RX_BUF_REQUEST:
		{

		}
		case UART_RX_BUF_RELEASED:
		{

		}
		case UART_RX_DISABLED:
		{

		}
		case UART_RX_STOPPED:
		{

		}
		default:
		{

		}
	}
}

// Interrupt for pausing temp sensor reading
static struct gpio_callback temp_pause_read_cb_data;

int main(void)
{

	return 0;
}

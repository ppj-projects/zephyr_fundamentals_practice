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
void temp_read_thread_cb(void);
void temp_avg_thread_cb(void);
void temp_avg_led_thread_cb(void);
void temp_avg_console_thread_cb(void);

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

// Queue to temperature data. The index will wrap around
#define TEMP_DATA_QUEUE_MAX 10
#define TEMP_DATA_QUEUE_MIN 0
static uint32_t temp_data_queue[TEMP_DATA_QUEUE_MAX];
// This index needs to wrap around the queue. We write over old entries. 
static uint32_t temp_data_index = TEMP_DATA_QUEUE_MIN;

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

// Interrupt for pausing temp sensor reading
static struct gpio_callback temp_pause_read_cb_data;

// UART event handler for writing to the console
static void temp_avg_console_uart_cb(const struct *dev, struct uart_event *evt, void *user_data)
{
	switch(evt->type)
	{
		case UART_TX_DONE:
		{
			LOG_INF("Transmission complete!\n");
			break;
		}
		case UART_TX_ABORTED:
		{
			LOG_ERR("Transmission aborted\n");
			break;
		}
		case UART_RX_RDY:
		{
			LOG_INF("Not expecting data in for this application\n");
			break;
		}
		case UART_RX_BUF_REQUEST:
		{
			LOG_INF("Not expecting data in for this application\n");
			break;
		}
		case UART_RX_BUF_RELEASED:
		{
			LOG_ERR("Buffer released\n");
			break;
		}
		case UART_RX_DISABLED:
		{
			LOG_INF("Not expecting data in for this application\n");
			break;
		}
		case UART_RX_STOPPED:
		{
			LOG_INF("Not expecting data in for this application\n");
			break;
		}
		default:
		{
			LOG_WARN("How did we end up here?\n");
			break;
		}
	}
}

// Borrowed from previous lessons 
// BME280 I2C device addresses and register offsets
#define CTRLMEAS_REG 	0xF4
#define CALIB00_REG  	0x88
#define ID_REG			0xD0
#define TEMPMSB_REG		0xFA

// BME280 constants
#define CHIP_ID  0x60
//#define SENSOR_CONFIG_VALUE 0x93
#define SENSOR_CONFIG_TEMP_OVERSAMP_8 0b100'000'00
#define SENSOR_CONFIG_PRESS_OVERSAMP_8 0b000'100'00
#define SENSOR_CONFIG_NORMAL_MODE 0b000'000'11

// Compensation values for obtaining true values
// These are fixed in NVM at production time
// Unsigned, signed, signed
struct bme280_compensation_data 
{
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
} bme_comp_data;

// Calibration data - use to calibrate raw readings from sensor
// Use BEFORE bme280_fixup_raw_temp()
void bme280_get_compensation_data(const struct i2c_dt_spec *spec, struct bme280_compensation_data *comp_data)
{
	uint8_t temperature_compensation_values[6];

	// Read in compensation values and store them for later use
	int ret = i2c_burst_read_dt(spec, CALIB00_REG, temperature_compensation_values, sizeof(temperature_compensation_values));
	if(ret != 0)
	{
		LOG_ERR("Failed to read register %x \n", CALIB00_REG);
		return;
	}

	comp_data->dig_t1 = ((uint16_t)temperature_compensation_values[1]) << 8 | temperature_compensation_values[0];
	comp_data->dig_t2 = ((uint16_t)temperature_compensation_values[3]) << 8 | temperature_compensation_values[2];
	comp_data->dig_t3 = ((uint16_t)temperature_compensation_values[5]) << 8 | temperature_compensation_values[4];
}

// Formula provided by data sheet - Use compensation data and raw readings to obtain actual readings
static int32_t bme280_fixup_raw_temp(struct bme280_compensation_data *data, int32_t adc_temp)
{
	int32_t var1, var2;

	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) * ((int32_t)data->dig_t2)) >> 11;

	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >>
		 12) *
		((int32_t)data->dig_t3)) >>
	       14;

	return ((var1 + var2) * 5 + 128) >> 8;
}

// Initializes the BME280
int init_bme280(void)
{
	// Check if I2C bus is ready
	if(!device_is_ready(temp_sens_intf.bus))
	{
		LOG_ERR("I2c bus %s is not ready!\n\r", temp_sens_intf.bus->name);
		return -1;
	}

	// Verify IC by checking it's ID
	uint8_t bme280_id = 0;
	uint8_t bme280_id_reg[] = {ID_REG};

	int ret = i2c_write_read_dt(&temp_sens_intf, bme280_id_reg, sizeof(bme280_id_reg), &bme280_id, sizeof(bme280_id));

	if(ret != 0)
	{
		LOG_ERR("Failed to read register %x \n", bme280_id_reg[0]);
		return -1;
	}
	if(bme280_id != CHIP_ID)
	{
		LOG_ERR("Invalid chip id! %x \n", bme280_id);
		return -1;
	}
	
	// Get calibration values
	bme280_get_compensation_data(&temp_sens_intf, &bme_comp_data);

	// Reduce noise. Set temperature and pressure data to oversampling x8
	// Change from default sleep mode to normal mode for continuous readings
	uint8_t sensor_config[] = {CTRLMEAS_REG, 
		(SENSOR_CONFIG_TEMP_OVERSAMP_8 | SENSOR_CONFIG_PRESS_OVERSAMP_8 | SENSOR_CONFIG_NORMAL_MODE)};

	ret = i2c_write_dt(&temp_sens_intf, sensor_config, sizeof(sensor_config));

	if (ret != 0) {
		LOG_ERR("Failed to write register %x \n", sensor_config[0]);
		return -1;
	}

	return 0;
}

void temp_read_thread_cb(void)
{
	int ret = 0;
	uint8_t raw_temperature_value[3] = {0};
	uint32_t combined_temperature_value = 0;
	uint32_t compensated_temperature = 0;
	float temperature_celsius = 0;
	double temperature_fahrenheit = 0;

	while (1) 
	{
		// Grab the raw temperature data and order the bytes
		ret = i2c_burst_read_dt(&temp_sens_intf, TEMPMSB_REG, raw_temperature_value, sizeof(raw_temperature_value));

		if (ret != 0) {
			LOG_WARN("Failed to read register %x \n", TEMPMSB_REG);
			k_msleep(SLEEP_TIME_MS);
			continue;
		}
		combined_temperature_value = (raw_temperature_value[0] << 12) | (raw_temperature_value[1] << 4) | ((raw_temperature_value[2] >> 4) & 0x0F);

		// Fixup the raw data using calibration values. Convert to celcius and farenheit
		compensated_temperature = bme280_fixup_raw_temp(&bme_comp_data, combined_temperature_value);
		temperature_celsius = (float)compensated_temperature / 100.0f;
		temperature_fahrenheit = (double)temperature_celsius * 1.8 + 32;

		// Print reading to console
		//LOG_INF("Temperature in Celsius : %8.2f C\n", (double)temperature_celsius);
		//LOG_INF("Temperature in Fahrenheit : %.2f F\n", temperature_fahrenheit);

		k_msleep(SLEEP_TIME_MS);
	}
}

int main(void)
{
	int ret = 0;

	// Initialize BME280
	ret = init_bme280();
	if(ret != 0)
	{
		LOG_ERR("Failed to initialize bme280\n");
		return ret;
	}
	
	return 0;
}

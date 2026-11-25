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

// Creating a queue for temp data. This should be treated as a circular queue... for now I'll
// put this down, but maybe I'll just make a struct to handle the head/tail later
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
// BM280 I2C device addresses and register offsets
#define CTRLMEAS 	0xF4
#define CALIB00  	0x88
#define ID_REG		0xD0
#define TEMPMSB		0xFA

#define CHIP_ID  0x60
#define SENSOR_CONFIG_VALUE 0x93

// I guess this finds the nodee within an i2c bus. So the "instantiation" is already referencing both
// the device and it's bus
#define I2C_NODE DT_NODELABEL(mysensor)

/* Data structure to store BME280 data */
struct bme280_data {
	/* Compensation parameters */
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
} bmedata;

// Calibration data - use to calibrate raw readings from sensor
// Use BEFORE bme280_compensate_temp()
void bme_calibrationdata(const struct i2c_dt_spec *spec, struct bme280_data *sensor_data_ptr)
{
	/* Step 10 - Put calibration function code */
	uint8_t values[6];

	// Read the callibration data
	int ret = i2c_burst_read_dt(spec, CALIB00, values, sizeof(values));

	if(ret != 0)
	{
		printk("Failed to read register %x \n", CALIB00);
		return;
	}

	// I guess it's neater to put a *piece* of data into a struct, rather than individually
	// Left shift is division. This is the compensation stuff
	sensor_data_ptr->dig_t1 = ((uint16_t)values[1]) << 8 | values[0];
	sensor_data_ptr->dig_t2 = ((uint16_t)values[3]) << 8 | values[2];
	sensor_data_ptr->dig_t3 = ((uint16_t)values[5]) << 8 | values[4];
}

/* Compensate current temperature using previously stored sensor calibration data */
// Uses calibration data and raw readings to obtain actual readings
static int32_t bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp)
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



// Interrupt for pausing temp sensor reading
static struct gpio_callback temp_pause_read_cb_data;

int main(void)
{

	// Borrowed from previous lessons
	/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is
	 * ready to use  */
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	if(!device_is_ready(dev_i2c.bus))
	{
		printk("I2c bus %s is not ready!\n\r", dev_i2c.bus->name);
		return -1;
	}

	/* STEP 9 - Verify it is proper device by reading device id  */
	// Use this in drivers to identify if it works. They should have ID registers that are
	// always the same value
	uint8_t bme280_id = 0;
	uint8_t id_reg[] = {ID_REG};

	// Grab the ID reg value to verify it's the correct IC (bme280)
	int ret = i2c_write_read_dt(&dev_i2c, id_reg, sizeof(id_reg), &bme280_id, sizeof(bme280_id));

	if(ret != 0)
	{
		printk("Failed to read register %x \n", id_reg[0]);
		return -1;
	}
	if(bme280_id != CHIP_ID)
	{
		printk("Invalid chip id! %x \n", bme280_id);
		return -1;
	}
	
	bme_calibrationdata(&dev_i2c, &bmedata);

	/* STEP 11 - Setup the sensor by writing the value 0x93 to the Configuration register */
	// Write the reg address, then the data
	uint8_t sensor_config[] = {CTRLMEAS, SENSOR_CONFIG_VALUE};

	ret = i2c_write_dt(&dev_i2c, sensor_config, sizeof(sensor_config));

	if (ret != 0) {
		printk("Failed to write register %x \n", sensor_config[0]);
		return -1;
	}

	while (1) {

		/* STEP 12 - Read the temperature from the sensor */
		uint8_t temp_val[3] = {0};

		// Grab raw data...
		int ret = i2c_burst_read_dt(&dev_i2c, TEMPMSB, temp_val, sizeof(temp_val));

		if (ret != 0) {
			printk("Failed to read register %x \n", TEMPMSB);
			k_msleep(SLEEP_TIME_MS);
			continue;
		}

		/* STEP 12.1 - Put the data read from registers into actual order (see datasheet) */
		// .. Order LSB and MSB...
		uint32_t adc_temp = (temp_val[0] << 12) | (temp_val[1] << 4) | ((temp_val[2] >> 4) & 0x0F);

		/* STEP 12.2 - Compensate temperature */
		// ... then compensate for real values
		uint32_t comp_temp = bme280_compensate_temp(&bmedata, adc_temp);

		/* STEP 12.3 - Convert temperature */
		// Convert data to F and C
		float temperature = (float)comp_temp / 100.0f;
		double fTemp = (double)temperature * 1.8 + 32;

		// Print reading to console
		printk("Temperature in Celsius : %8.2f C\n", (double)temperature);
		printk("Temperature in Fahrenheit : %.2f F\n", fTemp);

		k_msleep(SLEEP_TIME_MS);
	}


	
	return 0;
}

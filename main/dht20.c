#include <stdio.h>
#include <stdint.h> 
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/i2c_master.h" 
#include "esp_log.h" 

// i2c
#define SDA_GPIO 22
#define SCL_GPIO 21
#define DHT20_ADDR 0x38 // 7-bit i2c address
#define DHT20_SCL_SPEED_HZ 100000 // 100kHz -> slow mode, 400kHz -> fast mode

// commands
#define DHT20_CMD_MEASURE 0xAC // trigger measurement
#define DHT20_CMD_MEASURE_ARG1 0x33 // 1st argument of measure command
#define DHT20_CMD_MEASURE_ARG2 0x00 // 2nd argument of measure command

// status byte
#define DHT20_STATUS_BUSY 0x80 // bit[7]: 1 - busy, 0 - idle
#define DHT20_STATUS_CAL_MASK 0x18 // bit[3]: 1 - calibrated, 0 - uncalibrated 

// data layout
#define DHT20_DATA_LEN 7 // status + 5 data bytes + CRC
#define DHT20_NIBBLE_MASK 0x0F // keeps lower 4 bits

// conversion 
#define DHT20_RAW_FULL_SCALE 1048576.0f // 2^20, max value of 20-bit reading
#define DHT20_HUM_SCALE 100.0f // humidity range in %
#define DHT20_TEMP_SCALE 200.0f // temperature range in Celsius 
#define DHT20_TEMP_OFFSET 50.0f // temperature offset in Celsius

// delays in ms
#define DHT20_DELAY_POWER_ON_MS 300 // wait after power-on, datasheet needs >= 100
#define DHT20_DELAY_BEFORE_CMD_MS 10 // pause between trigger command
#define DHT20_DELAY_MEASURE_MS 80 // measurement time 
#define DHT20_PERIOD_MS 3000 // pause between measurements, datasheet advises >= 3000

// i2c transaction timeout
#define DHT20_XFER_TIMEOUT_MS 100 


static const char *TAG = "dht20";

void app_main(void) {
	vTaskDelay(pdMS_TO_TICKS(DHT20_DELAY_POWER_ON_MS));
	// adding bus
	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = SDA_GPIO, 
		.scl_io_num = SCL_GPIO, 
		.clk_source = I2C_CLK_SRC_DEFAULT, 
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t bus;
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
	
	
	// checking if anyone is on the bus
	for (uint8_t addr = 1; addr < 127; addr++) {
		if (i2c_master_probe(bus, addr, DHT20_XFER_TIMEOUT_MS) == ESP_OK) {
			ESP_LOGI(TAG, "find device at 0x%02X", addr);
		} 
	} 
	
	
	// adding device 
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = DHT20_ADDR,
		.scl_speed_hz = DHT20_SCL_SPEED_HZ,
	};
	i2c_master_dev_handle_t dht20;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dht20));
	
	// check if dht20 is calibrated
	uint8_t calibrate_output = 0;
	
	ESP_ERROR_CHECK(i2c_master_receive(dht20, &calibrate_output, 1, DHT20_XFER_TIMEOUT_MS));
	if ((calibrate_output & 0x18) == 0x18) {
		ESP_LOGI(TAG, "calibrate output: 0x%02X (expected 0x18)", calibrate_output);
	} else {
		ESP_LOGW(TAG, "calibrate output: 0x%02X (expected 0x18)", calibrate_output);
	}	
	
	
	// reading temp/hum in loop
	while (1) {
		// triggering temp and hum reading
		uint8_t trigger_reading[3] = {DHT20_CMD_MEASURE, DHT20_CMD_MEASURE_ARG1, DHT20_CMD_MEASURE_ARG2};
		vTaskDelay(pdMS_TO_TICKS(DHT20_DELAY_BEFORE_CMD_MS));
	
		esp_err_t err1 = (i2c_master_transmit(dht20, trigger_reading, 3, DHT20_XFER_TIMEOUT_MS));
		if (err1 != ESP_OK) {
			ESP_LOGW(TAG, "dht20 read failed: %s", esp_err_to_name(err1));
			vTaskDelay(pdMS_TO_TICKS(DHT20_PERIOD_MS));
			continue;
		}
		vTaskDelay(pdMS_TO_TICKS(DHT20_DELAY_MEASURE_MS));
		
		
		uint8_t data[7];
		
		// reading temp and hum bits
		esp_err_t err2 = (i2c_master_receive(dht20, data, 7, DHT20_XFER_TIMEOUT_MS));
		if (((data[0] & DHT20_STATUS_BUSY) == 0) && (err2 == ESP_OK))  {
			uint32_t raw_hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
			uint32_t raw_temp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | ((uint32_t)data[5]);
			
			ESP_LOGI(TAG, "hum: %.2f%% | temp: %.2f",
				(raw_hum / DHT20_RAW_FULL_SCALE * DHT20_HUM_SCALE), (raw_temp / DHT20_RAW_FULL_SCALE * DHT20_TEMP_SCALE - DHT20_TEMP_OFFSET));
		} else {
			ESP_LOGW(TAG, "dht20 read failed: %s", esp_err_to_name(err2));
		}
		
		vTaskDelay(pdMS_TO_TICKS(DHT20_PERIOD_MS));
		
		
		
	}	
	
	
	
	
	
}

#include <stdio.h>
#include <stdint.h> 
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/i2c_master.h" 
#include "esp_log.h" 

#define SDA_GPIO 22
#define SCL_GPIO 21
#define DHT20_ADDR 0x38

static const char *TAG = "dht20";

void app_main(void) {
	vTaskDelay(pdMS_TO_TICKS(300));
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
		if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
			ESP_LOGI(TAG, "find device at 0x%02X", addr);
		} 
	} 
	
	
	// adding device 
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = DHT20_ADDR,
		.scl_speed_hz = 100000 // slow mode
	};
	i2c_master_dev_handle_t dht20;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dht20));
	
	// check if dht20 is calibrated
	// uint8_t calibrate_reg = 0x71;
	uint8_t calibrate_output = 0;
	
	ESP_ERROR_CHECK(i2c_master_receive(dht20, &calibrate_output, 1, 100));
	ESP_LOGI(TAG, "calibrate output: 0x%02X (expected 0x18)", calibrate_output);
	
	
	// reading temp/hum in loop
	while (1) {
		// triggering temp and hum reading
		uint8_t trigger_reading[3] = {0xAC, 0x33, 0x00};
		vTaskDelay(pdMS_TO_TICKS(10));
	
		ESP_ERROR_CHECK(i2c_master_transmit(dht20, trigger_reading, 3, 100));
		vTaskDelay(pdMS_TO_TICKS(80));
		
		
		uint8_t data[7];
		
		ESP_ERROR_CHECK(i2c_master_receive(dht20, data, 7, 100));
		if ((data[0] & 0x80) == 0)  {
			uint32_t raw_hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
			uint32_t raw_temp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | ((uint32_t)data[5]);
			
			ESP_LOGI(TAG, "hum: %.2f%% | temp: %.2f",
				(raw_hum / 1048576.0f * 100.0f), (raw_temp / 1048576.0f * 200.0f - 50.0f));
		} else {
			ESP_LOGW(TAG, "dht20 is busy");
		}
		
		vTaskDelay(pdMS_TO_TICKS(3000));
		
		
		
	}	
	
	
	
	
	
}

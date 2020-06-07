#include <stdio.h>
#include "driver/i2c.h"

#include "bme280_idf.h"
#include "driver/bme280.h"

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


i2c_port_t _port_num;
uint8_t _dev_addr;
struct bme280_dev _dev;
struct bme280_data _comp_data;
uint32_t _req_delay;

// Delay function to provide to the bme280 driver
void bme280_delay_ms(uint32_t ms)
{
    vTaskDelay(ms/portTICK_PERIOD_MS);
}

// I2C Register write function, this pointer will be provided to the bme280 driver
int8_t bme280_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t count)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

    for(int i = 0; i < count; i++)
    {
    i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
    }

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_port_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret == ESP_OK)
        return 0;
    
    return -1;
}

// I2C Register read function, this pointer will be provided to the bme280 driver
int8_t bme280_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t count)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_dev_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    if (count > 1)
    {
        i2c_master_read(cmd, data, count - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + count - 1, NACK_VAL);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_port_num, cmd, 200 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret == ESP_OK)
        return 0;
    
    return -1;
}

esp_err_t  bme280_begin(i2c_port_t port_num, uint8_t dev_addr)
{
    if(!(dev_addr == 0x77 || dev_addr == 0x76))
        return ESP_ERR_INVALID_ARG;
    
    _port_num = port_num;
    _dev_addr = dev_addr;

    struct bme280_dev dev = {
        .intf = BME280_I2C_INTF,
		.write = bme280_write_reg,
		.read = bme280_read_reg,
		.dev_id = _dev_addr,
		.delay_ms =bme280_delay_ms 
	};

    int8_t ret = BME280_OK;

    ret = bme280_init(&dev);
    if(ret != BME280_OK)
        return ESP_ERR_NOT_FOUND;
    
    uint8_t settings_sel = 0;
    uint32_t req_delay;

    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    ret = bme280_set_sensor_settings(settings_sel,&dev);
    if(ret != BME280_OK)
        return ESP_ERR_INVALID_ARG;


    // how fast we can sample
    req_delay = bme280_cal_meas_delay(&dev.settings);


    // Save the setup to global scope
    _dev = dev;
    _req_delay = req_delay;

    return ESP_OK;
}

esp_err_t bme280_get_forced_data(struct bme280_data *comp)
{
    int8_t ret = bme280_set_sensor_mode(BME280_FORCED_MODE, &_dev);
    if(ret != BME280_OK)
        return ESP_ERR_INVALID_RESPONSE;
    
    ret = bme280_get_sensor_data(BME280_ALL,comp,&_dev);
    if(ret != BME280_OK)
        return ESP_ERR_INVALID_RESPONSE;
    
    return ESP_OK;
}
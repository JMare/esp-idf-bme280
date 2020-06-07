#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <driver/i2c.h>

#include "bme280_idf.h"
#include "bme280_defs.h"

static const char* TAG = "MAIN";

#define I2C_SCL_IO				22	//19               /*!< gpio number for I2C master clock */
#define I2C_SDA_IO				21	//18               /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ				400000           /*!< I2C master clock frequency */
#define I2C_PORT_NUM			I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */

#define BQ27441_I2C_ADDRESS 0x55

extern "C" {
    void app_main(void);
};

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(I2C_PORT_NUM, &conf);
    return i2c_driver_install(I2C_PORT_NUM, conf.mode, I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    ESP_LOGI(TAG,"MAIN ENTRY");

    i2c_master_init();
    i2c_set_timeout(I2C_PORT_NUM,0xFFFFF);

    ESP_ERROR_CHECK(bme280_begin(I2C_PORT_NUM,0x77));

    while(true)
    {
        vTaskDelay(300/portTICK_PERIOD_MS);

        struct bme280_data comp_data;
        bme280_get_forced_data(&comp_data);

        float temp, press, hum;

        temp = comp_data.temperature;
        press = 0.01 * comp_data.pressure;
        hum = comp_data.humidity;
        printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
    }
}

#ifndef BME280_IDF_H
#define BME280_IDF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <driver/i2c.h>

esp_err_t  bme280_begin(i2c_port_t port_num, uint8_t dev_addr);

#ifdef __cplusplus
}
#endif

#endif
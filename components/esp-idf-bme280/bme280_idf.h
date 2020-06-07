#ifndef BME280_IDF_H
#define BME280_IDF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <driver/i2c.h>
#include "driver/bme280_defs.h"

esp_err_t  bme280_begin(i2c_port_t port_num, uint8_t dev_addr);
esp_err_t bme280_get_forced_data(struct bme280_data *comp);

#ifdef __cplusplus
}
#endif

#endif
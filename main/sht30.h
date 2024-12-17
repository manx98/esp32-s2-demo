#ifndef _SHT30_H_
#define _SHT30_H_

#include <esp_err.h>

esp_err_t sht30_init();

esp_err_t sht30_read(float *temp_data, float *hum_data);
#endif
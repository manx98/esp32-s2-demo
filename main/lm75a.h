#ifndef LM75A_H
#define LM75A_H

esp_err_t lm75a_read_temperature(float *temp);

esp_err_t lm75a_set_tos(float tos);

esp_err_t lm75a_set_thys(float thys);

esp_err_t lm75a_get_tos(float *temp);

esp_err_t lm75a_get_thys(float *temp);

esp_err_t lm75a_set_interrupt(int en);
#endif
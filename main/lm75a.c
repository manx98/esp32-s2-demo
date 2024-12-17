#include <esp_log.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <u8g2_esp32_hal.h>

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "lm75a.h"

#define LM75A_SLAVE_ADDR            0x48                    /*!< LM75A slave address, you can set any 7bit value */
#define ACK_VAL                     0x0                     /*!< I2C ack value */
#define NACK_VAL                    0x1                     /*!< I2C nack value */
#define WRITE_BIT                   I2C_MASTER_WRITE        /*!< I2C master write */
#define READ_BIT                    I2C_MASTER_READ         /*!< I2C master read */
#define ACK_CHECK_EN                0x1                     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS               0x0                     /*!< I2C master will not check ack from slave */

#define TAG "LM75A"

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = i2c_master_start(cmd);
    if(ret != ESP_OK)
    {
        ESP_LOGW(TAG, "i2c_master_start: %d", ret);
        goto out;
    }
    ret = i2c_master_write_byte(cmd, ( LM75A_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if(ret != ESP_OK)
    {
        ESP_LOGW(TAG, "i2c_master_write_byte: %d", ret);
        goto out;
    }
    if (size > 1) {
        ret = i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
        if(ret != ESP_OK)
        {
            ESP_LOGW(TAG, "i2c_master_read: %d", ret);
            goto out;
        }
    }
    ret = i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    if(ret != ESP_OK)
    {
        ESP_LOGW(TAG, "i2c_master_read_byte: %d", ret);
        goto out;
    }
    ret = i2c_master_stop(cmd);
    if(ret != ESP_OK)
    {
        ESP_LOGW(TAG, "i2c_master_stop: %d", ret);
        goto out;
    }
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    if(ret != ESP_OK)
    {
        ESP_LOGW(TAG, "i2c_master_cmd_begin: %d", ret);
    }
out:
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_start: %d", ret);
        goto out;
    }
    ret = i2c_master_write_byte(cmd, ( LM75A_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_write_byte: %d", ret);
        goto out;
    }
    ret = i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_write: %d", ret);
        goto out;
    }
    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_stop: %d", ret);
        goto out;
    }
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_cmd_begin: %d", ret);
    }
out:
    i2c_cmd_link_delete(cmd);
    return ret;
}

static float convert_lm75a_data_to_temperature(uint8_t  data_read[2]) {
    int16_t i16 = (data_read[0] << 8) | data_read[1];
    return i16 / 256.0;
}

static void convert_temperature_to_lm75a_data(uint8_t  data_write[3], float temp) {
    int16_t i16 = (int16_t)(temp*256) & 0xFF80;
    data_write[1]=(i16 >> 8) & 0xff;
    data_write[2]=i16 & 0xff;
}

esp_err_t lm75a_read_temperature(float *temp) {
    uint8_t  buf[2];
    buf[0] = 0;
    buf[1] = 0;
    *temp = 0;
    esp_err_t ret = i2c_master_write_slave(I2C_MASTER_NUM, buf, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_write_slave: %d", ret);
        return ret;
    }
    ret = i2c_master_read_slave(I2C_MASTER_NUM, buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_read_slave: %d", ret);
    } else {
        *temp = convert_lm75a_data_to_temperature(buf);
        ESP_LOGD(TAG, "temperature: %f", *temp);
    }
    return ret;
}

esp_err_t lm75a_set_tos(float tos) {
    ESP_LOGD(TAG, "set tos: %.1f", tos);
    uint8_t buf[4];
    buf[0] = 0x3;
    convert_temperature_to_lm75a_data(buf, tos);
    return i2c_master_write_slave(I2C_MASTER_NUM, buf, 3);
}

esp_err_t lm75a_set_thys(float thys) {
    ESP_LOGD(TAG, "set thys: %.1f", thys);
    uint8_t buf[3];
    buf[0] = 0x2;
    convert_temperature_to_lm75a_data(buf, thys);
    return i2c_master_write_slave(I2C_MASTER_NUM, buf, 3);
}

esp_err_t lm75a_get_tos(float *temp) {
    uint8_t buf[4];
    buf[0] = 0x3;
    *temp = 0;
    esp_err_t ret = i2c_master_write_slave(I2C_MASTER_NUM, buf, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_write_slave: %d", ret);
        return ret;
    }
    ret = i2c_master_read_slave(I2C_MASTER_NUM, buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_read_slave: %d", ret);
    } else {
        *temp = convert_lm75a_data_to_temperature(buf);
        ESP_LOGD(TAG, "lm75a_get_tos: %f", *temp);
    }
    return ret;
}

esp_err_t lm75a_get_thys(float *temp) {
    uint8_t buf[4];
    buf[0] = 0x2;
    *temp = 0;
    esp_err_t ret = i2c_master_write_slave(I2C_MASTER_NUM, buf, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_write_slave: %d", ret);
        return ret;
    }
    ret = i2c_master_read_slave(I2C_MASTER_NUM, buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_read_slave: %d", ret);
    } else {
        *temp = convert_lm75a_data_to_temperature(buf);
        ESP_LOGD(TAG, "lm75a_get_thys: %f", *temp);
    }
    return ret;
}

esp_err_t lm75a_set_interrupt(int en) {
    uint8_t buf[2];
    en = !!en;
    if(en) {
        ESP_LOGD(TAG, "lm75a_set_interrupt: %d" , en);
        buf[0] = 0x1;
        buf[1] = 0x02;
    } else {
        ESP_LOGD(TAG, "lm75a_set_interrupt: %d" , en);
        buf[0] = 0x1;
        buf[1] = 0;
    }
    esp_err_t ret = i2c_master_write_slave(I2C_MASTER_NUM, buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_write_slave: %d", ret);
    } else
    {
        ret = i2c_master_read_slave(I2C_MASTER_NUM, buf, 2); //do one time read to clean interrupt before enter interrupt mode.
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "i2c_master_read_slave: %d", ret);
        }
    }
    return ret;
}

#include "sht30.h"
#include <u8g2_esp32_hal.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define SHT30_SENSOR_ADDR 0x44
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static const char *TAG = "SHT30";

const uint8_t g_polynom = 0x31;
static uint8_t crc8 (const uint8_t data[], int len)
{
    uint8_t crc = 0xff;
    for (int i=0; i < len; i++)
    {
        crc ^= data[i];

        for (int i = 0; i < 8; i++)
        {
            bool xor = crc & 0x80;
            crc = crc << 1;
            crc = xor ? crc ^ g_polynom : crc;
        }
    }
    return crc;
}

esp_err_t sht30_init()
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, 0xF3, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, 0x2D, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(30));

    uint8_t data[3];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR<< 1 | READ_BIT, ACK_CHECK_DIS);
    i2c_master_read(cmd, &data[0], sizeof(data), ACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(50));
    for (uint8_t i = 0; i < sizeof(data); i++)
    {
        printf("status:=%d:%d\n",i,data[i]);
    }
    if (crc8(data,2) != data[2])
    {
        ESP_LOGE(TAG,"CRC check for failed\n");
        return ret;
    }
    return ret;
}

esp_err_t sht30_read(float *temp_data, float *hum_data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x24, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(2000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"ESP WRITE CMD ERROR ");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    uint8_t data[6];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR<< 1 | READ_BIT, ACK_CHECK_EN);
    // i2c_master_read(cmd, &data[0], sizeof(data), ACK_VAL);
    i2c_master_read_byte(cmd, &data[0], ACK_VAL);
    i2c_master_read_byte(cmd, &data[1], ACK_VAL);
    i2c_master_read_byte(cmd, &data[2], ACK_VAL);
    i2c_master_read_byte(cmd, &data[3], ACK_VAL);
    i2c_master_read_byte(cmd, &data[4], ACK_VAL);
    i2c_master_read_byte(cmd, &data[5], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(5000));
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(30));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"ESP READ  CMD ERROR ");
        return ret;
    }
    if (crc8(data,2) != data[2])
    {
        ESP_LOGE(TAG,"CRC check for temp data failed");
        return ret;
    }
    // check humidity crc
    if (crc8(data+3,2) != data[5])
    {
        ESP_LOGE(TAG,"CRC check for humidity data failed");
        return ret;
    }
    *temp_data =((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
    *hum_data = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);
    return ret;
}
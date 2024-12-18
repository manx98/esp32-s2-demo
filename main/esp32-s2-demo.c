#include <esp_log.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <u8g2.h>
#include <u8g2_esp32_hal.h>

#include "config.h"
#include "wifi.h"
#include "lm75a.h"
#include "sht30.h"

#include <sys/unistd.h>

static void init_u8g2_esp32()
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
    u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    // 针对不同的屏幕使用不同的初始化函数
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2, U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8, OLED_I2C_ADDRESS << 1);
    u8g2_InitDisplay(&u8g2);     // send init sequence to the display, display is in
    // sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    u8g2_ClearBuffer(&u8g2);
}
static void init_wifi()
{
    //NVS初始化（WIFI底层驱动有用到NVS，所以这里要初始化）
    ESP_ERROR_CHECK(nvs_flash_init());
    //wifi STA工作模式初始化
    ESP_ERROR_CHECK(wifi_sta_init(WIFI_SSID, WIFI_PASSWORD));
}

static char* TAG = "main";

void i2c_scanner() {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    // Iterate over all possible I2C addresses
    for (int addr = 1; addr < 127; addr++) {
        // Create I2C command link
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // Send I2C address with write bit
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        // Execute I2C command and check for response
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);

        // If device responds, print the address
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02x", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan complete.");
}
void app_main(void)
{
    init_u8g2_esp32();
    init_wifi();
    i2c_scanner();
    ESP_ERROR_CHECK(sht30_init());
    float temp, sht30_temp, sht30_hum;
    char buf[100];
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        lm75a_read_temperature(&temp);
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
        sprintf(buf, "TMP: %.2f", temp);
        u8g2_DrawStr(&u8g2, 0, 10, buf);
        if(sht30_read(&sht30_temp, &sht30_hum) != ESP_OK) {
            sht30_temp = 0;
            sht30_hum = 0;
        }
        sprintf(buf, "T:%.2f H:%.2f", sht30_temp, sht30_hum);
        u8g2_DrawStr(&u8g2, 0, 20, buf);
        u8g2_SendBuffer(&u8g2);
        // 打印 PSRAM 的大小
        ESP_LOGI(TAG, "PSRAM Size: %zu/%zu bytes", heap_caps_get_total_size(MALLOC_CAP_SPIRAM), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI(TAG, "Free heap size: %lu", esp_get_free_heap_size());
    }
}

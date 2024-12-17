#include <esp_log.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <u8g2.h>
#include <u8g2_esp32_hal.h>
#include <esp_private/pm_impl.h>

#include "config.h"
#include "wifi.h"
#include "lm75a.h"
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

void app_main(void)
{
    init_u8g2_esp32();
    init_wifi();
    float temp;
    char buf[100];
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "lm75a_read_temperature");
        lm75a_read_temperature(&temp);
        ESP_LOGI(TAG, "u8g2_ClearBuffer");
        u8g2_ClearBuffer(&u8g2);
        ESP_LOGI(TAG, "u8g2_SetFont");
        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
        ESP_LOGI(TAG, "temp: %f", temp);
        sprintf(buf, "TMP: %.2f", temp);
        u8g2_DrawStr(&u8g2, 0, 16, buf);
        ESP_LOGI(TAG, "u8g2_SendBuffer start");
        u8g2_SendBuffer(&u8g2);
        ESP_LOGI(TAG, "u8g2_SendBuffer done");
    }
}

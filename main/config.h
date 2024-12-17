//
// Created by Shinelon on 2024/12/16.
//

#ifndef CONFIG_H
#define CONFIG_H
#include <u8g2.h>

#define PIN_SCL GPIO_NUM_1
#define PIN_SDA GPIO_NUM_2

#define I2C_SCAN_START_ADDR 1  // 起始地址
#define I2C_SCAN_END_ADDR   127 // 结束地址

#define OLED_I2C_ADDRESS 0x3c

static u8g2_t u8g2;

#endif //CONFIG_H

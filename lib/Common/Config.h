#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// #include "Config.h"
#include "Display.h"

/**** SD卡 GPIO 映射 ****/
#define SS_PIN          5
//#define MOSI_PIN        23
//#define MISO_PIN        19
//#define CLK_PIN         18
// #define CS    23
// #define DI    22
// #define SCLK  21
// #define DO    19

// #define CS    5
// #define DI    23
// #define SCLK  19
// #define DO    18

/****HUB75 GPIO 映射****/
//ESP32上的GPIO 34+仅用于输入！！
//https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// #define R1_PIN  14  // library default for the esp32, unchanged 25
// #define G1_PIN  27  // library default for the esp32, unchanged 26
// #define B1_PIN  26  // library default for the esp32, unchanged 27
// #define R2_PIN  25  // library default for the esp32, unchanged 14
// #define G2_PIN  33  // library default for the esp32, unchanged 12
// #define B2_PIN  32  // library default for the esp32, unchanged 13
// #define A_PIN   13  // remap esp32 library default from 23 to 33
// #define B_PIN   15  // remap esp32 library default from 19 to 32
// #define C_PIN   2   // remap esp32 library defaultfrom 5 to 22
// #define D_PIN   4  // library default for the esp32, unchanged 17
// #define E_PIN   16 // IMPORTANT: Change to a valid pin if using a 64x64px panel. 32
            
// #define LAT_PIN 5  // library default for the esp32, unchanged 4
// #define OE_PIN  18  // library default for the esp32, unchanged 15
// #define CLK_PIN 17  // library default for the esp32, unchanged 16

//  ESP32-HUB75E 开发板
#define R1_PIN 25
#define G1_PIN 26
#define B1_PIN 27
#define R2_PIN 14
#define G2_PIN 17
#define B2_PIN 13
#define A_PIN 32
#define B_PIN 33
#define C_PIN 21
#define D_PIN 15
#define E_PIN 12 
#define LAT_PIN 4
#define OE_PIN 22
#define CLK_PIN 16
/***************************************************************
 * HUB 75 LED DMA Matrix Panel Configuration
 **************************************************************/
#define PANEL_RES_X 64      // 每个独立面板模块的像素宽度
#define PANEL_RES_Y 64     //  每个独立面板模块的像素高度。
#define NUM_ROWS 1                // 显示屏独立面板的行数
#define NUM_COLS  1               // 显示屏独立面板的列数
#define PANEL_CHAIN NUM_ROWS *NUM_COLS // 一个面板链接到另一个面板的总数

// 根据您的需要更改此项，有关VirtualPanel的详细信息，请阅读PDF！
#define SERPENT false   //蛇形
#define TOPDOWN false   //上下
/**************************************************************/

// #define MATRIX_WIDTH PANEL_WIDTH
// #define MATRIX_HEIGHT PANEL_HEIGHT
#define NUM_LEDS PANEL_WIDTH*PANEL_HEIGHT

// 面板尺寸
#define PANEL_WIDTH PANEL_RES_X *NUM_COLS    //显示屏宽度
#define PANEL_HEIGHT PANEL_RES_Y *NUM_ROWS   //显示屏高度

// Clock
#define CLOCK_X 3
#define CLOCK_Y 21
#define CLOCK_SEGMENT_HEIGHT 8
#define CLOCK_SEGMENT_WIDTH 8
#define CLOCK_SEGMENT_SPACING 5
#define CLOCK_WIDTH 6*(CLOCK_SEGMENT_WIDTH+CLOCK_SEGMENT_SPACING)+4
#define CLOCK_HEIGHT 2*CLOCK_SEGMENT_HEIGHT+3
//color565 == ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3)
#define CLOCK_DIGIT_COLOR  ((0x00 & 0xF8) << 8) | ((0xFF & 0xFC) << 3) | (0xFF >> 3)
//时钟动画的延迟（毫秒）-对于8的片段大小，应低于30ms
#define CLOCK_ANIMATION_DELAY_MSEC 20

// Day of week
#define DOW_X 90
#define DOW_Y 21
#define DOW_COLOR ((0x00 & 0xF8) << 8) | ((0x40 & 0xFC) << 3) | (0xFF >> 3)
// Date
#define DATE_X DOW_X
#define DATE_Y DOW_Y+11
#define DATE_COLOR DOW_COLOR
//Width and height are for both DATE and DOW
#define DATE_WIDTH 40
#define DATE_HEIGHT 18


// 天气传感器数据
#define SENSOR_DATA_X 0
#define SENSOR_DATA_Y 0
#define SENSOR_DATA_WIDTH 192
#define SENSOR_DATA_HEIGHT 8
#define SENSOR_DATA_COLOR ((0x00 & 0xF8) << 8) | ((0x8F & 0xFC) << 3) | (0x00 >> 3)
#define SENSOR_ERROR_DATA_COLOR ((0xFF & 0xF8) << 8) | ((0x00 & 0xFC) << 3) | (0x00 >> 3)

// 光传感器数据
#define LIGHT_DATA_X 0
#define LIGHT_DATA_Y 9
#define LIGHT_DATA_WIDTH 44
#define LIGHT_DATA_HEIGHT 8
#define LIGHT_DATA_COLOR ((0x00 & 0xF8) << 8) | ((0xFF & 0xFC) << 3) | (0x00 >> 3)
//可接受为有效的最大照度值（有时传感器会返回错误值）
#define LIGHT_THRESHOLD 9999
#define LIGHT_READ_INTERVAL_SEC 10

// 在底部记录消息
#define LOG_MESSAGE_COLOR ((0xFF & 0xF8) << 8) | ((0x00 & 0xFC) << 3) | (0x00 >> 3)

#define BITMAP_X 0
#define BITMAP_Y 44

#define HEARTBEAT_X 120
#define HEARTBEAT_Y 21

// 看门狗设置
#define WDT_TIMEOUT 60   //如果WDT在X秒内未复位，请重新启动设备
                         //不要设置得太低，否则WDT将阻止OTA更新完成！！

// 天气-今天和5天预报
#define WEATHER_TODAY_X 56
#define WEATHER_TODAY_Y 1

#define WEATHER_FORECAST_X 90
#define WEATHER_FORECAST_Y 44

//今天的温度范围
#define TEMPRANGE_X 0
#define TEMPRANGE_Y 44
#define TEMPRANGE_WIDTH 64
#define TEMPRANGE_HEIGHT 8
#define TEMPRANGE_COLOR ((0x00 & 0xF8) << 8) | ((0xFF & 0xFC) << 3) | (0xFF >> 3)

//多久刷新一次天气预报数据
//（受API节流限制）
#define WEATHER_REFRESH_INTERVAL_SEC 14400

#endif //CONFIG_H
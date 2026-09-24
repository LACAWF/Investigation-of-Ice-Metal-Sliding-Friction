#ifndef _BSP_HX711_H_
#define _BSP_HX711_H_
 
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
 
// 1. 引脚定义：这里改成你实际连接的GPIO号！
#define HX711_SCK_PIN     1  // 时钟引脚，连接至HX711的SCK
#define HX711_DT_PIN      2  // 数据引脚，连接至HX711的DT
 
// 2. 操作宏定义：让代码更易读
#define DT_OUT()        gpio_set_direction(HX711_DT_PIN, GPIO_MODE_OUTPUT)
#define DT_IN()         gpio_set_direction(HX711_DT_PIN, GPIO_MODE_INPUT)
#define DT_GET()        gpio_get_level(HX711_DT_PIN)
#define DT(x)           gpio_set_level(HX711_DT_PIN, (x?1:0))
#define SCK(x)          gpio_set_level(HX711_SCK_PIN, (x?1:0))
 
// 3. 函数声明
void delay_us(unsigned int us);
void delay_ms(unsigned int ms);
void HX711_GPIO_Init(void);
float Get_Weight(void);
void Get_Maopi(void);
 
#endif
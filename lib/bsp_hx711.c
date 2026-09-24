#include "bsp_hx711.h"
#include "stdio.h"
 
unsigned int HX711_Buffer;    // 存储一次ADC读取的原始值
unsigned int Weight_Maopi;    // 存储“毛皮”（空载）时的ADC值
int Weight_Shiwu;             // 实物净重的ADC值（原始值-毛皮值）
unsigned char Flag_Error = 0; // 错误标志（示例代码未使用）
 
// ！！！核心校准参数 ！！！
// 这个值每个传感器都不一样，需要实际校准。重量偏大就调大它，偏小就调小它。
#define GapValue 208.05
 
// 简单的延时函数，基于FreeRTOS和ESP32的底层延时
void delay_ms(unsigned int ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
void delay_us(unsigned int us) {
    ets_delay_us(us); // 注意：ets_delay_us在中断中被禁用，对于主循环延时可用
}
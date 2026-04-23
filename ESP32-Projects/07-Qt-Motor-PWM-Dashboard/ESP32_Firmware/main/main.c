/*
 * ESP32 UART Motor PWM Tracker Firmware
 * ESP-IDF Framework kullanarak 4 motor verisini Qt Dashboard'ina yollar.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    // UART varsayilan olarak ESP-IDF tarafindan 115200 baud ile 
    // USB Uart portuna (UART0) configure edilir.
    // printf ile yapilan her cıktı, Qt Dashboard tarafindan yakalanacaktir.

    int pwm1 = 0;
    int pwm2 = 25;
    int pwm3 = 50;
    int pwm4 = 75;
    int dir = 1;

    while (1) {
        // ---- DEMO DEGER URETICI (Gercekte silip motor pini okuyun) ----
        pwm1 += dir; pwm2 += dir; pwm3 += dir; pwm4 += dir;
        if (pwm1 > 100 || pwm1 < 0) dir = -dir;
        // -------------------------------------------------------------

        // 1. Qt Arayuzune beklenen formatta veri gönderimi
        // Format: PWM:MOTOR1,MOTOR2,MOTOR3,MOTOR4\n
        printf("PWM:%d,%d,%d,%d\n", pwm1, pwm2, pwm3, pwm4);

        // 2. 100ms yenileme hizi
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

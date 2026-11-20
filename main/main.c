#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "tcs34725.h"

#define I2C_SCL_PIN 6
#define I2C_SDA_PIN 7

void app_main(void) {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    while (tcs34725_init(bus_handle) != ESP_OK) {
        printf("A procurar sensor de cor... (verifica fios e energia)\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (1) {
        cor_rgb_t rgb;
        if (tcs34725_ler_rgb(&rgb) == ESP_OK) {
            bool vermelho = tcs34725_detetar_vermelho();
            printf("Clear: %4d | R: %4d | G: %4d | B: %4d ---> Vermelho: %s\n",
                   rgb.clear, rgb.red, rgb.green, rgb.blue, vermelho ? "SIM" : "NAO");
        } else {
            printf("Falha momentanea na leitura, a religar sensor...\n");
            tcs34725_init(bus_handle);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
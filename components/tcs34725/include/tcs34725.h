#ifndef TCS34725_H
#define TCS34725_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define TCS34725_ADDR 0x29

typedef struct {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} cor_rgb_t;

esp_err_t tcs34725_init(i2c_master_bus_handle_t bus_handle);
esp_err_t tcs34725_ler_rgb(cor_rgb_t *dados);

// Devolve true se detetar vermelho, false caso contrário
bool tcs34725_detetar_vermelho(void);

#endif // TCS34725_H
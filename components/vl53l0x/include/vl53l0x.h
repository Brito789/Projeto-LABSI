#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Pinos definidos no teu esquema elétrico
#define I2C_SCL_PIN          6
#define I2C_SDA_PIN          7
#define XSHUT_ESQ_PIN        15
#define XSHUT_DIR_PIN        19

// Endereços I2C
#define VL53L0X_DEFAULT_ADDR 0x29
#define ADDR_SENSOR_ESQ      0x30
#define ADDR_SENSOR_DIR      0x31

// Limite para deteção de parede (150 mm = 15 cm)
#define DISTANCIA_PAREDE_MM  150

// Inicializa o barramento I2C e calibra os dois sensores
esp_err_t sensores_init(void);

// Lê a distância individual de cada sensor em milímetros
uint16_t sensor_ler_distancia_esq(void);
uint16_t sensor_ler_distancia_dir(void);

// Verifica se algum dos sensores detetou uma parede dentro do limite
void verificar_paredes(bool *parede_esq, bool *parede_dir);

#include "driver/i2c_master.h"
i2c_master_bus_handle_t sensores_get_i2c_bus(void);

#endif // VL53L0X_H
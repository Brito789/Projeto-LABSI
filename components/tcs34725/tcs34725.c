#include "tcs34725.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TCS34725";
static i2c_master_dev_handle_t dev_cor = NULL;

#define CMD_BIT        0x80
#define REG_ENABLE     0x00
#define REG_ATIME      0x01
#define REG_CONTROL    0x0F
#define REG_ID         0x12
#define REG_CDATAL     0x14

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {CMD_BIT | reg, val};
    return i2c_master_transmit(dev_cor, data, sizeof(data), 50);
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t cmd = CMD_BIT | reg;
    uint8_t val = 0;
    i2c_master_transmit_receive(dev_cor, &cmd, 1, &val, 1, 50);
    return val;
}

esp_err_t tcs34725_init(i2c_master_bus_handle_t bus_handle) {
    if (dev_cor == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = TCS34725_ADDR,
            .scl_speed_hz = 100000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_cor));
    }

    uint8_t id = read_reg(REG_ID);
    if (id != 0x44 && id != 0x4D) {
        ESP_LOGE(TAG, "Sensor de cor nao encontrado (ID: 0x%02X)", id);
        return ESP_FAIL;
    }

    write_reg(REG_ATIME, 0xF6);   // 24ms integração
    write_reg(REG_CONTROL, 0x01); // Ganho 4x
    write_reg(REG_ENABLE, 0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
    write_reg(REG_ENABLE, 0x03);

    ESP_LOGI(TAG, "Sensor TCS34725 pronto para detetar vermelho!");
    return ESP_OK;
}

esp_err_t tcs34725_ler_rgb(cor_rgb_t *dados) {
    uint8_t cmd = CMD_BIT | 0x20 | REG_CDATAL;
    uint8_t buf[8] = {0};

    esp_err_t err = i2c_master_transmit_receive(dev_cor, &cmd, 1, buf, 8, 50);
    if (err != ESP_OK) {
        return err;
    }

    dados->clear = ((uint16_t)buf[1] << 8) | buf[0];
    dados->red   = ((uint16_t)buf[3] << 8) | buf[2];
    dados->green = ((uint16_t)buf[5] << 8) | buf[4];
    dados->blue  = ((uint16_t)buf[7] << 8) | buf[6];

    return ESP_OK;
}

bool tcs34725_detetar_vermelho(void) {
    cor_rgb_t c;
    if (tcs34725_ler_rgb(&c) != ESP_OK) {
        return false;
    }

    // Luz mínima de 80 e Vermelho (R) pelo menos 20% maior que Verde (G) e Azul (B)
    if (c.clear > 80 && c.red > (c.green * 1.2) && c.red > (c.blue * 1.2)) {
        return true;
    }

    return false;
}
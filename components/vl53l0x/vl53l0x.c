#include "vl53l0x.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "VL53L0X";

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_esq = NULL;
static i2c_master_dev_handle_t dev_dir = NULL;

// --- Funções auxiliares de leitura e escrita I2C ---

static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_transmit(dev, data, sizeof(data), 100);
}

static uint8_t read_reg(i2c_master_dev_handle_t dev, uint8_t reg) {
    uint8_t val = 0;
    i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 100);
    return val;
}

static uint16_t read_reg_16(i2c_master_dev_handle_t dev, uint8_t reg) {
    uint8_t buf[2] = {0};
    if (i2c_master_transmit_receive(dev, &reg, 1, buf, 2, 100) != ESP_OK) {
        return 0;
    }
    return ((uint16_t)buf[0] << 8) | buf[1];
}

// --- Sequência de inicialização e calibração interna do VL53L0X ---

static esp_err_t vl53l0x_configurar_sensor(i2c_master_dev_handle_t dev) {
    // Verificar ID do modelo (deve retornar 0xEE)
    if (read_reg(dev, 0xC0) != 0xEE) {
        ESP_LOGE(TAG, "ID do sensor inválido! Verifica as ligações I2C.");
        return ESP_FAIL;
    }

    // Configurar modo I2C padrão
    write_reg(dev, 0x88, 0x00);
    write_reg(dev, 0x80, 0x01);
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);
    uint8_t stop_variable = read_reg(dev, 0x91);
    write_reg(dev, 0x00, 0x01);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x00);

    // Configurar interrupção para "novo dado pronto" (GPIO1)
    write_reg(dev, 0x0A, 0x04);
    uint8_t gpio_hv_mux = read_reg(dev, 0x84);
    write_reg(dev, 0x84, gpio_hv_mux & ~0x10); // Ativo a nível baixo
    write_reg(dev, 0x0B, 0x01);                // Limpar interrupção

    // Calibração de referência (VHV e Phase)
    write_reg(dev, 0x01, 0x01);
    write_reg(dev, 0x00, 0x01 | 0x40);
    vTaskDelay(pdMS_TO_TICKS(50));
    write_reg(dev, 0x0B, 0x01);
    write_reg(dev, 0x00, 0x00);

    write_reg(dev, 0x01, 0x02);
    write_reg(dev, 0x00, 0x01 | 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    write_reg(dev, 0x0B, 0x01);
    write_reg(dev, 0x00, 0x00);

    // Restaurar sequência de passos padrão
    write_reg(dev, 0x01, 0xE8);

    // Guardar stop_variable para as leituras futuras
    write_reg(dev, 0x80, 0x01);
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);
    write_reg(dev, 0x91, stop_variable);
    write_reg(dev, 0x00, 0x01);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x00);

    return ESP_OK;
}

static uint16_t ler_distancia_mm(i2c_master_dev_handle_t dev) {
    // Iniciar medição única
    write_reg(dev, 0x80, 0x01);
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);
    write_reg(dev, 0x91, 0x3C);
    write_reg(dev, 0x00, 0x01);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x00);

    write_reg(dev, 0x00, 0x01);

    // Aguardar fim da medição com timeout de segurança (50ms)
    int timeout = 50;
    while ((read_reg(dev, 0x13) & 0x07) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (--timeout == 0) {
            return 8190; // Valor de erro / fora de alcance
        }
    }

    // Ler distância em mm (registo 0x14 + 10 = 0x1E)
    uint16_t distancia = read_reg_16(dev, 0x1E);

    // Limpar interrupção para a próxima leitura
    write_reg(dev, 0x0B, 0x01);

    return distancia;
}

// --- Funções públicas ---

esp_err_t sensores_init(void) {
    // 1. Configurar pinos XSHUT como saída
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << XSHUT_ESQ_PIN) | (1ULL << XSHUT_DIR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Desligar ambos os sensores para evitar conflito no endereço 0x29
    gpio_set_level(XSHUT_ESQ_PIN, 0);
    gpio_set_level(XSHUT_DIR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. Configurar barramento I2C Master
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg_temp = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_DEFAULT_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t temp_handle;

    // 3. Ligar sensor ESQUERDO, mudar endereço para 0x30 e calibrar
    gpio_set_level(XSHUT_ESQ_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_temp, &temp_handle));
    write_reg(temp_handle, 0x8A, ADDR_SENSOR_ESQ);
    i2c_master_bus_rm_device(temp_handle);

    i2c_device_config_t dev_cfg_esq = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR_SENSOR_ESQ,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_esq, &dev_esq));
    if (vl53l0x_configurar_sensor(dev_esq) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar sensor ESQUERDO");
        return ESP_FAIL;
    }

    // 4. Ligar sensor DIREITO, mudar endereço para 0x31 e calibrar
    gpio_set_level(XSHUT_DIR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_temp, &temp_handle));
    write_reg(temp_handle, 0x8A, ADDR_SENSOR_DIR);
    i2c_master_bus_rm_device(temp_handle);

    i2c_device_config_t dev_cfg_dir = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR_SENSOR_DIR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_dir, &dev_dir));
    if (vl53l0x_configurar_sensor(dev_dir) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar sensor DIREITO");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Ambos os sensores VL53L0X inicializados e calibrados!");
    return ESP_OK;
}

uint16_t sensor_ler_distancia_esq(void) {
    return ler_distancia_mm(dev_esq);
}

uint16_t sensor_ler_distancia_dir(void) {
    return ler_distancia_mm(dev_dir);
}

void verificar_paredes(bool *parede_esq, bool *parede_dir) {
    uint16_t d_esq = sensor_ler_distancia_esq();
    uint16_t d_dir = sensor_ler_distancia_dir();

    // Filtra valores inválidos (abaixo de 20mm ou acima de 2000mm)
    *parede_esq = (d_esq >= 20 && d_esq <= DISTANCIA_PAREDE_MM);
    *parede_dir = (d_dir >= 20 && d_dir <= DISTANCIA_PAREDE_MM);
}

i2c_master_bus_handle_t sensores_get_i2c_bus(void) {
    return bus_handle;
}
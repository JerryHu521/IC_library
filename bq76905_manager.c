#include "bq76905_manager.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_SDA_PIN   GPIO_NUM_8   // SDA pin on ESP32-S3 DevKitC-1
#define I2C_MASTER_SCL_PIN   GPIO_NUM_9   // SCL pin on ESP32-S3 DevKitC-1
#define I2C_MASTER_FREQ_HZ   100000       // 100 kHz
#define I2C_TIMEOUT_MS       1000

static const char *TAG = "BQ76905";

/**
 * \brief Initialize ESP32 I2C master interface.
 */
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
    }
    return err;
}

/**
 * \brief Write a 16-bit word to a BQ76905 register over I2C.
 */
static int i2c_write_word(uint8_t dev_addr, uint8_t reg, uint16_t data) {
    uint8_t buf[3] = { reg, (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    esp_err_t ret = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        dev_addr,
        buf,
        sizeof(buf),
        I2C_TIMEOUT_MS / portTICK_PERIOD_MS
    );
    return (ret == ESP_OK) ? 0 : -1;
}

/**
 * \brief Read a 16-bit word from a BQ76905 register over I2C.
 */
static int i2c_read_word(uint8_t dev_addr, uint8_t reg, uint16_t* data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Null data pointer");
        return -1;
    }
    uint8_t buf[2];
    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        dev_addr,
        &reg,
        1,
        buf,
        sizeof(buf),
        I2C_TIMEOUT_MS / portTICK_PERIOD_MS
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed");
        return -1;
    }
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

int bq76905_init(void) {
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C master init failed");
        return BQ76905_ERR;
    }
    // Example: reset device via hypothetical reset command (0x00)
    if (i2c_write_word(BQ76905_I2C_ADDR, 0x00, 0x0001) != 0) {
        ESP_LOGE(TAG, "Device reset failed");
        return BQ76905_ERR;
    }
    // TODO: additional ADC/protection configuration
    ESP_LOGI(TAG, "BQ76905 initialized");
    return BQ76905_OK;
}

int bq76905_read_data(bq76905_data_t *data) {
    if (!data) {
        ESP_LOGE(TAG, "Null data struct");
        return BQ76905_ERR;
    }
    int rc;
    uint16_t raw;

    // Read each cell
    for (uint8_t i = 0; i < BQ76905_CELL_COUNT; i++) {
        uint8_t reg = (uint8_t)(BQ76905_REG_CELL0_VOLTAGE + i*2);
        rc = i2c_read_word(BQ76905_I2C_ADDR, reg, &raw);
        if (rc != 0) return BQ76905_ERR;
        data->cell_voltage[i] = raw;
    }

    // Pack voltage
    rc = i2c_read_word(BQ76905_I2C_ADDR, BQ76905_REG_PACK_VOLTAGE, &raw);
    if (rc != 0) return BQ76905_ERR;
    data->pack_voltage = raw;

    // Pack current
    rc = i2c_read_word(BQ76905_I2C_ADDR, BQ76905_REG_CURRENT, &raw);
    if (rc != 0) return BQ76905_ERR;
    data->pack_current = (int16_t)raw;

    // Temperature
    rc = i2c_read_word(BQ76905_I2C_ADDR, BQ76905_REG_TEMPERATURE, &raw);
    if (rc != 0) return BQ76905_ERR;
    data->temperature = (int16_t)raw;

    return BQ76905_OK;
}

int bq76905_set_voltage_thresholds(uint16_t ov_mV, uint16_t uv_mV) {
    if (i2c_write_word(BQ76905_I2C_ADDR, BQ76905_REG_OV_THRESHOLD, ov_mV) != 0) {
        ESP_LOGE(TAG, "OV threshold set failed");
        return BQ76905_ERR;
    }
    if (i2c_write_word(BQ76905_I2C_ADDR, BQ76905_REG_UV_THRESHOLD, uv_mV) != 0) {
        ESP_LOGE(TAG, "UV threshold set failed");
        return BQ76905_ERR;
    }
    ESP_LOGI(TAG, "Thresholds set: OV=%d mV UV=%d mV", ov_mV, uv_mV);
    return BQ76905_OK;
}

int bq76905_enable_cell_balancing(uint8_t cell_mask) {
    if (i2c_write_word(BQ76905_I2C_ADDR, BQ76905_REG_BALANCE_CTRL, cell_mask) != 0) {
        ESP_LOGE(TAG, "Cell balancing failed");
        return BQ76905_ERR;
    }
    ESP_LOGI(TAG, "Cell balancing mask=0x%02X", cell_mask);
    return BQ76905_OK;
}

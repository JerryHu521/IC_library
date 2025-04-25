#ifndef BQ76905_MANAGER_H
#define BQ76905_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
   BQ76905 Battery Monitor & Protector for 3-series LiPo Battery Packs
   UAV Charging Station Project
   ============================================================================= */

/** \brief Status codes returned by BQ76905 driver functions. */
typedef enum {
    BQ76905_OK  =  0,  /**< Operation succeeded */
    BQ76905_ERR = -1   /**< Operation failed */
} bq76905_status_t;

/** \brief 7-bit I2C address for the BQ76905. */
#define BQ76905_I2C_ADDR       0x18U

/** \brief Register addresses (partial). */
enum {
    BQ76905_REG_CELL0_VOLTAGE  = 0x04U,
    BQ76905_REG_CELL1_VOLTAGE  = 0x06U,
    BQ76905_REG_CELL2_VOLTAGE  = 0x08U,
    /* 0x0A,0x0C reserved for cells 4/5 in 5S packs */
    BQ76905_REG_PACK_VOLTAGE   = 0x0EU,
    BQ76905_REG_CURRENT        = 0x10U,
    BQ76905_REG_TEMPERATURE    = 0x12U,
    BQ76905_REG_OV_THRESHOLD   = 0x20U,
    BQ76905_REG_UV_THRESHOLD   = 0x22U,
    BQ76905_REG_BALANCE_CTRL   = 0x30U
};

/** \brief ADC conversion scales. */
#define BQ76905_VOLTAGE_LSB_MV   4U       /**< mV per LSB for cell & pack voltage */
#define BQ76905_CURRENT_LSB_MA   1U       /**< mA per LSB for pack current */
#define BQ76905_TEMP_LSB_10THC   1U       /**< 0.1 °C per LSB for temperature */

/** \brief Default configuration: 3 cells, 0.5Ω sense resistor. */
#define BQ76905_CELL_COUNT       3U
#define BQ76905_SENSE_RES_MOHM   500U     /**< milliohms */

/**
 * \brief Container for BQ76905 measurements.
 */
typedef struct {
    uint16_t cell_voltage[BQ76905_CELL_COUNT]; /**< Cell voltages in mV */
    uint16_t pack_voltage;                    /**< Pack voltage in mV */
    int16_t  pack_current;                    /**< Pack current in mA */
    int16_t  temperature;                     /**< Temperature in 0.1 °C units */
} bq76905_data_t;

/* Public API */

/**
 * \brief Initialize the BQ76905 device (sets up ADC, thresholds, etc.).
 * \return BQ76905_OK or BQ76905_ERR
 */
bq76905_status_t bq76905_init(void);

/**
 * \brief Read all sensor data from the BQ76905.
 * \param data Pointer to measurement struct to populate.
 * \return BQ76905_OK or BQ76905_ERR
 */
bq76905_status_t bq76905_read_data(bq76905_data_t *data);

/**
 * \brief Set overvoltage and undervoltage thresholds (in mV).
 * \param ov_mV Overvoltage threshold
 * \param uv_mV Undervoltage threshold
 * \return BQ76905_OK or BQ76905_ERR
 */
bq76905_status_t bq76905_set_voltage_thresholds(uint16_t ov_mV,
                                                uint16_t uv_mV);

/**
 * \brief Enable host-controlled cell balancing via bitmask.
 * \param cell_mask Bits 0–2 correspond to cells 1–3.
 * \return BQ76905_OK or BQ76905_ERR
 */
bq76905_status_t bq76905_enable_cell_balancing(uint8_t cell_mask);

/* Conversion helpers */

/**
 * \brief Convert raw mV to volts.
 */
static inline float bq76905_convert_voltage(uint16_t raw_mV) {
    return raw_mV * 1e-3f;
}

/**
 * \brief Convert raw mA to amps.
 */
static inline float bq76905_convert_current(int16_t raw_mA) {
    return raw_mA * 1e-3f;
}

/**
 * \brief Convert raw 0.1°C units to °C.
 */
static inline float bq76905_convert_temperature(int16_t raw) {
    return raw * 0.1f;
}

#ifdef __cplusplus
}
#endif

#endif /* BQ76905_MANAGER_H */

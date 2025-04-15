#ifndef BQ76905_MANAGER_H
#define BQ76905_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* =============================================================================
   BQ76905 Battery Monitor & Protector for 2-series to 5-series battery packs
   UAV Charging Station Project
   Developed based on project concept, definition & plan documents.
   ============================================================================= */

/* Return Codes */
#define BQ76905_SUCCESS 0
#define BQ76905_ERROR   -1

/* -----------------------------------------------------------------------------
   I2C Address & Command/ Register Definitions
   -----------------------------------------------------------------------------
   Update these addresses and command codes according to your board design
   and the BQ76905 datasheet ([46] :contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}).
-----------------------------------------------------------------------------*/

/* Example I2C address – adjust if different */
#define BQ76905_I2C_ADDR  0x18

/* Register addresses (placeholders):
   These registers are assumed for:
     - Cell voltage measurements (each cell: 16-bit, 2 bytes)
     - Pack voltage measurement (derived from cells or a dedicated register)
     - Current measurement
     - Temperature measurement (internal ADC/thermistor)
     - Protection thresholds (if configurable)
*/
#define BQ76905_REG_CELL0_VOLTAGE    0x04  // Voltage for Cell 1 (mV)
#define BQ76905_REG_CELL1_VOLTAGE    0x06  // Voltage for Cell 2 (mV)
#define BQ76905_REG_CELL2_VOLTAGE    0x08  // Voltage for Cell 3 (mV)
#define BQ76905_REG_CELL3_VOLTAGE    0x0A  // Voltage for Cell 4 (mV)
#define BQ76905_REG_CELL4_VOLTAGE    0x0C  // Voltage for Cell 5 (mV)
#define BQ76905_REG_PACK_VOLTAGE     0x0E  // Pack Voltage (mV)
#define BQ76905_REG_CURRENT          0x10  // Pack Current (mA; signed)
#define BQ76905_REG_TEMPERATURE      0x12  // Battery Temperature (raw value, units defined below)
#define BQ76905_REG_OV_THRESHOLD     0x20  // Overvoltage threshold (mV)
#define BQ76905_REG_UV_THRESHOLD     0x22  // Undervoltage threshold (mV)

/* Conversion constants – update as needed.
   For example, if the voltage ADC LSB is 4 mV, then:
*/
#define BQ76905_VOLTAGE_LSB    0.004f   // 4 mV per LSB (voltage)
#define BQ76905_CURRENT_LSB    0.001f   // 1 mA per LSB (current)
#define BQ76905_TEMPERATURE_LSB 0.1f    // Temperature measured in 0.1 °C units

/* -----------------------------------------------------------------------------
   Data Structures
-----------------------------------------------------------------------------*/

/* Data structure holding measurements from the BQ76905 */
typedef struct {
    /* Battery pack measurements */
    uint16_t pack_voltage;        // in mV
    int16_t  pack_current;        // in mA (signed; positive = charging, negative = discharging)
    int16_t  temperature;         // raw temperature reading (0.1°C units)
    /* Individual cell voltages (for up to 5 cells) */
    uint16_t cell_voltage[5];     // in mV; only the first ‘N’ cells are valid based on your battery configuration
} bq76905_data_t;

/* -----------------------------------------------------------------------------
   Public API Functions
-----------------------------------------------------------------------------*/

/* Initializes the BQ76905 device.
   Configures ADC modes and protection settings as required.
   Returns BQ76905_SUCCESS on success. */
int bq76905_init(void);

/* Reads battery data from the BQ76905.
   Populates the provided bq76905_data_t structure.
   Returns BQ76905_SUCCESS on success. */
int bq76905_read_data(bq76905_data_t *data);

/* Sets overvoltage and undervoltage thresholds for the battery pack.
   Thresholds are specified in millivolts.
   Returns BQ76905_SUCCESS on success. */
int bq76905_set_voltage_thresholds(uint16_t ov_threshold, uint16_t uv_threshold);

/* Enables host-controlled cell balancing for selected cells.
   The cell_mask argument is a bit mask indicating which cells to balance.
   For example, bit 0 for cell 1, bit 1 for cell 2, etc.
   Returns BQ76905_SUCCESS on success. */
int bq76905_enable_cell_balancing(uint8_t cell_mask);

/* Optional conversion functions
   These functions convert raw readings into common engineering units.
   For example, you can convert a raw pack voltage (in mV) to volts. */
static inline float bq76905_convert_voltage(uint16_t raw) {
    return raw * BQ76905_VOLTAGE_LSB; // V = (mV value)* (4e-3)
}
static inline float bq76905_convert_current(int16_t raw) {
    return raw * BQ76905_CURRENT_LSB; // A = (mA value)* (1e-3)
}
static inline float bq76905_convert_temperature(int16_t raw) {
    return raw * BQ76905_TEMPERATURE_LSB; // °C (if raw in 0.1°C units)
}

#ifdef __cplusplus
}
#endif

#endif // BQ76905_MANAGER_H

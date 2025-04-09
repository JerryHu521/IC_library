#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Return codes */
#define BAT_MGR_SUCCESS 0
#define BAT_MGR_ERROR   -1

/* ================= BQ40Z50 Definitions ================= */
/* I2C address for the BQ40Z50 (update to match your hardware) */
#define BQ40Z50_I2C_ADDR 0x0B   // Placeholder address

/* SMBus command codes for the BQ40Z50
   (see datasheet [34] :contentReference[oaicite:2]{index=2} for details) */
#define BQ40Z50_CMD_VOLTAGE         0x09  // 16-bit battery voltage in mV
#define BQ40Z50_CMD_CURRENT         0x0A  // 16-bit battery current in mA (signed)
#define BQ40Z50_CMD_TEMPERATURE     0x08  // 16-bit battery temperature in 0.1°C units
#define BQ40Z50_CMD_SOC             0x0D  // 16-bit state-of-charge (%)

/* Data structure for gauge readings from BQ40Z50 */
typedef struct {
    uint16_t voltage;       // in mV
    int16_t  current;       // in mA (signed)
    int16_t  temperature;   // in 0.1°C units
    uint8_t  soc;           // in percent
} bq40z50_data_t;

/* ================= BQ29330 Definitions ================= */
/* I2C address for the BQ29330 (update as needed) */
#define BQ29330_I2C_ADDR 0x0C   // Placeholder address

/* SMBus command codes for the BQ29330
   (see datasheet [20] :contentReference[oaicite:3]{index=3} for details; these values are placeholders) */
#define BQ29330_CMD_PROTECTION_STATUS 0x01 // Read protection status (16-bit)
#define BQ29330_CMD_SET_OV_THRESHOLD  0x02 // Set overvoltage threshold (16-bit, in mV)
#define BQ29330_CMD_SET_UV_THRESHOLD  0x03 // Set undervoltage threshold (16-bit, in mV)
#define BQ29330_CMD_CELL_VOLTAGE      0x04 // Read one cell voltage (16-bit in mV)

/* Data structure for protection data from BQ29330 */
typedef struct {
    uint8_t  protection_status;   // Bitfield status (placeholder)
    uint16_t cell_voltage[4];     // Array for up to 4 cell voltages (in mV)
} bq29330_data_t;

/* ================= Combined Battery Manager Data ================= */
typedef struct {
    bq40z50_data_t gauge;      // Battery parameters from the gauge (BQ40Z50)
    bq29330_data_t protection; // Protection and cell voltage data from the protection AFE (BQ29330)
} battery_manager_data_t;

/* ================= Public API Functions ================= */

/* Initialize both the battery gauge and protection devices */
int battery_manager_init(void);

/* Read combined battery data from the BQ40Z50 and BQ29330 */
int battery_manager_read(battery_manager_data_t *data);

/* For the BQ29330, set the overvoltage and undervoltage thresholds.
   Thresholds are specified in millivolts (mV). */
int bq29330_set_thresholds(uint16_t ov_threshold, uint16_t uv_threshold);

/* Optional conversion functions for gauge data */
float bq40z50_convert_voltage(uint16_t raw);        // Convert mV to V
float bq40z50_convert_current(int16_t raw);         // Convert mA to A
float bq40z50_convert_temperature(int16_t raw);     // Convert 0.1°C to °C

#ifdef __cplusplus
}
#endif

#endif // BATTERY_MANAGER_H

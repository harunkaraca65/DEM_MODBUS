/*
 * inputs.h
 *
 *  Created on: Nov 13, 2025
 *      Author: hrnkr
 * Brief: Header for the application's input processing layer.
 * Manages all read-only data sources (ADC, IMU, System Stats)
 * and updates the Modbus input databases.
 */

#ifndef INC_INPUTS_H_
#define INC_INPUTS_H_

#include "main.h"
#include "math.h"
#include "lis2dw12_reg.h"

/* ==================================================================== */
/* 1. TYPEDEFS (Data Structures) */
/* ==================================================================== */

/**
 * @brief Structure to hold calculated roll and pitch angles.
 */
typedef struct {
	float roll;
	float pitch;
} Angles;

/* ==================================================================== */
/* 2. DEFINES (Application Macros) */
/* ==================================================================== */
#define SENSOR_BUS 	hi2c1
#define BOOT_TIME 	20 //ms

/**
 * @brief Bit positions for Discrete Inputs (1xxxx) in Inputs_Database[0]
 */
typedef enum {
	DISCRETE_SYS_HEALTHY = 0, // 10001
	DISCRETE_MODBUS_COMM_OK = 1, // 10002
	DISCRETE_SENSOR_FAULT = 2, // 10003
	DISCRETE_WATCHDOG_OK = 3, // 10004
	DISCRETE_ANGLE_ALARM = 4, // 10005
	DISCRETE_TEMP_ALARM = 5, // 10006
	DISCRETE_CALIBRATION_REQ = 6, // 10007
} DiscreteInputBits;

/* ==================================================================== */
/* 3. EXTERNAL GLOBAL FLAGS */
/* ==================================================================== */

// These flags are set by main.c or other modules
extern volatile uint8_t g_modbus_activity_flag;
extern volatile uint8_t g_sensor_error_flag;

/* ==================================================================== */
/* 4. FUNCTION PROTOTYPES (Public Interface) */
/* ==================================================================== */

/**
 * @brief Initializes all input sensors (IMU) and application layer variables.
 */
void init_inputs(void);

/**
 * @brief Processes raw ADC data from DMA, scales it, and updates Modbus Input Registers.
 * @note  This function is designed to be called from HAL_ADC_ConvCpltCallback.
 */
void process_adc_data(void);

/**
 * @brief Reads IMU, calculates angles, and updates Modbus Input Registers.
 * @note  This function is designed to be called periodically (e.g., via SysTick).
 */
void update_imu_angles(void);

/**
 * @brief Updates system stats (Uptime, Counters) and writes to Modbus Input Registers.
 * @note  This function is designed to be called periodically (e.g., via SysTick).
 */
void update_register_inputs(void);

/**
 * @brief Updates system status flags (Discrete Inputs) and writes to Modbus database.
 * @note  This function is designed to be called periodically (e.g., via SysTick).
 */
void update_discrete_inputs(void);

/* ==================================================================== */
/* 5. MODBUS DIAGNOSTIC COUNTERS */
/* ==================================================================== */

/**
 * @brief Increments the successful Modbus transaction counter.
 * @note  Called by modbusSlave.c on a valid request.
 */
void increment_success_count(void);

/**
 * @brief Increments the Modbus exception (error) counter.
 * @note  Called by modbusSlave.c when sending an exception.
 */
void increment_error_count(void);

/* ==================================================================== */
/* 6. HELPER & DRIVER FUNCTIONS (Internal Logic) */
/* ==================================================================== */

/**
 * @brief Calculates NTC temperature (in Celsius) from raw ADC value.
 */
double process_ntc(uint32_t analogValue);

/**
 * @brief Maps a value from one range (0-in_max) to another (0-out_max).
 */
uint16_t map_adc_value(uint32_t val, uint32_t in_max, uint32_t out_max);

/**
 * @brief Calculates roll and pitch angles from accelerometer data (in mg).
 */
Angles calculate_angles(float x_mg, float y_mg, float z_mg);

/**
 * @brief Platform-specific I2C write function for LIS2DW12 driver.
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);

/**
 * @brief Platform-specific I2C read function for LIS2DW12 driver.
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

#endif /* INC_INPUTS_H_ */

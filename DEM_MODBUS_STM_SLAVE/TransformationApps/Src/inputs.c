/*
 * inputs.c
 *
 *  Created on: Nov 13, 2025
 *      Author: hrnkr
 */

#include "inputs.h"
#include "modbusSlave.h"// Required to access the Modbus database
#include <string.h>
/* ==================================================================== */
/* 1. EXTERNAL HARDWARE HANDLES */
/* ==================================================================== */

// These handles are defined in main.c
extern I2C_HandleTypeDef hi2c1;
extern volatile uint16_t ADC_VAL[4]; // ADC DMA Buffer defined in main.c

/* ==================================================================== */
/* 2. PRIVATE MODULE VARIABLES */
/* ==================================================================== */

static Angles angle;
static stmdev_ctx_t dev_ctx;
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;

// Modbus Diagnostic Counters
static volatile uint16_t modbus_success_count = 0;
static volatile uint16_t modbus_error_count = 0;

// Global flags (defined here)
volatile uint8_t g_modbus_activity_flag = 0;
volatile uint8_t g_sensor_error_flag = 0;
volatile uint8_t enter_flag = 0;

/* ==================================================================== */
/* 3. ADC & SENSOR PROCESSING */
/* ==================================================================== */

/**
 * @brief Maps a value from one range (0-in_max) to another (0-out_max).
 */
uint16_t map_adc_value(uint32_t val, uint32_t in_max, uint32_t out_max) {
	// Using float math for precision
	return (uint16_t) (((float) val / (float) in_max) * (float) out_max);
}

/**
 * @brief Calculates NTC temperature (in Celsius) from raw ADC value.
 */
double process_ntc(uint32_t analogValue) {
	double temperature;
	// Assuming 12-bit ADC (0-4095)
	uint32_t adcval = 4096 - analogValue;

	temperature = log((adcval * 10000) / (4095 - adcval));
	temperature = 1
			/ (0.001129148
					+ (0.000234125
							+ (0.0000000876741 * temperature * temperature))
							* temperature);
	temperature = temperature - 273.15;
	return temperature;
}

/**
 * @brief Processes raw ADC data from DMA and updates Modbus Input Registers.
 * @note  This is called by HAL_ADC_ConvCpltCallback in main.c
 * ADC Map: [0]=Trimpot1, [1]=Trimpot2, [2]=NTC, [3]=LDR
 */
void process_adc_data(void) {
	/* === MODBUS BRIDGE: INPUT REGISTERS (3xxxx) === */

	// 30001 (Index 0): Trimpot 1 (Map 0-4095 to 0-10000)
	Input_Registers_Database[0] = map_adc_value(ADC_VAL[0], 4095, 10000);

	// 30002 (Index 1): Trimpot 2 (Map 0-4095 to 0-10000)
	Input_Registers_Database[1] = map_adc_value(ADC_VAL[1], 4095, 10000);

	// 30003 (Index 2): NTC Temperature (Map 0-4095 to C°)
	double temp_c = process_ntc(ADC_VAL[2]);
	// Store 25.45 C as 2545
	Input_Registers_Database[2] = (uint16_t) (temp_c);

	// 30004 (Index 3): LDR (Map 0-4095 to 0-100%)
	Input_Registers_Database[3] = map_adc_value(ADC_VAL[3], 4095, 100);
}

/* ==================================================================== */
/* 4. IMU SENSOR LOGIC */
/* ==================================================================== */

/**
 * @brief Calculates roll and pitch angles from accelerometer data (in mg).
 */
Angles calculate_angles(float x_mg, float y_mg, float z_mg) {
	Angles angles;
	angles.roll = atan2(y_mg, z_mg) * 180.0 / M_PI;
	angles.pitch = atan2(-x_mg, sqrt(y_mg * y_mg + z_mg * z_mg)) * 180.0 / M_PI;

	// Normalize to 0-360 range (optional)
	if (angles.roll < 0) {
		angles.roll += 360;
	}
	if (angles.pitch < 0) {
		angles.pitch += 360;
	}

	return angles;
}

/**
 * @brief Reads IMU, calculates angles, and updates Modbus Input Registers.
 * @note  Called periodically by main loop (from SysTick flag)
 */
void update_imu_angles(void) {
	// If sensor failed at init, don't try to read it
	enter_flag = 1;

	uint8_t reg;
	/* Read output only if new value is available */
	lis2dw12_flag_data_ready_get(&dev_ctx, &reg);

	if (reg) {
		/* Read acceleration data */
		memset(data_raw_acceleration, 0, sizeof(data_raw_acceleration));
		lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		acceleration_mg[0] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[2]);
	}

	// Calculate angles using the correct axes
	angle = calculate_angles(acceleration_mg[1], acceleration_mg[0],
			acceleration_mg[2]);

	/* === MODBUS BRIDGE: INPUT REGISTERS (3xxxx) === */

	// 30005 (Index 4): Pitch (Scaled by 10)
	// Store 180.5 degrees as 1805
	Input_Registers_Database[4] = (uint16_t) (angle.pitch);

	// 30006 (Index 5): Roll (Scaled by 10)
	Input_Registers_Database[5] = (uint16_t) (angle.roll);
}

/**
 * @brief Initializes the LIS2DW12 IMU sensor.
 */
void init_inputs(void) {
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &SENSOR_BUS;

	lis2dw12_device_id_get(&dev_ctx, &whoamI);

	lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lis2dw12_reset_get(&dev_ctx, &rst);
	} while (rst);

	if (platform_read(&dev_ctx, LIS2DW12_WHO_AM_I, &whoamI, 1) != 0) {
		g_sensor_error_flag = 1;
		return;
	}

	if (whoamI != LIS2DW12_ID) {
		g_sensor_error_flag = 1;
		return;
	}

	/* Enable Block Data Update */
	lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set full scale */
	lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
	/* Configure filtering chain */
	lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
	lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);
	/* Configure power mode */
	lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
	/* Set Output Data Rate */
	lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_100Hz);
}

/* ==================================================================== */
/* 5. SYSTEM STATUS & STATS */
/* ==================================================================== */

// Defines for alarm thresholds
#define MAX_ANGLE_LIMIT_SCALED 45 // 45.0 degrees
#define MAX_TEMP_LIMIT_SCALED 50 // 50.00 C

/**
 * @brief Updates system stats (Uptime, Counters) and writes to Modbus Input Registers.
 * @note  Called periodically by main loop (from SysTick flag)
 */
void update_register_inputs(void) {
	/* === MODBUS BRIDGE: INPUT REGISTERS (3xxxx) === */

	// 30007 (Index 6): Uptime (Total Seconds)
	Input_Registers_Database[6] = (uint16_t) (HAL_GetTick() / 1000);

	// 30008 (Index 7): Modbus Success Counter
	Input_Registers_Database[7] = modbus_success_count;

	// 30009 (Index 8): Modbus Error Counter
	Input_Registers_Database[8] = modbus_error_count;
}

/**
 * @brief Updates system status flags (Discrete Inputs) and writes to Modbus database.
 * @note  Called periodically by main loop (from SysTick flag)
 */
void update_discrete_inputs(void) {
	/* === MODBUS BRIDGE: DISCRETE INPUTS (1xxxx) === */
	// (Address 10001-10007, Array Inputs_Database[0], Bits 0-6)
	// Clear all flags before recalculating
	Inputs_Database[0] = 0;

	// 10002 (Bit 1): Modbus Communication OK
	if (g_modbus_activity_flag) {
		Inputs_Database[0] |= (1 << DISCRETE_MODBUS_COMM_OK);
		g_modbus_activity_flag = 0; // Clear the flag (it's a one-shot)
	}

	// 10003 (Bit 2): Sensor Fault
	if (g_sensor_error_flag) {
		Inputs_Database[0] |= (1 << DISCRETE_SENSOR_FAULT);
	}

	// 10004 (Bit 3): Watchdog Status (Simulation)
	Inputs_Database[0] |= (1 << DISCRETE_WATCHDOG_OK); // Assume OK

	// 10005 (Bit 4): Angle Limit Alarm
	if (Input_Registers_Database[4] > MAX_ANGLE_LIMIT_SCALED
			|| Input_Registers_Database[5] > MAX_ANGLE_LIMIT_SCALED) {
		Inputs_Database[0] |= (1 << DISCRETE_ANGLE_ALARM);
	}

	// 10006 (Bit 5): Temperature Limit Alarm
	if (Input_Registers_Database[2] > MAX_TEMP_LIMIT_SCALED) {
		Inputs_Database[0] |= (1 << DISCRETE_TEMP_ALARM);
	}

	// 10007 (Bit 6): Calibration Required (Simulation)
	Inputs_Database[0] |= (1 << DISCRETE_CALIBRATION_REQ); // Assume True

	/* --- Summary Flag (Calculated Last) --- */

	// 10001 (Bit 0): System Healthy
	// Healthy = NO sensor fault
	if (!(Inputs_Database[0] & (1 << DISCRETE_SENSOR_FAULT))) {
		Inputs_Database[0] |= (1 << DISCRETE_SYS_HEALTHY);
	}
}

/* ==================================================================== */
/* 6. MODBUS DIAGNOSTIC COUNTERS */
/* ==================================================================== */

/**
 * @brief Increments the successful Modbus transaction counter.
 */
void increment_success_count(void) {
	modbus_success_count++;
}

/**
 * @brief Increments the Modbus exception (error) counter.
 */
void increment_error_count(void) {
	modbus_error_count++;
}

/* ==================================================================== */
/* 7. I2C DRIVER FUNCTIONS */
/* ==================================================================== */

/**
 * @brief Platform-specific I2C write function for LIS2DW12 driver.
 * (Using 10ms timeout)
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000); // 10ms timeout

	return 0;
}

/**
 * @brief Platform-specific I2C read function for LIS2DW12 driver.
 * (Using 10ms timeout)
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000); // 10ms timeout

	return 0;
}

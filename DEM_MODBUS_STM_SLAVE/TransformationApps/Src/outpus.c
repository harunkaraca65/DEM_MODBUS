/*
 * outpus.c
 *
 *  Created on: Nov 13, 2025
 *      Author: hrnkr
 * Brief: Source file for the application's output processing layer.
 */

#include "outputs.h"
#include "modbusSlave.h" // Required to access the Modbus database

/* ==================================================================== */
/* 1. EXTERNAL HARDWARE HANDLES (Defined in main.c)  */
/* ==================================================================== */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim16;

/* ==================================================================== */
/* 2. PRIVATE MODULE VARIABLES (To track current state)  */
/* ==================================================================== */

// Static variables to hold the *last known state* of the hardware.
// This prevents writing to hardware on every single update loop.
// 16-bit Register States
static uint16_t current_pwm_r = 0; // Init with invalid value to force first update
static uint16_t current_pwm_g = 0;
static uint16_t current_pwm_b = 0;

// 1-bit Coil States (We use one byte to track all 6 coils)
static uint8_t current_coil_states = 0; // Init with invalid value

/* ==================================================================== */
/* 3. OUTPUT INITIALIZATION  */
/* ==================================================================== */

/**
 * @brief Initializes all outputs (starts PWM timers).
 */
void init_outputs(void) {
	// Start all PWM channels required for RGB control
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // RGB_R (PA6)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // RGB_G (PA8)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // RGB_B (PB3)
}

/* ==================================================================== */
/* 4. OUTPUT UPDATE (Main polling function)  */
/* ==================================================================== */

/**
 * @brief Checks Modbus databases for changes and updates hardware (PWM/GPIO).
 * @note  Called periodically by HAL_SYSTICK_Callback in main.c
 */
void update_outputs(void) {
	/* --- 4.A: HOLDING REGISTER (PWM) UPDATE --- */
	// Check RGB Red (40001)
	uint16_t desired_pwm_r = Holding_Registers_Database[MODBUS_INDEX_PWM_R];
	if (current_pwm_r != desired_pwm_r) {
		// Value changed, update hardware
		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, desired_pwm_r);
		current_pwm_r = desired_pwm_r; // Store new state
	}

	// Check RGB Green (40002)
	uint16_t desired_pwm_g = Holding_Registers_Database[MODBUS_INDEX_PWM_G];
	if (current_pwm_g != desired_pwm_g) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, desired_pwm_g);
		current_pwm_g = desired_pwm_g;
	}

	// Check RGB Blue (40003)
	uint16_t desired_pwm_b = Holding_Registers_Database[MODBUS_INDEX_PWM_B];
	if (current_pwm_b != desired_pwm_b) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, desired_pwm_b);
		current_pwm_b = desired_pwm_b;
	}

	// TODO: Add logic for ADC parameters (Index 3, 4) if they control hardware

	/* --- 4.B: COILS (GPIO) UPDATE --- */

	// Read the entire byte (8 coils) from the Modbus database
	uint8_t desired_coil_states = Coils_Database[0];

	// Check if *any* coil state has changed
	if (current_coil_states != desired_coil_states) {
		// --- LED 1 (Coil 0 / PA15) ---
		if (((desired_coil_states >> MODBUS_COIL_LED_1) & 1)
				!= ((current_coil_states >> MODBUS_COIL_LED_1) & 1)) {
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
					(desired_coil_states >> MODBUS_COIL_LED_1) & 1);
		}

		// --- LED 2 (Coil 1 / PD0) ---
		if (((desired_coil_states >> MODBUS_COIL_LED_2) & 1)
				!= ((current_coil_states >> MODBUS_COIL_LED_2) & 1)) {
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
					(desired_coil_states >> MODBUS_COIL_LED_2) & 1);
		}

		// --- LED 3 (Coil 2 / PD1) ---
		if (((desired_coil_states >> MODBUS_COIL_LED_3) & 1)
				!= ((current_coil_states >> MODBUS_COIL_LED_3) & 1)) {
			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
					(desired_coil_states >> MODBUS_COIL_LED_3) & 1);
		}

		// --- LED 4 (Coil 3 / PD2) ---
		if (((desired_coil_states >> MODBUS_COIL_LED_4) & 1)
				!= ((current_coil_states >> MODBUS_COIL_LED_4) & 1)) {
			HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin,
					(desired_coil_states >> MODBUS_COIL_LED_4) & 1);
		}

		// --- LED 5 (Coil 4 / PD3) ---
		if (((desired_coil_states >> MODBUS_COIL_LED_5) & 1)
				!= ((current_coil_states >> MODBUS_COIL_LED_5) & 1)) {
			HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin,
					(desired_coil_states >> MODBUS_COIL_LED_5) & 1);
		}

		// --- Buzzer (Coil 5 / PB12) ---
		if (((desired_coil_states >> MODBUS_COIL_BUZZER) & 1)
				!= ((current_coil_states >> MODBUS_COIL_BUZZER) & 1)) {
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
					(desired_coil_states >> MODBUS_COIL_BUZZER) & 1);
		}

		// Store the new complete state
		current_coil_states = desired_coil_states;
	}
}

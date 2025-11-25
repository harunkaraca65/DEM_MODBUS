/*
 * outputs.h
 *
 *  Created on: Nov 13, 2025
 *      Author: hrnkr
 * Brief: Header for the application's output processing layer.
 * Manages all writeable data sinks (PWM, GPIO LEDs, Buzzer)
 * by reading the Modbus output databases.
 */

#ifndef INC_OUTPUTS_H_
#define INC_OUTPUTS_H_

#include "main.h"
#include "modbusSlave.h"

/* ==================================================================== */
/* 1. DEFINES (Hardware Mapping)  */
/* ==================================================================== */

/* Brief: Maps Modbus Holding Register Index to the correct Timer Channel
 * Note: These values MUST match your .ioc configuration.
 * RGB_R (PA6) -> TIM16_CH1
 * RGB_G (PA8) -> TIM1_CH1
 * RGB_B (PB3) -> TIM1_CH2
 */

typedef enum {
	MODBUS_INDEX_PWM_R = 0, // 10001
	MODBUS_INDEX_PWM_G = 1, // 10002
	MODBUS_INDEX_PWM_B = 2, // 10003
} IndexRegisterBits;

/* Brief: Maps Modbus Coil Index (Bit Position) to the correct GPIO Port/Pin
 * Note: These MUST match your .ioc configuration.
 * 5 LEDs + 1 Buzzer
 */

typedef enum {
	MODBUS_COIL_LED_1 = 0, // 10001
	MODBUS_COIL_LED_2 = 1, // 10002
	MODBUS_COIL_LED_3 = 2, // 10003
	MODBUS_COIL_LED_4 = 3, // 10001
	MODBUS_COIL_LED_5 = 4, // 10002
	MODBUS_COIL_BUZZER = 5, // 10003
} IndexCoilsBits;

/* ==================================================================== */
/* 2. FUNCTION PROTOTYPES (Public Interface)  */
/* ==================================================================== */

/**
 * @brief Initializes all outputs (starts PWM timers).
 * @note  Called once from main() after peripheral initialization.
 */
void init_outputs(void);

/**
 * @brief Checks the Modbus databases for changes and updates hardware (PWM/GPIO).
 * @note  This function is designed to be called periodically (e.g., via SysTick).
 */
void update_outputs(void);

#endif /* INC_OUTPUTS_H_ */

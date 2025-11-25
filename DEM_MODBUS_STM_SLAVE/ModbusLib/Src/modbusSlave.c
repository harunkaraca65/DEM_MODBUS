/*
 * modbusSlave.c
 *
 *  Created on: Nov 13, 2025
 *      Author: Controllerstech && hrnkrc
 */

#include "modbusSlave.h"
#include "string.h"
#include "inputs.h"

extern uint8_t RxData[256];
extern uint8_t TxData[256];
extern UART_HandleTypeDef huart1;

// Updated database sizes based on final Modbus Map
uint16_t Holding_Registers_Database[3] = {
		25000,  // 40001: RGB (R) PWM
		25000,  // 40002: RGB (G) PWM
		0,  // 40003: RGB (B) PWM
		};

// Updated database sizes based on final Modbus Map
uint16_t Input_Registers_Database[9] = { 0, // 30001: ADC 1
		0, // 30002: ADC 2
		0, // 30003: ADC 3
		0, // 30004: ADC 4
		0, // 30005: IMU Pitch
		0, // 30006: IMU Yaw
		0, // 30007: Uptime
		0, // 30008: Modbus Success Count
		0  // 30009: Modbus Error Count
		};

// Updated database sizes based on final Modbus Map
// 1 Byte = 8 Coils. We need 6 (5 LEDs + 1 Buzzer)
uint8_t Coils_Database[1] = { 0b00011111    // 0-7: Coils 1-8
		};

// Updated database sizes based on final Modbus Map
// 1 Byte = 8 Inputs. We need 7
uint8_t Inputs_Database[1] = { 0b00000000    // 0-7: Inputs 1-8
		};

void sendData(uint8_t *data, int size) {
	// we will calculate the CRC in this function itself
	uint16_t crc = crc16(data, size);
	data[size] = crc & 0xFF;   // CRC LOW
	data[size + 1] = (crc >> 8) & 0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart1, data, size + 2, 1000);
}

void modbusException(uint8_t exceptioncode) {
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	//| 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	TxData[0] = RxData[0];       // slave ID
	TxData[1] = RxData[1] | 0x80;  // adding 1 to the MSB of the function code
	TxData[2] = exceptioncode;   // Load the Exception code
	increment_error_count();
	sendData(TxData, 3);  // send Data... CRC will be calculated in the function
}

uint8_t readHoldingRegs(void) {
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]); // start Register Address

	uint16_t numRegs = ((RxData[4] << 8) | RxData[5]); // number to registers master has requested
	if ((numRegs < 1) || (numRegs > 125)) // maximum no. of Registers as per the PDF
			{
		modbusException(ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr + numRegs - 1;  // end Register
	// Updated address check for 5 registers (Index 0-3)
	if (endAddr > 2) // end Register can not be more than 2 as we only have record of 3 Registers in total
			{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs * 2;  // Byte count
	int indx = 3; // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i = 0; i < numRegs; i++) // Load the actual data into TxData buffer
			{
		TxData[indx++] = (Holding_Registers_Database[startAddr] >> 8) & 0xFF; // extract the higher byte
		TxData[indx++] = (Holding_Registers_Database[startAddr]) & 0xFF; // extract the lower byte
		startAddr++;  // increment the register address
	}

	increment_success_count();
	sendData(TxData, indx); // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readInputRegs(void) {
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]); // start Register Address

	uint16_t numRegs = ((RxData[4] << 8) | RxData[5]); // number to registers master has requested
	if ((numRegs < 1) || (numRegs > 125)) // maximum no. of Registers as per the PDF
			{
		modbusException(ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr + numRegs - 1;  // end Register
	// Updated address check for 9 registers (Index 0-8)
	if (endAddr > 8) // end Register can not be more than 8 as we only have record of 9 Registers in total
			{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs * 2;  // Byte count
	int indx = 3; // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i = 0; i < numRegs; i++) // Load the actual data into TxData buffer
			{
		TxData[indx++] = (Input_Registers_Database[startAddr] >> 8) & 0xFF; // extract the higher byte
		TxData[indx++] = (Input_Registers_Database[startAddr]) & 0xFF; // extract the lower byte
		startAddr++;  // increment the register address
	}
	increment_success_count();
	sendData(TxData, indx); // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readCoils(void) {
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Coil Address

	uint16_t numCoils = ((RxData[4] << 8) | RxData[5]); // number to coils master has requested
	if ((numCoils < 1) || (numCoils > 2000)) // maximum no. of coils as per the PDF
			{
		modbusException(ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr + numCoils - 1;  // Last coils address
	// Updated address check for 6 coils (Index 0-5)
	if (endAddr > 5) // end coil can not be more than 5 as we only have record of 6 (0-5) coils in total
			{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	//reset TxData buffer
	memset(TxData, '\0', 256);

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0);  // Byte count
	int indx = 3; // we need to keep track of how many bytes has been stored in TxData Buffer

	/* The approach is simple. We will read 1 bit at a time and store them in the Txdata buffer.
	 * First find the offset in the first byte we read from, for eg- if the start coil is 13,
	 * we will read from database[1] with an offset of 5. This bit will be stored in the TxData[0] at 0th position.
	 * Then we will keep shifting the database[1] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy into the next byte of the TxData
	 * This keeps going until the number of coils required have been copied
	 */
	int startByte = startAddr / 8; // which byte we have to start extracting the data from
	uint16_t bitPosition = startAddr % 8; // The shift position in the first byte
	int indxPosition = 0; // The shift position in the current indx of the TxData buffer

	// Load the actual data into TxData buffer
	for (int i = 0; i < numCoils; i++) {
		TxData[indx] |= ((Coils_Database[startByte] >> bitPosition) & 0x01)
				<< indxPosition;
		indxPosition++;
		bitPosition++;
		if (indxPosition > 7) // if the indxposition exceeds 7, we have to copy the data into the next byte position
				{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition > 7) // if the bitposition exceeds 7, we have to increment the startbyte
				{
			bitPosition = 0;
			startByte++;
		}
	}

	if (numCoils % 8 != 0)
		indx++; // increment the indx variable, only if the numcoils is not a multiple of 8
	increment_success_count();
	sendData(TxData, indx); // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readInputs(void) {
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]); // start Register Address

	uint16_t numCoils = ((RxData[4] << 8) | RxData[5]); // number to coils master has requested
	if ((numCoils < 1) || (numCoils > 2000)) // maximum no. of coils as per the PDF
			{
		modbusException(ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr + numCoils - 1;  // Last coils address
	// Updated address check for 7 inputs (Index 0-6)
	if (endAddr > 6) // end coil can not be more than 6 as we only have record of 7 (0-6) coils in total
			{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	//reset TxData buffer
	memset(TxData, '\0', 256);

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0);  // Byte count
	int indx = 3; // we need to keep track of how many bytes has been stored in TxData Buffer

	/* The approach is simple. We will read 1 bit at a time and store them in the Txdata buffer.
	 * First find the offset in the first byte we read from, for eg- if the start coil is 13,
	 * we will read from database[1] with an offset of 5. This bit will be stored in the TxData[0] at 0th position.
	 * Then we will keep shifting the database[1] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy into the next byte of the TxData
	 * This keeps going until the number of coils required have been copied
	 */
	int startByte = startAddr / 8; // which byte we have to start extracting the data from
	uint16_t bitPosition = startAddr % 8; // The shift position in the first byte
	int indxPosition = 0; // The shift position in the current indx of the TxData buffer

	// Load the actual data into TxData buffer
	for (int i = 0; i < numCoils; i++) {
		TxData[indx] |= ((Inputs_Database[startByte] >> bitPosition) & 0x01)
				<< indxPosition;
		indxPosition++;
		bitPosition++;
		if (indxPosition > 7) // if the indxposition exceeds 7, we have to copy the data into the next byte position
				{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition > 7) // if the bitposition exceeds 7, we have to increment the startbyte
				{
			bitPosition = 0;
			startByte++;
		}
	}

	if (numCoils % 8 != 0)
		indx++; // increment the indx variable, only if the numcoils is not a multiple of 8
	increment_success_count();
	sendData(TxData, indx); // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeHoldingRegs(void) {
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]); // start Register Address

	uint16_t numRegs = ((RxData[4] << 8) | RxData[5]); // number to registers master has requested
	if ((numRegs < 1) || (numRegs > 123)) // maximum no. of Registers as per the PDF
			{
		modbusException(ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr + numRegs - 1;  // end Register
	// Updated address check for 5 registers (Index 0-4)
	if (endAddr > 4) // end Register can not be more than 4 as we only have record of 5 Registers in total
			{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* start saving 16 bit data
	 * Data starts from RxData[7] and we need to combine 2 bytes together
	 * 16 bit Data = firstByte<<8|secondByte
	 */
	int indx = 7;  // we need to keep track of index in RxData
	for (int i = 0; i < numRegs; i++) {
		// 1. Read High Byte and increment indx
		uint8_t highByte = RxData[indx];
		indx++;

		// 2. Read Low Byte and increment indx
		uint8_t lowByte = RxData[indx];
		indx++;

		// 3. Combine the bytes and write to the database
		Holding_Registers_Database[startAddr++] = (highByte << 8) | lowByte;

	}

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | num of Regs    | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES      | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // num of Regs HIGH Byte
	TxData[5] = RxData[5];   // num of Regs LOW Byte

	increment_success_count();
	sendData(TxData, 6); // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeSingleReg(void) {
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]); // start Register Address

	// Updated address check for 5 registers (Index 0-2)
	if (startAddr > 2) // The Register Address can not be more than 2 as we only have record of 3 Registers in total
			{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* Save the 16 bit data
	 * Data is the combination of 2 bytes, RxData[4] and RxData[5]
	 */

	Holding_Registers_Database[startAddr] = (RxData[4] << 8) | RxData[5];

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // Reg Data HIGH Byte
	TxData[5] = RxData[5];   // Reg Data LOW  Byte

	increment_success_count();
	sendData(TxData, 6); // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeSingleCoil(void) {
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Coil Address

	// Updated address check for 6 coils (Index 0-5)
	if (startAddr > 5) // The Coil Address can not be more than 5 as we only have record of 6 Coils in total
			{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* Calculation for the bit in the database, where the modification will be done */
	int startByte = startAddr / 8; // which byte we have to start writing the data into
	uint16_t bitPosition = startAddr % 8; // The shift position in the first byte

	/* The next 2 bytes in the RxData determines the state of the coil
	 * A value of FF 00 hex requests the coil to be ON.
	 * A value of 00 00 requests it to be OFF.
	 * All other values are illegal and will not affect the coil.
	 */

	if ((RxData[4] == 0xFF) && (RxData[5] == 0x00)) {
		Coils_Database[startByte] |= 1 << bitPosition; // Replace that bit with 1
	}

	else if ((RxData[4] == 0x00) && (RxData[5] == 0x00)) {
		Coils_Database[startByte] &= ~(1 << bitPosition); // Replace that bit with 0
	}

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // Coil Data HIGH Byte
	TxData[5] = RxData[5];   // Coil Data LOW  Byte

	sendData(TxData, 6); // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeMultiCoils(void) {
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Coil Address

	uint16_t numCoils = ((RxData[4] << 8) | RxData[5]); // number to coils master has requested
	if ((numCoils < 1) || (numCoils > 1968)) // maximum no. of coils as per the PDF
			{
		modbusException(ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr + numCoils - 1;  // Last coils address
	// Updated address check for 6 coils (Index 0-5)
	if (endAddr > 5) // end coil can not be more than 5 as we only have record of 6 (0-5) coils in total
			{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* Calculation for the bit in the database, where the modification will be done */
	int startByte = startAddr / 8; // which byte we have to start writing the data into
	uint16_t bitPosition = startAddr % 8; // The shift position in the first byte
	int indxPosition = 0; // The shift position in the current indx of the RxData buffer

	int indx = 7;  // we need to keep track of index in RxData

	/* The approach is simple. We will read 1 bit (starting from the very first bit in the RxData Buffer)
	 * at a time and store them in the Database.
	 * First find the offset in the first byte we write into, for eg- if the start coil is 13,
	 * we will Write into database[1] with an offset of 5. This bit is read from the RxData[indx] at 0th indxposition.
	 * Then we will keep shifting the RxData[indx] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte and start modifying the next byte in the database
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy from the next byte of the RxData
	 * This keeps going until the number of coils required have been modified
	 */

	// Modify the bits as per the Byte received
	for (int i = 0; i < numCoils; i++) {
		if (((RxData[indx] >> indxPosition) & 0x01) == 1) {
			Coils_Database[startByte] |= 1 << bitPosition; // replace that bit with 1
		} else {
			Coils_Database[startByte] &= ~(1 << bitPosition); // replace that bit with 0
		}

		bitPosition++;
		indxPosition++;

		if (indxPosition > 7) // if the indxposition exceeds 7, we have to copy the data into the next byte position
				{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition > 7) // if the bitposition exceeds 7, we have to increment the startbyte
				{
			bitPosition = 0;
			startByte++;
		}
	}

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // num of coils HIGH Byte
	TxData[5] = RxData[5];   // num of coils LOW  Byte

	increment_success_count();
	sendData(TxData, 6); // send data... CRC will be calculated in the function itself
	return 1;   // success
}

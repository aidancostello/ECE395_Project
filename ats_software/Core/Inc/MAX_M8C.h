#ifndef __MAX_M8C_H
#define __MAX_M8C_H

#include <stdint.h>
#include <string.h>
#include "stm32l4xx_hal.h"

// function error codes
#define MAX_M8C_SUCCESS	0x00u
#define MAX_M8C_FAIL	0x01u

// i2c/ddc addresses
#define I2C_ADDR		(0x42u << 1)
#define REG_AVAIL_UPPER 0xFDu
#define REG_AVAIL_LOWER 0xFEu
#define REG_DATA		0xFFu

// sync bytes
#define SYNC_CHAR_1		0xB5u
#define SYNC_CHAR_2		0x62u

// class bytes
#define CLASS_NAV		0x01u
#define CLASS_ACK		0x05u
#define CLASS_CFG		0x06u

// id bytes
#define ID_NAV_PVT		0x07u
#define ID_ACK_ACK		0x01u
#define ID_ACK_NAK		0x00u
#define ID_CFG_PRT		0x00u
#define ID_CFG_MSG		0x01u

// packet sizes
#define UBX_MAX_PACKET_SIZE 100u
#define UBX_HEADER_SIZE 	6u
#define UBX_CHECKSUM_SIZE	2u
#define UBX_NUM_SYNC_BYTES	2u
#define UBX_NAV_PVT_PAYLOAD_SIZE 92u

// NAV-PVT offsets
#define LON_BYTE_OFFSET 24
#define LAT_BYTE_OFFSET 28
#define HEIGHT_BYTE_OFFSET 32

// lat, long, alt scalars
#define LAT_LON_SCALAR 10000000
#define ALT_SCALAR 1000

// trimmed packet structure (everything except sync chars and checksums)
struct ubx_packet_trimmed {
	uint8_t class;
	uint8_t id;
	uint8_t length_lower;
	uint8_t length_upper;
	uint8_t* payload;
};

// relevant gps data
struct max_m8c_gps_data {
	int32_t lat;
	int32_t lon;
	int32_t alt;
};

// toggle reset
uint8_t max_m8c_init(I2C_HandleTypeDef* i2c_handle, GPIO_TypeDef* reset_bank, uint16_t reset);

// sends blank nav pvt message to request data
uint8_t max_m8c_request_nav_pvt(I2C_HandleTypeDef* i2c_handle);

// reads bytes available for read
uint8_t max_m8c_available(I2C_HandleTypeDef* i2c_handle, uint16_t* bytes_avail);

// read a nav pvt packet
uint8_t max_m8c_read_nav_pvt(I2C_HandleTypeDef* i2c_handle, struct max_m8c_gps_data* gps_data);

// checksum calculator
void max_m8c_calc_checksum(uint8_t* buf, uint8_t len, uint8_t* CK_A, uint8_t* CK_B);

// reformats packet into a byte buffer, transmits over i2c
uint8_t max_m8c_i2c_transmit(I2C_HandleTypeDef* i2c_handle, struct ubx_packet_trimmed* packet);

// reads buffer from i2c, reformats into packet
uint8_t max_m8c_i2c_read_data(I2C_HandleTypeDef* i2c_handle, struct ubx_packet_trimmed* packet);

// reads data length
uint8_t max_m8c_i2c_read_avail(I2C_HandleTypeDef* i2c_handle, uint16_t* bytes_avail);

#endif

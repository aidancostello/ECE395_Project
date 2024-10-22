#include "MAX_M8C.h"

uint8_t max_m8c_init(I2C_HandleTypeDef* i2c_handle, GPIO_TypeDef* reset_bank, uint16_t reset) {
	// toggle reset
	HAL_GPIO_WritePin(reset_bank, reset, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(reset_bank, reset, GPIO_PIN_SET);

	return MAX_M8C_SUCCESS;
}

uint8_t max_m8c_request_nav_pvt(I2C_HandleTypeDef* i2c_handle) {
	// construct empty nav pvt packet
	struct ubx_packet_trimmed packet = {0};
	packet.class = CLASS_NAV;
	packet.id = ID_NAV_PVT;
	packet.length_lower = 0;
	packet.length_upper = 0;
	packet.payload = NULL;

	// transmit empty packet
	return(max_m8c_i2c_transmit(i2c_handle, &packet));
}

uint8_t max_m8c_available(I2C_HandleTypeDef* i2c_handle, uint16_t* bytes_avail) {
	return max_m8c_i2c_read_avail(i2c_handle, bytes_avail);
}

uint8_t max_m8c_read_nav_pvt(I2C_HandleTypeDef* i2c_handle, struct max_m8c_gps_data* gps_data) {
	// construct nav pvt packet
	struct ubx_packet_trimmed packet = {0};
	uint8_t buf[UBX_HEADER_SIZE+UBX_NAV_PVT_PAYLOAD_SIZE+UBX_CHECKSUM_SIZE];
	packet.payload = buf;

	// read packet
	if (max_m8c_i2c_read_data(i2c_handle, &packet) == MAX_M8C_FAIL) {
		return MAX_M8C_FAIL;
	}

	// check class, id, length
	if (packet.class != CLASS_NAV || packet.id != ID_NAV_PVT || ((packet.length_upper<<8)|packet.length_lower) != UBX_NAV_PVT_PAYLOAD_SIZE) {
		return MAX_M8C_FAIL;
	}

	gps_data->lon = *((int32_t*)(packet.payload+LON_BYTE_OFFSET));
	gps_data->lat = *((int32_t*)(packet.payload+LAT_BYTE_OFFSET));
	gps_data->alt = *((int32_t*)(packet.payload+HEIGHT_BYTE_OFFSET));

	return MAX_M8C_SUCCESS;
}

void max_m8c_calc_checksum(uint8_t* buf, uint8_t len, uint8_t* CK_A, uint8_t* CK_B) {
	*CK_A = 0;
	*CK_B = 0;
	for (uint8_t i = 0; i < len; i++) {
		*CK_A += buf[i];
		*CK_B += *CK_A;
	}
}

uint8_t max_m8c_i2c_transmit(I2C_HandleTypeDef* i2c_handle, struct ubx_packet_trimmed* packet) {
	uint8_t buf[UBX_MAX_PACKET_SIZE];
	uint16_t payload_size = (packet->length_upper << 8)|packet->length_lower;
	uint16_t buf_size = UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE + payload_size;

	// fill header
	buf[0] = SYNC_CHAR_1;
	buf[1] = SYNC_CHAR_2;
	buf[2] = packet->class;
	buf[3] = packet->id;
	buf[4] = packet->length_lower;
	buf[5] = packet->length_upper;

	// fill payload
	if (payload_size != 0) {
		memcpy(buf+UBX_HEADER_SIZE, packet->payload, payload_size);
	}

	// fill checksum
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	max_m8c_calc_checksum(buf+UBX_NUM_SYNC_BYTES, payload_size+(UBX_HEADER_SIZE-UBX_NUM_SYNC_BYTES), &CK_A, &CK_B);
	buf[buf_size-2] = CK_A;
	buf[buf_size-1] = CK_B;

	// transmit
	if (HAL_I2C_Master_Transmit(i2c_handle, I2C_ADDR, buf, buf_size, 1000) != HAL_OK){
		return MAX_M8C_FAIL;
	}

	return MAX_M8C_SUCCESS;
}

uint8_t max_m8c_i2c_read_data(I2C_HandleTypeDef* i2c_handle, struct ubx_packet_trimmed* packet) {
	uint8_t data_addr = REG_DATA;
	uint8_t buf[UBX_MAX_PACKET_SIZE];

	// request data
	if (HAL_I2C_Master_Transmit(i2c_handle, I2C_ADDR, &data_addr, 1, 1000) != HAL_OK) {
		return MAX_M8C_FAIL;
	}

	// receive data
	if (HAL_I2C_Master_Receive(i2c_handle, I2C_ADDR, buf, UBX_MAX_PACKET_SIZE, 1000) != HAL_OK) {
		return MAX_M8C_FAIL;
	}

	// check sync bytes
	if (buf[0] != SYNC_CHAR_1 || buf[1] != SYNC_CHAR_2) {
		return MAX_M8C_FAIL;
	}

	// validate checksum
	uint16_t payload_length = (packet->length_upper<<8)|packet->length_lower;
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	max_m8c_calc_checksum(buf+UBX_NUM_SYNC_BYTES, payload_length+(UBX_HEADER_SIZE-UBX_NUM_SYNC_BYTES), &CK_A, &CK_B);
	if (buf[UBX_HEADER_SIZE+payload_length] != CK_A || buf[UBX_HEADER_SIZE+payload_length+1] != CK_B) {
		return MAX_M8C_FAIL;
	}

	// fill packet
	packet->class = buf[2];
	packet->id = buf[3];
	packet->length_lower = buf[4];
	packet->length_upper = buf[5];
	memcpy(packet->payload, buf+UBX_HEADER_SIZE, payload_length);

	return MAX_M8C_SUCCESS;
}

uint8_t max_m8c_i2c_read_avail(I2C_HandleTypeDef* i2c_handle, uint16_t* bytes_avail) {
	uint8_t upper = 0;
	uint8_t lower = 0;
	uint8_t upper_addr = REG_AVAIL_UPPER;
	uint8_t lower_addr = REG_AVAIL_LOWER;

	// request upper data
	if (HAL_I2C_Master_Transmit(i2c_handle, I2C_ADDR, &upper_addr, 1, 1000) != HAL_OK) {
		return MAX_M8C_FAIL;
	}

	// receive upper data
	if (HAL_I2C_Master_Receive(i2c_handle, I2C_ADDR, &upper, 1, 1000) != HAL_OK) {
		return MAX_M8C_FAIL;
	}

	// request lower data
	if (HAL_I2C_Master_Transmit(i2c_handle, I2C_ADDR, &lower_addr, 1, 1000) != HAL_OK) {
		return MAX_M8C_FAIL;
	}

	// receive lower data
	if (HAL_I2C_Master_Receive(i2c_handle, I2C_ADDR, &lower, 1, 1000) != HAL_OK) {
		return MAX_M8C_FAIL;
	}

	*bytes_avail = (upper << 8) | lower;

	return MAX_M8C_SUCCESS;
}

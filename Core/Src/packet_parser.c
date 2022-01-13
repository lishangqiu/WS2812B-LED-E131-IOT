
#include "packet_parser.h"


const uint8_t ACN_ID[12] = { 0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00 };
uint8_t success_e[] = "Success!(E131)\n";


uint16_t convert_to_16(uint8_t d2, uint8_t d1);
uint32_t convert_to_32(uint8_t data_array[]);
e131_errors_t validate(e131_packet_t *E131_Object);

uint16_t convert_to_16(uint8_t d2, uint8_t d1) {
	uint16_t wd = ((uint16_t)d2 << 8) | d1;
  return wd;
}

uint32_t convert_to_32(uint8_t data_array[]) {
	uint16_t wd1 = convert_to_16(data_array[0],data_array[1]);
	uint16_t wd2 = convert_to_16(data_array[2], data_array[3]);
	
	uint32_t wd = ((uint32_t)wd1 << 16) | wd2;
  return wd;
}

e131_errors_t validate(e131_packet_t *E131_Object) {
	if (memcmp(E131_Object->E131_ACN, ACN_ID, sizeof(E131_Object->E131_ACN))) return E131_ACN_ID_ERROR;
	
	if (E131_Object->E131_Root_Vector != VECTOR_ROOT ) return E131_ROOT_VECTOR_ERROR;

	if (E131_Object->E131_Frame_Vector != VECTOR_FRAME) return E131_FRAME_VECTOR_ERROR;
	
	if (E131_Object->E131_DMP_Vector != VECTOR_DMP) return E131_DMP_VECTOR_ERROR;
	
	if (E131_Object->Data_Values[0] != 0) return E131_DATA_ERROR;
	
	return E131_STATUS_OK;
}

e131_errors_t parse_packet(e131_packet_t *Packet) {
	int i;
	uint8_t temp[4];
	
	
	for (i=0;i<E131_ROOT_ID_LENGTH;i++) {
		Packet->E131_ACN[i] = Packet->raw[i+E131_ROOT_ID];
	}
	
	for (i=0;i<E131_ROOT_VECTOR_LENGTH;i++) {
		temp[i] = Packet->raw[i+E131_ROOT_VECTOR];
	}
	Packet->E131_Root_Vector = convert_to_32(temp);
	
	for (i=0;i<E131_FRAME_VECTOR_LENGTH;i++) {
		temp[i] = Packet->raw[i+E131_FRAME_VECTOR];
	}
	Packet->E131_Frame_Vector = convert_to_32(temp);

	
	Packet->E131_Frame_Universe = convert_to_16(Packet->raw[E131_FRAME_UNIVERSE],Packet->raw[E131_FRAME_UNIVERSE+1]);
	Packet->E131_DMP_Vector = Packet->raw[E131_DMP_VECTOR];
	
	for (i=0;i<E131_DMP_DATA_LENGTH;i++) {
		Packet->Data_Values[i] = Packet->raw[i+E131_DMP_DATA];
	}
	
	//e131_errors_t status = validate(Packet);
	
	//return status;
	return E131_STATUS_OK;
}




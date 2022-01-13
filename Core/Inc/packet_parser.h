#ifndef __PACKET_PARSER_H
#define __PACKET_PARSER_H
 
#ifdef __cplusplus
 extern "C" {
#endif  

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stm32l4xx_hal.h"
#include "shared.h"



#define VECTOR_ROOT  0x00000004
#define VECTOR_FRAME  0x00000002
#define VECTOR_DMP  0x02

#define E131_ROOT_ID 4
#define E131_ROOT_ID_LENGTH 12
#define E131_ROOT_VECTOR 18
#define E131_ROOT_VECTOR_LENGTH 4
#define E131_FRAME_VECTOR 40
#define E131_FRAME_VECTOR_LENGTH 4
#define E131_FRAME_UNIVERSE 113
#define E131_FRAME_UNIVERSE_LENGTH 2
#define E131_DMP_VECTOR 117
#define E131_DMP_DATA 125
#define E131_DMP_DATA_LENGTH 513

typedef struct Packet {
	uint8_t E131_ACN[12];
	uint32_t E131_Root_Vector;
	uint32_t E131_Frame_Vector;
	uint16_t E131_Frame_Universe;
	uint8_t E131_DMP_Vector;
	uint8_t Data_Values[513];
	uint8_t raw[613];
} e131_packet_t;


typedef enum {
	E131_STATUS_OK = 0,
	E131_ACN_ID_ERROR = -1,
	E131_ROOT_VECTOR_ERROR = -2,
	E131_FRAME_VECTOR_ERROR = -3,
	E131_DMP_VECTOR_ERROR = -4,
	E131_DATA_ERROR = -5,
} e131_errors_t;
	

e131_errors_t parse_packet(e131_packet_t *Packet);
#ifdef __cplusplus
}
#endif
#endif /*__E131_H*/

/************************ (C) CORYPIGHT Nobody *****END OF FILE****/

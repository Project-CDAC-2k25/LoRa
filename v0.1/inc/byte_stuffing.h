/*
 * byte_stufffing.h
 *
 *  Created on: Aug 8, 2025
 *      Author: telum
 */
// byte_stuffing.h
#ifndef BYTE_STUFFING_H
#define BYTE_STUFFING_H
#include <stdint.h>
typedef void (*send_byte_func_t)(uint8_t byte);
void transmit_byte_stuffed(uint8_t byte, send_byte_func_t send_func);
typedef struct {
    uint8_t escape_flag;
} ByteUnstuffState;
int receive_byte_unstuffed(uint8_t received_byte, uint8_t* output_byte, ByteUnstuffState* state);
#endif
 /* INC_BYTE_STUFFING_H_ */

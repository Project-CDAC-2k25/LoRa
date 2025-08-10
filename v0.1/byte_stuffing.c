/*
 * byte_stuffing.c
 *
 *  Created on: Aug 8, 2025
 *      Author: telum
 */


// byte_stuffing.c
#include "byte_stuffing.h"

void transmit_byte_stuffed(uint8_t byte, send_byte_func_t send_func) {
    if (byte == 0x7E || byte == 0x7D) {
        send_func(0x7D);
        send_func(byte ^ 0x20);
    } else {
        send_func(byte);
    }
}

int receive_byte_unstuffed(uint8_t received_byte, uint8_t* output_byte, ByteUnstuffState* state) {
    if (state->escape_flag) {
        *output_byte = received_byte ^ 0x20;
        state->escape_flag = 0;
        return 1;
    } else if (received_byte == 0x7D) {
        state->escape_flag = 1;
        return 0;
    } else if (received_byte == 0x7E) {
        // Frame delimiter handled outside
        return 0;
    } else {
        *output_byte = received_byte;
        return 1;
    }
}

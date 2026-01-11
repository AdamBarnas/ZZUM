/*
 * gcode_rx.h
 *
 *  Created on: Jan 11, 2026
 *      Author: adamb
 */

#ifndef GCODE_RX_H
#define GCODE_RX_H

#include <stdint.h>
#include <stdbool.h>

#define GCODE_RX_BUF_SIZE 2048  // power of 2!

void gcode_rx_push(const uint8_t *data, uint16_t len);
bool gcode_rx_get_char(uint8_t *out);
uint16_t gcode_rx_available(void);

#endif

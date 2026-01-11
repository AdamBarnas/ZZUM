/*
 * gcode_rx.c
 *
 *  Created on: Jan 11, 2026
 *      Author: adamb
 */

#include "gcode_rx.h"

static uint8_t buf[GCODE_RX_BUF_SIZE];
static volatile uint16_t head = 0;
static volatile uint16_t tail = 0;

#define NEXT(i) ((uint16_t)((i + 1) & (GCODE_RX_BUF_SIZE - 1)))

void gcode_rx_push(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next = NEXT(head);
        if (next == tail)
            break;  // overflow

        buf[head] = data[i];
        head = next;
    }
}

bool gcode_rx_get_char(uint8_t *out)
{
    if (head == tail)
        return false;

    *out = buf[tail];
    tail = NEXT(tail);
    return true;
}

uint16_t gcode_rx_available(void)
{
    return (head >= tail) ? (head - tail)
                          : (GCODE_RX_BUF_SIZE - tail + head);
}

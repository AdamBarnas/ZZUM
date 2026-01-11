/*
 * gcode_rx.c
 *
 *  Created on: Jan 10, 2026
 *      Author: adamb
 */

#include "gcode_rx.h"

static uint8_t  rx_buf[GCODE_RX_BUFFER_SIZE];
static volatile uint16_t head = 0;
static volatile uint16_t tail = 0;

#define NEXT(idx) ((uint16_t)((idx + 1) & (GCODE_RX_BUFFER_SIZE - 1)))

/* Push data from UART ISR (DMA/IDLE callback) */
void gcode_rx_push(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next = NEXT(head);

        if (next == tail)
        {
            /* Buffer full: drop character */
            /* Optional: set overflow flag */
            break;
        }

        rx_buf[head] = data[i];
        head = next;
    }
}

/* Get one character (called from main loop) */
bool gcode_rx_get_char(uint8_t *out)
{
    if (head == tail)
        return false;

    *out = rx_buf[tail];
    tail = NEXT(tail);
    return true;
}

/* Number of bytes currently stored */
uint16_t gcode_rx_available(void)
{
    return (head >= tail)
           ? (head - tail)
           : (GCODE_RX_BUFFER_SIZE - tail + head);
}

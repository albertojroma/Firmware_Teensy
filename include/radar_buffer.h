/**
 * @file radar_buffer.h
 * @brief Memoria FIFO (Ring Buffer) para el desacoplo temporal del radar US-D1.
 * @author Alberto Jesús Rodríguez Machado
 * @date 2026
 */
#ifndef RADAR_BUFFER_H
#define RADAR_BUFFER_H

#include <Arduino.h>
#include "radar_uart.h" // Para conocer la estructura RadarFrame

#define BUFFER_SIZE 256
#define BUFFER_MASK (BUFFER_SIZE - 1)

void RadarBuffer_Init(void);
bool RadarBuffer_Push(const RadarFrame& frame);
bool RadarBuffer_Pop(RadarFrame& frame);

#endif // RADAR_BUFFER_H
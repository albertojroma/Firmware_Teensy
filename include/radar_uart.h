/**
 * @file radar_uart.h
 * @brief Módulo de adquisición UART para el radar Ainstein US-D1.
 */
#ifndef RADAR_UART_H
#define RADAR_UART_H

#include <Arduino.h>

/** @brief Estructura de datos de 8 bytes (alineada). */
typedef struct {
  uint32_t timestamp_us; 
  uint16_t altitud_cm;   
  uint8_t  snr;          
  bool     valida;       
} RadarFrame;

void RadarUART_Init(uint32_t baud = 115200);

/** @brief Procesa los bytes entrantes en la UART1. */
void RadarUART_Update(void);

#endif // RADAR_UART_H
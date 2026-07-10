/**
 * @file radar_uart.h
 * @brief Módulo de adquisición UART para el radar Ainstein US-D1.
 *
 * VERSIÓN HÍBRIDA: combina el buffer circular y la estrategia de
 * flush() diferido de la versión generada por otra IA, con la captura
 * por interrupción real sobre LPUART6 que ya validamos en mensajes
 * anteriores (necesaria para evitar el jitter de temporización que
 * introduce serialEvent1(), al no ser una interrupción real).
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

/**
 * @brief Inicializa Serial1 (LPUART6) e instala la interrupción real
 *        de recepción del radar. Las tramas válidas se empujan
 *        automáticamente al buffer circular (RadarBuffer_Push).
 * @param baud Velocidad en baudios (115200 según datasheet del US-D1).
 */
void RadarUART_Init(uint32_t baud = 115200);

#endif // RADAR_UART_H
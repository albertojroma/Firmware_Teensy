/**
 * @file radar_logger.h
 * @brief Registro de tramas en CSV sobre SD sin bloqueos.
 */
#ifndef RADAR_LOGGER_H
#define RADAR_LOGGER_H

#include <Arduino.h>
#include "radar_uart.h"

bool RadarLogger_Init(const char *nombreArchivo);
bool RadarLogger_Guardar(const RadarFrame& frame);
void RadarLogger_Flush(void);

#endif // RADAR_LOGGER_H
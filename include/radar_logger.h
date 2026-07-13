/**
 * @file radar_logger.h
 * @brief Registro de tramas del radar en CSV sobre la SD, sin bloqueo
 *        por escritura fisica en cada trama.
 *
 * @details
 * Separa deliberadamente escribir en el bufer de la libreria SD
 * (rapido, RadarLogger_Guardar()) de forzar su escritura fisica
 * (lento, RadarLogger_Flush()) -- ver razonamiento en radar_logger.cpp.
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef RADAR_LOGGER_H
#define RADAR_LOGGER_H

#include <Arduino.h>
#include "radar_uart.h"

/**
 * @brief Inicializa la SD y crea un fichero CSV nuevo con nombre
 *        "vuelo_N.csv", donde N es el primer numero disponible que
 *        no coincide con ningun fichero ya existente en la tarjeta.
 * @return true si SD y fichero quedaron listos; false si no.
 */
bool RadarLogger_Init(void);

/**
 * @brief Anade una fila al CSV, sin forzar su escritura fisica.
 * @details Registra tanto tramas validas como invalidas -- ver la
 * nota sobre el campo `valida` en radar_uart.h.
 * @param frame Trama a registrar.
 * @return true si se escribio en el bufer; false si el fichero no esta
 * disponible.
 * @see RadarLogger_Flush()
 */
bool RadarLogger_Guardar(const RadarFrame &frame);

/**
 * @brief Fuerza la escritura fisica del bufer a la SD.
 * @return true si la escritura fue correcta; false si se detecto un
 *         fallo (p. ej. SD retirada), en cuyo caso el fichero queda
 *         inutilizable para el resto de la sesion.
 */
bool RadarLogger_Flush(void);

#endif // RADAR_LOGGER_H

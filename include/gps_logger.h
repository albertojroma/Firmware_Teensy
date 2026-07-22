/**
 * @file gps_logger.h
 * @brief Registro del paso continuo binario de tramas UBX del GPS-PPK
 * sobre la SD, para post-proceso con RTKLIB.
 *
 * @details
 * Gestiona un unico fichero por sesion: "gps_logger_N.ubx", copia
 * binaria integra de cada trama UBX completa y valida, sin descomponer
 * ni renombrar por trama -- mismo patron que el proyecto de referencia
 * F9P_RAWX_Logger, que RTKLIB (RTKCONV) espera directamente como
 * fichero continuo.
 *
 * @note El par de sincronizacion (timestamp del MCU, tiempo GPS) que
 * antes gestionaba este modulo en un segundo fichero
 * ("correlacion_temporal_N.csv") se ha trasladado a
 * RadarLogger_GuardarSyncGps() (radar_logger.h), como filas de tipo
 * "GPS" dentro del mismo CSV que ya usa el radar -- ver la justificacion
 * completa (reduccion de 3 a 2 ficheros simultaneos en la SD) en
 * radar_logger.h.
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef GPS_LOGGER_H
#define GPS_LOGGER_H

#include <Arduino.h>

/**
 * @brief Abre el fichero de paso continuo binario de la sesion de GPS.
 * @param numeroVuelo Numero de sesion, para que "gps_logger_N.ubx"
 * comparta el mismo N que "radar_logger_N.csv" -- ver
 * RadarLogger_ObtenerNumeroVuelo().
 * @return true si el fichero quedo listo; false si falla.
 * @note NO llama a SD.begin(): se asume que RadarLogger_Init() ya
 * inicializo la tarjeta antes de que este modulo se use.
 */
bool GpsLogger_Init(int numeroVuelo);

/**
 * @brief Anade los bytes crudos de una trama UBX completa al fichero
 * de paso continuo, sin forzar su escritura fisica.
 * @param datos Puntero al inicio de la trama (byte de sync).
 * @param longitud Tamano total de la trama, en bytes.
 * @return true si se escribio en el bufer; false si el fichero no esta
 * disponible o no se escribieron todos los bytes.
 * @see GpsLogger_Flush()
 */
bool GpsLogger_GuardarTramaCruda(const uint8_t *datos, uint16_t longitud);

/**
 * @brief Fuerza la escritura fisica del fichero a la SD.
 * @return true si la escritura fue correcta; false si se detecto un fallo.
 */
bool GpsLogger_Flush(void);

#endif // GPS_LOGGER_H

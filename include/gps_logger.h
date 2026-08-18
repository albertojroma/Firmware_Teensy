/**
 * @file gps_logger.h
 * @brief Registro del paso continuo binario de tramas UBX del GPS-PPK
 * sobre la SD, para post-proceso con RTKLIB.
 *
 * @details
 * Gestiona un unico fichero por sesion: "gps_logger_N.ubx", copia
 * binaria integra de cada trama UBX completa y valida, sin descomponer
 * ni renombrar por trama
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
 * comparta el mismo N que "data_logger_N.csv" -- ver
 * DataLogger_ObtenerNumeroVuelo().
 * @return true si el fichero quedo listo; false si falla.
 * @note NO llama a SD.begin(): se asume que DataLogger_Init() ya
 * inicializo la tarjeta antes de que este modulo se use.
 */
bool GpsLogger_Init(int numeroVuelo);

/**
 * @brief Anyade los bytes crudos de una trama UBX completa al fichero
 * de paso continuo, sin forzar su escritura fisica.
 * @param datos Puntero al inicio de la trama (byte de sync).
 * @param longitud Tamano total de la trama, en bytes.
 * @return true si se escribio en el bufer; false si el fichero no esta
 * disponible o no se escribieron todos los bytes.
 * @details Escribe los bytes tal cual, sin ningun procesado.
 *
 * @note Los bytes son ya una trama UBX completa y con checksum valido
 * (verificado antes de llamar a esta funcion, en gps_uart.cpp).
 * @see GpsLogger_Flush()
 */
bool GpsLogger_GuardarTramaCruda(const uint8_t *datos, uint16_t longitud);

/**
 * @brief Fuerza la escritura fisica del fichero a la SD.
 * @return true si la escritura fue correcta; false si se detecto un fallo.
 */
bool GpsLogger_Flush(void);

#endif // GPS_LOGGER_H

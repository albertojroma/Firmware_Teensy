/**
 * @file data_logger.h
 * @brief Registro combinado de tramas del radar y filas de sincronizacion
 *        del GPS en un unico CSV sobre la SD
 *
 * @details
 * Separa deliberadamente escribir en el bufer de la libreria SD
 * (rapido, DataLogger_Guardar()/DataLogger_GuardarSyncGps()) de forzar
 * su escritura fisica (lento, DataLogger_Flush()) -- ver razonamiento
 * en data_logger.cpp.
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <Arduino.h>
#include "radar_uart.h"
#include "gps_uart.h" // Para GpsSyncPoint

/**
 * @brief Inicializa la SD y crea un fichero CSV nuevo con nombre
 *        "data_logger_N.csv", donde N es el primer numero disponible
 *        que no coincide con ningun fichero ya existente en la tarjeta.
 * @details La cabecera de columnas escrita es:
 *          tipo,timestamp_us,altitud_cm,snr,checksum_recibido,checksum_valido,
 *          trama_cruda,rcvTow,week
 *          Las filas de tipo "RADAR" dejan vacias rcvTow/week; las filas
 *          de tipo "GPS" dejan vacias las columnas propias del radar.
 * @return true si SD y fichero quedaron listos; false si no, o si se
 *         alcanzo el limite maximo de sesiones sin encontrar un nombre
 *         libre (ver el limite del bucle en data_logger.cpp).
 * @warning GpsLogger_Init() depende de que esta funcion se haya
 * ejecutado con exito antes: toma el numero de sesion de
 * DataLogger_ObtenerNumeroVuelo() para nombrar "gps_logger_N.ubx" con
 * el mismo N (ver gps_logger.h y el orden de inicializacion en
 * main.cpp).
 */
bool DataLogger_Init(void);

/**
 * @brief Anade una fila de tipo "RADAR" al CSV, sin forzar su escritura fisica.
 * @param frame Trama a registrar.
 * @return true si se escribio en el bufer; false si el fichero no esta
 * disponible.
 * @see DataLogger_Flush()
 */
bool DataLogger_Guardar(const RadarFrame &frame);

/**
 * @brief Anade una fila de tipo "GPS" al mismo CSV, con el par de
 *        sincronizacion (timestamp del MCU, tiempo GPS), sin forzar su
 *        escritura fisica.
 * @details Pensada para invocarse desde gestionarTramaCompleta()
 * (gps_uart.cpp) cada vez que se reensambla una UBX-RXM-RAWX valida.
 * Las columnas propias del radar (altitud_cm, snr, checksum_recibido,
 * checksum_valido, trama_cruda) quedan vacias en esta fila.
 * @param punto Par (timestamp del MCU, tiempo GPS) a registrar.
 * @return true si se escribio en el bufer; false si el fichero no esta
 * disponible.
 * @see DataLogger_Flush()
 */
bool DataLogger_GuardarSyncGps(const GpsSyncPoint &punto);

/**
 * @brief Fuerza la escritura fisica del bufer a la SD.
 * @return true si la escritura fue correcta; false si se detecto un
 *         fallo (p. ej. SD retirada), en cuyo caso el fichero queda
 *         inutilizable para el resto de la sesion.
 */
bool DataLogger_Flush(void);

/**
 * @brief Devuelve el numero de sesion ("vuelo") elegido por
 *        DataLogger_Init() al crear "data_logger_N.csv".
 * @details Pensada para que otros modulos (p. ej. GpsLogger_Init())
 * puedan usar el mismo numero N sin tener que escanear la tarjeta por
 * su cuenta -- evita que ambos numeros puedan desincronizarse.
 * @warning Debe llamarse DESPUES de un DataLogger_Init() con exito.
 * Si se llama antes, el valor devuelto no esta definido.
 * @return int: numero de sesion actual.
 */
int DataLogger_ObtenerNumeroVuelo(void);

#endif // DATA_LOGGER_H

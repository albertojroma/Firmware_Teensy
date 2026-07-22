/**
 * @file radar_logger.h
 * @brief Registro de tramas del radar en CSV sobre la SD, ahora tambien
 *        combinado con las filas de sincronizacion del GPS en el mismo
 *        fichero -- sin bloqueo por escritura fisica en cada trama.
 *
 * @details
 * Separa deliberadamente escribir en el bufer de la libreria SD
 * (rapido, RadarLogger_Guardar()/RadarLogger_GuardarSyncGps()) de forzar
 * su escritura fisica (lento, RadarLogger_Flush()) -- ver razonamiento
 * en radar_logger.cpp.
 *
 * @note CAMBIO DE DISENO: el CSV combina ahora dos tipos de fila (columna
 * `tipo`, valores "RADAR" o "GPS"), en vez de que el GPS tuviera su
 * propio fichero de sincronizacion aparte ("correlacion_temporal_N.csv").
 * Motivo: se detecto empiricamente, mediante biseccion exhaustiva, que
 * tener 3 ficheros abiertos simultaneamente en la SD, combinado con la
 * presencia de Serial3.write() en el binario, provocaba un cuelgue
 * deterministico en flush() cuya causa raiz exacta no se llego a
 * identificar pese a un analisis estatico exhaustivo (memoria,
 * direcciones, simbolos, desensamblado). Reducir a 2 ficheros
 * simultaneos (este CSV combinado + gps_logger_N.ubx) elimino el
 * problema de forma reproducible. Es un mitigante empirico, no una
 * solucion con causa raiz confirmada.
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef RADAR_LOGGER_H
#define RADAR_LOGGER_H

#include <Arduino.h>
#include "radar_uart.h"
#include "gps_uart.h" // Para GpsSyncPoint

/**
 * @brief Inicializa la SD y crea un fichero CSV nuevo con nombre
 *        "radar_logger_N.csv", donde N es el primer numero disponible
 *        que no coincide con ningun fichero ya existente en la tarjeta.
 * @details La cabecera de columnas escrita es:
 *          tipo,timestamp_us,altitud_cm,snr,checksum_recibido,checksum_valido,trama_cruda,rcvTow,week
 *          Las filas de tipo "RADAR" dejan vacias rcvTow/week; las filas
 *          de tipo "GPS" dejan vacias las columnas propias del radar.
 * @return true si SD y fichero quedaron listos; false si no.
 */
bool RadarLogger_Init(void);

/**
 * @brief Anade una fila de tipo "RADAR" al CSV, sin forzar su escritura fisica.
 * @details Registra tanto tramas validas como invalidas -- ver la
 * nota sobre el campo `valida` en radar_uart.h.
 * @param frame Trama a registrar.
 * @return true si se escribio en el bufer; false si el fichero no esta
 * disponible.
 * @see RadarLogger_Flush()
 */
bool RadarLogger_Guardar(const RadarFrame &frame);

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
 * @see RadarLogger_Flush()
 */
bool RadarLogger_GuardarSyncGps(const GpsSyncPoint &punto);

/**
 * @brief Fuerza la escritura fisica del bufer a la SD.
 * @return true si la escritura fue correcta; false si se detecto un
 *         fallo (p. ej. SD retirada), en cuyo caso el fichero queda
 *         inutilizable para el resto de la sesion.
 */
bool RadarLogger_Flush(void);

/**
 * @brief Devuelve el numero de sesion ("vuelo") elegido por
 *        RadarLogger_Init() al crear "radar_logger_N.csv".
 * @details Pensada para que otros modulos (p. ej. GpsLogger_Init())
 * puedan usar el mismo numero N sin tener que escanear la tarjeta por
 * su cuenta -- evita que ambos numeros puedan desincronizarse.
 * @warning Debe llamarse DESPUES de un RadarLogger_Init() con exito.
 * Si se llama antes, el valor devuelto no esta definido.
 * @return int: numero de sesion actual.
 */
int RadarLogger_ObtenerNumeroVuelo(void);

#endif // RADAR_LOGGER_H

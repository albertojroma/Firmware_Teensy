/**
 * @file gps_logger.cpp
 * @brief Implementacion del registro del paso continuo binario UBX sobre SD.
 * @see gps_logger.h para el proposito del modulo y por que ya no
 *      gestiona el fichero de sincronizacion (ver radar_logger.h).
 */
#include "gps_logger.h"
#include <SD.h>

/** @brief Fichero de paso continuo binario (tramas UBX crudas). */
static File s_archivoUbx;

/**
 * @copydoc GpsLogger_Init
 */
bool GpsLogger_Init(int numeroVuelo)
{
  char nombreUbx[24];
  snprintf(nombreUbx, sizeof(nombreUbx), "gps_logger_%d.ubx", numeroVuelo);

  s_archivoUbx = SD.open(nombreUbx, FILE_WRITE);
  if (!s_archivoUbx)
    return false;

  Serial.println("[DEBUG]: Logger del GPS inicializado.");
  return true;
}

/**
 * @copydoc GpsLogger_GuardarTramaCruda
 * @details Escribe los bytes tal cual, sin ningun procesado -- son ya
 * una trama UBX completa y con checksum valido (verificado antes de
 * llamar a esta funcion, en gps_uart.cpp).
 */
bool GpsLogger_GuardarTramaCruda(const uint8_t *datos, uint16_t longitud)
{
  if (!s_archivoUbx)
    return false;

  size_t escritos = s_archivoUbx.write(datos, longitud);
  // AVISO: aqui NO hay flush()! Mismo criterio de flush diferido que
  // RadarLogger_Guardar() -- ver GpsLogger_Flush().
  return escritos == longitud;
}

/**
 * @copydoc GpsLogger_Flush
 */
bool GpsLogger_Flush(void)
{
  if (!s_archivoUbx)
    return false;

  s_archivoUbx.flush();
  if (s_archivoUbx.getWriteError())
  {
    s_archivoUbx.clearWriteError();
    return false;
  }
  return true;
}

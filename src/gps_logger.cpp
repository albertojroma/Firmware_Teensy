/**
 * @file gps_logger.cpp
 * @brief Implementacion del registro del paso continuo binario UBX sobre SD.
 * @see gps_logger.h para el proposito del modulo y por que ya no
 *      gestiona el fichero de sincronizacion (ver data_logger.h).
 */
#include "gps_logger.h"
#include <SD.h>

/** @brief Fichero de paso continuo binario (tramas UBX crudas). */
static File s_archivoUbx;

/* Implementacion de GpsLogger_Init(). Documentacion en gps_logger.h */
bool GpsLogger_Init(int numeroVuelo)
{
  // numeroVuelo viene de DataLogger_Init(), limitado a
  // DATA_LOGGER_NUMERO_VUELO_MAX = 999 -> "gps_logger_999.ubx" son 18
  // caracteres + \0 = 19 bytes. 24 tiene margen de sobra sin ser
  // excesivo.
  char nombreUbx[24];
  snprintf(nombreUbx, sizeof(nombreUbx), "gps_logger_%d.ubx", numeroVuelo);

  s_archivoUbx = SD.open(nombreUbx, FILE_WRITE);
  if (!s_archivoUbx)
    return false;

  Serial.println("[DEBUG]: Logger del GPS inicializado.");
  return true;
}

/* Implementacion de GpsLogger_GuardarTramaCruda(). Documentacion en gps_logger.h */
bool GpsLogger_GuardarTramaCruda(const uint8_t *datos, uint16_t longitud)
{
  if (!s_archivoUbx)
    return false;

  // size_t porque es el tipo que devuelve File::write(): el tipo
  // entero sin signo estandar en C/C++ para tamanyos y recuentos.
  size_t escritos = s_archivoUbx.write(datos, longitud);
  // AVISO: aqui NO hay flush()! Mismo criterio de flush diferido que
  // DataLogger_Guardar(). Ver GpsLogger_Flush().
  return escritos == longitud;
}

/* Implementacion de GpsLogger_Flush(). Documentacion en gps_logger.h */
bool GpsLogger_Flush(void)
{
  if (!s_archivoUbx)
    return false;

  // flush() fuerza la escritura a la SD de lo que queda en el buffer
  // de la libreria, sin esperar a que se llene o a que se cierre el
  // fichero. Limita la perdida de datos ante un corte.
  s_archivoUbx.flush();

  // getWriteError() indica si la SD marco un fallo de escritura real
  // (el flag persiste hasta limpiarse); comprobarlo tras flush() es lo
  // que detecta el fallo fisico, ya que write() puede devolver
  // escritos == longitud sin que el dato haya llegado aun a la SD.
  // clearWriteError() limpia el flag.
  if (s_archivoUbx.getWriteError())
  {
    s_archivoUbx.clearWriteError();
    return false; // Fallo real detectado al volcar a la SD
  }
  return true;
}

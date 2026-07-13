/**
 * @file radar_logger.cpp
 * @brief Implementacion del registro de tramas del radar en CSV sobre SD.
 * @see radar_logger.h para el proposito general del modulo.
 */
#include "radar_logger.h"
#include <SD.h>

/** @brief Fichero CSV abierto durante toda la sesion de registro. */
static File s_archivo;

/**
 * @brief Escribe un byte en hexadecimal de 2 digitos (con cero a la izquierda si hace falta).
 * @details Auxiliar interno para formatear checksum_recibido y trama_cruda.
 */
static void imprimirHexByte(uint8_t valor)
{
  if (valor < 0x10)
    s_archivo.print("0");
  s_archivo.print(valor, HEX);
}

/**
 * @details `SD.begin(BUILTIN_SDCARD)` usa la ranura microSD integrada
 * en la propia Teensy 4.1 (no un lector SPI externo).
 */
bool RadarLogger_Init(void)
{
  if (!SD.begin(BUILTIN_SDCARD))
    return false;

  char nombreArchivo[20];
  int numeroVuelo = 1;
  do
  {
    snprintf(nombreArchivo, sizeof(nombreArchivo), "vuelo_%d.csv", numeroVuelo);
    numeroVuelo++;
  } while (SD.exists(nombreArchivo));
  // Al salir del bucle, nombreArchivo es garantizado nuevo -> siempre
  // hay que escribir la cabecera.

  s_archivo = SD.open(nombreArchivo, FILE_WRITE);
  if (!s_archivo)
    return false;

  s_archivo.println("timestamp_us,altitud_cm,snr,checksum_recibido,checksum_valido,trama_cruda");
  s_archivo.flush();
  return true;
}

/**
 * @details
 * `checksum_recibido` se extrae de `frame.trama_cruda[5]` (ultimo byte de la
 * trama cruda).
 * `trama_cruda` se escribe como 6 bytes en hexadecimal separados por espacios
 * (p. ej. "FE 02 3A 05 23 18"), sin comas dentro del campo para no romper el
 * formato CSV.
 *
 * **Por que NO llama a `flush()` en cada trama:** `flush()` es una escritura
 * fisica sincrona de varios milisegundos; a 100 Hz (una trama cada 10 ms),
 * hacerlo aqui arriesgaria que loop() no vaciara el buffer circular al ritmo
 * al que la ISR lo llena. Se escribe solo en el bufer RAM de la libreria SD;
 * la escritura fisica se difiere a RadarLogger_Flush(), invocada cada 1000 ms
 * desde main.cpp.
 *
 * @warning Trade-off: hasta ~1 s de datos (~100 tramas) quedan solo en
 * RAM hasta el siguiente flush. Un corte de alimentacion en ese intervalo los
 * perderia. Si esto no fuese viable, se puede reducir la cadencia de
 * `RadarLogger_Flush()` en main.cpp (p. ej. a 100-200 ms).
 */
bool RadarLogger_Guardar(const RadarFrame &frame)
{
  if (!s_archivo)
    return false;

  s_archivo.print(frame.timestamp_us);
  s_archivo.print(",");
  s_archivo.print(frame.altitud_cm);
  s_archivo.print(",");
  s_archivo.print(frame.snr);
  s_archivo.print(",");
  imprimirHexByte(frame.trama_cruda[5]); // checksum_recibido
  s_archivo.print(",");
  s_archivo.print(frame.valida ? "True" : "False");
  s_archivo.print(",");
  for (uint8_t i = 0; i < 6; i++)
  {
    imprimirHexByte(frame.trama_cruda[i]);
    if (i < 5)
      s_archivo.print(" ");
  }
  s_archivo.println();

  // AVISO: aqui NO hay flush()! Solo guardamos en cache (ver @details).
  return true;
}

bool RadarLogger_Flush(void)
{
  if (!s_archivo)
    return false;

  s_archivo.flush();

  if (s_archivo.getWriteError())
  {
    s_archivo.clearWriteError();
    return false; // Fallo real detectado al volcar a la SD
  }
  return true;
}

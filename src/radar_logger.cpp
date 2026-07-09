/**
 * @file radar_logger.cpp
 * @brief Implementación del registro SD.
 */
#include "radar_logger.h"
#include <SD.h>

static File s_archivo;

bool RadarLogger_Init(const char *nombreArchivo) {
  if (!SD.begin(BUILTIN_SDCARD)) return false;

  bool esNuevo = !SD.exists(nombreArchivo);
  s_archivo = SD.open(nombreArchivo, FILE_WRITE);
  
  if (!s_archivo) return false;

  if (esNuevo) {
    s_archivo.println("timestamp_us,altitud_cm,snr");
    s_archivo.flush();
  }
  return true;
}

bool RadarLogger_Guardar(const RadarFrame& frame) {
  if (!s_archivo) return false;

  s_archivo.print(frame.timestamp_us);
  s_archivo.print(",");
  s_archivo.print(frame.altitud_cm);
  s_archivo.print(",");
  s_archivo.println(frame.snr);
  
  // AVISO: ¡Aquí NO hay flush()! Solo guardamos en caché.
  return true;
}

void RadarLogger_Flush(void) {
  if (s_archivo) {
    s_archivo.flush(); // Fuerza la escritura física en la SD
  }
}
/**
 * @file radar_logger.cpp
 * @brief Implementacion del registro de tramas del radar en CSV sobre SD.
 * @see radar_logger.h para el proposito general del modulo.
 */
#include "radar_logger.h"
#include <SD.h>

/** @brief Manejador del fichero CSV abierto durante toda la sesion de registro. */
static File s_archivo;

/**
 * @copydoc RadarLogger_Init
 *
 * @details
 * SD.begin(BUILTIN_SDCARD) inicializa especificamente la ranura
 * microSD integrada en la propia placa Teensy 4.1 (a diferencia de un
 * lector SD externo conectado por SPI a pines genericos, que
 * requeriria pasar el numero de pin de chip select correspondiente en
 * vez de esta constante).
 *
 * El fichero se abre en modo FILE_WRITE, que en la libreria SD de
 * Teensyduino posiciona el cursor de escritura al final del fichero
 * si este ya existia (append), o lo crea vacio si no existia -- de
 * ahi que sea necesario comprobar esNuevo ANTES de abrir el fichero
 * (con SD.exists()), porque tras SD.open() ya no habria forma sencilla
 * de distinguir si el fichero se acaba de crear o ya tenia contenido
 * previo.
 */
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

/**
 * @copydoc RadarLogger_Guardar
 *
 * @details
 * **Por que esta funcion NO llama a flush() en cada trama:** flush()
 * fuerza una escritura fisica sincrona en la memoria flash de la
 * tarjeta SD, una operacion que puede tardar varios milisegundos (mas
 * aun si implica volcar una pagina fisica completa). El radar US-D1
 * transmite a 100 Hz, es decir, una trama cada 10 ms de media: si
 * cada llamada a esta funcion bloqueara ese tiempo variable de
 * flush(), existiria un riesgo real de que loop() no consiguiera
 * vaciar el buffer circular (radar_buffer.h) al ritmo al que la
 * interrupcion del radar lo llena, provocando perdida de tramas por
 * desbordamiento. En su lugar, esta funcion solo escribe en el bufer
 * interno que la propia libreria SD mantiene en memoria RAM
 * (operacion rapida), y la escritura fisica se difiere a
 * RadarLogger_Flush(), invocada desde loop() mediante un temporizador
 * no bloqueante cada 1000 ms (ver main.cpp).
 *
 * @warning **Trade-off asumido por este diseno:** al diferir la
 * escritura fisica, existe una ventana de hasta ~1 segundo de datos
 * (hasta ~100 tramas a 100 Hz) que permanecen unicamente en el bufer
 * en RAM de la libreria SD, sin haberse volcado todavia a la memoria
 * flash de la tarjeta. Si se produjera un corte de alimentacion
 * repentino en ese intervalo (por ejemplo, por vibracion en el
 * conector de la bateria durante un vuelo), esos datos se perderian
 * de forma irrecuperable. Esta es una decision de diseno consciente
 * que prioriza la fiabilidad de la escritura sostenida a 100 Hz frente
 * a la garantia de cero perdida de datos ante un corte de
 * alimentacion; si esta ventana de riesgo se considerara inaceptable
 * para el objetivo del proyecto, la cadencia de RadarLogger_Flush()
 * podria reducirse (p. ej. a 100-200 ms) a cambio de una mayor
 * frecuencia de escrituras fisicas.
 */
bool RadarLogger_Guardar(const RadarFrame& frame) {
  if (!s_archivo) return false;

  s_archivo.print(frame.timestamp_us);
  s_archivo.print(",");
  s_archivo.print(frame.altitud_cm);
  s_archivo.print(",");
  s_archivo.println(frame.snr);

  // AVISO: Aqui NO hay flush()! Solo guardamos en cache.
  return true;
}

/**
 * @copydoc RadarLogger_Flush
 */
void RadarLogger_Flush(void) {
  if (s_archivo) {
    s_archivo.flush(); // Fuerza la escritura fisica en la SD
  }
}

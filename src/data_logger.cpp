/**
 * @file data_logger.cpp
 * @brief Implementacion del registro combinado (radar + sincronizacion
 *        GPS) en CSV sobre SD.
 * @see data_logger.h para el proposito general del modulo y por que se
 *      combinaron ambos flujos en un unico fichero.
 */
#include "data_logger.h"
#include <SD.h>

/** @brief Fichero CSV abierto durante toda la sesion de registro (radar + GPS). */
static File s_archivo;

/**
 * @brief Numero de sesion ("vuelo") elegido por DataLogger_Init().
 * @details -1 mientras no se haya llamado a DataLogger_Init() con
 * exito -- ver DataLogger_ObtenerNumeroVuelo().
 */
static int s_numeroVueloActual = -1;

/**
 * @def DATA_LOGGER_NUMERO_VUELO_MAX
 * @brief Limite maximo de sesiones que DataLogger_Init() esta dispuesto
 * a probar al buscar un nombre de fichero libre.
 */
#define DATA_LOGGER_NUMERO_VUELO_MAX 999

/**
 * @brief Escribe un byte en hexadecimal de 2 digitos (con cero a la izquierda si hace falta).
 * @details Auxiliar interno para formatear checksum_recibido y trama_cruda.
 */
static void imprimirHexByte(uint8_t valor)
{
  // print(valor, HEX) no rellena con ceros a la izquierda: 0x05 se
  // imprimiria como "5". Se antepone "0" manualmente para mantener
  // siempre el ancho fijo de 2 caracteres por byte.
  if (valor < 0x10)
    s_archivo.print("0");
  s_archivo.print(valor, HEX);
}

/**
 * @details `SD.begin(BUILTIN_SDCARD)` usa la ranura microSD integrada
 * en la propia Teensy 4.1 (no un lector SPI externo).
 */
bool DataLogger_Init(void)
{
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("[ERROR]: Tarjeta SD no insertada");
    return false;
  }

  // "data_logger_999.csv" (999 = DATA_LOGGER_NUMERO_VUELO_MAX) son 20
  // caracteres + \0 = 21 bytes. 32 tiene margen de sobra, es multiplo
  // de 2 y no es excesivamente grande.
  char nombreArchivo[32];
  uint16_t numeroVuelo = 1;
  do
  {
    if (numeroVuelo > DATA_LOGGER_NUMERO_VUELO_MAX)
    {
      Serial.println("[ERROR]: Limite de sesiones alcanzado, no se encontro nombre de fichero libre");
      return false;
    }
    // snprintf formatea igual que sprintf (sustituye %d por numeroVuelo)
    // pero recibe el tamano del buffer y nunca escribe mas alla de el,
    // truncando si hiciera falta -> evita el desbordamiento de sprintf.
    snprintf(nombreArchivo, sizeof(nombreArchivo), "data_logger_%d.csv", numeroVuelo);
    numeroVuelo++;
  } while (SD.exists(nombreArchivo));
  // Al salir del bucle, nombreArchivo es garantizado nuevo -> siempre
  // hay que escribir la cabecera.

  // SD.open() abre el fichero y devuelve un objeto File. Con FILE_WRITE
  // lo crea si no existe y posiciona el cursor al final (append). Si
  // falla, el File devuelto se evalua como falso (operator bool()).
  s_archivo = SD.open(nombreArchivo, FILE_WRITE);
  if (!s_archivo)
  {
    Serial.println("[ERROR]: El archivo no se ha podido abrir");
    return false;
  }

  // numeroVuelo ya se incremento una vez de mas tras encontrar el hueco
  // libre en el bucle do/while -> el numero real usado es numeroVuelo-1.
  s_numeroVueloActual = numeroVuelo - 1;

  s_archivo.println("tipo,timestamp_us,altitud_cm,snr,checksum_recibido,checksum_valido,trama_cruda,rcvTow,week");
  s_archivo.flush();

  Serial.println("[DEBUG]: Logger de datos inicializado.");
  return true;
}

/**
 * @details
 * Antepone "RADAR," como columna `tipo`, y anade ",," al final de la
 * fila (columnas rcvTow/week vacias -- no aplican a una fila de radar).
 *
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
 * la escritura fisica se difiere a DataLogger_Flush(), invocada cada 1000 ms
 * desde main.cpp.
 *
 * @warning Trade-off: hasta ~1 s de datos (~100 tramas) quedan solo en
 * RAM hasta el siguiente flush. Un corte de alimentacion en ese intervalo los
 * perderia. Si esto no fuese viable, se puede reducir la cadencia de
 * `DataLogger_Flush()` en main.cpp (p. ej. a 100-200 ms).
 */
bool DataLogger_Guardar(const RadarFrame &frame)
{
  if (!s_archivo)
    return false;

  s_archivo.print("RADAR,");
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
      // se deja un espacio entre byte y byte de la trama cruda
      s_archivo.print(" ");
  }
  s_archivo.println(",,"); // columnas rcvTow,week vacias (no aplican a una fila RADAR)

  return true;
}

/**
 * @details
 * Antepone "GPS," como columna `tipo`, deja vacias las 5 columnas
 * propias del radar (altitud_cm, snr, checksum_recibido, checksum_valido,
 * trama_cruda), y anade rcvTow/week al final.
 * Mismo criterio de flush diferido que DataLogger_Guardar() -- ver
 * @details de esa funcion para el razonamiento completo, que aplica
 * igual aqui al compartir el mismo fichero.
 */
bool DataLogger_GuardarSyncGps(const GpsSyncPoint &punto)
{
  if (!s_archivo)
    return false;

  s_archivo.print("GPS,");
  s_archivo.print(punto.timestamp_us);
  s_archivo.print(",,,,,,"); // 5 columnas vacias del radar (altitud_cm..trama_cruda)
  s_archivo.print(punto.rcvTow, 6);
  s_archivo.print(",");
  s_archivo.println(punto.week);

  return true;
}

bool DataLogger_Flush(void)
{
  if (!s_archivo)
    return false;

  // flush() fuerza la escritura a la SD de lo que queda en el buffer
  // de la libreria, sin esperar a que se llene o a que se cierre el
  // fichero. Limita la perdida de datos ante un corte.
  s_archivo.flush();

  if (s_archivo.getWriteError())
  {
    s_archivo.clearWriteError();
    return false; // Fallo real detectado al volcar a la SD
  }
  return true;
}

/* Implementacion de DataLogger_ObtenerNumeroVuelo(). Documentacion en data_logger.h */
int DataLogger_ObtenerNumeroVuelo(void)
{
  return s_numeroVueloActual;
}

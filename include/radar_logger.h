/**
 * @file radar_logger.h
 * @brief Registro de tramas del radar en un fichero CSV sobre la
 *        tarjeta microSD integrada de la Teensy 4.1, sin bloqueos por
 *        escritura fisica en cada trama.
 *
 * @details
 * Este modulo separa deliberadamente dos operaciones que la libreria
 * SD.h combina si se usa de forma ingenua: (a) escribir datos en el
 * bufer de la libreria (rapido, RadarLogger_Guardar()), y (b) forzar
 * la escritura fisica de ese bufer en la memoria flash de la tarjeta
 * (lento, RadarLogger_Flush()). Ver la documentacion de cada funcion
 * en radar_logger.cpp para el razonamiento completo de por que se
 * separan y con que cadencia se invoca el flush.
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef RADAR_LOGGER_H
#define RADAR_LOGGER_H

#include <Arduino.h>
#include "radar_uart.h"

/**
 * @brief Inicializa la tarjeta SD y abre (o crea) el fichero CSV indicado.
 *
 * @details Si el fichero no existe, se crea y se escribe la cabecera
 * de columnas (timestamp_us,altitud_cm,snr). Si ya existe, las tramas
 * nuevas se anaden al final sin sobrescribir su contenido previo.
 *
 * @param nombreArchivo Nombre del fichero CSV a crear o abrir.
 * @return true si la SD y el fichero se abrieron/crearon
 *         correctamente; false si la SD no responde o el fichero no
 *         pudo abrirse.
 * @todo el nombre de fichero se pasa como parametro de esta funcion,
 * pero en main.cpp esta codificado como una cadena literal fija
 * ("vuelo_01.csv") -- no se ha documentado un esquema para nombrar
 * ficheros de forma distinta en vuelos sucesivos, lo que podria
 * sobrescribir datos de un vuelo anterior si no se cambia
 * manualmente ese nombre entre sesiones.
 */
bool RadarLogger_Init(const char *nombreArchivo);

/**
 * @brief Anade una fila al CSV con los datos de una trama, sin forzar
 *        su escritura fisica inmediata en la tarjeta.
 * @param frame Trama a registrar.
 * @return true si la escritura al bufer se realizo correctamente;
 *         false si el fichero no esta disponible (p. ej. la SD se ha
 *         desconectado).
 * @see RadarLogger_Flush() para forzar la escritura fisica periodica.
 */
bool RadarLogger_Guardar(const RadarFrame& frame);

/**
 * @brief Fuerza la escritura fisica inmediata del contenido en bufer a la tarjeta SD.
 * @details Pensada para invocarse periodicamente (no en cada trama)
 * desde loop(), mediante un temporizador no bloqueante -- ver
 * main.cpp y la documentacion de RadarLogger_Guardar() en
 * radar_logger.cpp para el razonamiento completo de esta separacion.
 */
void RadarLogger_Flush(void);

#endif // RADAR_LOGGER_H

/**
 * @file radar_uart.h
 * @brief Interfaz publica del modulo de adquisicion de datos del radar
 * altimetrico US-D1 de Ainstein, a traves de UART, sobre la Teensy 4.1.
 *
 * @details
 * Captura las tramas de 6 bytes del US-D1 y las traduce a estructura
 * "RadarFrame" para su uso por el resto del firmware (buffer circular, SD).
 * La captura ocurre mediante interrupcion hardware sobre el periferico
 * LPUART6, al que corresponde el puerto logico Serial1 de
 * Teensyduino (ver tabla de direcciones base de registro por puerto Serial de
 * Teensy 4.1 https://github.com/MicroControleurMonde/Teensy_4.1/blob/main/Teensy_LPUART.py).
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef RADAR_UART_H
#define RADAR_UART_H

/**
 * @def RADAR_BAUD_RATE
 * @brief Velocidad de transmision (b/s) del radar US-D1 de Ainstein.
 */
#define RADAR_BAUD_RATE 115200

#include <Arduino.h>

/**
 * @struct RadarFrame
 * @brief Trama del radar US-D1, decodificada y con su validez marcada.
 *
 * @details
 * `valida` refleja el resultado del checksum, pero la trama se conserva y se
 * registra igualmente aunque no sea valida `trama_cruda` guarda los 6 bytes
 * exactos recibidos, sin interpretar, como respaldo de auditoria independiente
 * de como el parser los haya decodificado.
 */
typedef struct
{
  uint32_t timestamp_us;  /**< Instante de llegada del ultimo byte (checksum), capturado en isrRadar(). */
  uint16_t altitud_cm;    /**< Altitud en centimetros. */
  uint8_t snr;            /**< Relacion senal-ruido del radar. */
  bool valida;            /**< true si el checksum recibido coincide con el calculado. */
  uint8_t trama_cruda[6]; /**< Bytes capturados: [cabecera, version, altL, altH, snr, checksum]. */
} RadarFrame;

/**
 * @brief Inicializa Serial1 (LPUART6) y activa la captura por interrupcion.
 *
 * @details
 * 1. `Serial1.begin(RADAR_BAUD_RATE)`: reloj, baudrate e IOMUX de
 *    TX1/RX1, delegados en Teensyduino.
 * 2. Sustituye el handler de interrupcion por defecto por `isrRadar()`
 *    (radar_uart.cpp), vía `attachInterruptVector()`.
 *
 * @warning Llamar una unica vez, desde setup(), antes de loop().
 * @note Tras esta llamada, Serial1.available()/read() dejan de
 * reflejar los bytes recibidos (toda la recepcion pasa a `isrRadar()`).
 *
 * @see isrRadar() (radar_uart.cpp)
 */
void RadarUART_Init();

#endif // RADAR_UART_H

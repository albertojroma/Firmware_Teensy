/**
 * @file radar_uart.h
 * @brief Parser de tramas UART del radar altimétrico Ainstein US-D1.
 *
 * Implementa la máquina de estados encargada de reconstruir y validar
 * las tramas de 6 bytes definidas en la sección 5.1 del manual técnico
 * del US-D1 (UART Data Protocol Specifications, D00.03.00).
 *
 * @author Alberto
 * @date 2026
 */

#ifndef RADAR_UART_H
#define RADAR_UART_H

#include <Arduino.h>

/**
 * @struct RadarFrame
 * @brief Representa una trama decodificada del radar US-D1.
 *
 * Contiene la altitud y el SNR extraídos de una trama válida, junto con
 * una marca de tiempo local de la Teensy y un indicador de validez del
 * checksum.
 */
struct RadarFrame {
  uint16_t altitud_cm;   /**< Altitud combinada en centímetros (LSB + MSB). */
  uint8_t  snr;          /**< Relación señal-ruido (SNR) en dB, según datasheet. */
  bool     valida;       /**< true si el checksum de la trama es correcto. */
  uint32_t timestamp_us; /**< Marca de tiempo local (micros()) de recepción. */
};

/**
 * @class RadarUART
 * @brief Gestiona la recepción y decodificación de tramas del radar US-D1 por UART.
 *
 * Implementa una máquina de estados a nivel de byte que reconstruye tramas
 * de 6 bytes (cabecera, versión, altitud LSB/MSB, SNR, checksum) según el
 * protocolo UART del US-D1. La verificación de checksum sigue la fórmula:
 *
 * checksum_calculado = (VersionID + Altitud_MSB + Altitud_LSB + SNR) & 0xFF
 *
 * Nota: la especificación original del fabricante indica de forma ambigua
 * "checksum = 1 → válido / checksum = 0 → inválido", lo cual es incoherente
 * con el rango de valores posibles de la operación AND. Se ha adoptado la
 * interpretación estándar de validación por coincidencia exacta entre el
 * checksum calculado y el byte recibido.
 */
class RadarUART {
public:
  /**
   * @brief Inicializa el puerto UART hardware asignado al radar.
   * @param serialPort Referencia al puerto Serial hardware (p. ej. Serial1).
   * @param baud Velocidad en baudios (por defecto 115200, según datasheet US-D1).
   */
  void begin(HardwareSerial &serialPort, uint32_t baud = 115200);

  /**
   * @brief Procesa los bytes disponibles en el buffer UART.
   *
   * Debe llamarse de forma periódica (idealmente en cada iteración del loop
   * principal) para no perder bytes por desbordamiento del buffer hardware.
   *
   * @return true si se ha completado y validado una nueva trama en esta llamada.
   */
  bool update();

  /**
   * @brief Devuelve la última trama procesada (válida o no).
   * @return Estructura RadarFrame con los últimos datos decodificados.
   */
  RadarFrame getLastFrame();

private:
  /**
   * @enum ParserState
   * @brief Estados internos de la máquina de estados del parser de bytes.
   */
  enum ParserState {
    WAIT_HEADER,   /**< Buscando el byte de cabecera (0xFE). */
    WAIT_VERSION,  /**< Esperando el byte de versión (0x02). */
    READ_ALT_L,    /**< Leyendo el byte menos significativo de altitud. */
    READ_ALT_H,    /**< Leyendo el byte más significativo de altitud. */
    READ_SNR,      /**< Leyendo el byte de SNR. */
    READ_CHECKSUM  /**< Leyendo y verificando el byte de checksum. */
  };

  HardwareSerial* _serial = nullptr; /**< Puntero al puerto UART hardware en uso. */
  ParserState _state = WAIT_HEADER;  /**< Estado actual del parser. */

  uint8_t _versionID = 0; /**< Byte de versión recibido en la trama actual. */
  uint8_t _altL = 0;      /**< Byte LSB de altitud recibido. */
  uint8_t _altH = 0;      /**< Byte MSB de altitud recibido. */
  uint8_t _snr  = 0;      /**< Byte de SNR recibido. */

  RadarFrame _lastFrame = {0, 0, false, 0}; /**< Última trama procesada. */

  static const uint8_t HEADER_BYTE  = 0xFE; /**< Valor esperado de cabecera. */
  static const uint8_t VERSION_BYTE = 0x02; /**< Valor esperado de versión. */
};

#endif // RADAR_UART_H
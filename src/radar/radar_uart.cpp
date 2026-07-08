/**
 * @file radar_uart.cpp
 * @brief Implementación del parser UART del radar US-D1.
 * @see radar_uart.h
 */

#include "radar_uart.h"

/**
 * @copydoc RadarUART::begin
 */
void RadarUART::begin(HardwareSerial &serialPort, uint32_t baud) {
  _serial = &serialPort;
  _serial->begin(baud);
  _state = WAIT_HEADER;
}

/**
 * @copydoc RadarUART::update
 *
 * Procesa byte a byte el contenido disponible en el buffer UART,
 * avanzando la máquina de estados interna. Al completar una trama,
 * calcula el checksum y lo compara con el byte recibido; si coinciden,
 * actualiza _lastFrame con los nuevos datos y marca la trama como válida.
 *
 * Si el checksum no coincide, la trama se descarta y el parser vuelve
 * a WAIT_HEADER, permitiendo resincronización automática sin intervención
 * externa (política de recuperación ante corrupción transitoria).
 */
bool RadarUART::update() {
  if (_serial == nullptr) return false;

  bool nuevaTramaValida = false;

  while (_serial->available() > 0) {
    uint8_t byte = _serial->read();

    switch (_state) {
      case WAIT_HEADER:
        if (byte == HEADER_BYTE) {
          _state = WAIT_VERSION;
        }
        break;

      case WAIT_VERSION:
        if (byte == VERSION_BYTE) {
          _versionID = byte;
          _state = READ_ALT_L;
        } else {
          // Byte de versión inesperado: posible falso positivo de cabecera.
          _state = WAIT_HEADER;
        }
        break;

      case READ_ALT_L:
        _altL = byte;
        _state = READ_ALT_H;
        break;

      case READ_ALT_H:
        _altH = byte;
        _state = READ_SNR;
        break;

      case READ_SNR:
        _snr = byte;
        _state = READ_CHECKSUM;
        break;

      case READ_CHECKSUM: {
        uint8_t checksumCalculado = (_versionID + _altH + _altL + _snr) & 0xFF;

        if (checksumCalculado == byte) {
          _lastFrame.altitud_cm   = (uint16_t)(_altH << 8) | _altL;
          _lastFrame.snr          = _snr;
          _lastFrame.valida       = true;
          _lastFrame.timestamp_us = micros();
          nuevaTramaValida = true;
        } else {
          // Checksum no coincide: trama corrupta, se descarta.
          _lastFrame.valida = false;
        }

        _state = WAIT_HEADER;
        break;
      }
    }
  }

  return nuevaTramaValida;
}

/**
 * @copydoc RadarUART::getLastFrame
 */
RadarFrame RadarUART::getLastFrame() {
  return _lastFrame;
}
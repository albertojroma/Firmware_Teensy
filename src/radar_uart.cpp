/**
 * @file radar_uart.cpp
 * @brief Captura por interrupcion los bytes de las tramas del radar US-D1.
 * @see radar_uart.h para la interfaz publica y el mapeo Serial1 -> LPUART6.
 */
#include "radar_uart.h"
#include "radar_buffer.h" // Para RadarBuffer_Push()
#include <imxrt.h>

/**
 * @def HEADER_BYTE
 * @brief Byte de cabecera esperado de la trama del radar (datasheet US-D1).
 * @note Definido aqui y no en `radar_uart.h` por encapsulacion.
 */
#define HEADER_BYTE ((uint8_t)0xFE)

/**
 * @def VERSION_BYTE
 * @brief Byte de version de protocolo esperado (datasheet US-D1).
 * @note Definido aqui y no en `radar_uart.h` por encapsulacion.
 */
#define VERSION_BYTE ((uint8_t)0x02)

/**
 * @enum ParserState
 * @brief Estados del parser de la trama de 6 bytes:
 *        [Cabecera][Version][Alt_LSB][Alt_MSB][SNR][Checksum].
 * @note Definido aqui y no en `radar_uart.h` por encapsulacion.
 */
typedef enum
{
  WAIT_HEADER,  /**< Buscando 0xFE. Cualquier otro byte se ignora (resincronizacion ante ruido). */
  WAIT_VERSION, /**< Esperando 0x02. Si no coincide, vuelve a WAIT_HEADER. */
  READ_ALT_L,   /**< Byte LSB de altitud. */
  READ_ALT_H,   /**< Byte MSB de altitud. */
  READ_SNR,     /**< Byte de SNR. */
  READ_CHECKSUM /**< Byte de checksum: cierra la trama y calcula su validez. */
} ParserState;

/** @brief Estado del parser, persistente entre invocaciones de la ISR. */
static ParserState s_estado = WAIT_HEADER;
static uint8_t s_altL = 0; /**< Byte LSB de altitud de la trama en curso. */
static uint8_t s_altH = 0; /**< Byte MSB de altitud de la trama en curso. */
static uint8_t s_snr = 0;  /**< Byte de SNR de la trama en curso. */

/**
 * @brief Handler de interrupcion de Serial1 (LPUART6).
 *
 * @details
 * 1. Reconoce (limpia) flags de error de STAT antes de leer datos:
 *    `IMXRT_LPUART6.STAT = IMXRT_LPUART6.STAT`. Las banderas OR
 *    (overrun), NF (ruido), FE (trama), PF (paridad) e IDLE siguen el
 *    convenio "escribir 1 para borrar"; si alguna queda activa sin
 *    reconocer, la interrupcion se dispara sin parar y bloquea todo
 *    el firmware -- fallo real, ya depurado en una version anterior
 *    de este modulo, y documentado con el mismo sintoma (banderas
 *    OR/FE sin reconocer -> interrupcion indefinida) en:
 *    https://github.com/phoenix-rtos/phoenix-rtos-devices/issues/55
 * 2. RDRF (Receive Data Register Full) activo implica la captura de
 *    `micros()` y lee `DATA` (la lectura desactiva RDRF).
 * 3. Avanza en la maquina de estados. Notar que el parseo/analisis de validez
 *    de la trama se realiza dentro del handler. Como es una operacion sencilla
 *    en relacion al envio de tramas del radar (100 Hz) y la capacidad del
 *    micro usado no hay problema.

 */
static void isrRadar(void)
{
  IMXRT_LPUART6.STAT = IMXRT_LPUART6.STAT;

  while (IMXRT_LPUART6.STAT & LPUART_STAT_RDRF)
  {
    // instante del byte actual
    uint32_t last_timestamp_us = micros();
    // se almacena el byte capturado temporalmente
    uint8_t byte = IMXRT_LPUART6.DATA;

    switch (s_estado)
    {
    case WAIT_HEADER:
      if (byte == HEADER_BYTE)
        s_estado = WAIT_VERSION;
      break;

    case WAIT_VERSION:
      s_estado = (byte == VERSION_BYTE) ? READ_ALT_L : WAIT_HEADER;
      break;

    case READ_ALT_L:
      s_altL = byte;
      s_estado = READ_ALT_H;
      break;

    case READ_ALT_H:
      s_altH = byte;
      s_estado = READ_SNR;
      break;

    case READ_SNR:
      s_snr = byte;
      s_estado = READ_CHECKSUM;
      break;

    case READ_CHECKSUM:
    {
      // checksum = (VersionID + Altitud_MSB + Altitud_LSB + SNR) & 0xFF.
      uint8_t checksum = (uint8_t)((VERSION_BYTE + s_altH + s_altL + s_snr) & 0xFF);

      RadarFrame nueva_trama;
      nueva_trama.timestamp_us = last_timestamp_us;
      nueva_trama.altitud_cm = (uint16_t)((s_altH << 8) | s_altL);
      nueva_trama.snr = s_snr;
      nueva_trama.valida = (byte == checksum);
      nueva_trama.trama_cruda[0] = HEADER_BYTE;
      nueva_trama.trama_cruda[1] = VERSION_BYTE;
      nueva_trama.trama_cruda[2] = s_altL;
      nueva_trama.trama_cruda[3] = s_altH;
      nueva_trama.trama_cruda[4] = s_snr;
      nueva_trama.trama_cruda[5] = byte;

      // Envio de datos al buffer
      RadarBuffer_Push(nueva_trama);

      s_estado = WAIT_HEADER;
      break;
    }
    }
  }
}

/**
 * @note RDRF y `LPUART_CTRL_RIE` (usado en RadarUART_Init()) son nombres de
 * campo definidos en `imxrt.h` (cabecera del propio framework Teensyduino); no
 * se ha localizado un enlace externo con numero de pagina exacto del manual de
 * referencia de NXP que los defina, mas alla de esa cabecera.
 */
void RadarUART_Init()
{
  Serial1.begin(RADAR_BAUD_RATE);

  attachInterruptVector(IRQ_LPUART6, isrRadar);
  NVIC_ENABLE_IRQ(IRQ_LPUART6);
  IMXRT_LPUART6.CTRL |= LPUART_CTRL_RIE; // Receiver Interrupt Enable

  s_estado = WAIT_HEADER;
}

/**
 * @file radar_uart.cpp
 * @brief Máquina de estados del protocolo UART del US-D1, capturada
 *        mediante interrupción real sobre LPUART6 (Serial1).
 *
 * Sustituye a serialEvent1(): esa función NO es una interrupción real
 * en Teensyduino (se invoca al terminar cada loop(), o en yield()),
 * por lo que el timestamp que se tomara ahí quedaría sujeto al tiempo
 * variable que tarde cada vuelta del bucle principal — justo el
 * jitter que este proyecto necesita evitar. Aquí, micros() se captura
 * dentro de la propia interrupción hardware, en el instante exacto de
 * llegada del byte.
 */
#include "radar_uart.h"
#include "radar_buffer.h" // Para RadarBuffer_Push
#include <imxrt.h>

#define HEADER_BYTE  ((uint8_t)0xFE)
#define VERSION_BYTE ((uint8_t)0x02)

typedef enum {
  WAIT_HEADER,
  WAIT_VERSION,
  READ_ALT_L,
  READ_ALT_H,
  READ_SNR,
  READ_CHECKSUM
} ParserState;

static ParserState s_estado = WAIT_HEADER;
static uint8_t s_altL = 0;
static uint8_t s_altH = 0;
static uint8_t s_snr  = 0;

/**
 * @brief Interrupción real de recepción de Serial1 (LPUART6).
 *
 * Avanza la máquina de estados del protocolo del US-D1 byte a byte.
 * Al completar una trama con checksum válido, la empuja al buffer
 * circular (RadarBuffer_Push) para su consumo posterior en loop().
 */
static void isrRadar(void) {
  /* Reconoce banderas de error (overrun, ruido, trama, paridad,
   * inactividad). Es imprescindible: si una de estas banderas quedara
   * sin reconocer, la interrupción se dispararía sin parar y
   * bloquearía el resto del firmware (bug ya depurado en una versión
   * anterior de este mismo módulo). */
  IMXRT_LPUART6.STAT = IMXRT_LPUART6.STAT;

  while (IMXRT_LPUART6.STAT & LPUART_STAT_RDRF) {
    uint32_t t = micros();
    uint8_t byte = IMXRT_LPUART6.DATA;

    switch (s_estado) {
      case WAIT_HEADER:
        if (byte == HEADER_BYTE) s_estado = WAIT_VERSION;
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

      case READ_CHECKSUM: {
        uint8_t checksum = (uint8_t)((VERSION_BYTE + s_altH + s_altL + s_snr) & 0xFF);

        if (byte == checksum) {
          RadarFrame nueva_trama;
          nueva_trama.timestamp_us = t; /* capturado al entrar en la ISR, no al salir */
          nueva_trama.altitud_cm   = (uint16_t)((s_altH << 8) | s_altL);
          nueva_trama.snr          = s_snr;
          nueva_trama.valida       = true;

          RadarBuffer_Push(nueva_trama);
        }
        /* Si el checksum no coincide, la trama se descarta en silencio
         * y el parser se resincroniza solo con el siguiente 0xFE. */
        s_estado = WAIT_HEADER;
        break;
      }
    }
  }
}

/**
 * @copydoc RadarUART_Init
 */
void RadarUART_Init(uint32_t baud) {
  Serial1.begin(baud); /* Teensyduino: reloj, baudrate, IOMUX de TX1/RX1 */

  /* Sustituye el manejador de interrupción por defecto de Teensyduino
   * para LPUART6 por el nuestro. No afecta a otros puertos serie. */
  attachInterruptVector(IRQ_LPUART6, isrRadar);
  NVIC_ENABLE_IRQ(IRQ_LPUART6);
  IMXRT_LPUART6.CTRL |= LPUART_CTRL_RIE; /* Receiver Interrupt Enable */

  s_estado = WAIT_HEADER;
}
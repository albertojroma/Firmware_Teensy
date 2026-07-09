/**
 * @file radar_uart.cpp
 * @brief Máquina de estados del protocolo UART del US-D1.
 */
#include "radar_uart.h"
#include "radar_buffer.h" // Necesario para hacer el Push

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

void RadarUART_Init(uint32_t baud) {
  Serial1.begin(baud);
  s_estado = WAIT_HEADER;
}

void RadarUART_Update(void) {
  while (Serial1.available() > 0) {
    uint8_t byte = Serial1.read();

    switch (s_estado) {
      case WAIT_HEADER:
        if (byte == HEADER_BYTE) s_estado = WAIT_VERSION;
        break;
      case WAIT_VERSION:
        if (byte == VERSION_BYTE) s_estado = READ_ALT_L;
        else s_estado = WAIT_HEADER;
        break;
      case READ_ALT_L:
        s_altL = byte; s_estado = READ_ALT_H;
        break;
      case READ_ALT_H:
        s_altH = byte; s_estado = READ_SNR;
        break;
      case READ_SNR:
        s_snr = byte; s_estado = READ_CHECKSUM;
        break;
      case READ_CHECKSUM:
        uint8_t checksum = (VERSION_BYTE + s_altH + s_altL + s_snr) & 0xFF;
        if (byte == checksum) {
          RadarFrame nueva_trama;
          nueva_trama.altitud_cm   = (uint16_t)((s_altH << 8) | s_altL);
          nueva_trama.snr          = s_snr;
          nueva_trama.valida       = true;
          nueva_trama.timestamp_us = micros();
          
          // ¡Introducimos la trama en el Buffer de forma segura!
          RadarBuffer_Push(nueva_trama);
        }
        s_estado = WAIT_HEADER;
        break;
    }
  }
}

/* * Usamos la función nativa serialEvent1 de Teensy.
 * Se ejecuta automáticamente al final de cada iteración del loop() 
 * si hay datos en el hardware de la Serial1.
 */
void serialEvent1() {
  RadarUART_Update();
}
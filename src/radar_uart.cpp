/**
 * @file radar_uart.cpp
 * @brief Implementacion de la captura por interrupcion hardware y del
 *        parser de tramas UART del radar Ainstein US-D1.
 * @see radar_uart.h para la documentacion de la interfaz publica y la
 *      justificacion general de por que se usa una interrupcion real
 *      en lugar de serialEvent1().
 */
#include "radar_uart.h"
#include "radar_buffer.h" // Para RadarBuffer_Push()
#include <imxrt.h>

/**
 * @def HEADER_BYTE
 * @brief Byte de cabecera que marca el inicio de una trama del US-D1.
 * @note Se define aqui y no en radar_uart.h por seguir el principio de 
 * encasulacion. El resto de archivos no necesitan esta macro
 */
#define HEADER_BYTE  ((uint8_t)0xFE)

/**
 * @def VERSION_BYTE
 * @brief Byte de version de protocolo esperado, segun el datasheet del US-D1.
 * @note Se define aqui y no en radar_uart.h por seguir el principio de 
 * encasulacion. El resto de archivos no necesitan esta macro
 * @details Se usa tanto para validar la trama entrante (segundo byte,
 * justo despues de la cabecera) como uno de los operandos del calculo
 * del checksum (ver el caso READ_CHECKSUM dentro de isrRadar()).
 */
#define VERSION_BYTE ((uint8_t)0x02)

/**
 * @enum ParserState
 * @brief Estados de la maquina de estados que reconstruye la trama de 6 bytes 
 * del US-D1 byte a byte.
 * @note Se define aqui y no en radar_uart.h por seguir el principio de 
 * encasulacion. El resto de archivos no necesitan este "enum".
 * @details
 * El protocolo del US-D1 en su version UART define una trama fija de 6 bytes:
 * [Cabecera][Version][Altitud_LSB][Altitud_MSB][SNR][Checksum].
 * Cada estado de este enumerado corresponde a "que byte se espera a
 * continuacion", y la transicion entre estados ocurre exactamente una
 * vez por byte recibido, dentro de isrRadar().
 */
typedef enum {
  WAIT_HEADER,   /**< Esperando el byte de cabecera (0xFE). Cualquier otro byte se ignora y se permanece en este estado. Este es un mecanismo de resincronizacion ante ruido o tramas corruptas. */
  WAIT_VERSION,  /**< Cabecera recibida; esperando el byte de version (0x02). Si no coincide, se vuelve a WAIT_HEADER (posible falso positivo de cabecera). */
  READ_ALT_L,    /**< Leyendo el byte menos significativo (LSB) de la altitud. */
  READ_ALT_H,    /**< Leyendo el byte mas significativo (MSB) de la altitud. */
  READ_SNR,      /**< Leyendo el byte de relacion senal-ruido (SNR). */
  READ_CHECKSUM  /**< Leyendo el byte de checksum y VERIFICANDOLO contra el valor calculado con los cuatro bytes anteriores. */
} ParserState;

/** @brief Estado actual del parser. Se conserva entre invocaciones sucesivas de la ISR, ya que una trama completa puede llegar repartida en varias rafagas de interrupcion. */
static ParserState s_estado = WAIT_HEADER;

/** @brief Byte LSB de altitud de la trama en curso. Guardado hasta completar el checksum. */
static uint8_t s_altL = 0;
/** @brief Byte MSB de altitud de la trama en curso. Guardado hasta completar el checksum. */
static uint8_t s_altH = 0;
/** @brief Byte de SNR de la trama en curso, retenido hasta completar el checksum. */
static uint8_t s_snr  = 0;

/**
 * @brief Manejador de interrupcion de recepcion de Serial1 (periferico LPUART6).
 * //! esto de DETAILS huele a IA que tira pa'tras
 * todo: mirar lo del puerto y justificarlo
 * @details
 * **Por que LPUART6:** en la Teensy 4.1, el puerto logico Serial1 de
 * Teensyduino no corresponde al periferico LPUART1, como cabria
 * esperar por analogia de nombres -- el mapeo real es
 * Serial1 -> LPUART6 (y de forma similarmente no intuitiva,
 * Serial2 -> LPUART3, Serial3 -> LPUART2, Serial4 -> LPUART4,
 * Serial5 -> LPUART8, etc.). Este proyecto verifico explicitamente ese
 * mapeo contra documentacion externa antes de escribir esta funcion,
 * precisamente porque usar el periferico equivocado (p. ej.
 * IMXRT_LPUART1) habria hecho que el codigo no reflejara el
 * comportamiento real de Serial1, con el consiguiente fallo de
 * recepcion en tiempo de ejecucion.
 *
 * **Que hace, paso a paso:**
 * todo: Averiguar de donde sale lo de IMXRT_LPUART6.STAT
 *  1. Reconoce (limpia) cualquier flag de error pendiente en el
 *     registro STAT del periferico, mediante la instruccion
 *     IMXRT_LPUART6.STAT = IMXRT_LPUART6.STAT.
 * todo: Averiguar de donde sale lo de RDRF
 *  2. Mientras la flag RDRF (Receive Data Register Full) este
 *     activa, hay al menos un byte disponible en el FIFO hardware:
 *      - Se captura micros() inmediatamente, antes de cualquier otro
 *        trabajo.
 *      - Se lee el byte del registro DATA (esta lectura, ademas,
 *        desactiva automaticamente la flag RDRF para ese byte).
 *      - Se avanza un paso la maquina de estados ParserState con ese
 *        byte.
 *      - Si el byte completa una trama con checksum valido, se
 *        construye una RadarFrame y se empuja al buffer circular
 *        mediante RadarBuffer_Push().
 *
 * **Por que se limpian las flags de error antes de leer datos**
 * (IMXRT_LPUART6.STAT = IMXRT_LPUART6.STAT): el registro STAT del
 * periferico LPUART no solo senaliza la llegada de datos (RDRF),
 * tambien senaliza condiciones de error -- overrun (OR), ruido (NF),
 * error de trama (FE), de paridad (PF) y linea inactiva (IDLE) --
 * cuyas flags siguen el convenio "escribir 1 para borrar"
 * (write-1-to-clear). Si cualquiera de esas flags quedara activa
 * sin ser reconocida, la interrupcion de LPUART6 volveria a
 * dispararse inmediatamente despues de retornar de esta misma
 * funcion, de forma indefinida, acaparando la CPU y bloqueando el
 * resto del firmware (incluido el propio arranque de setup()). Este
 * no es un riesgo teorico: es un fallo real que se produjo y depuro
 * en una version anterior de este modulo, manifestado como una placa
 * que compilaba y subia correctamente pero no llegaba a ejecutar ni
 * siquiera el primer Serial.println() de setup(). La expresion
 * IMXRT_LPUART6.STAT = IMXRT_LPUART6.STAT relee el registro y
 * reescribe exactamente el mismo valor leido: por el convenio
 * write-1-to-clear, esto reconoce (limpia) cualquier flag que
 * estuviera activa en el momento de la lectura, sin alterar ningun
 * bit de configuracion que pudiera compartir el mismo registro (por
 * ejemplo, LBKDE), ya que esos bits de configuracion se reescriben
 * con su propio valor actual, sin cambio neto.
 *
 * **Por que esta logica vive dentro de la propia interrupcion, y no
 * en un buffer intermedio de bytes crudos mas simple:** el trabajo
 * por byte es minimo (unas pocas comparaciones de la maquina de
 * estados), muy lejos de poder saturar la CPU incluso a la tasa de
 * 100 Hz / 6 bytes por trama del US-D1. Mantener el parseo completo
 * dentro de la interrupcion, en lugar de solo capturar el byte crudo
 * y diferir el parseo a loop(), simplifica el flujo de codigo sin
 * coste practico de rendimiento.
 */
static void isrRadar(void) {
  IMXRT_LPUART6.STAT = IMXRT_LPUART6.STAT;

  while (IMXRT_LPUART6.STAT & LPUART_STAT_RDRF) {
    //last_timestamp_us instante del ULTIMO byte recibido
    uint32_t last_timestamp_us = micros();
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
        /*
         * Formula del checksum: (VersionID + Altitud_MSB + Altitud_LSB + SNR) & 0xFF
         * Si el calculo del checksum en esta FSM coincide con el ultimo byte 
         * recibido, que corresponderia con el checksum enviado por el radar, la
         * trama es valida.
         */
        uint8_t checksum = (uint8_t)((VERSION_BYTE + s_altH + s_altL + s_snr) & 0xFF);

        if (byte == checksum) {
          RadarFrame nueva_trama;
          nueva_trama.timestamp_us = last_timestamp_us;
          nueva_trama.altitud_cm   = (uint16_t)((s_altH << 8) | s_altL);
          nueva_trama.snr          = s_snr;

          RadarBuffer_Push(nueva_trama);
        }

        /* Si el checksum no coincide, la trama se descarta en silencio
         * (no se llama a RadarBuffer_Push) y el parser se resincroniza
         * solo con el siguiente 0xFE que llegue -- politica de
         * recuperacion ante corrupcion transitoria sin intervencion
         * externa. */
        s_estado = WAIT_HEADER;
        break;
      }
    }
  }
}

/**
 * @copydoc RadarUART_Init
 */
void RadarUART_Init() {
  // Teensyduino: reloj, baudrate, IOMUX de TX1/RX1
  Serial1.begin(RADAR_BAUD_RATE); 
  // Se lanza el handler isrRadar()
  attachInterruptVector(IRQ_LPUART6, isrRadar);
  NVIC_ENABLE_IRQ(IRQ_LPUART6);
  //todo: Averiguar de donde saca esto
  // Receiver Interrupt Enable: habilita que RDRF dispare la interrupcion
  IMXRT_LPUART6.CTRL |= LPUART_CTRL_RIE; 

  s_estado = WAIT_HEADER;
}

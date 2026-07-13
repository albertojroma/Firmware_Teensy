/**
 * @file radar_uart.h
 * @brief Interfaz publica del modulo de adquisicion UART del radar
 *        altimetrico Ainstein US-D1 sobre la Teensy 4.1.
 *
 * @details
 * Este modulo captura las tramas de 6 bytes que emite el radar US-D1
 * por su interfaz UART y las traduce a una estructura RadarFrame lista
 * para su consumo por el resto del firmware (buffer circular, registro
 * en SD, etc.). 
 * 
 * La implementacion (ver radar_uart.cpp) captura los bytes mediante una 
 * interrupcion hardware real sobre el periferico LPUART6, al que corresponde 
 * el puerto logico Serial1 de Teensyduino
 * todo: el siguiente parrafo habria que referenciarlo
 * -- un mapeo no intuitivo (no sigue el orden numerico de los puertos
 * logicos: Serial2 -> LPUART3, Serial3 -> LPUART2, Serial4 -> LPUART4,
 * etc.), verificado explicitamente contra documentacion externa del
 * mapeo de perifericos de Teensy 4.1 antes de su uso en este proyecto.
 *
 * @note serialEvent1() no funciona como una interrupcion. Se ejecuta siempre 
 * al final de cada iteracion de loop()
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef RADAR_UART_H
#define RADAR_UART_H

/**
 * @def RADAR_BAUD_RATE
 * @brief Velocidad de transmision (b/s) a la que funciona el radar US-D1 de 
 * Ainstein
 */
#define RADAR_BAUD_RATE 115200

#include <Arduino.h>

/**
 * @struct RadarFrame
 * @brief Representa una trama decodificada y validada del radar US-D1 de 
 * Ainstein en su versión UART.
 *
 * @details
 * Contiene el instante de captura, parametros de medida del radar (altitud 
 * (cm) y SNR (dB)) y un indicador de validez de la trama. Este indicador es 
 * heredado del checksum especificado en el datasheet del radar. * 
 */
typedef struct {
  uint32_t timestamp_us; /**< Instante de llegada del ultimo byte (el checksum) 
  de la trama (micros()), capturado dentro de la interrupcion hardware (ver isrRadar() en radar_uart.cpp), no en el momento de su posterior procesado en loop(). */
  uint16_t altitud_cm;   /**< Altitud medida, en centimetros, reconstruida a partir de los bytes MSB/LSB de altura de la trama. */
  uint8_t  snr;          /**< Relacion senal-ruido (SNR) reportada por el radar, en las unidades definidas por el datasheet del US-D1 (no se realiza ninguna conversion adicional en este modulo). */
} RadarFrame;

/**
 * @brief Inicializa el puerto Serial1 (periferico LPUART6) y activa la captura de tramas del radar 
 * mediante interrupcion hardware.
 *
 * @details
 * Realiza dos tareas en secuencia:
 *  todo: convendria confirmar este punto 1
 *  1. Llama a Serial1.begin(RADAR_BAUD_RATE), que delega en Teensyduino la 
 *     configuracion de reloj, divisor de baudrate y multiplexado de pines 
 *     (IOMUX) de TX1/RX1 -- trabajo de bajo nivel que no aporta valor de 
 *     ingenieria especifico a este proyecto reimplementar.
 *  2. Sustituye el handler de interrupcion por defecto de Teensyduino 
 *     por uno propio (isrRadar(), definida en radar_uart.cpp), mediante 
 *     attachInterruptVector().
 *
 * @warning Esta función solo debe llamarse una sola vez, tipicamente desde 
 *          setup(), antes de que el firmware entre en loop().
 * 
 * @note Sustituir el handler por defecto de Teensyduino no afecta a otros    
 * puertos serie (Serial, Serial2...).
 *
 * @see isrRadar() (radar_uart.cpp) para el detalle de la captura.
 */
void RadarUART_Init();

#endif // RADAR_UART_H

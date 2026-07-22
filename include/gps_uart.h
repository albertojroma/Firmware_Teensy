/**
 * @file gps_uart.h
 * @brief Interfaz publica del modulo de adquisicion GPS-PPK (u-blox
 * H-RTK ZED-F9P Ultralight) por UART, sobre la Teensy 4.1.
 *
 * @details
 * Captura, configura y reensambla las tramas UBX del receptor GPS,
 * dejando los bytes crudos y validados listos para su registro en SD
 * (ver gps_logger.h). La captura ocurre por interrupcion hardware sobre
 * el periferico LPUART2, al que corresponde el puerto logico Serial3
 * de Teensyduino (mapeo verificado, mismo criterio que ya se aplico
 * para el radar en Serial1/LPUART6):
 * https://github.com/MicroControleurMonde/Teensy_4.1/blob/main/Teensy_LPUART.py
 *
 * @note A diferencia de radar_uart.cpp, aqui la ISR NO reensambla la
 * trama completa: solo captura byte + timestamp en un buffer circular
 * privado del propio gps_uart.cpp. El reensamblado (busqueda de sync,
 * lectura de longitud, verificacion de checksum) ocurre en
 * GpsUART_Process(), invocada desde loop(). Motivo: las tramas UBX del
 * GPS son de longitud VARIABLE (una UBX-RXM-RAWX puede superar 2900
 * bytes con muchos satelites -- calculo 8+16+32*N ya usado en este
 * proyecto), y reensamblar algo de ese tamano dentro de una interrupcion
 * (tiempo de ejecucion no acotado) no es deseable. El radar no tiene
 * este problema porque su trama es fija, de 6 bytes.
 *
 * @note Este modulo NO implementa UBX-NAV-PVT ni ningun concepto de
 * "fix" de posicion: se decidio explicitamente en este proyecto que el
 * post-proceso PPK no depende de fixType, y que la sincronizacion
 * temporal usa directamente rcvTow/week de UBX-RXM-RAWX, sin ninguna
 * condicion de espera antes de empezar a registrar.
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef GPS_UART_H
#define GPS_UART_H

#include <Arduino.h>

/**
 * @struct GpsSyncPoint
 * @brief Par (timestamp del MCU, tiempo GPS) extraido de una trama
 * UBX-RXM-RAWX valida, para la regresion de sincronizacion posterior.
 *
 * @details `timestamp_us` es el instante de llegada del ULTIMO byte de
 * la trama (el segundo byte de checksum), no del primero -- mismo
 * criterio ya usado en radar_uart.cpp (timestamp en la finalizacion de
 * la trama, no en su inicio).
 */
typedef struct
{
  uint32_t timestamp_us; /**< Instante de llegada del ultimo byte de la trama RAWX (micros()). */
  double rcvTow;         /**< Campo rcvTow del payload de RAWX: tiempo de recepcion GPS, en segundos de semana. */
  uint16_t week;         /**< Campo week del payload de RAWX: semana GPS. */
} GpsSyncPoint;

/**
 * @brief Inicializa Serial3 (LPUART2) y activa la captura por interrupcion.
 *
 * @details
 * Arranca al baudrate de fabrica del ZED-F9P (38400, ver ZED-F9P
 * Integration Manual UBX-18010802 R16, apartado 3.1.3, p. 13) y entra
 * en el estado inicial de la maquina de estados de configuracion (ver
 * GpsUART_Process()). La conmutacion al baudrate objetivo (230400)
 * ocurre automaticamente, dentro de la propia maquina de estados,
 * despues de confirmar la configuracion con el receptor.
 *
 * @warning Llamar una unica vez, desde setup(), antes de loop().
 * @see GpsUART_Process()
 */
void GpsUART_Init(void);

/**
 * @brief Avanza la maquina de estados de configuracion/registro del GPS
 * y reensambla las tramas UBX recibidas.
 *
 * @details Debe invocarse en cada iteracion de loop() (a diferencia del
 * radar, que no necesita ninguna llamada explicita porque todo ocurre
 * en su ISR). Consume el buffer circular de bytes que llena la
 * interrupcion, reconstruye tramas UBX completas, verifica su checksum,
 * y:
 * - Durante la configuracion inicial: procesa las respuestas
 *   UBX-ACK-ACK/NAK y UBX-CFG-VALGET necesarias para activar
 *   UBX-RXM-RAWX, UBX-RXM-SFRBX y el baudrate objetivo.
 * - Una vez en registro: entrega cada trama UBX-RXM-RAWX/SFRBX valida a
 *   GpsLogger (ver gps_logger.h), y cada UBX-RXM-RAWX ademas aporta un
 *   GpsSyncPoint.
 *
 * @see GpsUART_HayError()
 */
void GpsUART_Process(void);

/**
 * @brief Indica si la maquina de estados ha entrado en el estado de
 * error terminal (configuracion fallida tras agotar los reintentos).
 * @details Pensada para que main.cpp encienda un LED de error dedicado
 * -- ver la nota de diseno en gps_uart.cpp sobre por que este fallo NO
 * bloquea el resto del firmware (a diferencia de un fallo de SD).
 * @return true si el modulo esta en estado de error.
 */
bool GpsUART_HayError(void);

#endif // GPS_UART_H

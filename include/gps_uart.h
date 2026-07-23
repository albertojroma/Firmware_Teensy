/**
 * @file gps_uart.h
 * @brief Interfaz publica del modulo de adquisicion GPS-PPK (u-blox
 * H-RTK ZED-F9P Ultralight) por UART, sobre la Teensy 4.1.
 *
 * @details
 * Captura y reensambla las tramas UBX del receptor GPS, dejando los
 * bytes crudos y validados listos para su registro en SD (ver
 * gps_logger.h). La captura ocurre por interrupcion hardware sobre el
 * periferico LPUART2, al que corresponde el puerto logico Serial3 de
 * Teensyduino (mapeo verificado, mismo criterio que ya se aplico para
 * el radar en Serial1/LPUART6):
 * https://github.com/MicroControleurMonde/Teensy_4.1/blob/main/Teensy_LPUART.py
 *
 * @note CAMBIO DE DISENO: este modulo NO envia absolutamente nada por
 * Serial3 (ni configuracion, ni consultas) -- solo escucha. Se detecto
 * empiricamente, mediante una biseccion exhaustiva documentada en el
 * proyecto, que la sola presencia de una llamada a Serial3.write()
 * alcanzable en el binario (aunque nunca llegara a ejecutarse)
 * provocaba un cuelgue deterministico de la libreria SD al escribir en
 * la tarjeta. La causa raiz exacta no se llego a identificar pese a un
 * analisis exhaustivo (memoria estatica, direcciones, simbolos,
 * desensamblado, y una sesion de depuracion con GDB que resulto poco
 * fiable en este entorno concreto). La solucion adoptada es evitar POR
 * COMPLETO la escritura por Serial3 desde el firmware: el receptor se
 * configura una UNICA VEZ, de forma permanente, con u-center conectado
 * directamente a un PC (Receiver -> Action -> Save Config, que lo
 * guarda en la capa Flash del propio receptor -- ver ZED-F9P
 * Integration Manual, UBX-18010802, apartado 3.1: "Configuration
 * settings can be saved permanently in flash memory"; procedimiento
 * confirmado en la practica en
 * https://www.ardusimple.es/how-to-configure-ublox-zed-f9p/, paso 11),
 * no en cada arranque de la Teensy.
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
 * @details Arranca al unico baudrate soportado (115200), coincidiendo
 * con la configuracion permanente ya guardada en el receptor -- ver el
 * @note de este fichero. No envia nada por Serial3, solo escucha.
 * @warning Llamar una unica vez, desde setup(), antes de loop().
 * @warning El receptor GPS debe estar ya configurado de antemano
 * (RAWX/SFRBX activos, 115200 baudios, guardado permanentemente en
 * Flash) mediante u-center -- este modulo no lo comprueba ni lo fuerza.
 * @see GpsUART_Process()
 */
void GpsUART_Init(void);

/**
 * @brief Reensambla las tramas UBX recibidas y las entrega a GpsLogger.
 * @details Debe invocarse en cada iteracion de loop() (a diferencia del
 * radar, que no necesita ninguna llamada explicita porque todo ocurre
 * en su ISR). Consume el buffer circular de bytes que llena la
 * interrupcion, reconstruye tramas UBX completas, verifica su checksum,
 * y entrega cada UBX-RXM-RAWX/SFRBX valida a GpsLogger (ver
 * gps_logger.h); cada UBX-RXM-RAWX ademas aporta un GpsSyncPoint a
 * RadarLogger_GuardarSyncGps() (ver radar_logger.h). Cualquier otra
 * trama recibida se ignora en silencio.
 */
void GpsUART_Process(void);

#endif // GPS_UART_H
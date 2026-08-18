/**
 * @file gps_uart.h
 * @brief Interfaz publica del modulo de adquisicion GPS-PPK (u-blox
 * H-RTK ZED-F9P Ultralight) por UART, sobre la Teensy 4.1.
 *
 * @details
 * Captura y reensambla las tramas UBX del receptor GPS, dejando los
 * bytes crudos y validados listos para su registro en SD (ver
 * gps_logger.h). La captura ocurre por interrupcion hardware sobre el
 * periferico LPUART2, al que corresponde el puerto logico Serial3.
 *
 * @note A diferencia de radar_uart.cpp, aqui la ISR NO reensambla la
 * trama completa: solo captura byte + timestamp en un buffer circular
 * privado del propio gps_uart.cpp. El reensamblado (busqueda de sync,
 * lectura de longitud, verificacion de checksum) ocurre en
 * GpsUART_Process(), invocada desde loop(). Motivo: las tramas UBX del
 * GPS son de longitud VARIABLE (8+16+32*N bytes) y trabajar con tamanyos
 * variables dentro de una interrupcion no es deseable
 *
 * @note la marca temporal de un GpsSyncPoint es el instante de llegada del
 * PRIMER byte del sync (0xB5), no la del ultimo byte de la trama (como ocurre
 * en el radar) porque, como las tramas son variables, el tiempo no a va ser
 * consistente. La ISR sigue siendo minima (solo captura byte + timestamp, sin
 * logica anadida, ver isrGps() en gps_uart.cpp): el timestamp del primer byte
 * se  obtiene manteniendo, en gps_uart.cpp, un array s_tramaTimestamps[]
 * en paralelo con el buffer de ensamblado s_tramaBuffer[], desplazado
 * con los mismos memmove(), de forma que s_tramaTimestamps[0] senyale
 * siempre al timestamp (capturado en la ISR) del byte de sync que
 * encabeza la trama en curso.
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
 * @details `timestamp_us` es el instante de llegada del PRIMER byte del
 * sync (0xB5), no del ultimo. Ver el @note de este fichero sobre por
 * que este criterio difiere del usado en radar_uart.cpp.
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
 * con la configuracion permanente ya guardada en el receptor
 * @warning Llamar una unica vez, desde setup(), antes de loop().
 * @warning El receptor GPS debe estar ya configurado de antemano
 * (RAWX/SFRBX activos, 115200 baudios, guardado permanentemente en
 * Flash) mediante u-center
 * @note Este modulo no lo comprueba ni fuerza la configuracion del GPS.
 * @see GpsUART_Process()
 */
void GpsUART_Init(void);

/**
 * @brief Reensambla las tramas UBX recibidas y las entrega a GpsLogger.
 * @param[out] huboErrorSDGps Se pone a true si, durante esta llamada,
 * alguna trama valida no pudo escribirse en el fichero .ubx (ver
 * GpsLogger_GuardarTramaCruda()); en caso contrario se pone a false. El
 * llamador es responsable de fijar este error si procede (ver
 * s_errorSD_gps en main.cpp). Se pasa por referencia porque el valor de
 * retorno de la funcion ya se usa para indicar si se proceso alguna
 * trama valida; el paso por referencia permite devolver este segundo
 * valor de salida sin recurrir a una variable global ni a una struct.
 * @return true si se proceso al menos una trama valida en esta llamada
 * (para el LED de actividad); false en caso contrario.
 * @details Debe invocarse en cada iteracion de loop() (a diferencia del
 * radar, que no necesita ninguna llamada explicita porque todo ocurre
 * en su ISR). Consume el buffer circular de bytes que llena la
 * interrupcion, reconstruye tramas UBX completas, verifica su checksum,
 * y entrega cada UBX-RXM-RAWX/SFRBX valida a GpsLogger (ver
 * gps_logger.h); cada UBX-RXM-RAWX ademas aporta un GpsSyncPoint a
 * DataLogger_GuardarSyncGps() (ver data_logger.h). Cualquier otra
 * trama recibida se ignora en silencio.
 */
bool GpsUART_Process(bool &huboErrorSDGps);

#endif // GPS_UART_H
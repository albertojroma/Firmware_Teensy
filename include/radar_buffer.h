/**
 * @file radar_buffer.h
 * @brief Memoria FIFO (Ring Buffer) para el desacoplo temporal del radar US-D1.
 *
 * @details
 * Por que existe este buffer, en vez de que la interrupcion de
 * recepcion del radar (isrRadar(), en radar_uart.cpp) escriba
 * directamente en la tarjeta SD: la captura de cada byte debe ser lo
 * mas rapida posible, porque ocurre dentro de una interrupcion
 * hardware que no puede demorarse sin arriesgar la perdida de bytes
 * siguientes o interferir con otras interrupciones del sistema (p.
 * ej. la del GPS). Escribir en la tarjeta SD, en cambio, es una
 * operacion lenta y de duracion variable (puede tardar varios
 * milisegundos, mas aun si el propio buffer interno de la libreria SD
 * necesita volcar una pagina fisica de la memoria flash). Este buffer
 * circular actua de "colchon" entre ambos ritmos: la interrupcion
 * solo tiene que copiar 8 bytes (RadarBuffer_Push()) y seguir; el
 * consumo mas lento (registro en SD desde loop(), ver main.cpp) se
 * hace fuera de cualquier contexto de interrupcion, sin presion de
 * tiempo real.
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#ifndef RADAR_BUFFER_H
#define RADAR_BUFFER_H

#include <Arduino.h>
#include "radar_uart.h" // Para conocer la estructura RadarFrame

/**
 * @def BUFFER_SIZE
 * @brief Numero de tramas RadarFrame que caben simultaneamente en el buffer circular.
 *
 * @details
 * Se ha elegido una potencia de 2 (256) de forma deliberada: permite
 * calcular el indice siguiente mediante una mascara de bits
 * (BUFFER_MASK) en vez de una operacion de modulo (%). En un
 * microcontrolador, el operador modulo sobre un valor que no es
 * potencia de 2 requiere una division entera (costosa en ciclos de
 * reloj); con una potencia de 2, (x + 1) % BUFFER_SIZE es equivalente
 * a (x + 1) & BUFFER_MASK, una operacion AND de un solo ciclo. Dado
 * que este calculo se ejecuta en cada byte recibido dentro de una
 * interrupcion hardware (a traves de RadarBuffer_Push()), es
 * precisamente el tipo de operacion donde minimizar el coste por
 * ciclo importa.
 * @todo justificar por que el tamano concreto es 256 y no otro valor
 * potencia de 2 (p. ej. 128 o 512) -- no se ha documentado un calculo
 * explicito de cuantos segundos de margen a la tasa de 100 Hz del
 * radar representa este tamano frente a la duracion esperada de una
 * escritura en SD.
 */
#define BUFFER_SIZE 256

/**
 * @def BUFFER_MASK
 * @brief Mascara de bits usada para el calculo circular de indices del buffer.
 * @details Al ser BUFFER_SIZE una potencia de 2, BUFFER_MASK =
 * BUFFER_SIZE - 1 tiene todos sus bits menos significativos a 1 (para
 * 256, en binario 0b11111111), de forma que indice & BUFFER_MASK
 * envuelve correctamente el indice al llegar al final del array,
 * replicando el comportamiento de indice % BUFFER_SIZE sin usar
 * division.
 */
#define BUFFER_MASK (BUFFER_SIZE - 1)

/**
 * @brief Inicializa el buffer circular a su estado vacio.
 * @details Debe llamarse una vez desde setup(), antes de que
 * RadarUART_Init() active la interrupcion del radar (que es quien
 * llama a RadarBuffer_Push()).
 */
void RadarBuffer_Init(void);

/**
 * @brief Inserta una trama en el buffer circular (productor).
 *
 * @details Pensada para invocarse desde la interrupcion de recepcion
 * del radar (isrRadar(), en radar_uart.cpp) -- es, por tanto, la
 * unica funcion de este modulo que se ejecuta en contexto de
 * interrupcion real.
 *
 * @param frame Trama a insertar, pasada por referencia constante para
 *        evitar la copia de 8 bytes que supondria pasarla por valor.
 * @return true si se inserto correctamente; false si el buffer estaba
 *         lleno, en cuyo caso la trama se descarta (perdida de trama
 *         por falta de consumo a tiempo desde loop()).
 */
bool RadarBuffer_Push(const RadarFrame& frame);

/**
 * @brief Extrae la trama mas antigua del buffer circular (consumidor).
 *
 * @details Pensada para invocarse desde loop() (ver main.cpp), en un
 * bucle que la llame repetidamente hasta vaciar el buffer en cada
 * iteracion.
 *
 * @param[out] frame Variable donde se escribe la trama extraida, si
 *        la hay.
 * @return true si se extrajo una trama; false si el buffer estaba
 *         vacio (no hay tramas nuevas pendientes de procesar).
 */
bool RadarBuffer_Pop(RadarFrame& frame);

#endif // RADAR_BUFFER_H

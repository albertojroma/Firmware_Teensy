/**
 * @file radar_buffer.h
 * @brief Memoria FIFO (Ring Buffer) para datos del radar US-D1.
 *
 * @details
 * La razon de este buffer es porque la captura de cada byte debe ser lo
 * mas rapida posible, porque ocurre dentro de una interrupcion hardware que no
 * puede arriesgar la perdida de bytes o interferir con otras interrupciones
 * del sistema (p. ej. la del GPS). Escribir en la tarjeta SD, es una operacion
 * lenta y de duracion variable.
 * La interrupcion solo tiene que copiar una trama RadarFrame
 * (RadarBuffer_Push()) y seguir; el consumo mas lento (registro en SD
 * desde loop(), ver main.cpp) se hace fuera de cualquier contexto de
 * interrupcion, sin presion de tiempo real.
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
 * @brief Numero de tramas RadarFrame que caben simultaneamente en el buffer
 * circular.
 *
 * @details
 * Se ha elegido una potencia de 2 (256) de forma deliberada: permite
 * calcular el indice siguiente mediante una mascara de bits (BUFFER_MASK) en
 * vez de una operacion de modulo (%) porque esta es una operacion costosa en
 * recursos (en ciclos de reloj concretamente). Con una potencia de 2, (x + 1)
 * % BUFFER_SIZE es equivalente a (x + 1) & BUFFER_MASK, una operacion AND de
 * un solo ciclo.
 * Como este calculo se ejecuta en cada byte recibido dentro de una
 * interrupcion hardware (a traves de RadarBuffer_Push()), minimizar el coste
 * por ciclo importa.
 * El valor concreto (256, frente a otras potencias de 2 como 128 o 512)
 * es una eleccion conservadora: da margen de sobra para desacoplar la
 * ISR del consumo en loop() sin un coste relevante de memoria RAM.
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
 * del radar (isrRadar(), en radar_uart.cpp).
 * @param frame Trama a insertar, pasada por referencia para evitar la
 *        copia de la trama completa que supondria pasarla por valor.
 *        Es constante porque la funcion solo necesita leerla para
 *        copiarla al buffer, no modificar el original de la ISR.
 * @return true si se inserto correctamente; false si el buffer estaba
 *         lleno, en cuyo caso la trama se descarta (perdida de trama
 *         por falta de consumo a tiempo desde loop()).
 */
bool RadarBuffer_Push(const RadarFrame &frame);

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
bool RadarBuffer_Pop(RadarFrame &frame);

#endif // RADAR_BUFFER_H

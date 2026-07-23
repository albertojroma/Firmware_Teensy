/**
 * @file radar_buffer.cpp
 * @brief Implementacion del Ring Buffer con proteccion de secciones criticas.
 * @see radar_buffer.h para la justificacion general de la existencia
 *      de este buffer y de por que su tamanyo es una potencia de 2.
 */
#include "radar_buffer.h"

/**
 * @struct RingBuffer
 * @brief Estado interno del buffer circular: el array de tramas y los
 *        dos indices que delimitan la region ocupada.
 *
 * @details
 * head es el indice donde se escribira la PROXIMA trama (avanzado por
 * RadarBuffer_Push()); tail es el indice de la trama MAS ANTIGUA
 * todavia no consumida (avanzado por RadarBuffer_Pop()).
 * buffer vacio cuando `head == tail`, y lleno cuando avanzar head un paso
 * mas lo haria coincidir con tail (ver la comprobacion en
 * RadarBuffer_Push())
 * Este disenyo sacrifica una posicion del array (nunca se llega a usar el 100%
 * de BUFFER_SIZE) a cambio de que "vacio" y "lleno" se puedan distinguir sin
 * una variable de conteo adicional.
 *
 * head y tail son `volatile` porque se leen y escriben desde dos contextos de
 * ejecucion distintos: head se escribe desde la interrupcion del radar
 * (RadarBuffer_Push()) y se lee desde loop() (RadarBuffer_Pop()); tail a la
 * inversa, y volatile impide que el compilador optimice esas lecturas
 * asumiendo que su valor no puede cambiar entre una instruccion y la siguiente.
 */
typedef struct
{
    RadarFrame data[BUFFER_SIZE]; /**< Array circular de tramas. */
    volatile uint16_t head;       /**< Indice de proxima escritura (productor). */
    volatile uint16_t tail;       /**< Indice de proxima lectura (consumidor). */
} RingBuffer;

/** @brief Unica instancia del buffer circular del proyecto (solo 1 radar). */
static RingBuffer s_buffer;

void RadarBuffer_Init(void)
{
    s_buffer.head = 0;
    s_buffer.tail = 0;
    Serial.println("[DEBUG]: Buffer del radar inicializado.");
}

bool RadarBuffer_Push(const RadarFrame &frame)
{
    uint16_t next_head = (s_buffer.head + 1) & BUFFER_MASK;
    if (next_head == s_buffer.tail)
    {
        // Buffer lleno, perdida de trama
        return false;
    }
    s_buffer.data[s_buffer.head] = frame;
    s_buffer.head = next_head;
    return true;
}

/**
 * @details
 * **Justificacion de la copia de trama protegida con
 * noInterrupts()/interrupts():**
 * `RadarUART_Init()` (radar_uart.cpp) establece `isrRadar()` como handler de
 * interrupcion sobre LPUART6. Esto implica que `RadarBuffer_Push()` puede
 * ejecutarse en cualquier instante, de forma asincrona respecto al codigo que
 * se este ejecutando en loop().
 * Esto podria ocurrir, durante `s_buffer.data[s_buffer.tail]`.
 * Sin esta seccion critica, existiria una ventana de tiempo, aunque pequenya,
 * en la que una trama nueva podria interferir con la lectura de una trama en
 * curso de extraccion, dando lugar a una RadarFrame corrupta (con campos
 * mezclados de dos tramas distintas).
 * Deshabilitar las interrupciones durante la copia impide que isrRadar()
 * pueda ejecutarse mientras esta copia esta en curso, garantizando
 * que frame siempre contenga una trama completa y consistente.
 * El coste de este bloqueo es minimo: son solo unas pocas instrucciones
 * (la copia de 8 bytes), y el radar transmite a 100 Hz (muy por debajo de
 * cualquier frecuencia a la que este breve bloqueo pudiera suponer un problema
 * de temporizacion para otras interrupciones del sistema).
 *
 */
bool RadarBuffer_Pop(RadarFrame &frame)
{
    if (s_buffer.head == s_buffer.tail)
    {
        // Buffer vacio
        return false;
    }

    // SECCION CRITICA: Protegemos la lectura de 8 bytes de interrupciones
    noInterrupts();
    frame = s_buffer.data[s_buffer.tail];
    s_buffer.tail = (s_buffer.tail + 1) & BUFFER_MASK;
    interrupts();

    return true;
}

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
 * todavia no consumida (avanzado por RadarBuffer_Pop()). El buffer
 * esta vacio cuando head == tail, y lleno cuando avanzar head un paso
 * mas lo haria coincidir con tail (ver la comprobacion en
 * RadarBuffer_Push()) -- este diseno sacrifica deliberadamente una
 * posicion del array (nunca se llega a usar el 100% de BUFFER_SIZE) a
 * cambio de que "vacio" y "lleno" se puedan distinguir sin una
 * variable de conteo adicional.
 *
 * head y tail se declaran volatile porque se leen y escriben desde
 * dos contextos de ejecucion distintos -- head se escribe desde la
 * interrupcion del radar (RadarBuffer_Push()) y se lee desde loop()
 * (RadarBuffer_Pop()); tail a la inversa --, y volatile impide que el
 * compilador optimice esas lecturas asumiendo que su valor no puede
 * cambiar entre una instruccion y la siguiente.
 */
typedef struct {
    RadarFrame data[BUFFER_SIZE]; /**< Array circular de tramas. */
    volatile uint16_t head;       /**< Indice de proxima escritura (productor). */
    volatile uint16_t tail;       /**< Indice de proxima lectura (consumidor). */
} RingBuffer;

/** @brief Unica instancia del buffer circular del proyecto (no se prevee mas de un radar). */
static RingBuffer s_buffer;

/**
 * @copydoc RadarBuffer_Init
 */
void RadarBuffer_Init(void) {
    s_buffer.head = 0;
    s_buffer.tail = 0;
}

/**
 * @copydoc RadarBuffer_Push
 */
bool RadarBuffer_Push(const RadarFrame& frame) {
    uint16_t next_head = (s_buffer.head + 1) & BUFFER_MASK;
    if (next_head == s_buffer.tail) {
        return false; // Buffer lleno, perdida de trama
    }
    s_buffer.data[s_buffer.head] = frame;
    s_buffer.head = next_head;
    return true;
}

/**
 * @copydoc RadarBuffer_Pop
 *
 * @details
 * **Por que la copia de la trama esta protegida con
 * noInterrupts()/interrupts():** desde que RadarUART_Init()
 * (radar_uart.cpp) instala isrRadar() como manejador de interrupcion
 * real sobre LPUART6, RadarBuffer_Push() puede ejecutarse en
 * cualquier instante, de forma asincrona respecto al codigo que se
 * este ejecutando en loop() -- incluido, potencialmente, a mitad de
 * la copia de 8 bytes de s_buffer.data[s_buffer.tail] a frame que
 * realiza esta funcion. Sin esta seccion critica, existiria una
 * ventana de tiempo, por breve que sea, en la que una trama nueva
 * podria interferir con la lectura de una trama en curso de
 * extraccion, dando lugar a una RadarFrame corrupta (con campos
 * mezclados de dos tramas distintas). Deshabilitar las interrupciones
 * durante la copia (noInterrupts()/interrupts()) impide que isrRadar()
 * pueda ejecutarse mientras esta copia esta en curso, garantizando
 * que frame siempre contenga una trama completa y consistente. El
 * coste de este bloqueo es minimo: son solo unas pocas instrucciones
 * (la copia de 8 bytes), y el radar transmite a 100 Hz -- muy por
 * debajo de cualquier frecuencia a la que este breve bloqueo pudiera
 * suponer un problema de temporizacion para otras interrupciones del
 * sistema.
 *
 * @note Esta seccion critica no protegia nada de forma efectiva en
 * una version anterior de este proyecto, en la que la escritura al
 * buffer (entonces llamada desde serialEvent1()) se ejecutaba en el
 * mismo contexto que loop(), nunca de forma realmente concurrente.
 * Solo paso a tener efecto protector real tras sustituir
 * serialEvent1() por la interrupcion hardware isrRadar() (ver
 * radar_uart.cpp).
 */
bool RadarBuffer_Pop(RadarFrame& frame) {
    if (s_buffer.head == s_buffer.tail) {
        return false; // Buffer vacio
    }

    // SECCION CRITICA: Protegemos la lectura de 8 bytes de interrupciones
    noInterrupts();
    frame = s_buffer.data[s_buffer.tail];
    s_buffer.tail = (s_buffer.tail + 1) & BUFFER_MASK;
    interrupts();

    return true;
}

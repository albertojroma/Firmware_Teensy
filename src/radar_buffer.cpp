/**
 * @file radar_buffer.cpp
 * @brief Implementación del Ring Buffer con protección de secciones críticas.
 */
#include "radar_buffer.h"

typedef struct {
    RadarFrame data[BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

static RingBuffer s_buffer;

void RadarBuffer_Init(void) {
    s_buffer.head = 0;
    s_buffer.tail = 0;
}

bool RadarBuffer_Push(const RadarFrame& frame) {
    uint16_t next_head = (s_buffer.head + 1) & BUFFER_MASK;
    if (next_head == s_buffer.tail) {
        return false; // Buffer lleno, pérdida de trama
    }
    s_buffer.data[s_buffer.head] = frame;
    s_buffer.head = next_head;
    return true;
}

bool RadarBuffer_Pop(RadarFrame& frame) {
    if (s_buffer.head == s_buffer.tail) {
        return false; // Buffer vacío
    }
    
    // SECCIÓN CRÍTICA: Protegemos la lectura de 8 bytes de interrupciones
    noInterrupts();
    frame = s_buffer.data[s_buffer.tail];
    s_buffer.tail = (s_buffer.tail + 1) & BUFFER_MASK;
    interrupts();
    
    return true;
}
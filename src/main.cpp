/**
 * @file main.cpp
 * @brief Firmware de adquisición radar tolerante a latencias (TFM).
 */
#include <Arduino.h>
#include "radar_uart.h"
#include "radar_buffer.h"
#include "radar_logger.h"

const int LED_ACTIVIDAD_PIN = 13;
const int LED_ERROR_SD_PIN = 2;

static uint32_t s_ultimoFlushMs = 0;

void setup() {
    pinMode(LED_ACTIVIDAD_PIN, OUTPUT);
    pinMode(LED_ERROR_SD_PIN, OUTPUT);
    digitalWrite(LED_ERROR_SD_PIN, LOW);

    Serial.begin(115200); 

    RadarBuffer_Init();
    RadarUART_Init(115200); 

    if (!RadarLogger_Init("vuelo_01.csv")) {
        Serial.println("[ERROR CRÍTICO] Fallo de SD.");
        digitalWrite(LED_ERROR_SD_PIN, HIGH); 
        while (true) { delay(100); } // Bloqueo de seguridad
    }

    Serial.println("Sistema OK. Adquiriendo datos...");
    s_ultimoFlushMs = millis();
}

void loop() {
    RadarFrame trama_actual;

    /* 1. Consumimos todas las tramas que el radar haya metido en el buffer */
    while (RadarBuffer_Pop(trama_actual)) {
        bool ok = RadarLogger_Guardar(trama_actual);
        
        if (!ok) {
            digitalWrite(LED_ERROR_SD_PIN, HIGH);
        } else {
            digitalWrite(LED_ACTIVIDAD_PIN, !digitalRead(LED_ACTIVIDAD_PIN)); // Toggle
        }
    }

    /* 2. Temporizador no bloqueante (1 Hz) para guardado seguro */
    uint32_t tiempoActual = millis();
    if (tiempoActual - s_ultimoFlushMs >= 1000) {
        RadarLogger_Flush(); 
        s_ultimoFlushMs = tiempoActual;
    }
}
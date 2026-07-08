/**
 * @file main.cpp
 * @brief Punto de entrada del firmware Teensy 4.1 para adquisición del radar US-D1.
 *
 * Esta versión inicial valida el parser UART del radar (RadarUART) de forma
 * aislada, mostrando por USB-Serial las tramas válidas recibidas y usando
 * el LED integrado como indicador visual de actividad.
 *
 * @note Esta es una etapa de validación temprana. La lógica definitiva de
 * adquisición se integrará dentro de la FSM principal (fsm.cpp).
 */

#include <Arduino.h>
#include "radar_uart.h"

/** @brief Pin del LED integrado en la Teensy 4.1, usado como indicador visual. */
const int LED_PIN = 13;

/** @brief Instancia global del parser UART del radar US-D1. */
RadarUART radar;

/**
 * @brief Inicialización del firmware.
 *
 * Configura el pin del LED como salida, inicializa el puerto USB-Serial
 * para depuración en banco, e inicializa la UART hardware (Serial1)
 * dedicada a la recepción de tramas del radar US-D1.
 */
void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);       // USB, solo para depuración en banco
  radar.begin(Serial1, 115200); // UART hardware dedicada al radar (US-D1)

  // Pequeña espera para poder abrir el monitor serie sin perder los primeros mensajes
  delay(1000);
  Serial.println("Firmware Teensy - Validacion parser radar US-D1");
}

/**
 * @brief Bucle principal de ejecución.
 *
 * En cada iteración, invoca RadarUART::update() para procesar los bytes
 * disponibles en el buffer UART. Cuando se completa una trama válida,
 * la imprime por USB-Serial y activa brevemente el LED como indicador
 * visual de actividad.
 */
void loop() {
  if (radar.update()) {
    RadarFrame frame = radar.getLastFrame();

    // Parpadeo breve del LED como indicador visual de trama válida recibida
    digitalWrite(LED_PIN, HIGH);

    Serial.print("Altitud (cm): ");
    Serial.print(frame.altitud_cm);
    Serial.print(" | SNR: ");
    Serial.print(frame.snr);
    Serial.print(" | t (us): ");
    Serial.println(frame.timestamp_us);

    digitalWrite(LED_PIN, LOW);
  }
}
/**
 * @file main.cpp
 * @brief Firmware de adquisicion del radar US-D1, tolerante a latencias de
 *        escritura en SD.
 *
 * @details
 * Este fichero integra los tres modulos del firmware: captura del radar por
 * interrupcion (radar_uart.h), desacoplo temporal mediante buffer circular
 * (radar_buffer.h), y registro en CSV sobre SD con flush diferido
 * (radar_logger.h).
 * No implementa logica propia mas alla de la orquestacion de estos tres
 * modulos y de la senalizacion visual mediante dos LEDs (ver mas abajo).
 */
#include <Arduino.h>
#include "radar_uart.h"
#include "radar_buffer.h"
#include "radar_logger.h"

/**
 * @def PUERTO_SERIE_BAUD_RATE
 * @brief Velocidad de transmision (b/s) del puerto serie de la Teensy 4.1.
 */
#define PUERTO_SERIE_BAUD_RATE 115200

/**
 * @brief LED de actividad general del sistema.
 * @details Se alterna (toggle) cada vez que una trama se registra
 * correctamente en la SD (ver loop()), sirviendo como indicador
 * visual de que el sistema esta vivo y procesando datos.
 */
const int LED_ACTIVIDAD_PIN = 13;

/**
 * @brief LED dedicado, en un pin conectado a un LED para senyalizar errores
 * relacionados con la SD.
 *
 */
const int LED_ERROR_SD_PIN = 2;

/**
 * @brief Marca de tiempo (millis()) del ultimo flush() forzado a la SD.
 * @details Usada por el temporizador no bloqueante de loop() para
 * invocar RadarLogger_Flush() cada 1000 ms sin recurrir a delay().
 */
static uint32_t s_ultimoFlushMs = 0;

/**
 * @brief Inicializacion del firmware.
 *
 * @details
 * Orden de inicializacion, y por que importa:
 *  1. Se configuran ambos LEDs como salida, y el de error se deja
 *     explicitamente apagado (por si el pin arrancara en un estado no
 *     determinado).
 *  2. Se abre el puerto USB-Serial, unicamente para depuracion.
 *     En el sistema final desplegado en campo no habra ningun monitor
 *     conectado a este puerto.
 *  3. Se inicializa el buffer circular ANTES que la interrupcion del
 *     radar `RadarUART_Init()`, porque en cuanto esa interrupcion
 *     queda activa, podria dispararse en cualquier momento y llamar a
 *     `RadarBuffer_Push()` -- el buffer debe estar ya en un estado
 *     valido `RadarBuffer_Init()` antes de que eso pueda ocurrir.
 *  4. Se inicializa el registro en SD. **Si falla** (tarjeta no presente, no
 *     se puede crear/abrir el fichero), el firmware enciende el LED de error
 *     de forma fija y entra en un bucle infinito, sin llegar nunca a loop().
 *     Esta es una decision de disenyo deliberadamente conservadora: en el
 *     sistema final, sin monitor serie conectado en campo, la unica forma de
 *     que el operador se entere de un fallo de SD es un indicador visual
 *     persistente; continuar operando sin registrar datos supondria perder
 *     silenciosamente una salida de campo completa, un coste mucho mayor que
 *     detectar el fallo en tierra antes de despegar.
 */
void setup()
{
    // Configuracion de LEDs
    pinMode(LED_ACTIVIDAD_PIN, OUTPUT);
    pinMode(LED_ERROR_SD_PIN, OUTPUT);
    digitalWrite(LED_ERROR_SD_PIN, LOW);

    // Configuracion del puerto serie
    Serial.begin(RADAR_BAUD_RATE);

    RadarBuffer_Init();

    // Espera activa. Si la SD no esta insertada al arrancar, el LED de error queda encendido y el firmware reintenta periodicamente.

    while (!RadarLogger_Init())
    {
        digitalWrite(LED_ERROR_SD_PIN, HIGH);
        delay(400); // Margen entre reintentos. Aleatorio
        digitalWrite(LED_ERROR_SD_PIN, LOW);
    }
    digitalWrite(LED_ERROR_SD_PIN, LOW);

    RadarUART_Init();

    s_ultimoFlushMs = millis();
}

/**
 * @brief Bucle principal de ejecucion.
 *
 * @details
 * Realiza dos tareas independientes en cada iteracion. Ambas no bloqueantes:
 *
 *  1. **Consumo del buffer circular**: extrae con `RadarBuffer_Pop()` todas
 *     las tramas que la interrupcion del radar haya insertado desde la ultima
 *     iteracion, y las registra en la SD mediante `RadarLogger_Guardar()`.
 *     Se usa un while, no un if, precisamente porque en una sola vuelta de
 *     loop() podria haberse acumulado mas de una trama nueva (por ejemplo, si
 *     la iteracion anterior tardo mas de 10 ms). Por cada trama registrada con
 *     exito, se alterna el LED de actividad; si el registro falla (SD
 *     desconectada a mitad de sesion), se enciende el LED de error.
 *
 *  2. **Flush**: cada 1000 ms (comparacion de millis() sin usar delay(), para
 *     no bloquear la lectura del buffer circular durante esa espera), se invoca
 *     `RadarLogger_Flush()` para forzar la escritura fisica del contenido
 *     acumulado en la SD.
 *
 * @see RadarLogger_Guardar()
 */
void loop()
{
    RadarFrame trama_actual;

    /* 1. Consumimos todas las tramas que el radar haya metido en el buffer */
    while (RadarBuffer_Pop(trama_actual))
    {
        bool ok = RadarLogger_Guardar(trama_actual);

        if (!ok)
        {
            digitalWrite(LED_ERROR_SD_PIN, HIGH);
        }
        else
        {
            // Alternancia del LED del MCU
            digitalWrite(LED_ACTIVIDAD_PIN, !digitalRead(LED_ACTIVIDAD_PIN));
        }
    }

    /* 2. Temporizador no bloqueante (1 Hz) para guardado seguro */
    uint32_t tiempoActual = millis();
    if (tiempoActual - s_ultimoFlushMs >= 1000)
    {
        if (!RadarLogger_Flush())
        {
            digitalWrite(LED_ERROR_SD_PIN, HIGH);
        }
        s_ultimoFlushMs = tiempoActual;
    }
}

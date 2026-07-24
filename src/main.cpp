/**
 * @file main.cpp
 * @brief Firmware de adquisicion del radar US-D1 y del GPS-PPK ZED-F9P,
 *        tolerante a latencias de escritura en SD.
 *
 * @details
 * Este fichero integra cinco modulos: captura del radar por interrupcion
 * (radar_uart.h), desacoplo temporal mediante buffer circular
 * (radar_buffer.h), registro combinado radar+GPS en CSV sobre SD con
 * flush diferido (radar_logger.h), captura del GPS por interrupcion
 * (gps_uart.h), y registro del paso binario del GPS sobre SD
 * (gps_logger.h). No implementa logica propia mas alla de la
 * orquestacion de estos modulos y de la senalizacion visual mediante
 * tres LEDs (ver mas abajo).
 *
 * @note El receptor GPS debe estar configurado de antemano de forma
 * PERMANENTE (RAWX/SFRBX activos, 115200 baudios, guardado en Flash
 * con u-center) -- este firmware no envia absolutamente nada por
 * Serial3, solo escucha. Ver gps_uart.h para la justificacion completa
 * de esta decision de diseno (un cuelgue deterministico, detectado por
 * biseccion exhaustiva, asociado a la sola presencia de Serial3.write()
 * en el binario).
 */
#include <Arduino.h>
#include "radar_uart.h"
#include "radar_buffer.h"
#include "radar_logger.h"
#include "gps_uart.h"
#include "gps_logger.h"

/**
 * @def PUERTO_SERIE_BAUD_RATE
 * @brief Velocidad de transmision (b/s) del puerto serie de la Teensy 4.1.
 */
#define PUERTO_SERIE_BAUD_RATE 115200

/**
 * @brief LED de actividad del radar.
 * @details Se alterna (toggle) cada vez que una trama del radar se
 * registra correctamente en la SD (ver loop()), sirviendo como
 * indicador visual de que el sistema esta vivo y procesando datos.
 */
const int LED_ACTIVIDAD_RADAR_PIN = 13;

/**
 * @brief LED de actividad general del sistema.
 * @details Se alterna (toggle) cada vez que una trama del radar se
 * registra correctamente en la SD (ver loop()), sirviendo como
 * indicador visual de que el sistema esta vivo y procesando datos.
 */
const int LED_ACTIVIDAD_GPS_PIN = 6;

/**
 * @brief LED dedicado a errores relacionados con la SD del radar.
 */
const int LED_ERROR_SD_PIN = 2;

/**
 * @brief LED dedicado a errores de registro en SD del modulo GPS.
 * @details Unicamente refleja fallos de GpsLogger_Init()/
 * GpsLogger_Flush() -- ya no existe ningun estado de error de
 * configuracion del GPS en si, porque gps_uart.h ya no configura nada
 * (ver su documentacion, y la nota de este mismo fichero). Pin
 * fisicamente distinto de LED_ACTIVIDAD_RADAR_PIN y LED_ERROR_SD_PIN, mismo
 * patron de cableado (resistencia limitadora + LED hacia GND).
 */
const int LED_ERROR_GPS_PIN = 4;

/**
 * @brief Marca de tiempo (millis()) del ultimo flush() forzado a la SD.
 * @details Usada por el temporizador no bloqueante de loop() para
 * invocar RadarLogger_Flush()/GpsLogger_Flush() cada 1000 ms sin
 * recurrir a delay().
 */
static uint32_t s_ultimoFlushMs = 0;

/**
 * @brief Inicializacion del firmware.
 *
 * @details
 * Orden de inicializacion, y por que importa:
 *  1. Se configuran los tres LEDs como salida, con los de error
 *     explicitamente apagados.
 *  2. Se abre el puerto USB-Serial, unicamente para depuracion. En el
 *     sistema final desplegado en campo no habra ningun monitor
 *     conectado a este puerto.
 *  3. Se inicializa el buffer circular del radar ANTES que su
 *     interrupcion `RadarUART_Init()`, porque la interrupcion podria
 *     dispararse en cualquier momento y llamar a `RadarBuffer_Push()`.
 *  4. Se inicializa el registro combinado (radar+GPS) en SD, con
 *     espera activa y reintentos si la tarjeta no esta insertada (LED
 *     de error encendido durante la espera). Esta SI bloquea el
 *     firmware indefinidamente si falla de forma persistente --
 *     decision ya tomada: sin monitor serie en campo, es preferible
 *     detectar un fallo de SD en tierra que perder silenciosamente una
 *     salida de campo completa.
 *  5. Se consulta el numero de sesion ya elegido por RadarLogger_Init()
 *     y se usa para inicializar el registro del `.ubx` del GPS, de
 *     forma que ambos ficheros comparten el mismo numero de vuelo. Si
 *     GpsLogger_Init() falla, se enciende LED_ERROR_GPS_PIN pero NO se
 *     bloquea el firmware -- el radar debe poder seguir registrando
 *     sin GPS.
 *  6. Se inicializan las interrupciones del radar y del GPS.
 *     GpsUART_Init() no envia nada por Serial3 -- el receptor debe
 *     estar ya configurado de fabrica/manualmente de antemano (ver
 *     @note de este fichero).
 */
void setup()
{
  // Para que de tiempo a arrancar el monitor serie
  delay(10000);
  Serial.println("[DEBUG]: Inicia el sistema");

  pinMode(LED_ACTIVIDAD_RADAR_PIN, OUTPUT);
  pinMode(LED_ACTIVIDAD_GPS_PIN, OUTPUT);
  pinMode(LED_ERROR_SD_PIN, OUTPUT);
  pinMode(LED_ERROR_GPS_PIN, OUTPUT);

  digitalWrite(LED_ERROR_SD_PIN, LOW);
  digitalWrite(LED_ERROR_GPS_PIN, LOW);
  digitalWrite(LED_ACTIVIDAD_RADAR_PIN, LOW);
  digitalWrite(LED_ACTIVIDAD_GPS_PIN, LOW);

  Serial.begin(PUERTO_SERIE_BAUD_RATE);

  RadarBuffer_Init();

  while (!RadarLogger_Init())
  {
    digitalWrite(LED_ERROR_SD_PIN, HIGH);
  }
  digitalWrite(LED_ERROR_SD_PIN, LOW);

  int numeroVuelo = RadarLogger_ObtenerNumeroVuelo();
  if (!GpsLogger_Init(numeroVuelo))
  {
    // Fallo NO bloqueante: el radar debe poder seguir registrando sin GPS.
    digitalWrite(LED_ERROR_GPS_PIN, HIGH);
  }

  RadarUART_Init();
  GpsUART_Init();

  s_ultimoFlushMs = millis();
}

/**
 * @brief Bucle principal de ejecucion.
 *
 * @details
 * Realiza tres tareas independientes en cada iteracion. Ninguna bloqueante:
 *
 *  1. **Consumo del buffer circular del radar**: extrae con
 *     `RadarBuffer_Pop()` todas las tramas acumuladas, y las registra
 *     con `RadarLogger_Guardar()`.
 *
 *  2. **Procesado del GPS**: `GpsUART_Process()` reensambla las tramas
 *     UBX recibidas y las registra en SD una vez detectado flujo valido
 *     -- toda esa logica vive dentro del propio modulo (ver
 *     gps_uart.cpp). Sin ninguna comprobacion de error asociada: ya no
 *     existe ese concepto, al no haber ninguna configuracion que pueda
 *     fallar (ver gps_uart.h).
 *
 *  3. **Flush**: cada 1000 ms (comparacion de millis() sin usar
 *     delay()), se invoca `RadarLogger_Flush()` y `GpsLogger_Flush()`
 *     para forzar la escritura fisica del contenido acumulado en la SD.
 *
 * @see RadarLogger_Guardar(), GpsUART_Process()
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
      // Alternancia del LED de actividad
      digitalWrite(LED_ACTIVIDAD_RADAR_PIN, !digitalRead(LED_ACTIVIDAD_RADAR_PIN));
    }
  }

  /* 2. Procesado del GPS: reensamblado de tramas */
  if (GpsUART_Process())
  {
    digitalWrite(LED_ACTIVIDAD_GPS_PIN, !digitalRead(LED_ACTIVIDAD_GPS_PIN));
  }

  /* 3. Temporizador no bloqueante (1 Hz) para guardado seguro */
  uint32_t tiempoActual = millis();
  if (tiempoActual - s_ultimoFlushMs >= 1000)
  {
    if (!RadarLogger_Flush())
    {
      digitalWrite(LED_ERROR_SD_PIN, HIGH);
    }
    else
    {
      digitalWrite(LED_ERROR_SD_PIN, LOW);
    }

    if (!GpsLogger_Flush())
    {
      digitalWrite(LED_ERROR_GPS_PIN, HIGH);
    }
    else
    {
      digitalWrite(LED_ERROR_GPS_PIN, LOW);
    }
    s_ultimoFlushMs = tiempoActual;
  }
}
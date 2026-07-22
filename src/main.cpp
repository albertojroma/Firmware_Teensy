/**
 * @file main.cpp
 * @brief Firmware de adquisicion del radar US-D1 y del GPS-PPK ZED-F9P,
 *        tolerante a latencias de escritura en SD.
 *
 * @details
 * Este fichero integra cinco modulos: captura del radar por interrupcion
 * (radar_uart.h), desacoplo temporal mediante buffer circular
 * (radar_buffer.h), registro del radar en CSV sobre SD con flush
 * diferido (radar_logger.h), captura y configuracion del GPS por
 * interrupcion (gps_uart.h), y registro del GPS sobre SD (gps_logger.h).
 * No implementa logica propia mas alla de la orquestacion de estos
 * modulos y de la senalizacion visual mediante tres LEDs (ver mas abajo).
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
 * @brief LED de actividad general del sistema.
 * @details Se alterna (toggle) cada vez que una trama del radar se
 * registra correctamente en la SD (ver loop()), sirviendo como
 * indicador visual de que el sistema esta vivo y procesando datos.
 */
const int LED_ACTIVIDAD_PIN = 13;

/**
 * @brief LED dedicado, en un pin conectado a un LED para senyalizar
 * errores relacionados con la SD (radar).
 */
const int LED_ERROR_SD_PIN = 2;

/**
 * @brief LED dedicado a errores del modulo GPS (configuracion fallida
 * tras agotar reintentos, o fallo de escritura en sus ficheros).
 * @details Pin fisicamente distinto de LED_ACTIVIDAD_PIN y
 * LED_ERROR_SD_PIN, mismo patron de cableado (resistencia limitadora +
 * LED hacia GND). A diferencia de LED_ERROR_SD_PIN, un fallo de GPS NO
 * detiene el firmware -- ver la decision de diseno documentada en
 * gps_uart.cpp (GPS_ERROR).
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
 *     explicitamente apagados (por si el pin arrancara en un estado no
 *     determinado).
 *  2. Se abre el puerto USB-Serial, unicamente para depuracion. En el
 *     sistema final desplegado en campo no habra ningun monitor
 *     conectado a este puerto.
 *  3. Se inicializa el buffer circular del radar ANTES que su
 *     interrupcion `RadarUART_Init()`, por el mismo motivo ya
 *     documentado en versiones anteriores de este fichero: la
 *     interrupcion podria dispararse en cualquier momento y llamar a
 *     `RadarBuffer_Push()`.
 *  4. Se inicializa el registro del radar en SD, con espera activa y
 *     reintentos si la tarjeta no esta insertada (LED de error
 *     encendido durante la espera). Esta SI bloquea el firmware
 *     indefinidamente si falla de forma persistente -- decision ya
 *     tomada: sin monitor serie en campo, es preferible detectar un
 *     fallo de SD en tierra que perder silenciosamente una salida de
 *     campo completa.
 *  5. Se consulta el numero de sesion ya elegido por RadarLogger_Init()
 *     (`RadarLogger_ObtenerNumeroVuelo()`) y se usa para inicializar el
 *     registro del GPS, de forma que ambos conjuntos de ficheros
 *     comparten el mismo numero de vuelo. Si GpsLogger_Init() falla, se
 *     enciende LED_ERROR_GPS_PIN pero NO se bloquea el firmware -- el
 *     radar debe poder seguir registrando sin GPS.
 *  6. Se inicializan las interrupciones del radar y del GPS. El GPS
 *     arranca su propia maquina de estados de configuracion (ver
 *     gps_uart.h), que se completa de forma asincrona en loop().
 */
void setup()
{
  delay(10000);

  // Configuracion de LEDs
  pinMode(LED_ACTIVIDAD_PIN, OUTPUT);
  pinMode(LED_ERROR_SD_PIN, OUTPUT);
  pinMode(LED_ERROR_GPS_PIN, OUTPUT);
  digitalWrite(LED_ERROR_SD_PIN, LOW);
  digitalWrite(LED_ERROR_GPS_PIN, LOW);

  // Configuracion del puerto serie de depuracion
  Serial.begin(PUERTO_SERIE_BAUD_RATE);

  RadarBuffer_Init();

  // Espera activa. Si la SD no esta insertada al arrancar, el LED de
  // error queda encendido y el firmware reintenta periodicamente.
  while (!RadarLogger_Init())
  {
    digitalWrite(LED_ERROR_SD_PIN, HIGH);
    delay(400); // Margen entre reintentos.
    digitalWrite(LED_ERROR_SD_PIN, LOW);
  }
  digitalWrite(LED_ERROR_SD_PIN, LOW);
  Serial.println("[DEBUG] Logger del radar inicializado");

  int numeroVuelo = RadarLogger_ObtenerNumeroVuelo();

  if (!GpsLogger_Init(numeroVuelo))
  {
    // Fallo NO bloqueante: ver justificacion en el @details de esta
    // funcion y en gps_uart.cpp (GPS_ERROR).
    digitalWrite(LED_ERROR_GPS_PIN, HIGH);
  }
  Serial.println("[DEBUG] Logger del GPS inicializado");

  RadarUART_Init();
  Serial.println("[DEBUG] UART del radar inicializado");

  GpsUART_Init();
  Serial.println("[DEBUG] UART del GPS inicializado");

  s_ultimoFlushMs = millis();
}

/**
 * @brief Bucle principal de ejecucion.
 *
 * @details
 * Realiza tres tareas independientes en cada iteracion. Ninguna bloqueante:
 *
 *  1. **Consumo del buffer circular del radar**: igual que en versiones
 *     anteriores de este fichero -- extrae con `RadarBuffer_Pop()` todas
 *     las tramas acumuladas, y las registra con `RadarLogger_Guardar()`.
 *
 *  2. **Procesado del GPS**: `GpsUART_Process()` reensambla las tramas
 *     UBX recibidas, avanza su maquina de estados de configuracion, y
 *     registra en SD las tramas validas una vez en estado de registro
 *     -- toda esa logica vive dentro del propio modulo (ver
 *     gps_uart.cpp); loop() solo tiene que invocarla y comprobar
 *     `GpsUART_HayError()` para la senyalizacion visual.
 *
 *  3. **Flush**: cada 1000 ms (comparacion de millis() sin usar
 *     delay()), se invoca `RadarLogger_Flush()` y `GpsLogger_Flush()`
 *     para forzar la escritura fisica del contenido acumulado en la SD.
 *
 * @see RadarLogger_Guardar(), GpsUART_Process()
 */
void loop()
{
  Serial.println("[DEBUG] Inicio del loop");

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
      digitalWrite(LED_ACTIVIDAD_PIN, !digitalRead(LED_ACTIVIDAD_PIN));
    }
  }

  /* 2. Procesado del GPS: reensamblado de tramas + maquina de estados */
  GpsUART_Process();
  if (GpsUART_HayError())
  {
    digitalWrite(LED_ERROR_GPS_PIN, HIGH);
  }

  /* 3. Temporizador no bloqueante (1 Hz) para guardado seguro */
  uint32_t tiempoActual = millis();
  if (tiempoActual - s_ultimoFlushMs >= 1000)
  {
    if (!RadarLogger_Flush())
    {
      digitalWrite(LED_ERROR_SD_PIN, HIGH);
    }
    if (!GpsLogger_Flush())
    {
      digitalWrite(LED_ERROR_GPS_PIN, HIGH);
    }
    s_ultimoFlushMs = tiempoActual;
  }
}

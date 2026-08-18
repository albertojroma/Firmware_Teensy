/**
 * @file main.cpp
 * @brief Firmware de adquisicion de datos del radar US-D1 y del GPS-PPK
 *        ZED-F9P.
 *
 * @details
 * Este fichero integra cinco modulos: captura del radar por interrupcion
 * (radar_uart.h), desacoplo temporal mediante buffer circular
 * (radar_buffer.h), registro combinado radar+GPS en CSV sobre SD con
 * flush diferido (data_logger.h), captura del GPS por interrupcion
 * (gps_uart.h), y registro de datos binarios del GPS sobre SD
 * (gps_logger.h). No implementa logica propia mas alla de la
 * orquestacion de estos modulos y de la senalizacion visual mediante
 * cuatro LEDs (ver mas abajo).
 *
 * @note El receptor GPS debe estar configurado de antemano de forma
 * PERMANENTE (RAWX/SFRBX activos, 115200 baudios, guardado en Flash
 * con u-center)
 *
 * @author Alberto Jesus Rodriguez Machado
 * @date 2026
 */
#include <Arduino.h>
#include "radar_uart.h"
#include "radar_buffer.h"
#include "data_logger.h"
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
 * @brief LED de actividad del GPS.
 * @details Se alterna cada vez que GpsUART_Process() reensambla al menos una
 * trama UBX-RXM-RAWX/SFRBX valida en esa iteracion de loop(), sirviendo como
 * indicador visual de que el receptor GPS esta vivo y entregando datos.
 */
const int LED_ACTIVIDAD_GPS_PIN = 6;

/**
 * @brief LED dedicado a errores de escritura del .csv o problemas en la sd.
 * @details Este LED se enciende si hay problemas en la escritura de datos en el
 * archivo .csv o si la tarjeta sd no se detecta.
 */
const int LED_ERROR_SD_PIN = 2;

/**
 * @brief LED dedicado a errores de registro en SD del modulo GPS.
 * @details De enclavamiento (latch): una vez encendido permanece
 * encendido hasta el reinicio del firmware, reflejando el flag
 * persistente s_errorSD_gps.
 */
const int LED_ERROR_GPS_PIN = 4;

/**
 * @brief Marca de tiempo (millis()) del ultimo flush() forzado a la SD.
 * @details Usada por el temporizador no bloqueante de loop() para
 * invocar DataLogger_Flush()/GpsLogger_Flush() cada 1000 ms sin
 * recurrir a delay().
 */
static uint32_t s_ultimoFlushMs = 0;

/**
 * @brief Flag fija (latch) de error de SD del CSV combinado radar+GPS (data_logger.h).
 * @details Se pone a true ante cualquier fallo de DataLogger_Init(),
 * DataLogger_Guardar() o DataLogger_Flush(), y nunca vuelve a false:
 * una vez detectado un fallo, LED_ERROR_SD_PIN permanece encendido
 * hasta el reinicio del firmware, aunque los flush() posteriores
 * tengan exito.
 */
static bool s_errorSD_datalogger = false;

/**
 * @brief Flag fija (latch) de error de SD del GPS.
 * @details Se pone a true ante cualquier fallo de GpsLogger_Init(),
 * GpsLogger_Flush() o GpsLogger_GuardarTramaCruda(), y nunca vuelve a
 * false: una vez detectado un fallo, LED_ERROR_GPS_PIN permanece
 * encendido hasta el reinicio del firmware.
 */
static bool s_errorSD_gps = false;

/**
 * @brief Inicializacion del firmware.
 *
 * @details
 * Orden de inicializacion, y por que importa:
 *  1. Se configuran los cuatro LEDs como salida, con los de error
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
 *  5. Se consulta el numero de sesion ya elegido por DataLogger_Init()
 *     y se usa para inicializar el registro del `.ubx` del GPS, de
 *     forma que ambos ficheros comparten el mismo numero de vuelo. Si
 *     GpsLogger_Init() falla, se enclava s_errorSD_gps y se enciende
 *     LED_ERROR_GPS_PIN (permanece encendido hasta el reinicio) pero NO
 *     se bloquea el firmware -- el radar debe poder seguir registrando
 *     sin GPS.
 *  6. Se inicializan las interrupciones del radar y del GPS.
 *     GpsUART_Init() no envia nada por Serial3 -- el receptor debe
 *     estar ya configurado de fabrica/manualmente de antemano (ver
 *     @note de este fichero).
 */
void setup()
{
  // Para que de tiempo a arrancar el monitor serie
  // delay(10000);

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

  while (!DataLogger_Init())
  {
    digitalWrite(LED_ERROR_SD_PIN, HIGH);
  }
  digitalWrite(LED_ERROR_SD_PIN, LOW);

  int numeroVuelo = DataLogger_ObtenerNumeroVuelo();
  if (!GpsLogger_Init(numeroVuelo))
  {
    // Fallo NO bloqueante: el radar debe poder seguir registrando sin GPS.
    s_errorSD_gps = true;
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
 *     con `DataLogger_Guardar()`.
 *
 *  2. **Procesado del GPS**: `GpsUART_Process()` reensambla las tramas
 *     UBX recibidas y las registra en SD una vez detectado flujo valido
 *     -- toda esa logica vive dentro del propio modulo (ver
 *     gps_uart.cpp). Puede fallar la escritura de la trama en SD:
 *     `GpsUART_Process()` reporta ese fallo mediante su parametro de salida, y
 *     se enclava en s_errorSD_gps.
 *
 *  3. **Flush**: cada 1000 ms (comparacion de millis() sin usar
 *     delay()), se invoca `DataLogger_Flush()` y `GpsLogger_Flush()`
 *     para forzar la escritura fisica del contenido acumulado en la SD.
 *
 * @see DataLogger_Guardar(), GpsUART_Process()
 */
void loop()
{
  RadarFrame trama_actual;

  /* 1. Consumimos todas las tramas que el radar haya metido en el buffer */
  while (RadarBuffer_Pop(trama_actual))
  {
    bool ok = DataLogger_Guardar(trama_actual);

    if (!ok)
    {
      s_errorSD_datalogger = true;
      digitalWrite(LED_ERROR_SD_PIN, HIGH);
    }
    else
    {
      // Alternancia del LED de actividad
      digitalWrite(LED_ACTIVIDAD_RADAR_PIN, !digitalRead(LED_ACTIVIDAD_RADAR_PIN));
    }
  }

  /* 2. Procesado del GPS: reensamblado de tramas */
  bool huboErrorSDGps = false;
  if (GpsUART_Process(huboErrorSDGps))
  {
    digitalWrite(LED_ACTIVIDAD_GPS_PIN, !digitalRead(LED_ACTIVIDAD_GPS_PIN));
  }
  if (huboErrorSDGps)
  {
    s_errorSD_gps = true;
    digitalWrite(LED_ERROR_GPS_PIN, HIGH);
  }

  /* 3. Temporizador no bloqueante (1 Hz) para guardado seguro */
  uint32_t tiempoActual = millis();
  if (tiempoActual - s_ultimoFlushMs >= 1000)
  {
    if (!DataLogger_Flush())
    {
      s_errorSD_datalogger = true;
      digitalWrite(LED_ERROR_SD_PIN, HIGH);
    }

    if (!GpsLogger_Flush())
    {
      s_errorSD_gps = true;
      digitalWrite(LED_ERROR_GPS_PIN, HIGH);
    }
    s_ultimoFlushMs = tiempoActual;
  }
}
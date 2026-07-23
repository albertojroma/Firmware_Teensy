/**
 * @file gps_uart.cpp
 * @brief Captura por interrupcion y reensamblado de tramas UBX del GPS-PPK.
 * @see gps_uart.h para la interfaz publica y la justificacion completa
 * de por que este modulo no envia nada por Serial3.
 */
#include "gps_uart.h"
#include "gps_logger.h"
#include "radar_logger.h" // Para RadarLogger_GuardarSyncGps()
#include <imxrt.h>
#include <string.h>

//* ============================ Protocolo UBX ==============================
//* Documento base para todas las citas de esta seccion, salvo que se indique
//* otro: u-blox, "u-blox F9 HPG 1.51 Interface Description"
//* (UBXDOC-963802114-13124, revision R01, 8-nov-2024).

#define UBX_SYNC1 ((uint8_t)0xB5)
#define UBX_SYNC2 ((uint8_t)0x62)

// UBX-RXM (apartado 3.17) -- los unicos dos tipos de trama que este
// modulo reconoce; cualquier otra Class/ID se ignora en
// gestionarTramaCompleta().
#define CLASS_RXM ((uint8_t)0x02)
#define ID_RAWX ((uint8_t)0x15)
#define ID_SFRBX ((uint8_t)0x13)

/**
 * @def GPS_BAUD_RATE
 * @brief Unico baudrate soportado: coincide con la configuracion
 * permanente ya guardada en el receptor via u-center (ver gps_uart.h).
 * @details Margen de sobra para el peor caso de N=90 senyales
 * simultaneas a 1 Hz (~29040 baudios minimos, mismo calculo
 * 8+16+32*N ya usado en este proyecto para el analisis de ancho de
 * banda) -- 115200 deja un margen de x3.97 sobre ese minimo.
 */
#define GPS_BAUD_RATE ((uint32_t)115200)

//* ==================== Buffer circular de bytes (ISR) ======================
//* Privado a este fichero: tanto el productor (isrGps) como el consumidor
//* (GpsUART_Process) viven aqui; ningun otro modulo necesita ver bytes
//* crudos sin reensamblar -- ver nota de diseno en gps_uart.h.

#define GPS_BUFFER_BYTES_SIZE 4096 // potencia de 2 (igual criterio que radar_buffer.h)
#define GPS_BUFFER_BYTES_MASK (GPS_BUFFER_BYTES_SIZE - 1)

typedef struct
{
  uint8_t dato;
  uint32_t timestamp_us;
} ByteEvent;

DMAMEM static volatile ByteEvent s_bufferBytes[GPS_BUFFER_BYTES_SIZE];
static volatile uint16_t s_idxEscritura = 0;
static volatile uint16_t s_idxLectura = 0;

/**
 * @brief Handler de interrupcion de Serial3 (LPUART2).
 * @details Mismo patron ya depurado en radar_uart.cpp: reconoce
 * (limpia) las banderas de error de STAT antes de leer datos --
 * necesario para evitar un bucle de interrupcion indefinido, mismo
 * fallo real ya documentado en:
 * https://github.com/phoenix-rtos/phoenix-rtos-devices/issues/55
 * A diferencia del radar, aqui NO se parsea nada: solo se captura el
 * byte y su timestamp en el buffer circular, para minimizar el tiempo
 * de ejecucion de la interrupcion (ver justificacion en gps_uart.h).
 */
static void isrGps(void)
{
  IMXRT_LPUART2.STAT = IMXRT_LPUART2.STAT;

  uint8_t iteraciones = 0;
  while ((IMXRT_LPUART2.STAT & LPUART_STAT_RDRF) && iteraciones < 32)
  {
    uint32_t t = micros();
    uint8_t dato = IMXRT_LPUART2.DATA;

    uint16_t siguiente = (uint16_t)((s_idxEscritura + 1) & GPS_BUFFER_BYTES_MASK);
    if (siguiente != s_idxLectura)
    {
      s_bufferBytes[s_idxEscritura].dato = dato;
      s_bufferBytes[s_idxEscritura].timestamp_us = t;
      s_idxEscritura = siguiente;
    }
    // Si el buffer esta lleno, el byte se descarta. No hay contador de
    // overrun para este buffer -- @todo: anadir uno si en banco se
    // observan perdidas de trama no explicadas por otra causa.
    iteraciones++;
  }
}

//* =================== Buffer de ensamblado de trama =========================
//* Lineal (no circular): se desplaza con memmove() al consumir una trama.
//* 3072 bytes de margen sobre el peor caso calculado en el proyecto para
//* RAWX (8+16+32*90 = 2904 bytes).

#define GPS_TRAMA_MAX_BYTES 3072

DMAMEM static uint8_t s_tramaBuffer[GPS_TRAMA_MAX_BYTES];
static uint16_t s_tramaLongitud = 0;

//* ========================= Estado de registro ==============================
//* Ya no hay maquina de estados de configuracion (ver gps_uart.h): solo
//* dos estados posibles, sin ninguna transicion por fallo.

typedef enum
{
  GPS_VERIFICANDO_FLUJO, /**< Esperando la primera UBX-RXM-RAWX valida. */
  GPS_REGISTRANDO        /**< Ya llego al menos una RAWX valida: se registra todo lo que llegue. */
} EstadoGps;

static EstadoGps s_estadoGps = GPS_VERIFICANDO_FLUJO;

//* ============================ Utilidades UBX ================================

/**
 * @brief Calcula el checksum Fletcher de 8 bits de una trama UBX.
 * @param datos Puntero al primer byte a incluir (Class).
 * @param longitud Numero de bytes a incluir (4 + payload).
 * @param[out] ckA, ckB Bytes de checksum resultantes.
 * @details Algoritmo definido en el Interface Description, apartado
 * 3.4, p. 57 -- mismo algoritmo ya implementado en checksum_ubx() del
 * emulador Python de este proyecto.
 */
static void calcularChecksum(const uint8_t *datos, uint16_t longitud, uint8_t *ckA, uint8_t *ckB)
{
  uint8_t a = 0, b = 0;
  for (uint16_t i = 0; i < longitud; i++)
  {
    a = (uint8_t)(a + datos[i]);
    b = (uint8_t)(b + a);
  }
  *ckA = a;
  *ckB = b;
}

//* ==================== Gestion de tramas completas ============================

/**
 * @brief Procesa una trama UBX completa y con checksum valido.
 * @param msgClass, msgId Class e ID de la trama.
 * @param trama Puntero al inicio de la trama (byte de sync).
 * @param longitudTotal Tamano total de la trama, en bytes.
 * @param timestampUltimoByte Instante de llegada del ultimo byte de la
 * trama (segundo byte de checksum), capturado por isrGps().
 * @details Solo reconoce RAWX/SFRBX -- cualquier otra Class/ID se
 * ignora en silencio, ya que el receptor no deberia emitir nada mas
 * con la configuracion permanente ya guardada (ver gps_uart.h). Si el
 * estado es GPS_VERIFICANDO_FLUJO, una RAWX valida provoca la
 * transicion a GPS_REGISTRANDO. Una vez en GPS_REGISTRANDO, la trama
 * se entrega a GpsLogger (bytes crudos siempre; ademas rcvTow/week via
 * RadarLogger_GuardarSyncGps() si es RAWX).
 */
static void gestionarTramaCompleta(uint8_t msgClass, uint8_t msgId, const uint8_t *trama,
                                   uint16_t longitudTotal, uint32_t timestampUltimoByte)
{
  bool esRawx = (msgClass == CLASS_RXM && msgId == ID_RAWX);
  bool esSfrbx = (msgClass == CLASS_RXM && msgId == ID_SFRBX);

  if (!esRawx && !esSfrbx)
    return; // Class/ID no relevante: se ignora.

  if (s_estadoGps == GPS_VERIFICANDO_FLUJO && esRawx)
  {
    s_estadoGps = GPS_REGISTRANDO;
    Serial.println("[DEBUG]: Esperando trama RAWX valida");
  }

  if (s_estadoGps == GPS_REGISTRANDO)
  {
    Serial.println("[DEBUG]: Llego por lo menos 1 trama RAWX valida");
    GpsLogger_GuardarTramaCruda(trama, longitudTotal);

    if (esRawx)
    {
      // Payload de RAWX empieza en trama[6]; rcvTow (R8) en el
      // offset 0 del payload, week (U2) en el offset 8 -- apartado
      // 3.17.6, p. 198.
      GpsSyncPoint punto;
      punto.timestamp_us = timestampUltimoByte;
      memcpy(&punto.rcvTow, &trama[6], 8);
      memcpy(&punto.week, &trama[14], 2);
      RadarLogger_GuardarSyncGps(punto);
    }
  }
}

/**
 * @brief Busca la posicion del sync 0xB5 0x62 dentro de s_tramaBuffer.
 * @return Indice del primer byte del sync, o -1 si no hay ninguno completo.
 */
static int16_t buscarSync(void)
{
  for (uint16_t i = 0; (uint16_t)(i + 1) < s_tramaLongitud; i++)
  {
    if (s_tramaBuffer[i] == UBX_SYNC1 && s_tramaBuffer[i + 1] == UBX_SYNC2)
    {
      return (int16_t)i;
    }
  }
  return -1;
}

/**
 * @brief Anade un byte recien recibido al buffer de ensamblado y
 * resuelve cualquier trama completa que quede disponible.
 * @param dato Byte recibido.
 * @param t Timestamp de llegada de ESE byte (de la ISR).
 * @details El timestamp que se asocia a una trama completada es
 * siempre el del byte que la completa exactamente en esta misma
 * llamada -- por eso la comprobacion de trama completa ocurre aqui,
 * byte a byte, y no en un segundo paso por separado (que perderia la
 * asociacion exacta byte-timestamp).
 */
static void procesarByteGps(uint8_t dato, uint32_t t)
{
  if (s_tramaLongitud < GPS_TRAMA_MAX_BYTES)
  {
    s_tramaBuffer[s_tramaLongitud] = dato;
    s_tramaLongitud++;
  }
  else
  {
    // Desbordamiento del buffer de ensamblado (trama implausiblemente
    // larga, o perdida de sincronismo sostenida): se descarta todo y
    // se reinicia la busqueda.
    s_tramaLongitud = 0;
    return;
  }

  while (true)
  {
    int16_t idxSync = buscarSync();

    if (idxSync == -1)
    {
      // Sin sync completo: conserva como maximo el ultimo byte, por si
      // es el primero de un sync futuro.
      if (s_tramaLongitud > 0 && s_tramaBuffer[s_tramaLongitud - 1] == UBX_SYNC1)
      {
        s_tramaBuffer[0] = UBX_SYNC1;
        s_tramaLongitud = 1;
      }
      else
      {
        s_tramaLongitud = 0;
      }
      return;
    }

    if (idxSync > 0)
    {
      memmove(s_tramaBuffer, s_tramaBuffer + idxSync, (size_t)(s_tramaLongitud - idxSync));
      s_tramaLongitud = (uint16_t)(s_tramaLongitud - idxSync);
    }

    // Trama minima posible (payload vacio): 8 bytes -- ver
    // hil_gps_emulacion.md, verificado con UBX-MGA-DBD como poll
    // request real del Interface Description.
    if (s_tramaLongitud < 8)
      return;

    uint16_t longitudPayload;
    memcpy(&longitudPayload, &s_tramaBuffer[4], 2);
    uint16_t tramaTotal = (uint16_t)(6 + longitudPayload + 2);

    if (tramaTotal > GPS_TRAMA_MAX_BYTES)
    {
      // Longitud implausible: probablemente un sync falso dentro de
      // datos que no son una trama real. Se descarta solo el propio
      // sync (2 bytes) y se reintenta la busqueda desde ahi.
      memmove(s_tramaBuffer, s_tramaBuffer + 2, (size_t)(s_tramaLongitud - 2));
      s_tramaLongitud = (uint16_t)(s_tramaLongitud - 2);
      continue;
    }

    if (s_tramaLongitud < tramaTotal)
      return; // trama incompleta, esperar mas bytes

    uint8_t ckA, ckB;
    calcularChecksum(&s_tramaBuffer[2], (uint16_t)(4 + longitudPayload), &ckA, &ckB);
    bool checksumOk = (ckA == s_tramaBuffer[6 + longitudPayload]) &&
                      (ckB == s_tramaBuffer[7 + longitudPayload]);

    if (checksumOk)
    {
      gestionarTramaCompleta(s_tramaBuffer[2], s_tramaBuffer[3], s_tramaBuffer, tramaTotal, t);
    }
    // Si el checksum falla, la trama se descarta en silencio -- mismo
    // criterio de recuperacion que el parser del radar.

    memmove(s_tramaBuffer, s_tramaBuffer + tramaTotal, (size_t)(s_tramaLongitud - tramaTotal));
    s_tramaLongitud = (uint16_t)(s_tramaLongitud - tramaTotal);
    // Continua el bucle: podria haber otra trama completa arrastrada.
  }
}

//* ================================ API publica ================================

/**
 * @copydoc GpsUART_Init
 */
void GpsUART_Init(void)
{
  Serial3.begin(GPS_BAUD_RATE);

  attachInterruptVector(IRQ_LPUART2, isrGps);
  NVIC_ENABLE_IRQ(IRQ_LPUART2);
  IMXRT_LPUART2.CTRL |= LPUART_CTRL_RIE; // Receiver Interrupt Enable

  s_estadoGps = GPS_VERIFICANDO_FLUJO;
  s_tramaLongitud = 0;
  s_idxEscritura = 0;
  s_idxLectura = 0;
  Serial.println("[DEBUG]: UART del GPS inicializada.");
}

/**
 * @copydoc GpsUART_Process
 */
void GpsUART_Process(void)
{
  while (s_idxLectura != s_idxEscritura)
  {
    uint8_t dato;
    uint32_t t;

    noInterrupts();
    dato = s_bufferBytes[s_idxLectura].dato;
    t = s_bufferBytes[s_idxLectura].timestamp_us;
    s_idxLectura = (uint16_t)((s_idxLectura + 1) & GPS_BUFFER_BYTES_MASK);
    interrupts();

    procesarByteGps(dato, t);

    Serial.println("[DEBUG]: Byte de la UART del GPS procesado correctamente.");
  }
}
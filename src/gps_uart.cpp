/**
 * @file gps_uart.cpp
 * @brief Captura por interrupcion, reensamblado de tramas UBX, y maquina
 * de estados de configuracion/registro del GPS-PPK.
 * @see gps_uart.h para la interfaz publica y la justificacion de por
 * que el reensamblado ocurre fuera de la interrupcion.
 */
#include "gps_uart.h"
#include "gps_logger.h"
#include "radar_logger.h" // Para RadarLogger_GuardarSyncGps() -- ver nota de diseno en radar_logger.h
#include <imxrt.h>
#include <string.h>

//* ============================ Protocolo UBX ==============================
//* Documento base para todas las citas de esta seccion, salvo que se indique
//* otro: u-blox, "u-blox F9 HPG 1.51 Interface Description"
//* (UBXDOC-963802114-13124, revision R01, 8-nov-2024).

#define UBX_SYNC1 ((uint8_t)0xB5)
#define UBX_SYNC2 ((uint8_t)0x62)

// UBX-RXM (apartado 3.17)
#define CLASS_RXM ((uint8_t)0x02)
#define ID_RAWX ((uint8_t)0x15)
#define ID_SFRBX ((uint8_t)0x13)

// UBX-CFG (apartado 3.10)
#define CLASS_CFG ((uint8_t)0x06)
#define ID_VALSET ((uint8_t)0x8A) // apartado 3.10.25, p. 98
#define ID_VALGET ((uint8_t)0x8B) // apartado 3.10.24, p. 97

// UBX-ACK (apartado 3.9)
#define CLASS_ACK ((uint8_t)0x05)
#define ID_ACK ((uint8_t)0x01)
#define ID_NAK ((uint8_t)0x00)

// KeyID de configuracion (mismas que ya usa el emulador Python del proyecto)
#define KEY_MSGOUT_RXM_RAWX_UART1 ((uint32_t)0x209102A5)
#define KEY_MSGOUT_RXM_SFRBX_UART1 ((uint32_t)0x20910232)
#define KEY_UART1_BAUDRATE ((uint32_t)0x40520001) // apartado 6.9.31, p. 292

// Baudrates: fabrica segun ZED-F9P Integration Manual (UBX-18010802 R16,
// apartado 3.1.3, p. 13); objetivo elegido en este proyecto tras el
// analisis de margen de ancho de banda (documentado en hil_gps_emulacion.md).
#define GPS_BAUD_FABRICA ((uint32_t)38400)
#define GPS_BAUD_OBJETIVO ((uint32_t)230400)

//* ==================== Buffer circular de bytes (ISR) ======================
//* Privado a este fichero: tanto el productor (isrGps) como el consumidor
//* (GpsUART_Process) viven aqui; ningun otro modulo necesita ver bytes
//* crudos sin reensamblar -- ver nota de diseno en gps_uart.h.
//* DMAMEM: se descarto como causa de un cuelgue distinto ya investigado
//* (ver radar_logger.h) tras comprobar que ni el tamano ni la region de
//* estos arrays influian; se mantiene aqui por ser la region correcta
//* para bufferes grandes segun la convencion de Teensyduino, no como
//* mitigante de ese problema.

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

//* ========================= Maquina de estados ==============================

typedef enum
{
  GPS_VERIFICANDO_CONFIG,
  GPS_CFG_ACTIVAR_RAWX,
  GPS_CFG_ACTIVAR_SFRBX,
  GPS_CFG_CAMBIAR_BAUDRATE,
  GPS_ERROR,
  GPS_VERIFICANDO_FLUJO,
  GPS_REGISTRANDO
} EstadoGps;

static EstadoGps s_estadoGps = GPS_VERIFICANDO_CONFIG;

#define GPS_TIMEOUT_MS 300
#define GPS_MAX_REINTENTOS 3

static bool s_peticionEnviada = false;
static uint32_t s_tiempoInicioEstado = 0;
static uint8_t s_reintentos = 0;

// Resultado de la ultima respuesta UBX-ACK-ACK/NAK recibida.
static volatile bool s_ackPendiente = false;
static volatile bool s_ackPositivo = false;

// Resultado de la ultima respuesta UBX-CFG-VALGET recibida.
static volatile bool s_valgetRespondido = false;
static volatile bool s_valgetCoincide = false;

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

/**
 * @brief Construye una trama UBX completa (sync+cabecera+payload+checksum).
 * @param msgClass, msgId Class e ID del mensaje.
 * @param payload Puntero al payload.
 * @param longitudPayload Tamano del payload, en bytes.
 * @param[out] tramaSalida Buffer donde escribir la trama completa.
 * @param[out] longitudSalida Tamano total de la trama escrita.
 * @details Estructura definida en el Interface Description, apartado 3.2.
 * @warning No comprueba que tramaSalida tenga espacio suficiente -- las
 * unicas tramas que este modulo construye (UBX-CFG-VALSET/VALGET) tienen
 * un payload pequeno y acotado, verificado por inspeccion en las
 * funciones que llaman a esta.
 */
static void construirTramaUbx(uint8_t msgClass, uint8_t msgId, const uint8_t *payload,
                               uint16_t longitudPayload, uint8_t *tramaSalida, uint16_t *longitudSalida)
{
  tramaSalida[0] = UBX_SYNC1;
  tramaSalida[1] = UBX_SYNC2;
  tramaSalida[2] = msgClass;
  tramaSalida[3] = msgId;
  tramaSalida[4] = (uint8_t)(longitudPayload & 0xFF);
  tramaSalida[5] = (uint8_t)(longitudPayload >> 8);
  memcpy(&tramaSalida[6], payload, longitudPayload);

  uint8_t ckA, ckB;
  calcularChecksum(&tramaSalida[2], (uint16_t)(4 + longitudPayload), &ckA, &ckB);
  tramaSalida[6 + longitudPayload] = ckA;
  tramaSalida[7 + longitudPayload] = ckB;

  *longitudSalida = (uint16_t)(8 + longitudPayload);
}

/**
 * @brief Envia un UBX-CFG-VALSET de una unica clave con valor de 1 byte (U1).
 * @param keyId KeyID de configuracion a establecer.
 * @param valor Valor de 1 byte a asignar.
 * @details layers=0x01 (RAM unicamente) -- decision de diseno ya tomada
 * en este proyecto: no se escribe a Flash, porque GPS_VERIFICANDO_CONFIG
 * ya comprueba en cada arranque si la configuracion sigue vigente, sin
 * necesitar persistencia entre ciclos de alimentacion (ver
 * hil_gps_emulacion.md).
 */
static void enviarValsetU1(uint32_t keyId, uint8_t valor)
{
  uint8_t payload[9];
  payload[0] = 0x00; // version
  payload[1] = 0x01; // layers = RAM (bit 0)
  payload[2] = 0x00; // reserved
  payload[3] = 0x00; // reserved
  memcpy(&payload[4], &keyId, 4);
  payload[8] = valor;

  uint8_t trama[32];
  uint16_t longitud;
  construirTramaUbx(CLASS_CFG, ID_VALSET, payload, sizeof(payload), trama, &longitud);
  Serial3.write(trama, longitud);
}

/**
 * @brief Envia un UBX-CFG-VALSET de una unica clave con valor de 4 bytes (U4).
 * @param keyId KeyID de configuracion a establecer.
 * @param valor Valor de 4 bytes a asignar.
 * @see enviarValsetU1() para la justificacion de layers=0x01.
 */
static void enviarValsetU4(uint32_t keyId, uint32_t valor)
{
  uint8_t payload[12];
  payload[0] = 0x00;
  payload[1] = 0x01;
  payload[2] = 0x00;
  payload[3] = 0x00;
  memcpy(&payload[4], &keyId, 4);
  memcpy(&payload[8], &valor, 4);

  uint8_t trama[32];
  uint16_t longitud;
  construirTramaUbx(CLASS_CFG, ID_VALSET, payload, sizeof(payload), trama, &longitud);
  Serial3.write(trama, longitud);
}

/**
 * @brief Envia un UBX-CFG-VALGET consultando las 3 claves de interes,
 * sobre la capa RAM.
 * @details Es la comprobacion previa de GPS_VERIFICANDO_CONFIG: evita
 * renegociar la configuracion si el receptor ya la tenia activa de una
 * sesion anterior de la Teensy sin ciclo de alimentacion del GPS.
 */
static void enviarValgetComprobacion(void)
{
  const uint32_t claves[3] = {KEY_MSGOUT_RXM_RAWX_UART1, KEY_MSGOUT_RXM_SFRBX_UART1, KEY_UART1_BAUDRATE};

  uint8_t payload[4 + 3 * 4];
  payload[0] = 0x00; // version (peticion)
  payload[1] = 0x00; // layer = RAM
  payload[2] = 0x00; // position
  payload[3] = 0x00;
  for (uint8_t i = 0; i < 3; i++)
  {
    memcpy(&payload[4 + i * 4], &claves[i], 4);
  }

  uint8_t trama[32];
  uint16_t longitud;
  construirTramaUbx(CLASS_CFG, ID_VALGET, payload, sizeof(payload), trama, &longitud);
  Serial3.write(trama, longitud);
}

/**
 * @brief Analiza el payload de una respuesta UBX-CFG-VALGET y comprueba
 * si las 3 claves de interes ya tienen el valor deseado.
 * @param payload Payload de la respuesta (version+layer+position+cfgData).
 * @param longitudPayload Tamano del payload, en bytes.
 * @details Actualiza s_valgetCoincide/s_valgetRespondido. Una keyID
 * inesperada corta el analisis del resto (no se puede saber su tamano
 * sin una tabla de tamanyos, que este modulo no necesita mas que para
 * las 3 claves conocidas).
 */
static void procesarRespuestaValget(const uint8_t *payload, uint16_t longitudPayload)
{
  bool rawxOk = false, sfrbxOk = false, baudOk = false;
  uint16_t idx = 4; // tras version+layer+position

  while ((uint16_t)(idx + 4) <= longitudPayload)
  {
    uint32_t keyId;
    memcpy(&keyId, &payload[idx], 4);
    idx += 4;

    if (keyId == KEY_MSGOUT_RXM_RAWX_UART1)
    {
      rawxOk = (payload[idx] > 0);
      idx += 1;
    }
    else if (keyId == KEY_MSGOUT_RXM_SFRBX_UART1)
    {
      sfrbxOk = (payload[idx] > 0);
      idx += 1;
    }
    else if (keyId == KEY_UART1_BAUDRATE)
    {
      uint32_t baud;
      memcpy(&baud, &payload[idx], 4);
      baudOk = (baud == GPS_BAUD_OBJETIVO);
      idx += 4;
    }
    else
    {
      break; // KeyID no reconocida: no se puede seguir con garantias
    }
  }

  s_valgetCoincide = rawxOk && sfrbxOk && baudOk;
  s_valgetRespondido = true;
}

//* ==================== Gestion de tramas completas ============================

/**
 * @brief Procesa una trama UBX completa y con checksum valido.
 * @param msgClass, msgId Class e ID de la trama.
 * @param trama Puntero al inicio de la trama (byte de sync).
 * @param longitudTotal Tamano total de la trama, en bytes.
 * @param timestampUltimoByte Instante de llegada del ultimo byte de la
 * trama (segundo byte de checksum), capturado por isrGps().
 * @details Bifurca segun el tipo de trama:
 * - RAWX/SFRBX: si el estado es GPS_VERIFICANDO_FLUJO, una RAWX valida
 *   provoca la transicion a GPS_REGISTRANDO. Si el estado ya es
 *   GPS_REGISTRANDO, la trama se entrega a GpsLogger (bytes crudos
 *   siempre; ademas rcvTow/week via RadarLogger_GuardarSyncGps() si es
 *   RAWX -- ver nota de diseno en radar_logger.h sobre por que la
 *   sincronizacion vive en el mismo CSV del radar, no en un fichero
 *   propio del GPS).
 * - UBX-ACK-ACK/NAK: resultado de un UBX-CFG-VALSET pendiente.
 * - UBX-CFG-VALGET (respuesta, version=0x01 en el payload): resultado
 *   de la comprobacion previa de configuracion.
 * Cualquier otra trama (Class/ID no relevante para este proyecto) se
 * ignora en silencio.
 */
static void gestionarTramaCompleta(uint8_t msgClass, uint8_t msgId, const uint8_t *trama,
                                    uint16_t longitudTotal, uint32_t timestampUltimoByte)
{
  bool esRawx = (msgClass == CLASS_RXM && msgId == ID_RAWX);
  bool esSfrbx = (msgClass == CLASS_RXM && msgId == ID_SFRBX);

  if (esRawx || esSfrbx)
  {
    if (s_estadoGps == GPS_VERIFICANDO_FLUJO && esRawx)
    {
      s_estadoGps = GPS_REGISTRANDO;
    }

    if (s_estadoGps == GPS_REGISTRANDO)
    {
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
    return;
  }

  if (msgClass == CLASS_ACK && (msgId == ID_ACK || msgId == ID_NAK))
  {
    s_ackPositivo = (msgId == ID_ACK);
    s_ackPendiente = false;
    return;
  }

  if (msgClass == CLASS_CFG && msgId == ID_VALGET)
  {
    // El payload empieza en trama[6]; version de respuesta = 0x01.
    if (trama[6] == 0x01)
    {
      procesarRespuestaValget(&trama[6], (uint16_t)(longitudTotal - 8));
    }
    return;
  }

  // Class/ID no relevante para este proyecto: se ignora.
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

//* ============ Avance de la maquina de estados de configuracion ==============

/**
 * @brief Avanza un paso generico de la sub-maquina de configuracion
 * (envio de UBX-CFG-VALSET de 1 byte, espera de ACK, reintentos).
 * @param keyId Clave a activar.
 * @param valor Valor a asignar (1 byte).
 * @param siguienteEstado Estado al que pasar si se recibe ACK positivo.
 */
static void avanzarPasoU1(uint32_t keyId, uint8_t valor, EstadoGps siguienteEstado)
{
  uint32_t ahora = millis();

  if (!s_peticionEnviada)
  {
    enviarValsetU1(keyId, valor);
    s_peticionEnviada = true;
    s_ackPendiente = true;
    s_tiempoInicioEstado = ahora;
    return;
  }

  if (!s_ackPendiente)
  {
    s_peticionEnviada = false;
    if (s_ackPositivo)
    {
      s_reintentos = 0;
      s_estadoGps = siguienteEstado;
    }
    else
    {
      s_reintentos++;
      if (s_reintentos >= GPS_MAX_REINTENTOS)
        s_estadoGps = GPS_ERROR;
    }
    return;
  }

  if (ahora - s_tiempoInicioEstado >= GPS_TIMEOUT_MS)
  {
    s_peticionEnviada = false;
    s_reintentos++;
    if (s_reintentos >= GPS_MAX_REINTENTOS)
      s_estadoGps = GPS_ERROR;
  }
}

/**
 * @brief Avanza el paso de cambio de baudrate: igual que avanzarPasoU1()
 * pero, tras el ACK positivo, conmuta tambien el propio Serial3.
 * @details El receptor confirma el ACK con el baudrate ANTIGUO antes de
 * conmutar -- mismo orden ya implementado en el emulador Python de este
 * proyecto.
 */
static void avanzarPasoBaudrate(void)
{
  uint32_t ahora = millis();

  if (!s_peticionEnviada)
  {
    enviarValsetU4(KEY_UART1_BAUDRATE, GPS_BAUD_OBJETIVO);
    s_peticionEnviada = true;
    s_ackPendiente = true;
    s_tiempoInicioEstado = ahora;
    return;
  }

  if (!s_ackPendiente)
  {
    s_peticionEnviada = false;
    if (s_ackPositivo)
    {
      s_reintentos = 0;
      Serial3.begin(GPS_BAUD_OBJETIVO);
      s_estadoGps = GPS_VERIFICANDO_FLUJO;
    }
    else
    {
      s_reintentos++;
      if (s_reintentos >= GPS_MAX_REINTENTOS)
        s_estadoGps = GPS_ERROR;
    }
    return;
  }

  if (ahora - s_tiempoInicioEstado >= GPS_TIMEOUT_MS)
  {
    s_peticionEnviada = false;
    s_reintentos++;
    if (s_reintentos >= GPS_MAX_REINTENTOS)
      s_estadoGps = GPS_ERROR;
  }
}

/**
 * @brief Avanza el estado GPS_VERIFICANDO_CONFIG (comprobacion previa
 * por UBX-CFG-VALGET).
 */
static void avanzarVerificandoConfig(void)
{
  uint32_t ahora = millis();

  if (!s_peticionEnviada)
  {
    enviarValgetComprobacion();
    s_peticionEnviada = true;
    s_valgetRespondido = false;
    s_tiempoInicioEstado = ahora;
    return;
  }

  if (s_valgetRespondido)
  {
    s_peticionEnviada = false;
    s_reintentos = 0;
    s_estadoGps = s_valgetCoincide ? GPS_VERIFICANDO_FLUJO : GPS_CFG_ACTIVAR_RAWX;
    return;
  }

  if (ahora - s_tiempoInicioEstado >= GPS_TIMEOUT_MS)
  {
    // Sin respuesta: se asume que hay que configurar de todos modos,
    // sin tratarlo como un fallo (a diferencia de un NAK explicito en
    // los pasos de VALSET).
    s_peticionEnviada = false;
    s_estadoGps = GPS_CFG_ACTIVAR_RAWX;
  }
}

//* ================================ API publica ================================

/**
 * @copydoc GpsUART_Init
 */
void GpsUART_Init(void)
{
  Serial3.begin(GPS_BAUD_FABRICA);

  attachInterruptVector(IRQ_LPUART2, isrGps);
  NVIC_ENABLE_IRQ(IRQ_LPUART2);
  IMXRT_LPUART2.CTRL |= LPUART_CTRL_RIE; // Receiver Interrupt Enable

  s_estadoGps = GPS_VERIFICANDO_CONFIG;
  s_peticionEnviada = false;
  s_reintentos = 0;
  s_tramaLongitud = 0;
  s_idxEscritura = 0;
  s_idxLectura = 0;
}

/**
 * @copydoc GpsUART_Process
 */
void GpsUART_Process(void)
{
  // 1. Drenar el buffer circular de bytes hacia el ensamblado de trama.
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
  }

  // 2. Avanzar la maquina de estados de configuracion/registro.
  switch (s_estadoGps)
  {
  case GPS_VERIFICANDO_CONFIG:
    avanzarVerificandoConfig();
    break;
  case GPS_CFG_ACTIVAR_RAWX:
    avanzarPasoU1(KEY_MSGOUT_RXM_RAWX_UART1, 1, GPS_CFG_ACTIVAR_SFRBX);
    break;
  case GPS_CFG_ACTIVAR_SFRBX:
    avanzarPasoU1(KEY_MSGOUT_RXM_SFRBX_UART1, 1, GPS_CFG_CAMBIAR_BAUDRATE);
    break;
  case GPS_CFG_CAMBIAR_BAUDRATE:
    avanzarPasoBaudrate();
    break;
  case GPS_ERROR:
    // Estado terminal para esta sesion. Decision de diseno: un fallo
    // de configuracion del GPS NO bloquea el resto del firmware (a
    // diferencia de un fallo de SD) -- el radar debe poder seguir
    // registrando sin GPS. No se reintenta automaticamente sin limite.
    // @todo: evaluar si conviene un reintento manual/por temporizador
    // largo en vez de quedar bloqueado hasta el proximo reinicio.
    break;
  case GPS_VERIFICANDO_FLUJO:
    // Sin timeout: se espera indefinidamente a la primera RAWX valida
    // (la transicion ocurre dentro de gestionarTramaCompleta()).
    // @todo: considerar un timeout maximo razonable si nunca llega
    // ninguna trama, para poder senyalizar el problema en vez de
    // quedar esperando para siempre.
    break;
  case GPS_REGISTRANDO:
    // Nada que avanzar por temporizador: el registro ocurre
    // enteramente en gestionarTramaCompleta().
    break;
  }
}

/**
 * @copydoc GpsUART_HayError
 */
bool GpsUART_HayError(void)
{
  return s_estadoGps == GPS_ERROR;
}

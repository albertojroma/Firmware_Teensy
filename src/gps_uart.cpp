/**
 * @file gps_uart.cpp
 * @brief Captura por interrupcion y reensamblado de tramas UBX del GPS-PPK.
 * @see gps_uart.h para la interfaz publica y la justificacion completa
 * de por que este modulo no envia nada por Serial3.
 * @note Documento base para todas las citas de esta seccion, salvo que se
 * indique otro: u-blox, "u-blox F9 HPG 1.51 Interface Description"
 * (UBXDOC-963802114-13124, revision R01, 8-nov-2024).

 */
#include "gps_uart.h"
#include "gps_logger.h"
#include "data_logger.h" // Para DataLogger_GuardarSyncGps()
#include <imxrt.h>
#include <string.h>

//* ============================ Protocolo UBX ==============================
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

#define GPS_BUFFER_BYTES_SIZE 4096 // potencia de 2 (mismo criterio que radar_buffer.h)
#define GPS_BUFFER_BYTES_MASK (GPS_BUFFER_BYTES_SIZE - 1)

typedef struct
{
  uint8_t dato;
  uint32_t timestamp_us;
} ByteEvent;

static volatile ByteEvent s_bufferBytes[GPS_BUFFER_BYTES_SIZE];
static volatile uint16_t s_idxEscritura = 0;
static volatile uint16_t s_idxLectura = 0;

/**
 * @brief Handler de interrupcion de Serial3 (LPUART2).
 * @details Mismo patron ya depurado en radar_uart.cpp: reconoce
 * (limpia) las banderas de error de STAT antes de leer datos --
 * necesario para evitar un bucle de interrupcion indefinido, mismo
 * fallo real ya documentado en:
 * https://github.com/phoenix-rtos/phoenix-rtos-devices/issues/55
 *
 * @note A diferencia del radar, aqui NO se parsea nada: solo se captura el
 * byte y su timestamp en el buffer circular, para minimizar el tiempo
 * de ejecucion de la interrupcion (ver justificacion en gps_uart.h).
 */
static void isrGps(void)
{
  IMXRT_LPUART2.STAT = IMXRT_LPUART2.STAT;

  while ((IMXRT_LPUART2.STAT & LPUART_STAT_RDRF))
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
    // En las pruebas realizadas hay 0 checksums invalidos. Este es un
    // indicio indirecto, pero es conveniente saber que una trama con
    // byte perdido deberia fallar al reensamblarse.
  }
}

//* =================== Buffer de ensamblado de trama =========================
//* Lineal (no circular): se desplaza con memmove() al consumir una trama.
//* No necesita ser potencia de 2 (a diferencia de GPS_BUFFER_BYTES_SIZE):
//* al ser lineal no se indexa con mascara, solo con desplazamientos.
//* N=90 es el mismo peor caso ya usado en este fichero para el margen
//* del baudrate (ver GPS_BAUD_RATE): 8+16+32*90 = 2904 bytes maximo
//* para una RAWX. 3072 es el primer multiplo de 1 KiB por encima de
//* ese maximo, dejando 168 bytes (~5,8 %) de margen.
#define GPS_TRAMA_MAX_BYTES 3072

static uint8_t s_tramaBuffer[GPS_TRAMA_MAX_BYTES];
static uint16_t s_tramaLongitud = 0;

/**
 * @brief Timestamps de la ISR (micros()) en paralelo, byte a byte, con
 * s_tramaBuffer.
 * @details s_tramaTimestamps[i] es el instante de llegada (capturado en
 * isrGps(), nunca en el bucle) del byte que ocupa s_tramaBuffer[i]. Se
 * desplaza con los mismos memmove() que s_tramaBuffer, para que el
 * indice 0 siga senyalando siempre al timestamp del byte de sync que
 * encabeza la trama en curso -- ver el @note de gps_uart.h sobre por
 * que la marca de una trama es la del PRIMER byte, no la del ultimo.
 */
static uint32_t s_tramaTimestamps[GPS_TRAMA_MAX_BYTES];

//* ========================== Estado de registro ==============================
//* Maquina de estados con solo dos estados posibles, sin ninguna transicion
//* por fallo.

typedef enum
{
  GPS_VERIFICANDO_FLUJO, /**< Esperando la primera UBX-RXM-RAWX valida. */
  GPS_REGISTRANDO        /**< Ya llego al menos una RAWX valida: se registra todo lo que llegue. */
} EstadoGps;

static EstadoGps s_estadoGps = GPS_VERIFICANDO_FLUJO;
static bool s_huboRegistroReciente = false;
static bool s_errorSDGpsDetectado = false;

//* ============================ Utilidades UBX ================================

/**
 * @brief Calcula el checksum Fletcher de 8 bits de una trama UBX.
 * @param datos Puntero al primer byte a incluir (Class).
 * @param longitud Numero de bytes a incluir (4 + payload).
 * @param[out] ckA, ckB Bytes de checksum resultantes.
 * @details Algoritmo definido en el Interface Description, apartado
 * 3.4, p. 57
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

//* ====================== Gestion de tramas completas =========================

/**
 * @brief Procesa una trama UBX completa y con checksum valido.
 * @param msgClass, msgId Class e ID de la trama.
 * @param trama Puntero al inicio de la trama (byte de sync).
 * @param longitudTotal Tamanyo total de la trama, en bytes.
 * @param timestampPrimerByte Instante de llegada del primer byte de la
 * trama (0xB5, primero del sync), capturado por isrGps() y propagado
 * mediante s_tramaTimestamps -- ver el @note de gps_uart.h sobre por
 * que se usa el primer byte y no el ultimo.
 * @details Solo reconoce RAWX/SFRBX. Cualquier otra Class/ID se
 * ignora en silencio, ya que el receptor no deberia emitir nada mas
 * con la configuracion permanente ya guardada (ver gps_uart.h). Si el
 * estado es GPS_VERIFICANDO_FLUJO, una RAWX valida provoca la
 * transicion a GPS_REGISTRANDO. Una vez en GPS_REGISTRANDO, la trama
 * se entrega a GpsLogger (bytes crudos siempre; ademas rcvTow/week via
 * DataLogger_GuardarSyncGps() si es RAWX). Si GpsLogger_GuardarTramaCruda()
 * falla, se marca s_errorSDGpsDetectado para que GpsUART_Process() lo
 * reporte al llamador -- ver su parametro huboErrorSDGps.
 */
static void gestionarTramaCompleta(uint8_t msgClass, uint8_t msgId, const uint8_t *trama, uint16_t longitudTotal, uint32_t timestampPrimerByte)
{
  bool esRawx = (msgClass == CLASS_RXM && msgId == ID_RAWX);
  bool esSfrbx = (msgClass == CLASS_RXM && msgId == ID_SFRBX);

  if (!esRawx && !esSfrbx)
    return; // Class/ID no relevante: se ignora.

  if (s_estadoGps == GPS_VERIFICANDO_FLUJO && esRawx)
  {
    s_estadoGps = GPS_REGISTRANDO;
    // Serial.println("[DEBUG]: Esperando trama RAWX valida");
  }

  if (s_estadoGps == GPS_REGISTRANDO)
  {
    // Serial.println("[DEBUG]: Llego por lo menos 1 trama RAWX valida");
    if (!GpsLogger_GuardarTramaCruda(trama, longitudTotal))
    {
      s_errorSDGpsDetectado = true;
    }
    s_huboRegistroReciente = true;

    if (esRawx)
    {
      // Payload de RAWX empieza en trama[6]; rcvTow (R8) en el
      // offset 0 del payload, week (U2) en el offset 8 -- apartado
      // 3.17.6, p. 198.
      GpsSyncPoint punto;
      punto.timestamp_us = timestampPrimerByte;
      // memcpy() copia byte a byte sin exigir alineacion de memoria:
      // &trama[6] y &trama[14] son offsets arbitrarios dentro de un
      // buffer de bytes, no alineados para double/uint16_t. Leerlos
      // con un cast de puntero directo seria un acceso desalineado
      // (comportamiento indefinido). No hace falta convertir el orden
      // de bytes: UBX es little-endian (ver cabecera del fichero) y
      // el Cortex-M7 de la Teensy tambien, asi que la copia cruda ya
      // da el valor correcto.
      memcpy(&punto.rcvTow, &trama[6], 8);
      memcpy(&punto.week, &trama[14], 2);
      DataLogger_GuardarSyncGps(punto);
    }
  }
}

/**
 * @brief Busca la posicion del sync 0xB5 0x62 dentro de s_tramaBuffer.
 * @return Indice del primer byte del sync, o -1 si no hay ninguno completo.
 */
static int16_t buscarSync(void)
{
  // "i" es uint16_t porque recorre s_tramaLongitud (tambien uint16_t)
  // y asi se evita comparar signo con sin signo. Se devuelve como
  // int16_t porque hace falta un valor diferenciador negativo para
  // "no encontrado" (-1); el cast es seguro porque i no supera 3071
  // (GPS_TRAMA_MAX_BYTES - 1), muy por debajo del limite de int16_t.
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
 * @brief Anyade un byte recien recibido al buffer de ensamblado y
 * resuelve cualquier trama completa que quede disponible.
 * @param dato Byte recibido.
 * @param t Timestamp de llegada de ESE byte (de la ISR).
 * @details El timestamp que se asocia a una trama completada es el del
 * PRIMER byte de su secuencia de sync (0xB5), no el del byte que la
 * completa -- ver el @note de gps_uart.h. s_tramaTimestamps[i] se
 * mantiene en paralelo con s_tramaBuffer[i] byte a byte, y se
 * desplaza con los mismos memmove(), para que s_tramaTimestamps[0]
 * senale siempre al instante de llegada (capturado en isrGps()) del
 * byte de sync que encabeza la trama en curso.
 */
static void procesarByteGps(uint8_t dato, uint32_t t)
{
  if (s_tramaLongitud < GPS_TRAMA_MAX_BYTES)
  {
    s_tramaBuffer[s_tramaLongitud] = dato;
    s_tramaTimestamps[s_tramaLongitud] = t;
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
        s_tramaTimestamps[0] = s_tramaTimestamps[s_tramaLongitud - 1];
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
      // memmove() (no memcpy) porque origen y destino se solapan: se
      // desplaza el propio buffer sobre si mismo. Firma: memmove(destino,
      // origen, bytes_a_mover).
      // Descarta los idxSync bytes previos al sync desplazando el resto
      // al inicio (junto con s_tramaTimestamps en paralelo, ver
      // @details); (size_t) evita conversion de signo en el 3er parametro.
      memmove(s_tramaBuffer, s_tramaBuffer + idxSync, (size_t)(s_tramaLongitud - idxSync));
      memmove(s_tramaTimestamps, s_tramaTimestamps + idxSync,
              (size_t)(s_tramaLongitud - idxSync) * sizeof(uint32_t));
      s_tramaLongitud = (uint16_t)(s_tramaLongitud - idxSync);
    }

    // Trama minima posible (payload vacio): 8 bytes
    if (s_tramaLongitud < 8)
      return;

    uint16_t longitudPayload;
    // Mismo motivo que con rcvTow/week: &s_tramaBuffer[4] es un offset
    // arbitrario sin garantia de alineacion para uint16_t. Asignar
    // directamente con un cast de puntero seria un acceso desalineado
    // (comportamiento indefinido); memcpy() copia hacia una variable
    // ya alineada. No hace falta conversion de bytes: el campo length
    // de UBX es little-endian, igual que la Teensy.
    memcpy(&longitudPayload, &s_tramaBuffer[4], 2);
    uint16_t tramaTotal = (uint16_t)(6 + longitudPayload + 2);

    if (tramaTotal > GPS_TRAMA_MAX_BYTES)
    {
      // Longitud implausible: probablemente un sync falso dentro de
      // datos que no son una trama real. Se descarta solo el propio
      // sync (2 bytes) y se reintenta la busqueda desde ahi.
      // Mismo patron de memmove() que en procesarByteGps() (ver mas
      // arriba), pero descartando solo los 2 bytes del sync falso, no
      // idxSync: al ser la longitud implausible, se asume que no era
      // un sync real. "continue;" reintenta buscarSync() sobre lo que
      // queda, sin esperar a que llegue el siguiente byte.
      memmove(s_tramaBuffer, s_tramaBuffer + 2, (size_t)(s_tramaLongitud - 2));
      memmove(s_tramaTimestamps, s_tramaTimestamps + 2, (size_t)(s_tramaLongitud - 2) * sizeof(uint32_t));
      s_tramaLongitud = (uint16_t)(s_tramaLongitud - 2);
      continue;
    }

    if (s_tramaLongitud < tramaTotal)
      return; // trama incompleta, esperar mas bytes

    uint8_t ckA, ckB;
    // El checksum UBX cubre Class+ID+Length+Payload, no los 2 bytes de
    // sync ni los propios ckA/ckB. Bytes: [0]=sync1 [1]=sync2
    // [2]=class [3]=id [4:5]=length [6:]=payload -> empieza en [2]
    // (salta el sync) y cubre 4+longitudPayload bytes (Class+ID+Length
    // + Payload), hasta justo antes de donde empieza el checksum.
    calcularChecksum(&s_tramaBuffer[2], (uint16_t)(4 + longitudPayload), &ckA, &ckB);
    bool checksumOk = (ckA == s_tramaBuffer[6 + longitudPayload]) &&
                      (ckB == s_tramaBuffer[7 + longitudPayload]);

    if (checksumOk)
    {
      gestionarTramaCompleta(s_tramaBuffer[2], s_tramaBuffer[3], s_tramaBuffer, tramaTotal, s_tramaTimestamps[0]);
    }
    // Si el checksum falla, la trama se descarta en silencio -- mismo
    // criterio de recuperacion que el parser del radar.

    // Mismo patron de memmove() que en el resto de la funcion. Aqui se
    // descarta la trama que acaba de procesarse completa (tramaTotal
    // bytes, tenga o no checksum valido), desplazando al inicio del
    // buffer lo que quede despues -- podria ser ya el principio de
    // otra trama arrastrada. s_tramaTimestamps se desplaza en paralelo.
    memmove(s_tramaBuffer, s_tramaBuffer + tramaTotal, (size_t)(s_tramaLongitud - tramaTotal));
    memmove(s_tramaTimestamps, s_tramaTimestamps + tramaTotal,
            (size_t)(s_tramaLongitud - tramaTotal) * sizeof(uint32_t));
    // Los memmove() ya desplazaron los bytes; esta linea actualiza el
    // tamanyo de la trama. El cast es necesario porque la resta de dos
    // uint16_t se cambia a int antes de restar.
    s_tramaLongitud = (uint16_t)(s_tramaLongitud - tramaTotal);
    // Continua el bucle: podria haber otra trama completa arrastrada.
  }
}

//* =============================== API publica ================================

/* Implementacion de GpsUART_Init(). Documentacion en gps_uart.h */
void GpsUART_Init(void)
{
  Serial3.begin(GPS_BAUD_RATE);

  // attachInterruptVector() define como "handler" de la interrupcion
  // IRQ_LPUART2 a la funcion "isrGps()" (constante del NVIC del
  // Cortex-M7; ver tabla de vectores en el capitulo de interrupciones
  // del manual). NVIC_ENABLE_IRQ() habilita esa interrupcion a nivel
  // del NVIC. IMXRT_LPUART2.CTRL |= LPUART_CTRL_RIE habilita, dentro
  // del propio periferico LPUART2, el bit Receiver Interrupt Enable de
  // su registro CTRL (ver capitulo LPUART del manual) -- sin el, el
  // periferico no pide la interrupcion aunque el NVIC la tenga activa.
  // Los tres pasos son necesarios: periferico, NVIC y manejador.
  attachInterruptVector(IRQ_LPUART2, isrGps);
  NVIC_ENABLE_IRQ(IRQ_LPUART2);
  IMXRT_LPUART2.CTRL |= LPUART_CTRL_RIE; // Receiver Interrupt Enable

  s_estadoGps = GPS_VERIFICANDO_FLUJO;
  s_tramaLongitud = 0;
  s_idxEscritura = 0;
  s_idxLectura = 0;
  Serial.println("[DEBUG]: UART del GPS inicializada.");
}

/* Implementacion de GpsUART_Process(). Documentacion en gps_uart.h */
bool GpsUART_Process(bool &huboErrorSDGps)
{
  s_huboRegistroReciente = false;
  s_errorSDGpsDetectado = false;
  while (s_idxLectura != s_idxEscritura)
  {
    uint8_t dato;
    uint32_t t;

    // SECCION CRITICA: isrGps() puede ejecutarse en cualquier instante,
    // de forma asincrona respecto a este bucle. Sin esta proteccion,
    // existiria una ventana entre leer dato/timestamp_us y actualizar
    // s_idxLectura en la que la ISR podria escribir sobre el mismo
    // ByteEvent que se esta leyendo, dando una lectura corrupta
    // (campos mezclados de dos eventos distintos). Mismo criterio que
    // RadarBuffer_Pop() (radar_buffer.cpp); coste minimo (pocas
    // instrucciones).
    noInterrupts();
    dato = s_bufferBytes[s_idxLectura].dato;
    t = s_bufferBytes[s_idxLectura].timestamp_us;
    s_idxLectura = (uint16_t)((s_idxLectura + 1) & GPS_BUFFER_BYTES_MASK);
    interrupts();

    procesarByteGps(dato, t);

    // Serial.println("[DEBUG]: Byte de la UART del GPS procesado correctamente.");
  }
  huboErrorSDGps = s_errorSDGpsDetectado;
  return s_huboRegistroReciente;
}
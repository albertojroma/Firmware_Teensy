# Plan de pruebas — Firmware Teensy 4.1 (radar US-D1 + GPS-PPK ZED-F9P)

Plan de verificación en banco para la primera versión del firmware ya
compilada y cargada en la placa. Organizado en fases progresivas: cada
fase asume que las anteriores han pasado, y un fallo en una fase
temprana debe diagnosticarse ahí antes de avanzar.

## Tabla resumen

| Fase | Objetivo | Duración estimada | ¿Requiere GPS real? |
|---|---|---|---|
| 1. Arranque en banco | LEDs y comportamiento ante SD ausente/presente | 10 min | No |
| 2. GPS aislado — configuración | Máquina de estados VERIFICANDO_CONFIG → CONFIGURANDO → VERIFICANDO_FLUJO | 45 min | No (emulador) |
| 3. GPS aislado — registro | Validez de `gps_logger_N.ubx` y `correlacion_temporal_N.csv` | 5-10 min | No (emulador) |
| 4. Radar aislado | Referencia de comportamiento "solo radar" | 15 min | No |
| 5. Integración completa | Radar + GPS simultáneos, sin degradación mutua | 20 min | No (emulador) |
| 6. Validación post-proceso (opcional) | Regresión lineal timestamp_MCU ↔ rcvTow | 15 min | No |
| — | Precisión final de PPK (fuera de este plan) | — | **Sí, imprescindible** |

**Nota importante**: este plan valida que el firmware genera datos con
la *estructura* y el *comportamiento* correctos. La precisión real de
la solución PPK (centimétrica o no) solo puede confirmarse con datos de
un GPS físico y post-proceso en RTKLIB — el emulador Python genera
observables sintéticas, válidas para probar el parser pero no para
evaluar precisión de posicionamiento.

---

## Fase 1 — Arranque en banco, sin GPS ni radar conectados

### Prueba 1.1 — Comportamiento ante SD ausente

**Objetivo**: confirmar que `RadarLogger_Init()` reintenta correctamente
y que `LED_ERROR_SD_PIN` (pin 2) queda encendido mientras no haya SD.

**Precondiciones**: Teensy alimentada por USB, sin tarjeta SD insertada,
sin GPS ni radar conectados. Monitor serie abierto (115200 baudios).

**Pasos**:
1. Alimentar la Teensy (o pulsar reset).
2. Observar `LED_ERROR_SD_PIN` durante al menos 5 segundos.
3. Insertar la tarjeta SD sin reiniciar la placa.

**Criterio de éxito**:
- `LED_ERROR_SD_PIN` parpadea con el patrón de reintento (encendido
  ~400 ms, apagado brevemente, repetido) mientras no hay SD.
- Al insertar la SD, el firmware sale del bucle de espera en la
  siguiente pasada del `do/while` (puede tardar hasta ~400 ms desde la
  inserción) y `LED_ERROR_SD_PIN` se apaga.
- El monitor serie no muestra ningún mensaje hasta después de
  `RadarLogger_Init()` con éxito (el `Serial.begin()` ocurre antes, pero
  no hay ningún `Serial.println()` en el bucle de espera de SD).

**Si falla**: revisar que la tarjeta esté formateada en FAT32 y que
`BUILTIN_SDCARD` sea el modo correcto para su placa. No avanzar a la
Fase 2 hasta que esta prueba pase.

### Prueba 1.2 — Arranque normal con SD presente

**Objetivo**: confirmar el arranque completo hasta `loop()`, incluyendo
la inicialización del GPS.

**Precondiciones**: SD insertada y vacía (o sin ficheros `radar_logger_*.csv`
previos, para partir de `N=1`). Sin GPS ni radar conectados.

**Pasos**:
1. Reiniciar la Teensy.
2. Observar los tres LEDs durante 10 segundos.
3. Extraer la SD y comprobar su contenido en un PC.

**Criterio de éxito**:
- `LED_ERROR_SD_PIN` permanece apagado tras el arranque.
- `LED_ERROR_GPS_PIN` se enciende y **permanece encendido** (sin GPS
  físico ni emulador conectado, la fase `VERIFICANDO_CONFIG` nunca
  recibirá respuesta, agotará los 3 reintentos de `CONFIGURANDO`, y
  entrará en `GPS_ERROR` — comportamiento esperado, no un fallo).
- En la SD existen `radar_logger_1.csv` (con solo la fila de cabecera,
  al no haber radar conectado), `gps_logger_1.ubx` (vacío, 0 bytes) y
  `correlacion_temporal_1.csv` (con solo la fila de cabecera).
- El firmware sigue respondiendo (no se ha quedado bloqueado) — se
  puede comprobar reconectando el radar en caliente y viendo si
  `LED_ACTIVIDAD_PIN` reacciona.

**Si falla**: si `LED_ERROR_GPS_PIN` NO se enciende sin GPS conectado,
revisar el timeout de `GPS_TIMEOUT_MS` y la lógica de reintentos en
`avanzarPasoU1()`/`avanzarVerificandoConfig()` — podría estar
esperando indefinidamente en vez de fallar tras 3 intentos.

---

## Fase 2 — GPS aislado, fase de configuración

Conectar únicamente el GPS (emulador Python en la Raspberry Pi 4, pines
14/15 de la Teensy) — radar todavía desconectado, para aislar cualquier
problema de configuración del GPS sin interferencia del radar.

### Prueba 2.1 — Detección de que hace falta configurar

**Objetivo**: confirmar que `VERIFICANDO_CONFIG` detecta correctamente,
mediante `UBX-CFG-VALGET`, que el emulador (en estado de fábrica) no
tiene activado nada, y transiciona a `CONFIGURANDO`.

**Precondiciones**: emulador Python recién arrancado (`python3
emulador_gps.py`), sin ninguna configuración previa. Monitor serie de
la Teensy y terminal del emulador visibles a la vez.

**Pasos**:
1. Reiniciar la Teensy con el emulador ya corriendo.
2. Observar la terminal del emulador durante los primeros 2 segundos.

**Criterio de éxito**:
- La terminal del emulador muestra la recepción de un `UBX-CFG-VALGET`
  (verificar mediante un `print()` adicional en el emulador si no es ya
  visible — el emulador actual no imprime nada explícito al *responder*
  un VALGET más allá de `"[CFG] VALGET respondido"`, que sí debería
  aparecer).
- Justo después, la terminal del emulador muestra las tres líneas
  `[CFG] RXM-RAWX activado`, `[CFG] RXM-SFRBX activado`, `[CFG]
  Solicitud de cambio de baud rate a 230400` — confirma que la Teensy
  pasó a `CONFIGURANDO` en vez de saltarse esa fase.

**Si falla**: si la Teensy activa las claves sin haber consultado antes
(sin ver ningún log de VALGET), revisar `avanzarVerificandoConfig()` —
podría estar entrando directamente en `CONFIGURANDO` sin enviar el
`VALGET` inicial.

### Prueba 2.2 — Secuencia completa de `CONFIGURANDO`

**Objetivo**: confirmar que los tres pasos (`RAWX` → `SFRBX` →
`BAUDRATE`) se completan en orden, cada uno con su `ACK`.

**Precondiciones**: continuación de la prueba 2.1.

**Pasos**: observar la terminal del emulador hasta ver los tres
mensajes `[CFG] ...` y el de conmutación de baudrate.

**Criterio de éxito**:
- Los tres mensajes aparecen en el orden RAWX → SFRBX → BAUDRATE (no
  en otro orden, ni intercalados).
- Tras el mensaje de baudrate, aparece `[CFG] Baud rate del emulador
  conmutado a 230400` — confirma que el emulador aplicó el cambio
  después del ACK, no antes.
- El tiempo total desde el arranque de la Teensy hasta este punto es
  inferior a ~2 segundos (3 pasos × timeout máximo 300 ms si todo va
  bien a la primera, más margen).

**Si falla**: si algún paso no llega, comprobar con un analizador
lógico o el propio monitor serie de depuración (añadir temporalmente
`Serial.println()` en `avanzarPasoU1()`) en qué paso concreto se
detiene. Revisar que `KEY_MSGOUT_RXM_RAWX_UART1`/`SFRBX`/`BAUDRATE`
coincidan exactamente entre `gps_uart.cpp` y `emulador_gps.py`.

### Prueba 2.3 — Cambio de baudrate sin pérdida de sincronía

**Objetivo**: confirmar que, tras conmutar a 230400 baudios en ambos
lados, la comunicación sigue siendo válida (no hay tramas corruptas por
un desajuste de temporización durante la conmutación).

**Precondiciones**: continuación de la prueba 2.2, dejando correr 10
segundos más tras el cambio de baudrate.

**Pasos**: observar si el emulador empieza a recibir e imprimir avisos
de `[AVISO] KeyID desconocido...` (indicaría tramas corruptas
interpretadas erróneamente) tras el cambio de baudrate.

**Criterio de éxito**: no aparece ningún `[AVISO]` en los 10 segundos
posteriores al cambio de baudrate. El firmware avanza a
`VERIFICANDO_FLUJO` (no hay log explícito de esta transición
actualmente — @todo: añadir un `Serial.println()` temporal de
depuración en `gestionarTramaCompleta()` si se quiere confirmar
visualmente este instante exacto durante la prueba).

**Si falla**: revisar que `Serial3.begin(GPS_BAUD_OBJETIVO)` en
`avanzarPasoBaudrate()` se ejecute exactamente una vez, y no se repita
en cada llamada a `GpsUART_Process()` (reiniciar la UART repetidamente
podría causar la pérdida de bytes en tránsito).

### Prueba 2.4 — CASO CRÍTICO: comprobación previa tras reinicio de la Teensy

**Objetivo**: confirmar que `VERIFICANDO_CONFIG` detecta que el
emulador ya está configurado (de la prueba 2.2) y **no repite**
`CONFIGURANDO` — es el comportamiento que justifica la existencia de
este estado en el diseño.

**Precondiciones**: emulador Python **sin reiniciar** tras completar la
prueba 2.2 (debe seguir corriendo, con `RAWX`/`SFRBX` activos y a
230400 baudios).

**Pasos**:
1. Pulsar el botón físico de reset de la Teensy (no desconectar el
   emulador).
2. Observar la terminal del emulador.

**Criterio de éxito**: **no** aparece ninguno de los tres mensajes
`[CFG] ...` de activación tras el reset — solo el `VALGET` de
comprobación (o ni siquiera un log visible de él, según cómo se
instrumente). El tiempo hasta que el firmware llegue a
`VERIFICANDO_FLUJO`/`REGISTRANDO` debe ser sensiblemente menor que en
la prueba 2.2 (un único `VALGET` con respuesta, en vez de tres
`VALSET` secuenciales con sus `ACK`).

**Si falla**: si el firmware repite `CONFIGURANDO` a pesar de que el
emulador ya tenía todo activado, el fallo más probable está en
`procesarRespuestaValget()` — revisar que la comparación de los 3
valores (`rawxOk && sfrbxOk && baudOk`) sea correcta y que el baudrate
objetivo comparado coincida exactamente con `GPS_BAUD_OBJETIVO`.

### Prueba 2.5 — CASO NEGATIVO: fallo de configuración forzado

**Objetivo**: confirmar que, si el GPS no responde durante
`CONFIGURANDO`, el firmware entra en `GPS_ERROR` tras 3 reintentos, sin
bloquearse.

**Precondiciones**: emulador Python detenido (`Ctrl+C`) o desconectado
físicamente, Teensy sin reiniciar todavía (o recién reiniciada sin
emulador).

**Pasos**:
1. Reiniciar la Teensy sin el emulador conectado/corriendo.
2. Medir con un cronómetro el tiempo hasta que `LED_ERROR_GPS_PIN` se
   enciende de forma sostenida.
3. Tras ese encendido, reconectar el radar (si estuviera disponible) y
   comprobar que `LED_ACTIVIDAD_PIN` sigue respondiendo con normalidad.

**Criterio de éxito**:
- `LED_ERROR_GPS_PIN` se enciende entre ~0.9 s y ~4 s tras el arranque
  (1 intento de `VALGET` con timeout de 300 ms, más hasta 3 intentos ×
  3 pasos × 300 ms en el peor caso si `VERIFICANDO_CONFIG` decide
  configurar igualmente — @todo: el rango exacto depende de si
  `VERIFICANDO_CONFIG` cuenta como un reintento adicional o no; medir
  empíricamente y documentar el valor real observado).
- El firmware **no se bloquea**: el radar (si está conectado) sigue
  registrando con normalidad, confirmando el carácter no bloqueante de
  `GPS_ERROR`.

**Si falla**: si `LED_ERROR_GPS_PIN` nunca se enciende (espera
indefinida), revisar los contadores `s_reintentos` en `gps_uart.cpp` —
podrían no incrementarse correctamente en el camino de timeout.

---

## Fase 3 — GPS aislado, fase de registro

**Precondiciones para toda la fase**: emulador Python arrancado desde
cero (`--num_sats 10 --ttff 30`, valores por defecto), Teensy reiniciada
con el emulador ya corriendo. Dejar correr al menos 45 segundos (30 s
de rampa + margen) antes de extraer la SD.

### Prueba 3.1 — Validez del fichero binario `.ubx`

**Objetivo**: confirmar que `gps_logger_N.ubx` es una secuencia válida
de tramas UBX, sin corrupción.

**Pasos**: extraer la SD, abrir `gps_logger_N.ubx` con u-center (`File
→ Open Log File`) o intentar convertirlo con `RTKCONV` de RTKLIB.

**Criterio de éxito**: u-center reproduce el fichero mostrando mensajes
`UBX-RXM-RAWX` y `UBX-RXM-SFRBX` reconocidos, sin errores de checksum
reportados. `RTKCONV` genera un fichero `.obs` sin errores de parseo
(aunque el contenido de las observables sea sintético y no produzca una
solución PPK con sentido físico).

**Si falla**: si u-center reporta tramas corruptas, revisar
`GpsLogger_GuardarTramaCruda()` — comprobar que `longitud` coincide
exactamente con los bytes realmente escritos (`s_archivoUbx.write()`
podría no escribir todos los bytes si el buffer interno de la librería
SD se llenara, aunque es poco probable a este ritmo).

### Prueba 3.2 — Monotonía de `correlacion_temporal_N.csv`

**Objetivo**: confirmar que las columnas `timestamp_us` y `rcvTow`
crecen de forma estrictamente monótona, sin saltos hacia atrás ni
repeticiones.

**Pasos**: abrir `correlacion_temporal_N.csv` en una hoja de cálculo o
un script Python; comprobar que cada fila tiene `timestamp_us` y
`rcvTow` mayores que la fila anterior.

**Criterio de éxito**: monotonía estricta en ambas columnas, en todas
las filas. El incremento de `rcvTow` entre filas consecutivas debe ser
razonablemente constante (el emulador emite a 5 Hz una vez configurado,
así que el incremento esperado es de ~0.2 s entre filas — @todo:
verificar contra la tasa real configurada en el emulador si se ha
cambiado del valor por defecto).

**Si falla**: si `timestamp_us` no es monótono, revisar que
`micros()` no se esté desbordando de forma no gestionada (se desborda
cada ~71 minutos; para una prueba de 45 segundos no debería ser la
causa, pero merece la pena descartarlo). Si `rcvTow` no es monótono,
el problema está en el propio emulador o en la extracción de bytes
(offset 6 del payload) en `gestionarTramaCompleta()`.

### Prueba 3.3 — Rampa de satélites

**Objetivo**: confirmar que el número de mediciones por trama `RAWX`
(`numMeas`) crece progresivamente durante los primeros ~30 s, en vez de
aparecer siempre al máximo — valida que el firmware no asume un tamaño
fijo de trama.

**Pasos**: en u-center, con el log cargado, observar el campo `numMeas`
de los mensajes `UBX-RXM-RAWX` a lo largo del tiempo (o extraerlo con
un script analizando `gps_logger_N.ubx` byte a byte).

**Criterio de éxito**: `numMeas` empieza en 1 y crece hasta 10 (el
`--num_sats` por defecto) a lo largo de los primeros ~30 segundos,
coincidiendo con `--ttff`. No debe verse `numMeas=10` desde la primera
trama.

**Si falla**: si `numMeas` es siempre 10 desde el principio, el
problema está en el propio emulador (`num_satelites_activos()`), no en
el firmware — descartar primero revisando la salida cruda del
emulador antes de sospechar del firmware.

---

## Fase 4 — Radar aislado

### Prueba 4.1 — Referencia de comportamiento "solo radar"

**Objetivo**: obtener una línea base de `radar_logger_N.csv` sin GPS
conectado, para comparar contra la Fase 5.

**Precondiciones**: solo el radar conectado (Serial1, pines TX1/RX1),
sin GPS ni emulador.

**Pasos**: dejar registrar 30 segundos, extraer la SD, contar filas de
`radar_logger_N.csv` y contrastar contra la duración real cronometrada.

**Criterio de éxito**: número de filas ≈ 100 × segundos transcurridos,
con una tolerancia razonable (@todo: definir el margen de tolerancia
exacto tras la primera medida empírica — no se ha establecido en este
proyecto un porcentaje de pérdida de tramas aceptable).

**Si falla**: si ya existe una validación equivalente de una iteración
anterior del proyecto, referenciarla en vez de repetirla.

---

## Fase 5 — Integración completa (radar + GPS simultáneos)

**Precondiciones para toda la fase**: radar y emulador GPS conectados a
la vez, SD vacía o con numeración limpia.

### Prueba 5.1 — Tasa de registro del radar sin degradación

**Objetivo**: confirmar que `GpsUART_Process()` en `loop()` no ralentiza
lo suficiente el consumo del buffer del radar como para perder tramas.

**Pasos**: repetir la prueba 4.1 exactamente igual, pero con el GPS
también conectado y registrando.

**Criterio de éxito**: número de filas de `radar_logger_N.csv` dentro
del mismo margen de tolerancia que en la prueba 4.1 (comparar
directamente los dos resultados, no solo contra el valor teórico de
100 Hz).

**Si falla**: indica que `GpsUART_Process()` está tardando demasiado en
alguna iteración de `loop()` (posiblemente el reensamblado de una trama
`RAWX` larga) — revisar con un pin de depuración (toggle antes/después
de `GpsUART_Process()` y medir con osciloscopio o analizador lógico) 
cuánto tarda esa función en el peor caso.

### Prueba 5.2 — Coherencia de la numeración compartida

**Objetivo**: confirmar que `radar_logger_N.csv`, `gps_logger_N.ubx` y
`correlacion_temporal_N.csv` comparten el mismo `N` tras el mismo
arranque.

**Pasos**: extraer la SD tras la prueba 5.1, comprobar los tres nombres
de fichero.

**Criterio de éxito**: los tres ficheros generados en la misma sesión
tienen exactamente el mismo número `N`.

**Si falla**: revisar la llamada a `RadarLogger_ObtenerNumeroVuelo()`
en `setup()` — debe ocurrir después de un `RadarLogger_Init()` exitoso
y antes de `GpsLogger_Init()`.

### Prueba 5.3 — LED de actividad sin interferencia visible

**Objetivo**: confirmar que el parpadeo de `LED_ACTIVIDAD_PIN` es
indistinguible, a simple vista, entre la Fase 4 (solo radar) y esta
fase (radar + GPS).

**Pasos**: observación visual comparativa, sin instrumentación.

**Criterio de éxito**: valoración cualitativa — el LED no muestra
parones ni parpadeos irregulares nuevos al añadir el GPS.

---

## Fase 6 — Validación de datos post-proceso (opcional)

### Prueba 6.1 — Regresión lineal timestamp_MCU ↔ rcvTow

**Objetivo**: confirmar que los datos de `correlacion_temporal_N.csv`
son aptos para el ajuste por mínimos cuadrados que se aplicará en
post-proceso real.

**Pasos**: cargar `correlacion_temporal_N.csv` de la prueba de la Fase
3 en un script Python, ejecutar `numpy.polyfit(timestamp_us, rcvTow, 1)`.

**Criterio de éxito**: el ajuste se ejecuta sin excepciones. La
pendiente resultante está razonablemente próxima a la relación de
unidades esperada (rcvTow en segundos, timestamp_us en microsegundos →
pendiente teórica ≈ 1×10⁻⁶, con una desviación pequeña reflejando la
diferencia de frecuencia real entre el cristal de la Teensy y el reloj
GPS). @todo: no se ha establecido en este proyecto un margen de
tolerancia cuantitativo para esa desviación — definir tras la primera
medida empírica con GPS real, no con el emulador (el emulador no
modela deriva de reloj alguna).

---

## Registro de resultados

Se recomienda documentar, para cada prueba, además del resultado
PASA/FALLA: fecha, versión exacta del firmware (commit de git), y
cualquier valor cuantitativo medido — especialmente los marcados con
`@todo` en este plan, que deben convertirse en criterios concretos una
vez se disponga de la primera medida empírica.

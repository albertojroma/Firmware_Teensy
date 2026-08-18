# Plan de pruebas — Firmware Teensy 4.1 (radar US-D1 + GPS-PPK ZED-F9P)
## Versión 2 — arquitectura simplificada (sin configuración del GPS por firmware)

Este plan sustituye por completo al anterior. Cambios de fondo respecto a
la versión previa del firmware:
- El GPS ya **no se configura desde la Teensy**: el receptor debe estar
  configurado de antemano, de forma **permanente**, con `u-center`
  (`RAWX`/`SFRBX` activos, 115200 baudios, guardado en capa `Flash`).
- Ya no existe ningún estado de error de configuración del GPS
  (`GPS_ERROR` ha desaparecido). `LED_ERROR_GPS_PIN` ahora refleja
  **únicamente** fallos de escritura en la SD del propio GPS.
- Ya no hay fichero `correlacion_temporal_N.csv` aparte: las filas de
  sincronización del GPS viven dentro de `data_logger_N.csv`, con una
  columna `tipo` (`RADAR`/`GPS`).
- Solo quedan **dos** ficheros por sesión: `data_logger_N.csv`
  (combinado) y `gps_logger_N.ubx`.

## Tabla resumen

| Fase | Objetivo | Duración estimada | ¿Requiere GPS real? |
|---|---|---|---|
| 0. Configuración permanente del GPS | Activar RAWX/SFRBX en Flash con u-center | 15-20 min | **Sí, imprescindible** |
| 1. Arranque en banco | LEDs y comportamiento ante SD ausente/presente | 10 min | No |
| 2. GPS aislado (emulador) | Detección de flujo válido, transición a registro | 15 min | No (emulador) |
| 3. GPS aislado — validez de datos | Estructura de `gps_logger_N.ubx` y filas `GPS` del CSV | 5-10 min | No (emulador) |
| 4. Radar aislado | Referencia de comportamiento "solo radar" | 15 min | No |
| 5. Integración completa | Radar + GPS simultáneos, sin degradación mutua | 20 min | No (emulador) |
| 6. GPS real, configuración permanente | Confirmar que el receptor real emite sin handshake | 15 min | **Sí, imprescindible** |
| 7. Validación post-proceso (opcional) | Regresión lineal timestamp_MCU ↔ rcvTow | 15 min | No |
| — | Precisión final de PPK (fuera de este plan) | — | **Sí, imprescindible** |

---

## Fase 0 — Configuración permanente del GPS (prerrequisito de todo lo demás)

**Objetivo**: dejar el receptor ZED-F9P configurado de forma que
sobreviva a cortes de alimentación, sin que el firmware tenga que
pedirle nada.

**Pasos**:
1. Conecte el módulo GPS directamente a un PC por USB (o al puerto que
   use habitualmente para `u-center`), **sin la Teensy de por medio**.
2. Abra `u-center`. Confirme que reconoce el receptor y ve tramas
   entrantes.
3. `Tools → Receiver Configuration` — active `UBX-RXM-RAWX` y
   `UBX-RXM-SFRBX` en `UART1`, y fije el baudrate de `UART1` a 115200.
   (Puede partir del fichero de ejemplo "Procesamiento posterior de
   datos RAW (PPK)" de ArduSimple como plantilla, ajustando el
   baudrate si no coincide.)
4. `Receiver → Action → Save Config` — confirma el guardado permanente
   en `Flash`.
5. **Desconecte la alimentación del receptor por completo** (no solo
   cierre `u-center` — retire el USB), espere unos segundos, y vuelva
   a conectarlo.
6. En `u-center`, confirme que `RAWX`/`SFRBX` siguen activos y el
   baudrate sigue en 115200 **sin haber enviado ninguna configuración
   nueva** — es la prueba real de que quedó guardado en `Flash`, no
   solo en `RAM`.

**Criterio de éxito**: tras el ciclo de apagado/encendido, el receptor
sigue emitiendo `RAWX`/`SFRBX` a 115200 sin intervención.

**Si falla**: si la configuración no sobrevive al corte de
alimentación, el receptor podría no tener realmente disponible la capa
`Flash` a pesar de lo indicado en su documentación — en ese caso, antes
de continuar con el resto del plan, habría que reconsiderar el diseño
(por ejemplo, evaluar si el Holybro Ultralight expone el pin `V_BCKP`
para al menos persistencia por batería `BBR`, o si haría falta volver a
una configuración por firmware, con las implicaciones ya conocidas).

---

## Fase 1 — Arranque en banco, sin GPS ni radar conectados

### Prueba 1.1 — Comportamiento ante SD ausente

**Objetivo**: confirmar que `DataLogger_Init()` reintenta correctamente
y que `LED_ERROR_SD_PIN` (pin 2) queda encendido mientras no haya SD.

**Precondiciones**: Teensy alimentada por USB, sin tarjeta SD insertada,
sin GPS ni radar conectados.

**Pasos**: alimentar la Teensy, observar `LED_ERROR_SD_PIN` durante al
menos 5 segundos, insertar la SD sin reiniciar.

**Criterio de éxito**: `LED_ERROR_SD_PIN` parpadea con el patrón de
reintento mientras no hay SD, y se apaga en cuanto se inserta.

### Prueba 1.2 — Arranque normal con SD presente

**Objetivo**: confirmar el arranque completo hasta `loop()`.

**Precondiciones**: SD insertada y vacía. Sin GPS ni radar conectados.

**Pasos**: reiniciar la Teensy, observar los LEDs durante 10 segundos,
extraer la SD y comprobar su contenido.

**Criterio de éxito**, **distinto del plan anterior** —léalo con
atención, ha cambiado con el rediseño:
- `LED_ERROR_SD_PIN` permanece apagado.
- **`LED_ERROR_GPS_PIN` también debe permanecer apagado** — a
  diferencia de la versión anterior del firmware, `GpsLogger_Init()` no
  depende en absoluto de que haya un GPS físico conectado, solo de que
  la SD funcione; sin ningún fallo de SD, este LED no tiene ningún
  motivo para encenderse, con o sin GPS conectado.
- En la SD existen `data_logger_1.csv` (con la cabecera de 9 columnas,
  incluida la columna `tipo`) y `gps_logger_1.ubx` (vacío, 0 bytes).

**Si falla**: si `LED_ERROR_GPS_PIN` se enciende sin que haya ningún
problema de SD, revisar `GpsLogger_Init()` — podría estar fallando por
algún motivo no relacionado con el GPS en sí (por ejemplo, algún límite
de ficheros abiertos simultáneos, aunque ya se investigó extensamente
que dos ficheros no presentan ese problema en la versión actual).

---

## Fase 2 — GPS aislado, con el emulador (sin handshake de configuración)

Conecte únicamente el GPS (emulador Python en la Raspberry Pi 4, pines
14/15 de la Teensy) — radar todavía desconectado.

### Prueba 2.1 — Transición a registro sin ningún envío previo

**Objetivo**: confirmar que, sin que la Teensy envíe nada, en cuanto
llega la primera `RAWX` válida del emulador, el firmware pasa a
`GPS_REGISTRANDO`.

**Precondiciones**: emulador arrancado (`python3 emulador_gps.py`),
Teensy reiniciada con el emulador ya corriendo.

**Pasos**: dejar correr 5-10 segundos, extraer la SD.

**Criterio de éxito**: `data_logger_N.csv` contiene filas de tipo
`GPS` desde prácticamente el primer segundo (el emulador ya emite
`RAWX`/`SFRBX` desde el arranque, sin ningún estado de fábrica que
esperar). `gps_logger_N.ubx` no está vacío.

**Si falla**: si no aparece ninguna fila `GPS`, revisar el cableado
(pines 14/15, cruce TX↔RX) y el baudrate configurado en el emulador
(`--baudrate`, por defecto 115200 — debe coincidir exactamente con
`GPS_BAUD_RATE` en `gps_uart.cpp`).

---

## Fase 3 — GPS aislado, validez de los datos registrados

**Precondiciones**: emulador arrancado con los valores por defecto
(`--num_sats 10 --ttff 30`), Teensy reiniciada con el emulador ya
corriendo. Dejar correr al menos 45 segundos antes de extraer la SD.

### Prueba 3.1 — Validez del fichero binario `.ubx`

**Objetivo**: confirmar que `gps_logger_N.ubx` es una secuencia válida
de tramas UBX.

**Pasos**: abrir con u-center (`File → Open Log File`) o convertir con
`RTKCONV` de RTKLIB.

**Criterio de éxito**: mensajes `UBX-RXM-RAWX`/`SFRBX` reconocidos sin
errores de checksum. `RTKCONV` genera un `.obs` sin errores de parseo.

### Prueba 3.2 — Monotonía de las filas `GPS` en el CSV combinado

**Objetivo**: confirmar que `timestamp_us` y `rcvTow` crecen de forma
estrictamente monótona en las filas de tipo `GPS`.

**Pasos**: abrir `data_logger_N.csv`, filtrar por `tipo == "GPS"`,
comprobar la monotonía de ambas columnas.

**Criterio de éxito**: monotonía estricta. Incremento de `rcvTow` entre
filas consecutivas ≈ 1 s, la tasa adoptada como criterio de diseño para
validar primero el funcionamiento del sistema; la mejora de la tasa de
refresco queda como trabajo futuro.

### Prueba 3.3 — Rampa de satélites

**Objetivo**: confirmar que `numMeas` de las tramas `RAWX` crece
progresivamente durante los primeros ~30 s.

**Pasos**: en u-center, observar el campo `numMeas` a lo largo del
tiempo.

**Criterio de éxito**: `numMeas` empieza en 1 y crece hasta 10 a lo
largo de los primeros ~30 segundos.

---

## Fase 4 — Radar aislado

### Prueba 4.1 — Referencia de comportamiento "solo radar"

**Objetivo**: obtener una línea base de las filas `RADAR` en
`data_logger_N.csv` sin GPS conectado.

**Precondiciones**: solo el radar conectado, sin GPS ni emulador.

**Pasos**: registrar 30 segundos, extraer la SD, contar filas de tipo
`RADAR` y contrastar contra la duración cronometrada.

**Criterio de éxito**: número de filas `RADAR` ≈ 100 × segundos
transcurridos, con tolerancia razonable.

---

## Fase 5 — Integración completa (radar + GPS simultáneos)

**Precondiciones**: radar y emulador GPS conectados a la vez.

### Prueba 5.1 — Tasa de registro del radar sin degradación

**Objetivo**: confirmar que `GpsUART_Process()` no ralentiza el
consumo del buffer del radar.

**Pasos**: repetir la prueba 4.1, pero con el GPS también conectado.

**Criterio de éxito**: número de filas `RADAR` dentro del mismo margen
que en la prueba 4.1.

### Prueba 5.2 — Coherencia de la numeración compartida

**Objetivo**: confirmar que `data_logger_N.csv` y `gps_logger_N.ubx`
comparten el mismo `N`.

**Criterio de éxito**: mismo número `N` en ambos ficheros de la misma
sesión.

### Prueba 5.3 — LED de actividad sin interferencia visible

**Objetivo**: confirmar que `LED_ACTIVIDAD_PIN` se comporta igual que
en la Fase 4, sin parones nuevos al añadir el GPS.

---

## Fase 6 — GPS real, confirmación de funcionamiento sin handshake

**Esta fase es nueva respecto al plan anterior, y es la más importante
del rediseño.**

### Prueba 6.1 — El firmware recibe datos del receptor real sin enviar nada

**Objetivo**: confirmar, con hardware real (no el emulador), que un
receptor configurado según la Fase 0 empieza a registrar sin que la
Teensy tenga que enviarle nada.

**Precondiciones**: GPS real, ya configurado y verificado en la Fase 0,
conectado a los pines 14/15 de la Teensy con antena y buena vista del
cielo.

**Pasos**: alimentar el conjunto, esperar al TTFF real (hasta ~30 s en
frío), extraer la SD tras unos minutos.

**Criterio de éxito**: aparecen filas `GPS` en `data_logger_N.csv` y
contenido real en `gps_logger_N.ubx`, con datos de satélites reales
(no sintéticos como en el emulador).

**Si falla**: revisar de nuevo la Fase 0 (¿sigue realmente configurado
tras el último corte de alimentación?) antes de sospechar del
firmware — dado todo lo investigado en este proyecto, es más probable
un problema de configuración del receptor que del propio código en
este punto.

---

## Fase 7 — Validación de datos post-proceso (opcional)

### Prueba 7.1 — Regresión lineal timestamp_MCU ↔ rcvTow

**Objetivo**: confirmar que las filas `GPS` de `data_logger_N.csv`
son aptas para el ajuste por mínimos cuadrados del post-proceso real.

**Pasos**: cargar `data_logger_N.csv`, filtrar por `tipo == "GPS"`,
ejecutar `numpy.polyfit(timestamp_us, rcvTow, 1)`.

**Criterio de éxito**: ajuste sin excepciones, pendiente cercana a
1×10⁻⁶ (µs → s).

---

## Registro de resultados

Para cada prueba: fecha, versión del firmware (commit de git), y
cualquier valor cuantitativo medido — especialmente los marcados con
`@todo`, que deben convertirse en criterios concretos tras la primera
medida empírica.
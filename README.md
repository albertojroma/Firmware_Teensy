# TFM Firmware Teensy — Estimación de lámina de agua (Albufera)

Firmware para Teensy 4.1 encargado de la adquisición sincronizada de datos
del radar altimétrico Ainstein US-D1 (UART, 100 Hz) y del receptor GNSS
Holybro H-RTK ZED-F9P (UBX, PPK), en el marco del TFM de estimación de
lámina de agua en arrozales de la Albufera mediante UAV.

## Repositorios relacionados

- Emulador HIL (Raspberry Pi 4): [albertojroma/Emulador_RPi4](https://github.com/albertojroma/Emulador_RPi4)
  — Emula ambos sensores para validar este firmware sin hardware físico.

## Estructura

- `src/` — Código fuente (FSM, parsers de radar y GPS, logging SD)
- `include/` — Cabeceras compartidas

## Entorno

- Plataforma: Teensy 4.1
- Framework: Arduino (vía PlatformIO)

## Generación de documentación técnica (Doxygen)

Para regenerar la documentación del código:

```
cd docs
doxygen Doxyfile
```

La documentación se genera en `docs/html/index.html`.

## Conexiones con el emulador

Los siguientes LEDs de estado son parte del hardware del sistema y no son
exclusivos de las pruebas con el emulador; se comportan igual con el radar
y el GPS reales.

| LED | Pin (Teensy) | Función |
|---|---|---|
| Actividad radar | 13 | Se alterna con cada trama del radar registrada correctamente en la SD |
| Actividad GPS | 6 | Se alterna con cada trama UBX-RXM-RAWX/SFRBX válida reensamblada |
| Error SD — CSV | 2 | Encendido durante la espera si la SD no se detecta al arrancar (se apaga al detectarla y continúa la ejecución normal); enclavado (permanece encendido hasta el reinicio) si falla una escritura del `.csv` ya en funcionamiento |
| Error SD — GPS (`.ubx`) | 4 | Enclavado: permanece encendido hasta el reinicio tras un fallo de escritura del `.ubx` |

Para que todo funcione correctamente con el emulador se deben realizar las
conexiones de acuerdo a las imágenes inferior donde:
* El TX del emulador del radar (PIN 8 en la RPi4, GPIO14/TXD0 de UART0) se conecta al RX del radar en la Teensy 4.1 (**PIN 0**, RX1/Serial1).
* El TX del emulador del GPS (PIN 7 en la RPi4, GPIO4/TXD3 de UART3) se conecta al RX del GPS-PPK en la Teensy 4.1 (**PIN 15**, RX3/Serial3).

| Pinout RPi4 | Pinout Teensy |
|-------------|---------------|
| ![Pinout_RPi4](docs/imgs/Pinout_RPi4.png) | ![Pinout_Teensy](docs/imgs/Pinout_Teensy.png) |
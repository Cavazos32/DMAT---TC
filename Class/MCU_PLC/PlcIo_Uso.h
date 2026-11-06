#pragma once

/*
 * PlcIo_Uso.h — MCU PLCA (ESP32) ↔ sensores + válvulas neumáticas (GPIO)
 *
 * No hay Modbus ni CAN hacia el PLC: entradas digitales y salidas por pulso.
 * Órdenes por Serial. Sin WiFi.
 * Referencia: Parameters.h · PlcStates.h · MCU_PLC.ino
 *
 * Segmentos:
 *   [1] Enlace físico (pines entrada/salida)
 *   [2] Códigos lógicos (bytes PlcStates — solo etiqueta, no trama serial)
 *   [3] Salidas válvulas — protocolo de pulso
 *   [4] Entradas sensores — lectura y debounce
 *   [5] Blower y reset
 *   [6] Home / all off
 *   [7] Ejemplos numéricos
 */

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// [1] Enlace físico — MCU ↔ I/O
// ═══════════════════════════════════════════════════════════════════════════
//
// VALVE_ACTIVE_HIGH = 1 (Parameters.h): pulso activo = GPIO HIGH.
//
// Entradas (sensor cerrado / falla → lectura LOW → bit en máscara):
//
// | Señal    | GPIO | Bit máscara | Byte error (solo reporte lógico) |
// |----------|------|-------------|----------------------------------|
// | Cutter   |  26  | BIT_CUTTER  | 0x1F                             |
// | Gripper  |  27  | BIT_GRIPPER | 0x20                             |
// | Holder   |  32  | BIT_HOLDER  | 0x21                             |
// | Encoder* |  34  | BIT_ENCODER | 0x22                             |
//
// * GPIO 26, 27 y 32: pull-up interno. Activo en LOW.
// * GPIO34: input-only, sin pull-up interno — pull-up externa en PCB.
//
// Salidas (válvula / relé — pulso KEEP neumático):
//
// | Función     | GPIO | Byte cmd (referencia) |
// |-------------|------|------------------------|
// | Cutter R    |  25  | 0x19                   |
// | Cutter L    |  14  | 0x1A                   |
// | Gripper     |  16  | 0x1B                   |
// | Holder      |  17  | 0x1C                   |
// | Encoder tray|  21  | 0x1D                   |
// | Reset PLC   |  33  | 0x1E                   |
// | Blower      |   4  | 0x23 (nivel temporal)  |
//
// Combinados (un solo pulso paralelo en varios GPIO):
//   0x2A — Holder + Encoder (pines 17 y 21 a la vez)
//   0x2B — Cutter R + L (pines 25 y 14) — uso interno setOut "CUTTERS"

// Si Parameters.h ya definió estos nombres, se usan esos (una sola fuente).

#ifndef PLC_PIN_IN_CUTTER
static const uint8_t PLC_PIN_IN_CUTTER  = 26;
static const uint8_t PLC_PIN_IN_GRIPPER = 27;
static const uint8_t PLC_PIN_IN_HOLDER  = 32;
static const uint8_t PLC_PIN_IN_ENCODER = 34;

static const uint8_t PLC_PIN_OUT_CUTTER_R = 25;
static const uint8_t PLC_PIN_OUT_CUTTER_L = 14;
static const uint8_t PLC_PIN_OUT_GRIPPER  = 16;
static const uint8_t PLC_PIN_OUT_HOLDER   = 17;
static const uint8_t PLC_PIN_OUT_ENCODER  = 21;
static const uint8_t PLC_PIN_OUT_BLOWER   = 4;
static const uint8_t PLC_PIN_OUT_RESET    = 33;

static const uint8_t PLC_BIT_GRIPPER = (1 << 0);
static const uint8_t PLC_BIT_HOLDER  = (1 << 1);
static const uint8_t PLC_BIT_CUTTER  = (1 << 2);
static const uint8_t PLC_BIT_ENCODER = (1 << 5);
#endif

// ═══════════════════════════════════════════════════════════════════════════
// [2] Códigos lógicos (PlcStates.h — mapa nombre ↔ byte)
// ═══════════════════════════════════════════════════════════════════════════

#ifndef PLC_CMD_CUTTER_R
static const uint8_t PLC_CMD_CUTTER_R        = 0x19;
static const uint8_t PLC_CMD_CUTTER_L        = 0x1A;
static const uint8_t PLC_CMD_GRIPPER         = 0x1B;
static const uint8_t PLC_CMD_HOLDER          = 0x1C;
static const uint8_t PLC_CMD_ENCODER_VALVE   = 0x1D;
static const uint8_t PLC_CMD_RESET           = 0x1E;
static const uint8_t PLC_CMD_BLOWER          = 0x23;
static const uint8_t PLC_CMD_HOLDER_ENCODER  = 0x2A;
static const uint8_t PLC_CMD_CUTTERS         = 0x2B;

static const uint8_t PLC_ERR_CUTTER  = 0x1F;
static const uint8_t PLC_ERR_GRIPPER = 0x20;
static const uint8_t PLC_ERR_HOLDER  = 0x21;
static const uint8_t PLC_ERR_ENCODER = 0x22;
#endif

// ═══════════════════════════════════════════════════════════════════════════
// [3] Válvulas — protocolo de pulso (MCU → coil/driver)
// ═══════════════════════════════════════════════════════════════════════════
//
// Las válvulas NO se dejan en HIGH continuo: cada cambio lógico ON↔OFF genera
// un impulso en el GPIO (KEEP neumático enclava en el relé).
//
// Parámetros (Parameters.h):
#ifndef PLC_VALVE_PULSE_MS
static const unsigned long PLC_VALVE_PULSE_MS     = 100;
static const unsigned long PLC_VALVE_PULSE_GAP_MS = 50;
#endif

// Operación VALVE_SET(logicalName, wantOn):
//   1) Si estado lógico ya == wantOn → no pulso (opcional no-op).
//   2) Si no: encolar pulso → GPIO HIGH durante PLC_VALVE_PULSE_MS → LOW.
//   3) Esperar ≥ PLC_VALVE_PULSE_GAP_MS antes del siguiente pulso (otro GPIO).
//   4) Actualizar estado lógico interno (stCutterR, stGrippers, …).
//
// Integrador en otro MCU: replicar máquina IDLE → GAP → HIGH(100ms) → LOW.

#ifndef VALVE_ACTIVE_HIGH
#define VALVE_ACTIVE_HIGH 1
#endif

#if VALVE_ACTIVE_HIGH
#define PLC_VALVE_GPIO_ON(pin)  digitalWrite((pin), HIGH)
#define PLC_VALVE_GPIO_OFF(pin) digitalWrite((pin), LOW)
#else
#define PLC_VALVE_GPIO_ON(pin)  digitalWrite((pin), LOW)
#define PLC_VALVE_GPIO_OFF(pin) digitalWrite((pin), HIGH)
#endif

// Boot: todas las salidas válvula en idle OFF; estados lógicos en false.

// ═══════════════════════════════════════════════════════════════════════════
// [4] Sensores — lectura (drive → MCU)
// ═══════════════════════════════════════════════════════════════════════════

#ifndef PLC_SENSOR_DEBOUNCE_MS
static const unsigned long PLC_SENSOR_DEBOUNCE_MS = 150;
static const unsigned long PLC_BOOT_GRACE_MS      = 15000;
#endif

// READ_SENSORS_RAW():
//   mask = 0;
//   if (digitalRead(PIN_CUTTER)  == LOW) mask |= PLC_BIT_CUTTER;
//   if (digitalRead(PIN_GRIPPER) == LOW) mask |= PLC_BIT_GRIPPER;
//   if (digitalRead(PIN_HOLDER)  == LOW) mask |= PLC_BIT_HOLDER;
//   if (digitalRead(PIN_ENCODER)== LOW) mask |= PLC_BIT_ENCODER;
//   return mask;
//
// Debounce: el mask debe ser estable PLC_SENSOR_DEBOUNCE_MS antes de aceptar cambio.
// Tras boot, ignorar fallas encoder en 34 hasta haber visto HIGH al menos una vez
// (encoderFaultArmed en firmware).
// Durante PLC_BOOT_GRACE_MS no emitir errores de sensor por arranque lento.

// ═══════════════════════════════════════════════════════════════════════════
// [5] Blower y reset hardware
// ═══════════════════════════════════════════════════════════════════════════
//
// Blower (GPIO 4): distinto a válvulas — nivel HIGH sostenido, luego OFF auto.
//   BLOWER_ON(durationMs): default 2000 ms, min 200, max 300000.
//   BLOWER_OFF(): GPIO LOW de inmediato.
//
// RESET (0x1E, GPIO 33):
//   Abortar pulsos pendientes; estados lógicos válvulas → false; blower off;
//   encolar pulso 100 ms en PIN_OUT_RESET (mismo timing [3]).

#ifndef PLC_BLOWER_DEFAULT_MS
static const unsigned long PLC_BLOWER_DEFAULT_MS = 2000;
static const unsigned long PLC_BLOWER_MIN_MS     = 200;
static const unsigned long PLC_BLOWER_MAX_MS     = 300000;
#endif

// ═══════════════════════════════════════════════════════════════════════════
// [6] Home / all off (desactivar todo lo que esté ON lógico)
// ═══════════════════════════════════════════════════════════════════════════
//
// Para cada válvula con estado lógico ON: encolar pulso OFF (mismo [3]), una
// tras otra respetando GAP. Orden típico: Cutter R/L, Gripper, Encoder tray, Holder.
// Blower: stop inmediato sin pulso KEEP.

// ═══════════════════════════════════════════════════════════════════════════
// [7] Ejemplos numéricos
// ═══════════════════════════════════════════════════════════════════════════
//
// ── Ejemplo A: Gripper ON (cmd 0x1B, wantOn = true) ──
//   Estado previo: stGrippers = false
//   Acción MCU:
//     t=0 ms:    digitalWrite(16, HIGH)     // PLC_VALVE_GPIO_ON
//     t=100 ms:  digitalWrite(16, LOW)
//     stGrippers = true
//   No mantener HIGH después de 100 ms.
//
// ── Ejemplo B: Gripper OFF ──
//   Estado previo: stGrippers = true
//   Mismo pulso 100 ms en GPIO 16; stGrippers = false.
//
// ── Ejemplo C: Holder + Encoder juntos (0x2A, wantOn = true) ──
//   t=0 ms:    GPIO 17 y 21 HIGH simultáneos
//   t=100 ms:  ambos LOW
//   stHolder = true; stEncoder = true
//   Si otro pulso sigue: esperar ≥ 50 ms desde fin del anterior.
//
// ── Ejemplo D: Blower 3 s ──
//   GPIO 4 HIGH; mantener 3000 ms; GPIO 4 LOW (no usa pulso 100 ms).
//
// ── Ejemplo E: Leer sensor Cutter ──
//   digitalRead(26) == LOW  →  falla cortador activa (bit Cutter en mask)
//   Tras debounce 150 ms estable → asociar a PLC_ERR_CUTTER (0x1F) en lógica HMI.
//
// ── Ejemplo F: Reset ──
//   Todas válvulas lógicas false; pulso 100 ms GPIO 33; salidas 25/14/16/17/21 en LOW.
//
// Integración sin TubeCutter: implementar [3]+[4] en loop(); los bytes 0x19…
// son solo IDs para alinear nombres con PlcStates.h si comparten documentación con HMI.

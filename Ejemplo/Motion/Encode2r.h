#pragma once

/*
 * Encode2r.h — MCU (ESP32) ↔ encoders OM (cuadratura A/B/Z)
 *
 * No hay bus serial: el “protocolo” es señal incremental + lectura PCNT en firmware.
 * Sin TCP/HMI. Referencia: Encoder.h · Motion.ino (initEncodersHw, updateEncoderSide).
 *
 * Segmentos:
 *   [1] Enlace físico (pines, señales, lados L/R)
 *   [2] Contador hardware (PCNT / pulse_cnt)
 *   [3] Conversión counts ↔ mm y lectura “oficial”
 *   [4] Operaciones MCU (init, leer, Set0, reset)
 *   [5] Detección de quieto (settle)
 *   [6] Ejemplo numérico
 */

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// [1] Enlace físico — MCU ↔ encoder OM
// ═══════════════════════════════════════════════════════════════════════════
//
// Dos encoders independientes (misma mecánica / constante counts↔mm):
//
// | Lado lógico | GPIO A | GPIO B | GPIO Z (índice) | PCNT (ESP32)   |
// |-------------|--------|--------|-----------------|----------------|
// | R           |   18   |   19   |       21        | unidad 0 / hdl |
// | L           |   22   |   23   |       25        | unidad 1 / hdl |
//
// Cableado típico OM incremental:
//   A, B → cuadratura 90° (PCNT cuenta según flancos en A con nivel B).
//   Z    → pulso índice (opcional); en Motion: attachInterrupt FALLING en Z.
//   Entradas: INPUT_PULLUP en A, B, Z.
//
// Modelo referencia en firmware: RE30 / 2000 PPR, cuadratura configurada ENC_QUAD=1
// → ENC_CPR = 2000 counts/rev en la constante del proyecto.
// Polea Ø 50 mm en constantes; escala lineal calibrada:
//   1273 counts = 100 mm  →  mm = counts × (100 / 1273)

static const uint16_t ENC_OM_PPR              = 2000;
static const uint8_t  ENC_OM_QUAD             = 1;
static const uint16_t ENC_OM_CPR              = ENC_OM_PPR * ENC_OM_QUAD;
static const float    ENC_OM_PULLEY_DIAM_MM   = 50.0f;
static const int32_t  ENC_OM_COUNTS_PER_100MM = 1273;

static const int ENC_OM_PIN_A_R = 18;
static const int ENC_OM_PIN_B_R = 19;
static const int ENC_OM_PIN_Z_R = 21;
static const int ENC_OM_PIN_A_L = 22;
static const int ENC_OM_PIN_B_L = 23;
static const int ENC_OM_PIN_Z_L = 25;

// ═══════════════════════════════════════════════════════════════════════════
// [2] Contador hardware (PCNT)
// ═══════════════════════════════════════════════════════════════════════════
//
// Lectura bruta (por lado):
//   countAbs = pcntOverflow + hwCounter16
//   hwCounter16 ← pcnt_unit_get_count / pcnt_get_counter_value (−30000…+30000)
//   pcntOverflow ← suma de saltos al llegar a high/low limit (±30000)
//
// Filtro: glitch ~12000 ns (IDF5) o filter 1023 ticks (legacy).
//
// No existe trama TX/RX; “comando” = llamada firmware o replicar config PCNT en IDF.

static const int16_t ENC_OM_PCNT_HIGH_LIM = 30000;
static const int16_t ENC_OM_PCNT_LOW_LIM  = -30000;

// ═══════════════════════════════════════════════════════════════════════════
// [3] Counts ↔ mm (después de Set0)
// ═══════════════════════════════════════════════════════════════════════════
//
//   countRel = countAbs − zeroRef
//   mmSigned = countRel × (100.0f / ENC_OM_COUNTS_PER_100MM)
//   mmAbs    = |countRel| × (100.0f / ENC_OM_COUNTS_PER_100MM)
//
// “Oficial” (display / tolerancia): omRoundMm sobre mmAbs — redondeo a 0.5 mm
//   fracción decimal (décimas): 0–2 → .0 ; 3–7 → .5 ; 8–9 → siguiente entero .0
//
// Offset opcional por lado (calibración interna): mmOfficial = omRoundMm(mmAbs) + offsetMm
//   rango típico offset ±5 mm (no necesario para jog/lectura cruda).

#define ENC_OM_COUNTS_TO_MM(counts) \
  ((float)(counts) * (100.0f / (float)ENC_OM_COUNTS_PER_100MM))

// ═══════════════════════════════════════════════════════════════════════════
// [4] Operaciones MCU (equivalente a “comandos” — no hay Modbus/CAN)
// ═══════════════════════════════════════════════════════════════════════════
//
// INIT (una vez en setup):
//   pinMode A/B/Z; configurar PCNT cuadratura; clear count; overflow=0;
//   opcional: attachInterrupt(Z, FALLING) → incrementa zCount (diagnóstico).
//
// READ_LIVE(side R|L):
//   countAbs = readEncoderCountSide(ix);
//   countRel = countAbs − zeroRef;
//   devolver mmSigned / mmAbs según [3].
//
// SET0 (marcar cero lógico en posición actual):
//   zeroRef = countAbs;
//   settledCount = countAbs; settled = true; (lectura base → 0 mm)
//
// RESET_HW (borrar contador PCNT + estado software):
//   pcnt clear; pcntOverflow=0; zeroRef=0; settled=false; …
//
// POLL (cada ~80 ms en loop — updateEncoderSide):
//   delta = count − lastCount → mm/s; actualiza settled si mm/s < ENC_MOVE_MMS
//   durante ENC_SETTLE_MS tras haberse movido.

// ═══════════════════════════════════════════════════════════════════════════
// [5] Settle (quieto después de movimiento)
// ═══════════════════════════════════════════════════════════════════════════

static const float    ENC_OM_MOVE_MMS  = 20.0f;   // por debajo → candidato a parado
static const uint32_t ENC_OM_SETTLE_MS = 250;     // quieto continuo para settled=true

// settled=true ⇒ lectura “oficial” congelada en settledCount (no el live jitter).

// ═══════════════════════════════════════════════════════════════════════════
// [6] Ejemplo numérico — lado R, Set0 y lectura
// ═══════════════════════════════════════════════════════════════════════════
//
// Parámetros:
//   ENC_OM_COUNTS_PER_100MM = 1273
//   Tras SET0 en countAbs = 48 000 (ej. cerca de referencia mecánica)
//
// Operación SET0 (pseudo):
//   zeroRef ← 48000
//   countRel ← 0  →  mmSigned = 0.0 mm
//
// El eje avanza; lectura READ_LIVE:
//   countAbs = 49 273
//   countRel = 49273 − 48000 = 1273
//   mmSigned = 1273 × (100/1273) = 100.0 mm
//   mmAbs    = 100.0 mm
//   omRoundMm(100.0) = 100.0 mm  (valor “oficial” mostrado)
//
// Si mm/s ≥ 20 durante el movimiento y luego < 20 por 250 ms:
//   settled = true; settledCount = 49273 → official estable = 100.0 mm
//
// Pulso Z (diagnóstico): cada flanco FALLING en GPIO 21 incrementa zCount
//   (no sustituye Set0; solo conteo de pasadas por índice).
//
// Integrador en otro MCU:
//   Replicar A/B en timer/quadrature decoder; usar la misma fórmula mm y Set0
//   guardando zeroRef en RAM/NVS. No se envía ningún frame al encoder.

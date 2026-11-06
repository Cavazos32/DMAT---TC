#pragma once

/*
 * ServoFeeder.h — MCU (ESP32) ↔ drives feeder por CAN (CiA402 / SDO)
 *
 * Solo tramas en el bus: COB-ID + datos; respuestas SDO del drive.
 * Sin capa de aplicación ni tiempos de espera obligatorios (ajustar en banco).
 */

#include <stdint.h>

// Segmentos:
//   [1] Enlace físico (TWAI, pinout, COB-IDs, nodos)
//   [2] NMT CiA301
//   [3] SDO expedited (TX/RX MCU ↔ drive)
//   [4] Objetos OD CiA402
//   [5] Nodos CAN (mapa y configuración drive)
//   [6] Secuencias arranque + “home” lógico (607A)
//   [7] Jog mínimo
//   [8] 6081 profile velocity ↔ mm/s ↔ RPM rodillo
//   [9] Encender / apagar / re-inicializar

// ═══════════════════════════════════════════════════════════════════════════
// [1] Enlace físico — MCU ↔ drives CAN
// ═══════════════════════════════════════════════════════════════════════════
//   GPIO 4   CAN TX  →  TX del transceiver (ej. SN65HVD230)
//   GPIO 5   CAN RX  ←  RX del transceiver
//   CANH/CANL a ambos drives; terminación 120 Ω en un extremo del bus (típ. último nodo).
//   500 kbit/s · nodos CiA402: derecho (R) = 1, izquierdo (L) = 2.

static const int      SERVO_CAN_TX_GPIO      = 4;
static const int      SERVO_CAN_RX_GPIO      = 5;
static const uint32_t SERVO_CAN_BITRATE_KBPS = 500;

static const uint32_t SERVO_CAN_NMT_ID       = 0x000u;
static const uint32_t SERVO_CAN_SDO_TX_BASE  = 0x600u;  // master → slave
static const uint32_t SERVO_CAN_SDO_RX_BASE  = 0x580u;  // slave → master

static const uint8_t  SERVO_CAN_NODE_R       = 1;
static const uint8_t  SERVO_CAN_NODE_L       = 2;

#define SERVO_CAN_COB_TX(node)  (SERVO_CAN_SDO_TX_BASE + (uint32_t)(node))
#define SERVO_CAN_COB_RX(node)  (SERVO_CAN_SDO_RX_BASE + (uint32_t)(node))

// En este proyecto: TX R = 0x601, TX L = 0x602; RX R = 0x581, RX L = 0x582.

// ═══════════════════════════════════════════════════════════════════════════
// [5] Nodos CAN — mapa MCU ↔ drives
// ═══════════════════════════════════════════════════════════════════════════
//
// | Lado   | Node ID | COB SDO TX (MCU→drv) | COB SDO RX (drv→MCU) |
// |--------|---------|----------------------|----------------------|
// | R feed |    1    |       0x601          |       0x581          |
// | L feed |    2    |       0x602          |       0x582          |
//
// Configuración en cada drive (manual fabricante, no por SDO en runtime):
//   Node ID = 1 o 2, bitrate 500 kbit/s, perfil CiA402 / DS402.
//   Bus: CANH/CANL común; terminación 120 Ω en un extremo.
//
// Encoder interno (referencia counts): 131072 inc/rev (0x20000).
// Signo comando movimiento relativo (convención firmware feed):
//   R → +1 (más counts = avance feed), L → −1.

static const uint32_t SERVO_CAN_ENC_INC_PER_REV = 131072u;
static const int8_t   SERVO_CAN_CMD_SIGN_R      = 1;
static const int8_t   SERVO_CAN_CMD_SIGN_L      = -1;

// Perfil / halt usados en arranque (valores Motion; tunear en banco):
static const uint32_t SERVO_CAN_HOME_PP           = 42500u;   // 607A “home” lógico
static const uint32_t SERVO_CAN_MAX_RPM           = 3000u;
static const uint32_t SERVO_CAN_MAX_PPS           =
    (uint32_t)((uint64_t)SERVO_CAN_MAX_RPM * SERVO_CAN_ENC_INC_PER_REV / 60u);
static const uint8_t  SERVO_CAN_HALT_OPTION_CODE  = 2;
static const uint32_t SERVO_CAN_HALT_DECEL_PP     = 8000000u;

/*
 * ── Ejemplo numérico: nodo R (ID 1, COB TX 0x601) ──
 * Mover a posición absoluta 100000 counts con perfil:
 *   6081 profile velocity = 2_000_000 counts/s  (0x001E8480)
 *   6083 profile accel      =   800_000           (0x000C3500)
 *   6084 profile decel      =   800_000
 *   607A target position    = 100_000           (0x000186A0)
 * Precondición: NMT Operational + CiA402 enable (6040→0x000F) + 6060=1 (PP).
 *
 * | Paso | Objeto   | Parámetro              | ID 0x601 DLC 8 (hex)              |
 * |------|----------|------------------------|-----------------------------------|
 * |  1   | 0x6081:0 | vel = 2000000          | 23 81 60 00 80 84 1E 00           |
 * |  2   | 0x6083:0 | acc = 800000           | 23 83 60 00 00 35 0C 00           |
 * |  3   | 0x6084:0 | dec = 800000           | 23 84 60 00 00 35 0C 00           |
 * |  4   | 0x6040:0 | halt hold CW=0x010F    | 2B 40 60 00 0F 01 00 00           |
 * |  5   | 0x607A:0 | pos = 100000           | 23 7A 60 00 A0 86 01 00           |
 * |  6   | 0x6040:0 | reset halt CW=0x012F   | 2B 40 60 00 2F 01 00 00           |
 * |  7   | 0x6040:0 | new set-point CW=0x013F| 2B 40 60 00 3F 01 00 00           |
 * |  8   | 0x6040:0 | run absolute CW=0x003F | 2B 40 60 00 3F 00 00 00           |
 *
 * Lectura posición (upload 0x6064:0) — esperar RX en ID 0x581:
 *   TX 0x601: 40 64 60 00 00 00 00 00
 *   RX 0x581: 43 64 60 00 [pos int32 LE]  (≈100000 al terminar; bit target en 6041)
 *
 * Mismo ejemplo en nodo L: sustituir 0x601→0x602 y RX 0x581→0x582.
 * Unidades 6081/6083/6084: counts/s y counts/s² del encoder del drive (131072/rev aquí).
 */
// ═══════════════════════════════════════════════════════════════════════════
// [2] NMT — MCU → bus (COB 0x000, DLC 2)
// ═══════════════════════════════════════════════════════════════════════════
//   80 00  → Pre-Operational (broadcast)
//   01 00  → Operational (broadcast)

// ═══════════════════════════════════════════════════════════════════════════
// [3] SDO expedited — MCU ↔ drive (DLC = 8)
// ═══════════════════════════════════════════════════════════════════════════
//
// Campos comunes en bytes [1..3]: índice OD little-endian, subíndice en [3].
//
// Download (escritura) — byte [0]:
//   0x2F  escribir 1 byte  → dato en [4]
//   0x2B  escribir 2 bytes → dato en [4][5] LE
//   0x23  escribir 4 bytes → dato en [4..7] LE
//
// Upload (lectura) — TX byte [0] = 0x40, [4..7] = 0
//
// Upload respuesta — RX en COB 0x580+node:
//   0x43 + 4 bytes de dato en [4..7] LE     (canReadSdoI32)
//   expedited 2 bytes: índice en [1][2], dato uint16 LE en [4][5] (canReadStatusWord)
//
// Confirmación download (si el drive responde): CS 0x60, eco de índice/sub.
// En feed muchas escrituras SDO no esperan confirmación; solo lecturas hacen poll RX.

// ═══════════════════════════════════════════════════════════════════════════
// [4] Objetos OD (CiA402) y control/status word
// ═══════════════════════════════════════════════════════════════════════════

static const uint16_t SERVO_OD_CONTROL_WORD       = 0x6040u;
static const uint16_t SERVO_OD_STATUS_WORD        = 0x6041u;
static const uint16_t SERVO_OD_MODE_OF_OPERATION  = 0x6060u;
static const uint16_t SERVO_OD_POSITION_ACTUAL    = 0x6064u;
static const uint16_t SERVO_OD_VELOCITY_ACTUAL    = 0x606Cu;
static const uint16_t SERVO_OD_TARGET_POSITION    = 0x607Au;
static const uint16_t SERVO_OD_MAX_PROFILE_VEL    = 0x607Fu;
static const uint16_t SERVO_OD_PROFILE_VELOCITY   = 0x6081u;
static const uint16_t SERVO_OD_PROFILE_ACCEL      = 0x6083u;
static const uint16_t SERVO_OD_PROFILE_DECEL      = 0x6084u;
static const uint16_t SERVO_OD_QUICK_STOP_DECEL   = 0x6085u;
static const uint16_t SERVO_OD_HALT_OPTION        = 0x605Du;
static const uint16_t SERVO_OD_TARGET_VELOCITY    = 0x60FFu;

static const int8_t   SERVO_MODE_PP = 1;
static const int8_t   SERVO_MODE_PV = 3;

// Control word (uint16 LE en bytes [4][5] de SDO 0x2B 40 60 00 …):
//   0x0006 Shutdown    0x0007 Switch on    0x000F Enable operation
//   0x002F Reset set-point   0x003F Run absolute (PP)
//   0x010F Halt hold   0x012F Reset halt   0x013F New set-point + abs + halt
//   0x0080 Fault reset (en trama setup)
// Halt rápido (PDO-like repetido): 2B 40 60 00 0F 01 00 00  → CW = 0x010F

// Status word 0x6041 (lectura): bit 3 = fault; patrón 0x0027 ≈ operation enabled.

// ── Ejemplos TX (hex) por objeto ─────────────────────────────────────────────
//
// Leer posición actual 0x6064:0
//   ID 0x600+node  40 64 60 00 00 00 00 00
//   RX ID 0x580+node  43 64 60 00 [pos int32 LE]
//
// Leer status 0x6041:0
//   ID 0x600+node  40 41 60 00 00 00 00 00
//   RX ID 0x580+node  … 41 60 …  [SW uint16 LE en bytes 4-5]
//
// Modo PP (1 byte):  2F 60 60 00 01 00 00 00
// Modo PV (3):       2F 60 60 00 03 00 00 00
//
// Target position 0x607A (int32 LE):
//   23 7A 60 00  [4 bytes pos]
//
// Profile velocity / accel / decel 0x6081 / 0x6083 / 0x6084 (uint32 LE):
//   23 81 60 00 …   /   23 83 60 00 …   /   23 84 60 00 …
//
// Max profile velocity 0x607F:  23 7F 60 00 [uint32 LE]
// Target velocity 0x60FF:       23 FF 60 00 [int32 LE]
// Halt option 0x605D (16):      2B 5D 60 00 [code] 00 00 00  (código 2 en feed)
// Quick stop decel 0x6085:      23 85 60 00 [uint32 LE]

// ═══════════════════════════════════════════════════════════════════════════
// [6] Arranque bus + “home” lógico (607A) — por nodo o ambos
// ═══════════════════════════════════════════════════════════════════════════
//
// No es homing CiA402 (0x6098); es poner set-point 607A y reset SP (referencia
// de software al encender). Repetir tras fault si hace falta re-sincronizar.

/*
 * 1) Broadcast NMT Pre-Op:     ID 0x000  80 00
 * 2) Broadcast NMT Operational: ID 0x000  01 00
 * 3) Por cada nodo (0x601 y 0x602):
 *      SDO 6040 ← 0x0080   2B 40 60 00 80 00 00 00   (fault reset)
 * 4) Por nodo — habilitar CiA402 PP (pausas ~200 ms entre pasos recomendadas):
 *      6040 ← 0x0006   2B 40 60 00 06 00 00 00
 *      6040 ← 0x0007   2B 40 60 00 07 00 00 00
 *      6040 ← 0x000F   2B 40 60 00 0F 00 00 00
 *      6060 ← 1        2F 60 60 00 01 00 00 00
 *      607F ← SERVO_CAN_MAX_PPS   (23 7F 60 00 + uint32 LE)
 *      605D ← 2        2B 5D 60 00 02 00 00 00
 *      6085 ← 8000000  23 85 60 00 + uint32 LE (SERVO_CAN_HALT_DECEL_PP)
 * 5) Perfil movimiento (6081/6083/6084): escribir antes de mover; en Motion
 *    se derivan de SERVO_CAN_HOME_PP para el snap inicial (42500 counts/s escala).
 * 6) “Home” lógico por nodo:
 *      607A ← SERVO_CAN_HOME_PP (42500)   23 7A 60 00 24 A6 00 00
 *      6040 ← 0x002F (reset set-point)    2B 40 60 00 2F 00 00 00
 * 7) Verificar: upload 6041 → status ≈ operation enabled (ej. 0x0027), sin fault bit 3.
 *
 * Movimiento relativo PP (jog por pasos, un nodo):
 *   Upload 6064 → pos; 6040 halt 010F; 607A ← pos ± pasos;
 *   6040 ← 012F, 013F, 003F (ver [7]).
 *
 * Paro: 6040 ← 0x010F en COB del nodo activo.
 *
 * Modo velocidad (PV): 6060←3, perfil 6081/6083/6084, 60FF, 6040←0x000F run.
 */

/*
 * ── Ejemplo numérico PV (jog a velocidad fija) — nodo R, COB 0x601 ──
 *   6060 = 3 (PV):     2F 60 60 00 03 00 00 00
 *   6081 = 1500000:    23 81 60 00 60 E3 16 00
 *   6083 / 6084 = 800000 (mismas tramas que ejemplo PP pasos 2–3)
 *   60FF = +500000 (int32): 23 FF 60 00 20 A1 07 00
 *   6040 run 0x000F:     2B 40 60 00 0F 00 00 00
 * Parar: 6040 halt 0x010F + 60FF=0 (23 FF 60 00 00 00 00 00)
 */

// ═══════════════════════════════════════════════════════════════════════════
// [7] Jog mínimo
// ═══════════════════════════════════════════════════════════════════════════
//
// A) Habilitar eje (una vez por nodo, tras NMT 80 00 → 01 00):
//   SDO 6040:0 ← 0x0006, 0x0007, 0x000F  (Shutdown → Switch on → Enable)
//   SDO 6060:0 ← 1 (PP) o 3 (PV jog continuo)
//   SDO 6081 / 6083 / 6084 — perfil no cero (ej. vel/accel/dec en counts/s;
//        valores concretos dependen del drive; probar bajo en banco)
//
// B) Jog por pasos (modo PP = 1) — repetir botón + / −:
//   Upload 6064 → pos
//   SDO 6040 ← 0x010F (halt hold)     trama: 2B 40 60 00 0F 01 00 00
//   SDO 607A ← pos ± jogCounts       trama: 23 7A 60 00 [int32 LE]
//   SDO 6040 ← 0x012F                 2B 40 60 00 2F 01 00 00
//   SDO 6040 ← 0x013F                 2B 40 60 00 3F 01 00 00
//   SDO 6040 ← 0x003F (run abs)       2B 40 60 00 3F 00 00 00
//   Signo jog: +counts = sentido R en convención R; en banco probar ± en un nodo.
//
// C) Jog continuo (modo PV = 3) — mientras botón pulsado:
//   SDO 6060 ← 3
//   SDO 6081/6083/6084 ← perfil
//   SDO 60FF ← ±velocidad (int32; 0 = parado)
//   SDO 6040 ← 0x000F (run)
//   Al soltar: SDO 6040 ← 0x010F (halt) y opc. 60FF ← 0
//
// Paro rápido: SDO 6040 ← 0x010F (misma trama halt).
// Diagnóstico: upload 6041 (fault bit 3); upload 6064 (posición).
//
// Para jog sin paso [6]: al menos NMT + enable 6040 + 6060=PP; home 607A opcional.

// ═══════════════════════════════════════════════════════════════════════════
// [8] 0x6081 profile velocity (PP) ↔ mm/s ↔ RPM rodillo
// ═══════════════════════════════════════════════════════════════════════════
//
// Escala usada al mapear velocidad de cinta → valor escrito en 6081 (counts/s
// del perfil CiA402; en setup coincide con unidades del encoder 131072/rev):
//
//   FEED_STEPS_PER_MM = 3600 / 55 ≈ 65.4545   (PP por cada mm/s de cinta)
//   mm_s = PP / FEED_STEPS_PER_MM
//   PP   = mm_s × FEED_STEPS_PER_MM
//
// RPM mecánico del rodillo (Ø polea 50 mm):
//   RPM_roll = mm_s × 60 / (π × 50)
//   PP       = RPM_roll × (π × 50) / 60 × FEED_STEPS_PER_MM
//
// Ejemplos (nominal 650 mm/s):
//   PP ≈ 42545, RPM_roll ≈ 249
// Tope ~7854 mm/s ≈ SERVO_CAN_MAX_PPS / (131072 / (π×50)).
// 607F en arranque suele = SERVO_CAN_MAX_PPS; límite práctico de PP según mecánica.

static const float    SERVO_FEED_NOMINAL_MM           = 55.0f;
static const uint32_t SERVO_FEED_NOMINAL_STEPS        = 3600u;
static const float    SERVO_FEED_STEPS_PER_MM_S       =
    (float)SERVO_FEED_NOMINAL_STEPS / SERVO_FEED_NOMINAL_MM;
static const float    SERVO_FEED_PULLEY_DIAM_MM       = 50.0f;
static const float    SERVO_FEED_MM_S_DEFAULT         = 650.0f;
static const uint32_t SERVO_FEED_PP_DEFAULT           = 42545u;  // ≈650 mm/s

// ═══════════════════════════════════════════════════════════════════════════
// [9] Encender / apagar / re-inicializar
// ═══════════════════════════════════════════════════════════════════════════
//
// Encender (por nodo, tras TWAI 500 kbit/s activo):
//   Secuencia [6] pasos 1–4 (NMT → fault reset → Shutdown→Switch on→Enable→PP…).
//   Verificar 6041: (status & 0x006F) == 0x0027.
//
// Apagar (deshabilitar variador, un nodo o ambos):
//   SDO 6040 ← 0x0006   2B 40 60 00 06 00 00 00   (Shutdown)
//   Opcional paro rápido antes: 6040 ← 0x010F (halt).
//
// “Home” lógico (referencia software, no homing CiA402):
//   Tras encendido: 6081/6083/6084 con perfil deseado → 607A ← SERVO_CAN_HOME_PP
//   → 6040 ← 0x002F. Repetir si hay que re-sincronizar set-point sin reiniciar TWAI.
//
// Re-inicialización completa del bus (fallo TWAI o tras cambio de cableado):
//   Apagar driver TWAI → instalar de nuevo 500 kbit/s → repetir [6] entero en R y L.

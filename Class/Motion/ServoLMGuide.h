#pragma once

/*
 * ServoLMGuide.h — MCU (ESP32) ↔ drive Delta ASDA-B3 por RS-485 Modbus RTU
 *
 * Solo tramas en el bus: qué envía el MCU y qué responde el drive.
 * Sin capa de aplicación; tiempos de asentamiento indicados como guía de banco.
 */

#include <stdint.h>

// Segmentos:
//   [1] Enlace físico (UART, pinout, parámetros RTU)
//   [2] Funciones Modbus (FC 0x03 / 0x06 / 0x10)
//   [3] Registros P.xxx
//   [4] Secuencias típicas MCU → drive
//   [5] Homing (secuencia completa)
//   [6] Jog mínimo
//   [7] PUU ↔ mm (calibración lineal)
//   [8] Inicialización UART / encender / apagar / HOME (resumen)
//   (ejemplo numérico MOVE en [1] tras pinout)

// ═══════════════════════════════════════════════════════════════════════════
// [1] Enlace físico — MCU ↔ ASDA-B3
// ═══════════════════════════════════════════════════════════════════════════
// Esclavo MODBUS_ID, 9600 8N2, CRC-16 Modbus (init 0xFFFF, poly 0xA001), CRC en wire LSB primero.

#ifndef MODBUS_ASDA_SLAVE_ID
static const uint8_t  MODBUS_ASDA_SLAVE_ID = 0x01;
static const uint32_t MODBUS_ASDA_BAUD     = 9600;
#endif

// “Nodo” en Modbus = dirección de esclavo RTU (un solo ASDA lineal en este hardware).
// Configurar en el drive Pn que fija ID de comunicación = 1 (manual ASDA-B3).

// ── Pinout (ESP32 Motion → ASDA-B3 RS-485) ───────────────────────────────────
//   GPIO 16  UART RX  ←  RO del transceiver  ←  A/B del drive
//   GPIO 17  UART TX  →  DI del transceiver  →  A/B del drive
//   DE/RE      no usado (-1): transceiver con auto-dirección o DE atado a VCC según HW
// Cable: par trenzado A/B al CN3/CN4 del servo; GND común; terminación 120 Ω en el
// extremo lejano del bus si hay más de un nodo.

/*
 * ── Ejemplo numérico: mover PR1 a PUU 50000 @ 1200 rpm ──
 * Parámetros:
 *   speedRpm = 1200.0  →  P5.060 int32 speed01rpm = 12000 (0x00002EE0)
 *   modo PR1 absoluto  →  P6.002 = 0x00000002
 *   destino            →  P6.003 = 50000 PUU (0x0000C350)
 *   disparo            →  P5.007 = 1
 * Precondición: P2.030 = 1 (Servo ON) y ~2 s de asentamiento.
 *
 * Paso 1 — velocidad PR (FC 0x10 @ 0x0578):
 *   TX: 01 10 05 78 00 02 04 E0 2E 00 00 9D B4
 *   RX: 01 10 05 78 00 02 [CRC]  (eco 8 bytes)
 *
 * Paso 2 — modo PR1 (FC 0x10 @ 0x0604):
 *   TX: 01 10 06 04 00 02 04 00 02 00 00 78 3C
 *   RX: eco 0x10
 *
 * Paso 3 — objetivo PUU (FC 0x10 @ 0x0606):
 *   TX: 01 10 06 06 00 02 04 50 C3 00 00 B9 19
 *   RX: eco 0x10
 *
 * Paso 4 — ejecutar movimiento (FC 0x06 @ 0x050E):
 *   TX: 01 06 05 0E 00 01 29 05
 *   RX: 01 06 05 0E 00 01 29 05  (eco idéntico)
 *
 * Seguimiento — posición actual (FC 0x03 @ 0x0520, 2 regs):
 *   TX: 01 03 05 20 00 02 C5 0D
 *   RX: 01 03 04 LL LH HL HH [CRC]  → PUU int32; esperar ≈50000 y ZSPD en P0.046
 *
 * Paro en marcha: FC 0x06 P5.007=1000 → TX: 01 06 05 0E 03 E8 [CRC]
 *   (1000 = 0x03E8; calcular CRC en MCU igual que el resto)
 */
// ═══════════════════════════════════════════════════════════════════════════
// [2] Funciones Modbus (FC)
// ═══════════════════════════════════════════════════════════════════════════

/*
 * FC 0x03 — Leer registros holding (16 bit c/u)
 *
 * TX (8 bytes):
 *   [0] slave
 *   [1] 0x03
 *   [2][3] dirección reg (big-endian)
 *   [4][5] cantidad de registros (big-endian), típico 1 o 2
 *   [6][7] CRC
 *
 * RX OK (5 + 2*N bytes), N = cantidad pedida:
 *   [0] slave
 *   [1] 0x03
 *   [2] byte count = 2*N
 *   [3..] datos: por cada registro, high byte luego low byte
 *   [..] CRC
 *
 * Ejemplo leer 1 reg en 0x050E (P5.007):
 *   TX: 01 03 05 0E 00 01 CRC CRC
 *   RX: 01 03 02 HH LL CRC CRC  → valor uint16 = (HH<<8)|LL
 */

/*
 * FC 0x06 — Escribir un registro (16 bit)
 *
 * TX (8 bytes):
 *   [0] slave, [1] 0x06, [2][3] addr BE, [4][5] valor BE, [6][7] CRC
 *
 * RX OK: eco idéntico al TX (8 bytes).
 */

/*
 * FC 0x10 — Escribir varios registros (32 bit = 2 registros consecutivos)
 *
 * TX (13 bytes) para 2 regs / 4 bytes de dato:
 *   [0] slave, [1] 0x10, [2][3] addr BE, [4][5] 0x00 0x02 (qty regs),
 *   [6] 0x04 (byte count),
 *   [7][8] reg bajo:  high, low  (bits 15..0 del int32),
 *   [9][10] reg alto: high, low  (bits 31..16 del int32),
 *   [11][12] CRC
 *
 * RX OK (8 bytes):
 *   [0] slave, [1] 0x10, [2][3] addr BE, [4][5] qty regs BE, [6][7] CRC
 *
 * Lectura int32 (FC 0x03, count=2): reg[0] = bits 15..0, reg[1] = bits 31..16.
 */

/*
 * RX error Modbus: [0] slave, [1] función | 0x80, [2] código excepción, CRC.
 */

// ═══════════════════════════════════════════════════════════════════════════
// [3] Registros holding (addr Modbus = columna P del manual)
// ═══════════════════════════════════════════════════════════════════════════

static const uint16_t ASDA_REG_P0_046 = 0x005C;  // P0.046 — imagen DO (16 bit, lectura)
static const uint16_t ASDA_REG_P1_087 = 0x01AE;  // P1.087 — torque homing %
static const uint16_t ASDA_REG_P1_088 = 0x01B0;  // P1.088 — tiempo homing ms
static const uint16_t ASDA_REG_P2_030 = 0x023C;  // P2.030 — Servo ON/OFF (16)
static const uint16_t ASDA_REG_P3_000 = 0x0300;  // P3.000 — test comunicación (lectura)
static const uint16_t ASDA_REG_P3_001 = 0x0302;
static const uint16_t ASDA_REG_P3_002 = 0x0304;
static const uint16_t ASDA_REG_P5_004 = 0x0508;  // P5.004 — método homing (16)
static const uint16_t ASDA_REG_P5_005 = 0x050A;  // P5.005 — vel. homing 1 (32, 0.1 rpm)
static const uint16_t ASDA_REG_P5_006 = 0x050C;  // P5.006 — vel. homing 2 (32, 0.1 rpm)
static const uint16_t ASDA_REG_P5_007 = 0x050E;  // P5.007 — disparo PR / home / paro (16)
static const uint16_t ASDA_REG_P5_016 = 0x0520;  // P5.016 — posición feedback PUU (32, signed)
static const uint16_t ASDA_REG_P5_060 = 0x0578;  // P5.060 — vel. movimiento PR (32, 0.1 rpm)
static const uint16_t ASDA_REG_P6_000 = 0x0600;  // P6.000 — posición PR origen (32)
static const uint16_t ASDA_REG_P6_001 = 0x0602;  // P6.001 — posición PR destino (32)
static const uint16_t ASDA_REG_P6_002 = 0x0604;  // P6.002 — modo PR1 (32)
static const uint16_t ASDA_REG_P6_003 = 0x0606;  // P6.003 — objetivo PR1 PUU (32, signed)

// Bits útiles en P0.046 (respuesta FC 0x03):
static const uint16_t ASDA_DO_BIT_ZSPD = 0x0004;  // bit 2 — eje parado
static const uint16_t ASDA_DO_BIT_HOME = 0x0100;  // bit 8 — homing completado

// Valores típicos escritos en P5.007 (FC 0x06):
//   0    → ejecutar secuencia HOME (tras preparar P1/P5/P6)
//   1    → ejecutar movimiento PR1 (tras P5.060, P6.002, P6.003)
//   1000 → paro / cancelar PR

// Valores típicos P2.030 (FC 0x06):
//   1      → Servo ON
//   0      → Servo OFF
//   0xFFFF → paso intermedio antes de OFF (secuencia firmware)

// Valores típicos P6.002 (FC 0x10, 32 bit):
//   0x00000002 → PR1 absoluto

// P5.004 homing (16): 0x0029 forward / 0x002A reverse (según sentido máquina)

// ═══════════════════════════════════════════════════════════════════════════
// [4] Secuencias Modbus (orden de writes/reads, sin tiempos de espera)
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Servo ON:
 *   W P2.030 ← 1
 *
 * Servo OFF:
 *   W P2.030 ← 0xFFFF
 *   W P2.030 ← 0
 *
 * Paro movimiento:
 *   W P5.007 ← 1000
 *
 * Movimiento PR1 a PUU objetivo:
 *   W P5.060 ← velocidad (int32, unidad 0.1 rpm)
 *   W P6.002 ← 0x00000002
 *   W P6.003 ← posición PUU (int32)
 *   W P5.007 ← 1
 *
 * Homing: ver segmento [5] (orden completo de registros).
 *
 * Lecturas de estado / posición:
 *   R P5.007  (16) — fase comando PR (p. ej. 10000/20000 al terminar job)
 *   R P5.016  (32) — posición actual PUU
 *   R P0.046  (16) — DO (ZSPD, HOME, …)
 *
 * Test enlace (solo lectura):
 *   R P3.000, R P3.001, R P3.002
 */

// ═══════════════════════════════════════════════════════════════════════════
// [5] Homing ASDA — secuencia Modbus completa (MCU → drive)
// ═══════════════════════════════════════════════════════════════════════════
//
// Valores de ejemplo en banco (ajustar torque y rpm al mecanismo):
#ifndef ASDA_HOME_TORQUE_PCT_DEFAULT
static const uint16_t ASDA_HOME_TORQUE_PCT_DEFAULT = 10;
static const uint16_t ASDA_HOME_TIME_MS_DEFAULT    = 150;
static const float    ASDA_HOME_SPEED_RPM_DEFAULT  = 10.0f;   // → speed01rpm = 100
static const uint16_t ASDA_HOME_METHOD_FORWARD     = 0x0029;
static const uint16_t ASDA_HOME_METHOD_REVERSE     = 0x002A;
static const uint32_t ASDA_SERVO_ON_SETTLE_MS      = 2000;    // tras P2.030=1, antes de home
#endif

/*
 * Orden exacto de writes (cada uno = una trama FC 0x06 o 0x10 + RX eco):
 *
 *   1) W P2.030 ← 1                         (Servo ON)
 *   2) esperar ASDA_SERVO_ON_SETTLE_MS
 *   3) W P1.087 ← torquePct                 (16, ej. 10)
 *   4) W P1.088 ← timeMs                    (16, ej. 150)
 *   5) W P5.004 ← 0x0029 | 0x002A         (16, forward / reverse)
 *   6) W P5.005 ← speed01rpm                (32, rpm×10, ej. 100 = 10 rpm)
 *   7) W P5.006 ← speed01rpm                (32, mismo valor que P5.005)
 *   8) W P6.000 ← 0                         (32, origen PR)
 *   9) W P6.001 ← 0                         (32, destino PR)
 *  10) W P5.007 ← 0                         (16, disparar homing)
 *
 * Fin de homing (poll R P0.046, FC 0x03):
 *   bit HOME (0x0100) = 1 y bit ZSPD (0x0004) = 1 → buscar referencia mecánica.
 *   Confirmar R P5.016 ≈ 0 PUU cuando el eje ya está en el tope.
 *   P5.007 en HOME no siempre pasa a 10000/20000; no depender solo de P5.007.
 *
 * Paro durante home: W P5.007 ← 1000
 *
 * ── Ejemplo numérico homing forward @ 10 rpm ──
 *   torquePct=10, timeMs=150, P5.004=0x0029, speed01rpm=100 (10.0 rpm)
 *   Tras P2.030=1 y 2000 ms:
 *     W P1.087 ← 10      FC 0x06 01 AE 00 0A …
 *     W P1.088 ← 150     FC 0x06 01 B0 00 96 …
 *     W P5.004 ← 0x0029  FC 0x06 05 08 00 29 …
 *     W P5.005 ← 100     FC 0x10 05 0A … 64 00 00 00 …
 *     W P5.006 ← 100     (igual P5.005)
 *     W P6.000 ← 0, W P6.001 ← 0, W P5.007 ← 0
 *   (CRC en cada trama como en el ejemplo MOVE; usar calculadora Modbus o Motion.ino)
 */

// ═══════════════════════════════════════════════════════════════════════════
// [6] Jog mínimo (sin homing ni calibración mm — solo PUU en el drive)
// ═══════════════════════════════════════════════════════════════════════════
//
// Prerrequisitos drive (manual ASDA-B3, una vez en banco):
//   Esclavo Modbus = 0x01, 9600 8N2, RS-485 OK, sin alarma activa.
//   Servo ON (P2.030=1) al menos una vez por sesión.
//
// Implementación MCU mínima:
//   Una trama Modbus a la vez; CRC correcto; esperar RX eco/datos (~300 ms máx).
//   Tras P2.030=1, esperar ~2 s antes del primer movimiento (drive asentando).
//
/*
 * Encendido (una vez por sesión):
 *   W P2.030 ← 1
 *
 * Jog + (un paso):
 *   R P5.016 → pos
 *   W P5.060 ← speed01rpm (ej. 12000 = 1200 rpm; bajar para jog lento)
 *   W P6.002 ← 0x00000002
 *   W P6.003 ← pos + deltaPuu   (ej. deltaPuu = 500…5000 según prueba mecánica)
 *   W P5.007 ← 1
 *   Repetir lectura R P5.016 o R P0.046 (bit ZSPD) hasta parado → listo otro jog.
 *
 * Jog −:
 *   Igual con P6.003 ← pos - deltaPuu
 *
 * Soltar / parar:
 *   W P5.007 ← 1000
 *
 * Apagar servo (opcional):
 *   W P2.030 ← 0xFFFF, luego W P2.030 ← 0
 *
 * No incluido aquí (no necesario para jog): HOME, mm↔PUU, P3 test más allá del
 * primer smoke test, códigos 10000/20000 de P5.007 — basta ZSPD + posición estable.
 */

// ═══════════════════════════════════════════════════════════════════════════
// [7] PUU ↔ mm (P5.016 lectura, P6.003 comando PR1)
// ═══════════════════════════════════════════════════════════════════════════
//
// Calibración de fábrica en firmware (sin offset NVS):
//   770 PUU en 45 mm de carrera de referencia → stepsPerMm ≈ 17.111 PUU/mm
//   electronic gear N = 1, offsetSteps = 0
//
// Lectura posición (FC 0x03, P5.016 int32):
//   pasos = |PUU|
//   mm    = (pasos − offsetSteps) / stepsPerMm
//
// Comando destino PR1 (escribir P6.003 antes de P5.007=1):
//   pasos = offsetSteps + mm × stepsPerMm
//   PUU   = −(int32)pasos   (signo “work” hacia el corte)
//
// Compensación de medición (interfaz, mm con signo + o −):
//   La posición del modelo es la longitud del modelo, en mm.
//   mm comandado = posición del modelo + compensación
//   Ese mm entra en la fórmula de arriba (offsetSteps de fábrica = 0).
//   Home, jog y un PUU escrito a mano no suman esta compensación.
//
// Ejemplo: mm = 50 → pasos ≈ 856 → P6.003 = −856
//          |P5.016| / 17.111 ≈ 50 mm tras el movimiento
//
// Velocidad PR (P5.060): int32 en 0.1 rpm → rpm = valor / 10
// Avance tornillo ~5 mm/rev motor → mm/s ≈ (rpm / 60) × 5

static const float    ASDA_FACTORY_STEPS_PER_MM     = 770.0f / 45.0f;
static const float    ASDA_LINEAR_MM_PER_MOTOR_REV  = 5.0f;
static const uint32_t ASDA_EGEAR_N                  = 1u;

// ═══════════════════════════════════════════════════════════════════════════
// [8] Inicialización UART / encender / apagar / HOME (resumen MCU)
// ═══════════════════════════════════════════════════════════════════════════
//
// Init bus (una vez al energizar MCU):
//   UART RX=16, TX=17, 9600 8N2, esclavo 0x01.
//   No hay secuencia Modbus obligatoria al boot; el drive puede estar OFF.
//   Smoke test: R P3.000, P3.001, P3.002 (FC 0x03).
//
// Encender servo:
//   W P2.030 ← 1 (FC 0x06). Esperar ASDA_SERVO_ON_SETTLE_MS antes de HOME/MOVE.
//
// Apagar servo:
//   W P2.030 ← 0xFFFF → W P2.030 ← 0
//
// HOME actuador (homing mecánico del drive):
//   Secuencia completa [5]: ON → parámetros P1/P5/P6 → P5.007 ← 0.
//   Fin: R P0.046 (HOME+ZSPD) y R P5.016 ≈ 0 PUU.
//
// MOVE PR1:
//   W P5.060, P6.002=2, P6.003, P5.007=1 — ver [4] y ejemplo en [1].
//
// Paro:
//   W P5.007 ← 1000

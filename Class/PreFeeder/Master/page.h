#pragma once

// Valores por defecto (solo si NVS está vacío).
// La fuente de edición es el HMI; Guardar persiste en NVS del maestro
// hasta el próximo Guardar. L/R leen cfg por GET /api/config.

struct PfSideParams {
  float rpm;
  float pullerRpm;
  uint16_t settleMs;
  float onSec;
  float offFaultSec;
  float stallFaultSec;
};

static constexpr PfSideParams PF_PARAMS_L = { 60.0f, 60.0f, 150, 2.0f, 5.0f, 2.0f };
static constexpr PfSideParams PF_PARAMS_R = { 60.0f, 60.0f, 150, 2.0f, 5.0f, 2.0f };

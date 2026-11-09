#pragma once

#include <stdint.h>

// Fallas compartidas por el lado y el maestro. No cambiar los números.
enum SystemFault : uint8_t {
  FAULT_NONE = 0,
  FAULT_CYLINDER = 3,
  FAULT_HOSE = 4,
  FAULT_HOLGURA = 6,
};

// Estado que ve el maestro. Error gana sobre busy.
enum PfRunState : uint8_t {
  PF_RUN_IDLE = 0,
  PF_RUN_BUSY = 1,
  PF_RUN_ERROR = 2,
};

#define PF_HTTP_STATUS "/api/status"
#define PF_HTTP_CONFIG "/api/config"

static inline PfRunState pfResolveState(SystemFault fault, bool feeding) {
  if (fault != FAULT_NONE) return PF_RUN_ERROR;
  return feeding ? PF_RUN_BUSY : PF_RUN_IDLE;
}

static inline const char* pfStateName(PfRunState state) {
  switch (state) {
    case PF_RUN_BUSY: return "busy";
    case PF_RUN_ERROR: return "error";
    default: return "idle";
  }
}

static inline const char* pfFaultName(SystemFault fault) {
  switch (fault) {
    case FAULT_CYLINDER: return "Cilindro";
    case FAULT_HOSE: return "Manguera";
    case FAULT_HOLGURA: return "Holgura";
    default: return "";
  }
}

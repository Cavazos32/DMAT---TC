import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

from flask import Flask, jsonify, render_template, request
from werkzeug.serving import WSGIRequestHandler

# Peticiones de sondeo del HMI: no escribir cada GET en la consola (ruido).
_QUIET_HTTP_LOG_PREFIXES = (
    "/api/motion/status",
    "/api/plc/status",
    "/api/prefeeder/status",
    "/api/state",
)


class _QuietRequestHandler(WSGIRequestHandler):
    def log_request(self, code="-", size="-"):
        path = (getattr(self, "path", "") or "").split("?", 1)[0]
        if code == 200 and any(path == p or path.startswith(p + "/") for p in _QUIET_HTTP_LOG_PREFIXES):
            return
        super().log_request(code, size)

_CLASS_DIR = Path(__file__).resolve().parent / "Class"
if str(_CLASS_DIR) not in sys.path:
    sys.path.insert(0, str(_CLASS_DIR))

import error_codes  # noqa: E402

app = Flask(__name__)

DATA_DIR = Path(__file__).resolve().parent / "data"
MODELOS_PATH = DATA_DIR / "modelos.json"
MOTION_PATH = DATA_DIR / "Motion.json"
PFEEDER_PATH = DATA_DIR / "PreFeeder.json"
CYCLE_PATH = DATA_DIR / "cycle.json"
PROD_CYCLE_LOG_PATH = DATA_DIR / "cycle_production.log"
SERIAL_PATH = DATA_DIR / "serial.json"
MANT_PATH = DATA_DIR / "mantenimiento.json"
CHECKLIST_LOG_PATH = DATA_DIR / "checklist.json"

# Vías de operador. No son parámetros del ciclo automático.
PURGE_PARK_MM = 250.0
PURGE_DEPOSIT_MM = 100.0
PURGE_CLEAN_MS = 3000
PURGE_AFTER_CUT_MS = 500
# La longitud del modelo es la pieza. La LM Guide recorre esa longitud menos este offset.
LMGUIDE_PIECE_OFFSET_MM = 55.0
CHECKLIST_MAX_CYCLES = 10000

# ServoLMGuide.h [7]: 770 PUU en 45 mm. El sentido de trabajo es PUU negativo.
LMGUIDE_STEPS_PER_MM = 770.0 / 45.0
LMGUIDE_PUU_SIGN = -1
LMGUIDE_PR_RPM_MIN = 400.0
LMGUIDE_PR_RPM_MAX = 3000.0

# Reset de error: condiciones PLC/Motion despejadas de forma estable (ms).
_ERROR_RESET_STABLE_MS = 200

# ServoFeeder.h [8]: 3600 counts = 55 mm de cinta. Vale para mm/s → 6081 y mm → 607A.
FEED_STEPS_PER_MM = 3600.0 / 55.0

# -----------------------------------------------------------------------------
# ANDON_SKIP_OPCIONAL — TODO_BORRAR
# Puente de banco: permitir ciclo sin COM del Andon (env HMI_ANDON_SKIP=1 o UI
# "No usar Andon"). Cuando en planta siempre haya cable, borrar todo el bloque
# marcado con este tag en app.py, static/js/main.js y el párrafo en rutinas-operador.md.
# -----------------------------------------------------------------------------
_ANDON_SKIPPED_ENV = os.environ.get("HMI_ANDON_SKIP", "0").strip() == "1"
_ANDON_NONE_TOKEN = "__none__"
# --- fin constantes ANDON_SKIP_OPCIONAL ---

_serial_lock = threading.Lock()
_motion_write_lock = threading.Lock()
_motion_serial = None
_serial_cfg_lock = threading.Lock()
_serial_cfg = {
    "motion": "",
    "plc": "",
    "andon": "",
    "prefeeder": "",
    "andon_disabled": False,  # ANDON_SKIP_OPCIONAL — TODO_BORRAR (quitar clave y usos)
    "checklist_bypass": False,
}

# Orquestador del ciclo automático (rules.md §4.1–4.8) — app.py coordina Motion + PLC.
_STATE_LOCK = threading.Lock()
# Candado de arranque: un solo Start/Resume/Charola a la vez (evita workers gemelos).
_CYCLE_SPAWN_LOCK = threading.Lock()
_CYCLE_WORKER_TLS = threading.local()
_CYCLE_CTRL = {
    "thread": None,
    # Generación del worker oficial. Un hilo huérfano ve gen ≠ la suya y se detiene.
    "gen": 0,
    "stop": threading.Event(),
    "pause_after_step": threading.Event(),
    # Prueba desde Parámetros → Ciclo: un paso, pausa, «Siguiente paso».
    "step_mode": False,
}

# Hilo de purga, posición inicial, alimentar un lado y piezas de prueba.
# El ciclo automático no usa este hilo.
_ROUTINE = {
    "thread": None,
    "stop": threading.Event(),
    "from_lote": False,
}

# Interlocks del orquestador (estado lógico tras pasos completados en este ciclo).
_CYCLE_LATCH = {
    "holder_encoder_active": False,
    "feed_validated": False,
    "gripper_active": False,
    "holder_encoder_released": False,
    "lm_at_cut": False,
    "cut_done": False,
    "discharge_done": False,
}
# Pinzas cerradas con pieza en proceso (paso 4 → antes de soltar en paso 7).
# Ante error se retiene; no se re-cierra si ya estaban abiertas.
_GRIPPER_HOLDING = False

# Lote en curso. El batch (montón de la charola) se calcula al pulsar Start.
_LOT = {
    "active": False,
    "tray_wait": False,
    "home_before": False,
    "cycles_total": 0,
    "cycles_done": 0,
    "stacks": 0,
    "deposits_per_tray": 0,
    "batch_size": 50,
    "cut_mm": 0.0,
    "cut_offset_mm": 0.0,
    "cut_comp_mode": False,
    "cut_comp_r_mm": 0.0,
    "cut_comp_l_mm": 0.0,
    "length_mm": 0.0,
    "gap_mm": 0.0,
    "clearance_mm": 0.0,
    "sides": "R",
}
_LAST_PIECE_S = 0.0
# Marca de fin de la pieza anterior (lote). Tiempo de ciclo en régimen = cadencia entre salidas.
_PREV_PIECE_END_T: float | None = None
# Reloj vivo de la pieza / lote (para HMI: estimado y corriendo).
_PIECE_TICK0: float | None = None
_LOT_T0: float | None = None
# Longitud programada de la pieza en curso (para duración del blower).
# Se conserva aunque el paso 3 del ciclo siguiente termine durante el regreso a home.
_PIECE_BLOWER_MM = None
# Lectura de encoder L/R de la pieza en curso (no se pisa con el solape).
_PIECE_FEED = {"R": None, "L": None}

# Siguiente ciclo arrancado al soltar pinzas (paso 7).
# Los pasos 1 a 3 pueden terminar mientras el eje sigue hacia home.
# El paso 4 no cierra pinzas hasta que el paso 8 anterior acabó:
# eje en home y pinzas abiertas.
_OVERLAP = {
    "gen": 0,
    "on": False,
    "feed_sent": False,
    "feed_done": False,
    "post_enc_reset_done": False,
    "holder_error": "",
    "holder_ready": threading.Event(),
    "feed": None,
    "blower_mm": None,
    "t0": None,
    "feed_error": False,
    "feed_task_started": False,
    "feed_t0": None,
}
# El siguiente paso 4 solo cierra: home ya está hecho y las pinzas siguen abiertas.
_STEP4_CLOSES = False

# Pasos ejecutables en orden (1 = arranque / set_data al Motion).
# Con otro ciclo pendiente, el paso 7 dispara de nuevo el 1 sin esperar al 8.
CYCLE_STEPS = (
    {"id": "1", "label": "Start — inicio de secuencia", "actor": "coordinator"},
    {"id": "2", "label": "Holder dual + encoder dual", "actor": "plc"},
    {"id": "3", "label": "Alimentación y medición L/R", "actor": "motion"},
    {"id": "4", "label": "Pinzas — liberar holder", "actor": "plc"},
    {"id": "5", "label": "LM Guide — posición de corte", "actor": "motion"},
    {"id": "6", "label": "Corte R/L", "actor": "plc"},
    {"id": "7", "label": "Descarga de pieza", "actor": "motion+plc"},
    {"id": "8", "label": "Home LM Guide + blower", "actor": "motion+plc"},
)

# ── Registro de producción (una línea por operación; no sustituye [Motion]/[PLC]) ──
_LOT_SEQ = 0
_PROD_CADENCE_SAMPLES: list[float] = []
_PROD_LOG_LOCK = threading.Lock()
_PROD_CTX = {"piece": 1, "step_id": "1", "t0": 0.0}
_PROD_STEP_DESC = {
    "1": "Configuración inicial",
    "2": "Holder + encoder",
    "3": "Alimentación L/R",
    "4": "Pinzas — liberar holder",
    "5": "LM Guide — corte",
    "6": "Corte R/L",
    "7": "Descarga de pieza",
    "8": "Regreso Home + blower",
}


def _prod_lot_tag() -> str:
    """LOTE = piezas totales del pedido (cantidad), no un contador secuencial de arranques."""
    if _LOT.get("active"):
        total = int(_LOT.get("cycles_total") or 0)
    else:
        total = int(_MACHINE_STATE.get("cantidad") or 0)
    return f"LOTE {max(total, 0):03d}"


def _prod_piece_tag(piece: int | None = None, total: int | None = None) -> str:
    """PIEZA = realizadas/totales (durante el paso, la pieza en curso cuenta como realizada)."""
    if total is None:
        if _LOT.get("active"):
            total = int(_LOT["cycles_total"] or 0)
        else:
            total = int(_MACHINE_STATE.get("cantidad") or 0)
        total = max(total, 1)
    if piece is None:
        piece = int(_LOT["cycles_done"]) + 1 if _LOT.get("active") else 1
    piece = max(0, min(int(piece), int(total)))
    return f"PIEZA {piece:03d}/{total:03d}"


def _prod_actor(step_id: str) -> str:
    _, meta = _cycle_step_meta(step_id)
    if not meta:
        return "MOTION"
    actor = str(meta.get("actor") or "motion").lower()
    if actor == "coordinator":
        return "MOTION"
    if actor == "plc":
        return "PLC"
    if actor == "motion":
        return "MOTION"
    if "+" in actor or "motion" in actor and "plc" in actor:
        return "PLC+MOTION"
    return actor.upper()


def _prod_write(line: str) -> None:
    with _PROD_LOG_LOCK:
        PROD_CYCLE_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with PROD_CYCLE_LOG_PATH.open("a", encoding="utf-8") as f:
            f.write(line + "\n")
    print(f"[PROD] {line}", flush=True)
    _push_event(line, "prod")


def _prod_enc_mm(feed: dict | None, side: str) -> str:
    block = (feed or {}).get(side)
    if not isinstance(block, dict) or block.get("mm") is None:
        return "-"
    try:
        return f"{float(block['mm']):.2f}"
    except (TypeError, ValueError):
        return "-"


def _prod_enc_tag(feed: dict | None = None) -> str:
    src = feed if feed is not None else _PIECE_FEED
    return f"ENC R={_prod_enc_mm(src, 'R')} mm L={_prod_enc_mm(src, 'L')} mm"


def _prod_log_op(
    step_id: str,
    duration_s: float,
    *,
    piece: int | None = None,
    parallel: bool = False,
    status: str = "OK",
    detail: str = "",
    feed: dict | None = None,
) -> None:
    total = int(_LOT["cycles_total"] or 1) if _LOT.get("active") else 1
    if piece is None:
        piece = int(_LOT["cycles_done"]) + 1 if _LOT.get("active") else 1
    try:
        step_no = f"STEP {int(step_id):02d}"
    except (TypeError, ValueError):
        step_no = f"STEP {step_id or '?'}"
    desc = _PROD_STEP_DESC.get(str(step_id), str(step_id or "?"))
    actor = _prod_actor(step_id)
    flag = status if status not in ("OK",) else ("PARALLEL" if parallel else "OK")
    line = (
        f"{_prod_lot_tag()} | {_prod_piece_tag(piece, total)} | {step_no} | {desc} | "
        f"{actor} | {duration_s:.3f} s | {flag}"
    )
    if detail and flag == "ERROR":
        line += f" | {detail}"
    if step_id == "3" and flag in ("OK", "PARALLEL"):
        line += f" | {_prod_enc_tag(feed)}"
    _prod_write(line)


def _prod_log_complete(duration_s: float, kind: str) -> None:
    piece = int(_LOT["cycles_done"]) + 1 if _LOT.get("active") else 1
    total = int(_LOT["cycles_total"] or piece) if _LOT.get("active") else 1
    line = (
        f"{_prod_lot_tag()} | {_prod_piece_tag(piece, total)} | COMPLETE | "
        f"{duration_s:.3f} s | {kind} | {_prod_enc_tag()}"
    )
    _prod_write(line)


def _prod_log_summary(done: int, total: int, avg_cadence: float) -> None:
    line = (
        f"{_prod_lot_tag()} | SUMMARY | {done}/{total} | "
        f"AVG CADENCE: {avg_cadence:.3f} s"
    )
    _prod_write(line)


def _prod_log_error(code: str, mensaje: str) -> None:
    """Registra error de producción; alimentación en paralelo usa pieza/paso del solape."""
    detail = mensaje or code
    if (
        _OVERLAP.get("on")
        and _OVERLAP.get("feed_sent")
        and not _OVERLAP.get("feed_done")
    ):
        piece = int(_LOT["cycles_done"]) + 2
        step_id = "3"
        t0 = float(_OVERLAP.get("feed_t0") or time.perf_counter())
        dt = max(0.0, time.perf_counter() - t0)
        parallel = True
    elif _cycle_thread_alive():
        piece = int(_LOT["cycles_done"]) + 1 if _LOT.get("active") else 1
        with _STATE_LOCK:
            step_id = str(_MACHINE_STATE.get("ciclo_paso_id") or "?")
        ctx = _PROD_CTX
        t0 = float(ctx.get("t0") or time.perf_counter())
        dt = max(0.0, time.perf_counter() - t0)
        parallel = bool(ctx.get("parallel"))
    else:
        ctx = _PROD_CTX
        step_id = str(ctx.get("step_id") or "?")
        t0 = float(ctx.get("t0") or time.perf_counter())
        dt = max(0.0, time.perf_counter() - t0)
        piece = int(ctx.get("piece") or 1)
        parallel = bool(ctx.get("parallel"))
    _prod_log_op(
        step_id,
        dt,
        piece=piece,
        parallel=parallel,
        status="ERROR",
        detail=detail,
    )

_DEFAULT_CYCLE_PARAMS = {
    "ms_holder_enc_stable": 100,
    "ms_after_feed": 100,
    "ms_lm_at_cut": 100,
    "ms_lm_at_discharge": 100,
    "ms_cutter_on": 50,
    "ms_after_cutter": 100,
    "ms_after_grip_release": 100,
    "ms_blower_delay": 200,
    "ms_at_home_end": 50,
    "tolerance_mm": 2.0,
    "cut_offset_mm": 0.0,  # compensación general (modo dual apagado)
    "cut_comp_mode": False,
    "cut_comp_r_mm": 0.0,
    "cut_comp_l_mm": 0.0,
    "deposit_batch_size": 50,
    "deposit_stack_gap_mm": 100.0,
    "deposit_max_travel_mm": 1500.0,
    "gripper_clearance_mm": 20.0,
    "feed_sides": "R",
    "blower_ms_per_mm": 8.0,
    "blower_ms_min": 200,
}

# Persistidos en cycle.json junto al ciclo (no son tiempos del orquestador).
_HMI_CYCLE_EXTRA_KEYS = ("cantidad", "numero_modelo")

# Estado inicial (orquestador + UI)
_MACHINE_STATE = {
    "estado": "idle",
    "estado_label": "En espera",
    "mensaje": "Sistema listo. Sin errores activos.",
    "progreso_pct": 0,
    "ciclo_paso_id": "",
    "ciclo_paso_label": "",
    "ciclo_paso_indice": 0,
    "ciclo_paralelo_id": "",
    "ciclo_en_curso": False,
    "ciclo_error": "",
    "ciclo_feed": {"R": None, "L": None},
    "numero_modelo": "",
    "cantidad": 1,
    "longitud_modelo_mm": None,
    "compensacion_medicion_mm": 0.0,  # compensación general; se usa si cut_comp_mode está apagado
    "tiempo_ciclo_s": None,
    "tiempo_estimado_s": 0,
    "tiempo_corriendo_s": 0,
    "eventos": [],
    "rutina": "",
    "rutina_fase": "",
    "rutina_manos": False,
    # Tras cualquier error: Reset → purga → Reanudar reintenta la pieza desde el paso 1
    # (el lote y cycles_done se conservan; no se cuenta ni se descarta la pieza).
    "post_error_purga": False,
    # Flujo guiado de recuperación (popup): error → reset → piezas → manguera → purga → listo.
    "recovery": {
        "active": False,
        "step": "",
        "code": "",
        "mensaje": "",
    },
    "omitir_holgura_l": False,
    "omitir_holgura_r": False,
    "ciclos_mant": 0,
    "checklist": {
        "ok": os.environ.get("HMI_CHECKLIST_SKIP", "0").strip() == "1",
        "paso": 1,
        "empleado": "",
        "aire_ok": False,
        "eje_ok": False,
        "recorrido_ok": False,
        "alimentador_ok": False,
        "purga_ok": False,
        "piezas_cortadas": False,
    },
}

def _dig(src, path, default):
    cur = src
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _as_float(src, path, default):
    try:
        return float(_dig(src, path, default))
    except (TypeError, ValueError):
        return float(default)


def _as_int(src, path, default):
    try:
        return int(round(float(_dig(src, path, default))))
    except (TypeError, ValueError):
        return int(default)


def _clamp_lm_pr_speed_rpm(value) -> float:
    try:
        x = float(value)
    except (TypeError, ValueError):
        x = 1200.0
    return max(LMGUIDE_PR_RPM_MIN, min(LMGUIDE_PR_RPM_MAX, x))


def _as_bool(src, path, default=False) -> bool:
    value = _dig(src, path, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "on", "yes")
    return bool(value)


def _canonical_motion_tune(src):
    """Parámetros persistidos en Motion.json. Defaults alineados con Parameters.h."""
    src = src if isinstance(src, dict) else {}
    return {
        "feeder": {
            "R": {
                "profile_velocity_pps": _as_int(src, ("feeder", "R", "profile_velocity_pps"), 42500),
                "target_position_counts": _as_int(src, ("feeder", "R", "target_position_counts"), 42500),
                "compensation_mm": _as_float(src, ("feeder", "R", "compensation_mm"), 0.0),
            },
            "L": {
                "profile_velocity_pps": _as_int(src, ("feeder", "L", "profile_velocity_pps"), 42500),
                "target_position_counts": _as_int(src, ("feeder", "L", "target_position_counts"), 42500),
                "compensation_mm": _as_float(src, ("feeder", "L", "compensation_mm"), 0.0),
            },
        },
        "lmguide": {
            "pr_speed_rpm": _clamp_lm_pr_speed_rpm(_as_float(src, ("lmguide", "pr_speed_rpm"), 1200.0)),
            "pos_puu": _as_int(src, ("lmguide", "pos_puu"), 0),
        },
        "feed_stage": {
            "validate_encoder": _as_bool(src, ("feed_stage", "validate_encoder"), True),
        },
    }


def _save_motion_tune(tune):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    MOTION_PATH.write_text(
        json.dumps(tune, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _load_motion_tune():
    if not MOTION_PATH.exists():
        tune = _canonical_motion_tune({})
        _save_motion_tune(tune)
        return tune
    with MOTION_PATH.open(encoding="utf-8") as f:
        data = json.load(f)
    return _canonical_motion_tune(data)


def _canonical_prefeeder_side(src: dict, side: str) -> dict:
    src = src if isinstance(src, dict) else {}
    defaults = {
        "L": {
            "rpm": 60.0,
            "puller_rpm": 60.0,
            "settle_ms": 150,
            "on_s": 2.0,
            "off_fault_s": 5.0,
            "stall_fault_s": 2.0,
        },
        "R": {
            "rpm": 60.0,
            "puller_rpm": 60.0,
            "settle_ms": 150,
            "on_s": 2.0,
            "off_fault_s": 5.0,
            "stall_fault_s": 2.0,
        },
    }[side]
    block = src.get(side) if isinstance(src.get(side), dict) else {}
    return {
        "rpm": _as_float(block, ("rpm",), defaults["rpm"]),
        "puller_rpm": _as_float(block, ("puller_rpm",), defaults["puller_rpm"]),
        "settle_ms": _as_int(block, ("settle_ms",), defaults["settle_ms"]),
        "on_s": _as_float(block, ("on_s",), defaults["on_s"]),
        "off_fault_s": _as_float(block, ("off_fault_s",), defaults["off_fault_s"]),
        "stall_fault_s": _as_float(block, ("stall_fault_s",), defaults["stall_fault_s"]),
    }


def _canonical_prefeeder_tune(src):
    src = src if isinstance(src, dict) else {}
    return {
        "L": _canonical_prefeeder_side(src, "L"),
        "R": _canonical_prefeeder_side(src, "R"),
    }


def _save_prefeeder_tune(tune: dict) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PFEEDER_PATH.write_text(
        json.dumps(tune, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _load_prefeeder_tune():
    if not PFEEDER_PATH.exists():
        tune = _canonical_prefeeder_tune({})
        _save_prefeeder_tune(tune)
        return tune
    with PFEEDER_PATH.open(encoding="utf-8") as f:
        data = json.load(f)
    return _canonical_prefeeder_tune(data)


def _prefeeder_cfg_signature(side_cfg: dict) -> tuple:
    c = side_cfg
    return (
        round(c["rpm"], 2),
        round(c["puller_rpm"], 2),
        c["settle_ms"],
        round(c["on_s"], 3),
        round(c["off_fault_s"], 3),
        round(c["stall_fault_s"], 3),
    )


# Datos de ciclo en ESP32 (NVS); la app los empuja con set_data / modelo.
_MOTION_DATA = {
    "target_length_mm": None,
    "tolerance_mm": 2.0,
    "modelo_id": "",
    "measure_comp_mm": 0.0,
}


def _motion_tune_flat(tune: dict) -> dict:
    tune = _canonical_motion_tune(tune)
    feed = tune["feeder"]
    lm = tune["lmguide"]
    return {
        "op": "set_tune",
        "feed_r_vel_pps": feed["R"]["profile_velocity_pps"],
        "feed_r_target_counts": feed["R"]["target_position_counts"],
        "feed_l_vel_pps": feed["L"]["profile_velocity_pps"],
        "feed_l_target_counts": feed["L"]["target_position_counts"],
        "feed_r_comp_mm": feed["R"]["compensation_mm"],
        "feed_l_comp_mm": feed["L"]["compensation_mm"],
        "lm_pr_speed_rpm": lm["pr_speed_rpm"],
        "lm_pos_puu": lm["pos_puu"],
        "feed_validate_encoder": tune["feed_stage"]["validate_encoder"],
    }


def _compact(obj) -> str:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False)


def _mm_to_puu(mm) -> int:
    return int(round(LMGUIDE_PUU_SIGN * float(mm) * LMGUIDE_STEPS_PER_MM))


def _num(value, default=0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _feed_sides_of(params: dict) -> str:
    raw = str(params.get("feed_sides") or "R").strip().upper().replace(" ", "").replace("+", "")
    if raw in ("LR", "RL", "BOTH", "AMBOS"):
        return "LR"
    if raw == "L":
        return "L"
    return "R"


def _holgura_omit_effective(
    sides: str | None = None,
    *,
    omit_l: bool | None = None,
    omit_r: bool | None = None,
) -> tuple[bool, bool]:
    """Omit manual + auto: un solo lado activo omite holgura del contrario."""
    if omit_l is None or omit_r is None:
        with _STATE_LOCK:
            if omit_l is None:
                omit_l = bool(_MACHINE_STATE.get("omitir_holgura_l"))
            if omit_r is None:
                omit_r = bool(_MACHINE_STATE.get("omitir_holgura_r"))
    raw = str(sides if sides is not None else _run_sides() or "R").strip().upper()
    if raw == "L":
        omit_r = True
    elif raw == "R":
        omit_l = True
    return bool(omit_l), bool(omit_r)


_PFEEDER_CODES_L = frozenset({"E093", "E094", "E095"})
_PFEEDER_CODES_R = frozenset({"E096", "E097", "E098"})


def _active_feed_sides(sides: str | None = None) -> tuple[str, ...]:
    """Lados que cuentan para Motion/PreFeeder según feed_sides."""
    raw = str(sides if sides is not None else _run_sides() or "R").strip().upper()
    if raw == "LR":
        return ("L", "R")
    if raw == "L":
        return ("L",)
    return ("R",)


def _parse_bool(value, default: bool = False) -> bool:
    """Convierte checkbox/JSON suelto a bool. No confundir con _as_bool(src, path, default)."""
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return value != 0
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "si", "sí"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return default


def _measure_comp_mm() -> float:
    """Compensación general de corte (pantalla principal). Se usa con modo dual apagado."""
    # Sin lock: lectura de float; evita deadlock si se llama desde _public_machine_state.
    return _num(_MACHINE_STATE.get("compensacion_medicion_mm"), 0.0)


def _mirror_comp_to_cycle_json(mm: float) -> None:
    """Mantiene cut_offset_mm en cycle.json alineado con la compensación general."""
    _ = mm
    _save_hmi_cycle_extras()


def _cut_base_mm(length: float) -> float:
    """Posición base de corte: |largo| − 55 mm (sin compensación)."""
    return abs(length) - LMGUIDE_PIECE_OFFSET_MM


def _cut_comp_side_mm(params: dict, side: str) -> float:
    if side == "R":
        return _num(params.get("cut_comp_r_mm"), 0.0)
    return _num(params.get("cut_comp_l_mm"), 0.0)


def _general_cut_comp_mm(params: dict | None = None) -> float:
    """Offset general vivo del HMI. En lote activo se congela con _lot_cut_offset."""
    if isinstance(params, dict) and params.get("_lot_cut_offset") is not None:
        return _num(params.get("_lot_cut_offset"), 0.0)
    return _measure_comp_mm()


def _cut_comp_params_of(params: dict | None = None) -> dict:
    src = params if isinstance(params, dict) else _load_cycle_params()
    return {
        "cut_comp_mode": _parse_bool(src.get("cut_comp_mode"), False),
        "cut_comp_r_mm": _num(src.get("cut_comp_r_mm"), 0.0),
        "cut_comp_l_mm": _num(src.get("cut_comp_l_mm"), 0.0),
        "cut_offset_mm": _general_cut_comp_mm(src),
    }


def _build_cut_plan(length: float, params: dict | None, sides: str) -> dict:
    """
    Plan de corte LM Guide.
    - Modo off: un solo movimiento a base + compensación general.
    - Modo on + un lado: base + compensación de ese lado (sin general).
    - Modo on + LR y compensaciones distintas: primero el offset menor, luego el mayor.
    - Modo on + LR e iguales: un movimiento a base+offset; ambos cortadores.
    last_mm es la última posición de corte (referencia de depósito/despeje).
    """
    base = _cut_base_mm(length)
    sides = str(sides or "R").strip().upper()
    if sides not in ("R", "L", "LR"):
        sides = "R"
    comps = _cut_comp_params_of(params)
    mode = bool(comps["cut_comp_mode"])
    general = float(comps["cut_offset_mm"]) if not mode else 0.0
    comp_r = float(comps["cut_comp_r_mm"]) if mode else 0.0
    comp_l = float(comps["cut_comp_l_mm"]) if mode else 0.0

    def side_name(side: str) -> str:
        return {"R": "derecho", "L": "izquierdo", "LR": "L y R"}[side]

    def valve_of(side: str) -> str:
        return {"R": "cutter_r", "L": "cutter_l", "LR": "cutters"}[side]

    def single(side: str, mm: float) -> dict:
        return {
            "mode": mode,
            "sequential": False,
            "base_mm": base,
            "general_mm": general,
            "last_mm": mm,
            "first_side": side,
            "first_mm": mm,
            "first_valve": valve_of(side),
            "first_label": side_name(side),
            "second_side": "",
            "second_mm": mm,
            "second_valve": "",
            "second_label": "",
            "comp_r_mm": comp_r,
            "comp_l_mm": comp_l,
        }

    if not mode:
        mm = base + general
        return single(sides if sides in ("R", "L") else "LR", mm)

    if sides == "R":
        return single("R", base + comp_r)
    if sides == "L":
        return single("L", base + comp_l)

    # Ambos lados, modo on.
    applicable = abs(comp_r - comp_l) >= 0.001
    if not applicable:
        return single("LR", base + comp_r)

    # Offset menor primero, mayor al final (despeje sobre la última posición).
    first_side, second_side = ("R", "L") if comp_r <= comp_l else ("L", "R")
    first_comp = comp_r if first_side == "R" else comp_l
    second_comp = comp_r if second_side == "R" else comp_l
    first_mm = base + first_comp
    second_mm = base + second_comp
    return {
        "mode": mode,
        "sequential": True,
        "base_mm": base,
        "general_mm": 0.0,
        "last_mm": second_mm,
        "first_side": first_side,
        "first_mm": first_mm,
        "first_valve": valve_of(first_side),
        "first_label": side_name(first_side),
        "second_side": second_side,
        "second_mm": second_mm,
        "second_valve": valve_of(second_side),
        "second_label": side_name(second_side),
        "comp_r_mm": comp_r,
        "comp_l_mm": comp_l,
    }


def _cut_mm_of(length: float, params: dict | None = None) -> float:
    """Última posición de corte (referencia de depósito/despeje)."""
    src = params if isinstance(params, dict) else _load_cycle_params()
    plan = _build_cut_plan(length, src, _feed_sides_of(src))
    return float(plan["last_mm"])


def _active_cut_plan() -> dict | None:
    """Plan de corte del ciclo en curso. None si no hay longitud."""
    if _LOT["active"]:
        length = float(_LOT.get("length_mm") or 0.0)
        if length <= 0:
            return None
        params = {
            "_lot_cut_offset": _LOT.get("cut_offset_mm"),
            "cut_comp_mode": _LOT.get("cut_comp_mode"),
            "cut_comp_r_mm": _LOT.get("cut_comp_r_mm"),
            "cut_comp_l_mm": _LOT.get("cut_comp_l_mm"),
        }
        return _build_cut_plan(length, params, _LOT.get("sides") or "R")
    params = _load_cycle_params()
    length = _target_length_mm()
    if length <= 0:
        return None
    return _build_cut_plan(length, params, _feed_sides_of(params))


def _clear_of(deposit: float, clearance: float) -> float:
    clearance = abs(clearance)
    if deposit < 0:
        return deposit - clearance
    return deposit + clearance


def _stack_count(cut: float, length: float, gap: float, clearance: float, travel: float) -> int:
    """Montones que caben. 0 = sin tope de carrera."""
    if travel <= 0:
        return 0
    step = abs(length) + gap
    deposit1 = cut + gap
    if gap >= 0 and deposit1 >= 0 and step > 0:
        room = travel - abs(clearance) - deposit1
        if room < 0:
            return 0
        return 1 + int(room // step)
    if step == 0:
        return 1
    n = 0
    while n < 100000:
        deposit = cut + gap + n * step
        if abs(_clear_of(deposit, clearance)) > travel:
            break
        n += 1
    return n


def _tray_plan(length: float, params: dict):
    """Plan de charola para esta longitud. None si el corte o el primer montón no caben."""
    batch = int(_num(params.get("deposit_batch_size"), 50))
    if batch < 1:
        return None, "El tamaño del batch tiene que ser al menos 1."
    gap = _num(params.get("deposit_stack_gap_mm"), 100.0)
    clearance = abs(_num(params.get("gripper_clearance_mm"), 20.0))
    travel = _num(params.get("deposit_max_travel_mm"), 0.0)
    cut = _cut_mm_of(length, params)
    deposit1 = cut + gap
    clear1 = _clear_of(deposit1, clearance)
    if travel > 0:
        passed = []
        if abs(cut) > travel:
            passed.append(f"corte en {cut:.1f} mm")
        if abs(clear1) > travel:
            passed.append(f"primer montón con despeje en {clear1:.1f} mm")
        if passed:
            return None, (
                f"No cabe en la carrera de {travel:.1f} mm: " + " y ".join(passed) + "."
            )
    stacks = _stack_count(cut, length, gap, clearance, travel)
    if travel > 0 and stacks < 1:
        return None, (
            f"No cabe en la carrera de {travel:.1f} mm: "
            f"primer montón con despeje en {clear1:.1f} mm."
        )
    return {
        "batch_size": batch,
        "gap_mm": gap,
        "clearance_mm": clearance,
        "cut_mm": cut,
        "length_mm": abs(length),
        "stacks": stacks,
        "deposits": (stacks * batch) if stacks else 0,
    }, ""


def _order_cycles(qty, sides: str):
    try:
        n = int(qty)
    except (TypeError, ValueError):
        n = 0
    if sides == "LR":
        if n < 2 or n % 2:
            return 0, "Con ambos lados la cantidad tiene que ser par y de al menos 2 piezas."
        return n // 2, ""
    if n < 1:
        return 0, "La cantidad es el número de ciclos y tiene que ser al menos 1."
    return n, ""


def _deposit_at(cut: float, length: float, gap: float, batch: int, stacks: int, cycle: int):
    idx0 = (max(cycle, 1) - 1) // max(batch, 1)
    if stacks:
        idx0 %= stacks
    deposit = cut + gap + idx0 * (abs(length) + gap)
    return deposit, idx0 + 1


def _active_cut_deposit():
    """Corte y depósito del ciclo que va a cortarse. None si no hay longitud."""
    if _LOT["active"]:
        cycle = _LOT["cycles_done"] + 1
        deposit, stack = _deposit_at(
            _LOT["cut_mm"],
            _LOT["length_mm"],
            _LOT["gap_mm"],
            _LOT["batch_size"],
            _LOT["stacks"],
            cycle,
        )
        return _LOT["cut_mm"], deposit, stack
    params = _load_cycle_params()
    length = _target_length_mm()
    if length <= 0:
        return None
    cut = _cut_mm_of(length, params)
    gap = _num(params.get("deposit_stack_gap_mm"), 0.0)
    batch = int(_num(params.get("deposit_batch_size"), 50))
    deposit, stack = _deposit_at(cut, length, gap, max(batch, 1), 0, 1)
    return cut, deposit, stack


def _lm_cut_and_discharge(params: dict) -> tuple[float, float] | None:
    """Corte = |L| − 55 mm + offset. Descarga = depósito del primer montón."""
    length = _target_length_mm()
    if length <= 0:
        return None
    cut = _cut_mm_of(length, params)
    gap = _num(params.get("deposit_stack_gap_mm"), 0.0)
    return cut, cut + gap


def _run_sides() -> str:
    if _MACHINE_STATE.get("rutina") == "checklist":
        return "LR"
    if _LOT["active"]:
        return _LOT["sides"]
    return _feed_sides_of(_load_cycle_params())


def _cycle_piece_prefix() -> str:
    if not _LOT["active"]:
        return ""
    return f"Ciclo {_LOT['cycles_done'] + 1} de {_LOT['cycles_total']}. "


def _announce_trays(cycles: int, plan: dict) -> None:
    deposits = int(plan["deposits"] or 0)
    if not deposits or cycles <= deposits:
        return
    trays = math.ceil(cycles / deposits)
    _push_event(
        f"Caben {plan['stacks']} montones, {deposits} depósitos por charola. "
        f"El lote pide {trays} charolas y pausa cada {deposits} ciclos."
    )


def _motion_scale() -> dict:
    return {
        "steps_per_mm": LMGUIDE_STEPS_PER_MM,
        "puu_sign": LMGUIDE_PUU_SIGN,
        "feed_steps_per_mm": FEED_STEPS_PER_MM,
    }


def _build_motion_lines(op: str, tune: dict, body: dict):
    """Líneas Serial compactas. El firmware busca \"op\":\"...\" sin espacios."""
    tune = _canonical_motion_tune(tune)
    tune_line = _compact(_motion_tune_flat(tune))

    if op == "set_tune":
        return tune, [tune_line]
    if op == "encoder_reset":
        side = str(body.get("side") or "").strip().upper()
        if side in ("R", "L"):
            return tune, [_compact({"op": "encoder_reset", "side": side})]
        # Sin lado / both / LR: firmware resetea R y L.
        return tune, [_compact({"op": "encoder_reset"})]
    if op == "feeder_init":
        return tune, [tune_line, '{"op":"feeder_init","side":"both"}']
    if op == "motion_hw_init":
        # set_tune + un solo bloque en firmware (CAN + feeders + LM home)
        return tune, [tune_line, '{"op":"motion_hw_init"}']
    if op == "lmguide_init":
        return tune, ['{"op":"lmguide_servo_on"}']
    if op == "lmguide_servo":
        if body.get("on"):
            return tune, ['{"op":"lmguide_servo_on"}']
        return tune, ['{"op":"lmguide_servo_off"}']
    if op == "lmguide_home":
        return tune, ['{"op":"lmguide_home"}']
    if op == "lmguide_move_home":
        return tune, [
            _compact(
                {
                    "op": "lmguide_move",
                    "target": "home",
                    "lm_pr_speed_rpm": tune["lmguide"]["pr_speed_rpm"],
                }
            )
        ]
    if op == "lmguide_move":
        if body.get("mm") is None or body.get("mm") == "":
            raise ValueError("Falta la posición en mm.")
        puu = _mm_to_puu(body.get("mm"))
        tune["lmguide"]["pos_puu"] = puu
        return tune, [
            _compact(
                {
                    "op": "lmguide_move",
                    "puu": puu,
                    "lm_pr_speed_rpm": tune["lmguide"]["pr_speed_rpm"],
                }
            )
        ]
    if op == "lmguide_jog":
        if body.get("jog_mm") is None or body.get("jog_mm") == "":
            raise ValueError("Falta el jog en mm.")
        jog_mm = float(body.get("jog_mm"))
        if jog_mm == 0:
            raise ValueError("El jog en mm tiene que ser distinto de cero.")
        direction = 1 if int(body.get("dir") or 1) >= 0 else -1
        delta = _mm_to_puu(direction * abs(jog_mm))
        return tune, [
            _compact(
                {
                    "op": "lmguide_jog",
                    "delta_puu": delta,
                    "lm_pr_speed_rpm": tune["lmguide"]["pr_speed_rpm"],
                }
            )
        ]
    if op == "lmguide_stop":
        return tune, ['{"op":"lmguide_stop"}']
    raise ValueError("Orden Motion no reconocida.")


# Firmware: PARAM_LM_MOVE_DONE_TIMEOUT_MS (45s) × PARAM_LM_MOVE_MAX_ATTEMPTS (1).
# Margen corto sobre el timeout del drive; si se corta antes, aparece un falso E043.
_MOTION_LM_MOVE_TIMEOUT_S = 50.0
_MOTION_STATUS_TIMEOUT_S = 3.0


def _reply_wait_s(line: str) -> float:
    if "lmguide_home" in line or "lmguide_move" in line:
        return _MOTION_LM_MOVE_TIMEOUT_S
    if "motion_hw_init" in line:
        return 55.0
    if "feeder_init" in line or "lmguide_servo_on" in line:
        return 8.0
    if '"op":"status"' in line:
        return _MOTION_STATUS_TIMEOUT_S
    return 2.0


def _load_serial_cfg() -> None:
    motion = os.environ.get("MOTION_SERIAL", "").strip()
    plc = os.environ.get("PLC_SERIAL", "").strip()
    andon = os.environ.get("ANDON_SERIAL", "").strip()
    prefeeder = os.environ.get("PREFEEDER_SERIAL", "").strip()
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR ---
    andon_disabled = False
    checklist_bypass = False
    if SERIAL_PATH.exists():
        try:
            saved = json.loads(SERIAL_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            saved = None
        if isinstance(saved, dict):
            motion = str(saved.get("motion") or "").strip()
            plc = str(saved.get("plc") or "").strip()
            andon = str(saved.get("andon") or "").strip()
            prefeeder = str(saved.get("prefeeder") or "").strip()
            andon_disabled = bool(saved.get("andon_disabled"))
            checklist_bypass = bool(saved.get("checklist_bypass"))
    if andon_disabled:
        andon = ""
    with _serial_cfg_lock:
        _serial_cfg["motion"] = motion
        _serial_cfg["plc"] = plc
        _serial_cfg["andon"] = andon
        _serial_cfg["prefeeder"] = prefeeder
        _serial_cfg["andon_disabled"] = andon_disabled
        _serial_cfg["checklist_bypass"] = checklist_bypass
    # --- fin ANDON_SKIP_OPCIONAL (_load_serial_cfg) ---


def _andon_skipped_effective() -> bool:
    """ANDON_SKIP_OPCIONAL — TODO_BORRAR: True si no se exige/enlaza el MCU Andon."""
    with _serial_cfg_lock:
        disabled = bool(_serial_cfg.get("andon_disabled"))
    return _ANDON_SKIPPED_ENV or disabled


def _checklist_bypassed() -> bool:
    with _serial_cfg_lock:
        return bool(_serial_cfg.get("checklist_bypass"))


def _serial_port(which: str) -> str:
    with _serial_cfg_lock:
        return _serial_cfg.get(which, "").strip()


def _close_motion_serial() -> None:
    global _motion_serial
    with _serial_lock:
        if _motion_serial is not None:
            try:
                _motion_serial.close()
            except Exception:
                pass
            _motion_serial = None


def _close_plc_serial() -> None:
    global _plc_serial
    with _plc_serial_lock:
        if _plc_serial is not None:
            try:
                _plc_serial.close()
            except Exception:
                pass
            _plc_serial = None


def _close_andon_serial() -> None:
    global _andon_serial, _andon_state_sent, _andon_attempted
    with _andon_serial_lock:
        if _andon_serial is not None:
            try:
                _andon_serial.close()
            except Exception:
                pass
            _andon_serial = None
        _andon_state_sent = ""
        _andon_attempted = ""


def _close_prefeeder_serial() -> None:
    global _prefeeder_serial
    with _prefeeder_serial_lock:
        if _prefeeder_serial is not None:
            try:
                _prefeeder_serial.close()
            except Exception:
                pass
            _prefeeder_serial = None


def _invalidate_serial_handle(ser) -> None:
    """Cierra un handle que Windows dejó zombie tras Monitor Serial / desconexión USB."""
    if ser is None:
        return
    try:
        ser.close()
    except Exception:
        pass


def _list_com_ports() -> list[dict]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    found = []
    for port in list_ports.comports():
        device = str(port.device or "").strip()
        if not device:
            continue
        found.append({"device": device, "description": str(port.description or "").strip()})
    found.sort(key=lambda item: item["device"])
    return found


def _match_listed_port(value: str, ports: list[dict]) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    for port in ports:
        if port["device"].upper() == text.upper():
            return port["device"]
    return ""


def _save_serial_cfg(
    motion: str,
    plc: str,
    andon: str,
    andon_disabled: bool,
    checklist_bypass: bool,
    prefeeder: str | None = None,
) -> None:
    """Guarda serial.json. Parámetro andon_disabled: ANDON_SKIP_OPCIONAL — TODO_BORRAR."""
    motion = str(motion or "").strip()
    plc = str(plc or "").strip()
    andon = str(andon or "").strip()
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR ---
    if andon_disabled:
        andon = ""
    # ---
    with _serial_cfg_lock:
        if prefeeder is None:
            prefeeder = str(_serial_cfg.get("prefeeder") or "").strip()
        else:
            prefeeder = str(prefeeder or "").strip()
        prev_motion = _serial_cfg["motion"]
        prev_plc = _serial_cfg["plc"]
        prev_andon = _serial_cfg["andon"]
        prev_pf = str(_serial_cfg.get("prefeeder") or "").strip()
        prev_ad = bool(_serial_cfg.get("andon_disabled"))
        _serial_cfg["motion"] = motion
        _serial_cfg["plc"] = plc
        _serial_cfg["andon"] = andon
        _serial_cfg["prefeeder"] = prefeeder
        _serial_cfg["andon_disabled"] = bool(andon_disabled)
        _serial_cfg["checklist_bypass"] = bool(checklist_bypass)
    SERIAL_PATH.parent.mkdir(parents=True, exist_ok=True)
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR: quitar clave andon_disabled del JSON ---
    SERIAL_PATH.write_text(
        json.dumps(
            {
                "motion": motion,
                "plc": plc,
                "andon": andon,
                "prefeeder": prefeeder,
                "andon_disabled": bool(andon_disabled),
                "checklist_bypass": bool(checklist_bypass),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    if motion != prev_motion:
        _close_motion_serial()
    if plc != prev_plc:
        _close_plc_serial()
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR: la condición incluye prev_ad ---
    if andon != prev_andon or bool(andon_disabled) != prev_ad:
        _close_andon_serial()
    if prefeeder != prev_pf:
        _close_prefeeder_serial()
        _pf_pushed_signature["L"] = None
        _pf_pushed_signature["R"] = None


def _serial_rx_flush(ser) -> None:
    """Descarta datos ya en cola RX (boot/status al conectar) para no mezclar con la respuesta al comando."""
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    try:
        t_end = time.time() + 0.05
        while time.time() < t_end:
            n = getattr(ser, "in_waiting", 0) or 0
            if n:
                ser.read(n)
            else:
                time.sleep(0.01)
    except Exception:
        pass


def _read_serial_line(
    ser, timeout_s: float, carry: bytearray | None = None
) -> str:
    """Lee una línea terminada en \\n.

    Si se pasa carry, los bytes sin \\n se conservan entre llamadas (no se
    reportan como línea). Sin carry, al timeout se devuelve lo parcial — solo
    para lecturas one-shot con espera larga.
    """
    buf = carry if carry is not None else bytearray()
    deadline = time.time() + max(0.0, float(timeout_s))
    while time.time() < deadline:
        # Leer hasta el fin de línea sin consumir bytes de la respuesta
        # siguiente. read(in_waiting) descartaba todo lo posterior al primer \n.
        chunk = ser.read(1)
        if chunk:
            if chunk == b"\n":
                line = bytes(buf).decode("utf-8", "replace").strip()
                del buf[:]
                return line
            if chunk != b"\r":
                buf.extend(chunk)
        else:
            time.sleep(0.02)
    if carry is None and buf:
        return bytes(buf).decode("utf-8", "replace").strip()
    return ""


def _read_lines_until_op(
    ser,
    want_op: str,
    budget_s: float,
    max_lines: int = 800,
    trace_prefix: str | None = None,
    inject=None,
    on_line=None,
) -> list[str]:
    """Lee líneas completas hasta la respuesta pedida, conservando las intermedias.

    inject() corre con el candado de escritura: None sigue esperando, [] no
    escribe, una lista se manda una sola vez. Con inject activo se usan
    rebanadas cortas de lectura; carry evita partir un JSON a mitad de línea
    (p. ej. `{` + resto) y colgar el paso 8 del solape.
    """
    out: list[str] = []
    seen: set[str] = set()
    injected = False
    carry = bytearray()
    deadline = time.time() + max(0.05, budget_s)
    abort_drain_until = 0.0

    def _consume(line: str) -> bool:
        """True si ya tenemos la respuesta pedida (y no falta feed_stage del solape)."""
        if not line:
            return False
        out.append(line)
        if trace_prefix:
            print(f"{trace_prefix} {line}", flush=True)
        try:
            msg = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            return False
        op = msg.get("op")
        if op:
            seen.add(op)
            if on_line is not None:
                on_line(msg)
        if want_op not in seen:
            return False
        overlap_feed_pending = (
            inject is not None
            and _OVERLAP.get("on")
            and _OVERLAP.get("feed_sent")
            and not _OVERLAP.get("feed_done")
            and "feed_stage" not in seen
        )
        return not overlap_feed_pending

    while time.time() < deadline and len(out) < max_lines:
        if _cycle_should_abort():
            # Drenar ~0.4 s lo ya encolado: un paro urgente puede hacer que el
            # Motion responda just en ese momento; no declarar E043 en seco.
            if abort_drain_until <= 0.0:
                abort_drain_until = time.time() + 0.4
            if time.time() >= abort_drain_until:
                break
        if inject is not None and not injected and not _cycle_should_abort():
            with _motion_write_lock:
                extra = inject()
                if extra is not None:
                    injected = True
                    if extra:
                        for line in extra:
                            print(f"[Motion TX] {line}", flush=True)
                            ser.write((line + "\n").encode("ascii", "replace"))
                        ser.flush()
            if injected and extra:
                for line in extra:
                    _push_event(f"[Motion] {line}")
        rem = deadline - time.time()
        if rem <= 0:
            break
        if abort_drain_until > 0.0:
            rem = min(rem, max(0.02, abort_drain_until - time.time()))
        # Con inject pendiente (holder aún no listo) rebanar para reintentar inject;
        # si ya hay bytes parciales, alargar la rebanada para no cortar el JSON.
        if inject is not None and not injected and not carry:
            slice_s = 0.15
        else:
            slice_s = min(2.0, rem)
        line = _read_serial_line(ser, min(slice_s, rem), carry)
        if _consume(line):
            break
        if not line:
            time.sleep(0.02)
    return out


_plc_serial = None
_plc_serial_lock = threading.Lock()
_andon_serial = None
_andon_serial_lock = threading.Lock()
_andon_state_sent = ""
_andon_attempted = ""
_prefeeder_serial = None
_prefeeder_serial_lock = threading.Lock()
_pf_pushed_signature: dict[str, tuple | None] = {"L": None, "R": None}
_pf_last_fault_sig: tuple[str, ...] = ()
_PF_CACHED_STATUS: dict | None = None
_PF_ERROR_LATCH = False
_PF_STABILIZE_TIMEOUT_S = 45.0
_PF_SERIAL_REPLY_S = 6.0
_PF_SOFT_ERRORS = frozenset({"E090", "E091", "E092", "E099"})
# Errores de enlace/COM: el Reset de recuperación no puede exigir status del módulo caído.
_RECOVERY_LINK_SOFT_RESET = frozenset({
    "E017", "E018", "E019",
    "E041", "E042", "E043",
    "E070", "E071", "E072",
    "E090", "E091", "E092",
})
_PF_MANUAL_HOLD = {"L": False, "R": False}
_PF_MANUAL_HOLD_LOCK = threading.Lock()
_PF_UI_REFRESH_AT = 0.0
_PLC_VALVES = ("holder", "encoder", "cutter_r", "cutter_l", "gripper")
# Estado lógico reportado por el MCU (gLogical) tras cada pulso KEEP/RES.
_PLC_VALVE_STATUS_KEYS = {
    "holder": ("holder",),
    "encoder": ("encoder",),
    "cutter_r": ("cutter_r",),
    "cutter_l": ("cutter_l",),
    "gripper": ("gripper",),
    "holder_encoder": ("holder", "encoder"),
    "cutters": ("cutter_r", "cutter_l"),
}
_load_serial_cfg()


def _send_plc_serial(lines):
    port = _serial_port("plc")
    if not port:
        return {
            "sent": False,
            "reason": "Sin puerto del PLC. Elígelo en Parámetros, pestaña Conexión.",
            "replies": [],
        }
    try:
        import serial
    except ImportError:
        return {
            "sent": False,
            "reason": "Falta pyserial para abrir el puerto.",
            "replies": [],
        }

    replies = []
    global _plc_serial
    try:
        with _plc_serial_lock:
            if (
                _plc_serial is None
                or not _plc_serial.is_open
                or getattr(_plc_serial, "port", None) != port
            ):
                if _plc_serial is not None:
                    try:
                        _plc_serial.close()
                    except Exception:
                        pass
                ser = serial.Serial()
                ser.port = port
                ser.baudrate = int(os.environ.get("PLC_SERIAL_BAUD", "115200"))
                ser.timeout = 0.05
                ser.write_timeout = 2
                ser.dtr = False
                ser.rts = False
                ser.open()
                time.sleep(0.05)
                _serial_rx_flush(ser)
                _plc_serial = ser
            ser = _plc_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                try:
                    expected_op = str(json.loads(line).get("op") or "")
                except (TypeError, json.JSONDecodeError):
                    expected_op = ""
                if expected_op:
                    replies.extend(_read_lines_until_op(ser, expected_op, 2.0))
                else:
                    reply = _read_serial_line(ser, 2.0)
                    if reply:
                        replies.append(reply)
    except Exception as exc:
        with _plc_serial_lock:
            _invalidate_serial_handle(_plc_serial)
            _plc_serial = None
        return {"sent": False, "reason": str(exc), "replies": replies}
    return {"sent": True, "reason": "", "replies": replies}


def _send_andon_serial(lines):
    port = _serial_port("andon")
    if not port:
        return {
            "sent": False,
            "reason": "Sin puerto del Andon. Elígelo en Parámetros, pestaña Conexión.",
            "replies": [],
        }
    try:
        import serial
    except ImportError:
        return {
            "sent": False,
            "reason": "Falta pyserial para abrir el puerto.",
            "replies": [],
        }

    replies = []
    global _andon_serial
    try:
        with _andon_serial_lock:
            if (
                _andon_serial is None
                or not _andon_serial.is_open
                or getattr(_andon_serial, "port", None) != port
            ):
                if _andon_serial is not None:
                    try:
                        _andon_serial.close()
                    except Exception:
                        pass
                ser = serial.Serial()
                ser.port = port
                ser.baudrate = 115200
                ser.timeout = 0.05
                ser.write_timeout = 2
                ser.dtr = False
                ser.rts = False
                ser.open()
                time.sleep(0.05)
                _serial_rx_flush(ser)
                _andon_serial = ser
            ser = _andon_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                try:
                    expected_op = str(json.loads(line).get("op") or "")
                except (TypeError, json.JSONDecodeError):
                    expected_op = ""
                if expected_op:
                    replies.extend(_read_lines_until_op(ser, expected_op, 2.0))
                else:
                    reply = _read_serial_line(ser, 2.0)
                    if reply:
                        replies.append(reply)
    except Exception as exc:
        with _andon_serial_lock:
            _invalidate_serial_handle(_andon_serial)
            _andon_serial = None
        return {"sent": False, "reason": str(exc), "replies": replies}
    return {"sent": True, "reason": "", "replies": replies}


def _open_prefeeder_serial_unlocked(port: str):
    """Abre el COM del PreFeeder. Caller debe tener _prefeeder_serial_lock."""
    import serial

    global _prefeeder_serial
    _invalidate_serial_handle(_prefeeder_serial)
    _prefeeder_serial = None
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.05
    ser.write_timeout = 2
    # Evitar reset por DTR al abrir (el Monitor Serial sí suele resetear).
    ser.dtr = False
    ser.rts = False
    ser.open()
    # Tras cerrar el Monitor, Windows/USB-UART a veces necesita un instante.
    time.sleep(0.25)
    _serial_rx_flush(ser)
    _prefeeder_serial = ser
    return ser


def _send_prefeeder_serial(lines, *, force_reopen: bool = False):
    port = _serial_port("prefeeder")
    if not port:
        return {
            "sent": False,
            "reason": "Sin puerto del PreAlimentador. Elígelo en Parámetros, pestaña Conexión.",
            "replies": [],
        }
    try:
        import serial  # noqa: F401
    except ImportError:
        return {
            "sent": False,
            "reason": "Falta pyserial para abrir el puerto.",
            "replies": [],
        }

    replies = []
    global _prefeeder_serial
    try:
        with _prefeeder_serial_lock:
            need_open = (
                force_reopen
                or _prefeeder_serial is None
                or not _prefeeder_serial.is_open
                or getattr(_prefeeder_serial, "port", None) != port
            )
            if need_open:
                ser = _open_prefeeder_serial_unlocked(port)
            else:
                ser = _prefeeder_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                try:
                    expected_op = str(json.loads(line).get("op") or "")
                except (TypeError, json.JSONDecodeError):
                    expected_op = ""
                if expected_op:
                    replies.extend(
                        _read_lines_until_op(ser, expected_op, _PF_SERIAL_REPLY_S)
                    )
                else:
                    reply = _read_serial_line(ser, _PF_SERIAL_REPLY_S)
                    if reply:
                        replies.append(reply)
    except Exception as exc:
        # Handle zombie: Access denied / dispositivo desconectado tras el Monitor.
        with _prefeeder_serial_lock:
            _invalidate_serial_handle(_prefeeder_serial)
            _prefeeder_serial = None
        return {"sent": False, "reason": str(exc), "replies": replies}
    return {"sent": True, "reason": "", "replies": replies}


def _prefeeder_status_from_replies(replies):
    for raw in reversed(replies or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "status" and msg.get("mcu") == "prefeeder":
            return msg
    return None


def _prefeeder_status_get() -> dict | None:
    """Pide status; si el COM quedó mudo (p. ej. tras Monitor Serial), reabre el puerto."""
    for attempt in range(3):
        # Intento 0: handle actual. 1 y 2: forzar reopen (caso típico tras
        # cerrar el Monitor Serial en Windows: is_open=True pero sin RX/TX).
        sent = _send_prefeeder_serial(
            ['{"op":"status"}'], force_reopen=(attempt > 0)
        )
        if not sent.get("sent"):
            if attempt < 2:
                time.sleep(0.35)
            continue
        st = _prefeeder_status_from_replies(sent.get("replies"))
        if st:
            return st
        # Envió OK pero sin status: handle zombie o MCU reiniciando por DTR.
        if attempt < 2:
            _close_prefeeder_serial()
            time.sleep(0.35)
    return None


def _prefeeder_fault_codes_active(
    status: dict | None,
    *,
    omit_l: bool | None = None,
    omit_r: bool | None = None,
) -> list[str]:
    """Fallas vigentes del PreAlimentador.

    - Lado fuera de feed_sides: se ignora por completo (no espera ni bloquea).
    - E095/E098 se ignoran si ese lado tiene holgura omitida (manual o auto).

    omit_l/omit_r opcionales: si ya se leyeron bajo _STATE_LOCK, pasarlos para
    no reentrar el candado (deadlock en /api/state con falla activa).
    """
    codes = error_codes.prefeeder_fault_codes(status)
    if not codes:
        return []
    watched = set(_active_feed_sides())
    omit_l, omit_r = _holgura_omit_effective(omit_l=omit_l, omit_r=omit_r)
    out: list[str] = []
    for code in codes:
        if code in _PFEEDER_CODES_L and "L" not in watched:
            continue
        if code in _PFEEDER_CODES_R and "R" not in watched:
            continue
        if code == "E095" and omit_l:
            continue
        if code == "E098" and omit_r:
            continue
        out.append(code)
    return out


def _prefeeder_holgura_omitted_for_code(code: str) -> bool:
    code = str(code or "").strip().upper()
    omit_l, omit_r = _holgura_omit_effective()
    if code == "E095":
        return omit_l
    if code == "E098":
        return omit_r
    return False


def _prefeeder_active_errors(status: dict | None) -> list[dict]:
    if not status:
        return []
    items = []
    for code in _prefeeder_fault_codes_active(status):
        items.append({"code": code, "message": error_codes.format_message(code)})
    return items


def _prefeeder_cfg_from_status_side(st: dict | None, side: str) -> dict | None:
    if not isinstance(st, dict):
        return None
    block = st.get(side)
    if not isinstance(block, dict):
        return None
    cfg = block.get("cfg")
    if not isinstance(cfg, dict):
        return None
    return _canonical_prefeeder_side({side: cfg}, side)


def _prefeeder_reconcile_pushed_from_status(st: dict | None) -> None:
    """Alinea la caché de envío con el cfg real del maestro."""
    if not isinstance(st, dict):
        return
    for side in ("L", "R"):
        live = _prefeeder_cfg_from_status_side(st, side)
        if live is not None:
            _pf_pushed_signature[side] = _prefeeder_cfg_signature(live)


def _prefeeder_set_config_side(side: str, cfg: dict) -> tuple[bool, bool]:
    """Envía set_config al maestro solo si la firma cambió. Retorna (ok, changed)."""
    side = side.upper()
    if side not in ("L", "R"):
        return False, False
    canonical = _canonical_prefeeder_side({side: cfg}, side)
    sig = _prefeeder_cfg_signature(canonical)
    if _pf_pushed_signature.get(side) == sig:
        return True, False
    payload = {"op": "set_config", "side": side, **canonical}
    sent = _send_prefeeder_serial([_compact(payload)])
    if not sent.get("sent"):
        return False, False
    changed = False
    ok = False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "set_config" and str(msg.get("side") or "").upper() == side:
            ok = bool(msg.get("ok", False))
            changed = bool(msg.get("changed", False))
            break
    if ok and (changed or _pf_pushed_signature.get(side) is None):
        _pf_pushed_signature[side] = sig
    return ok, changed


def _prefeeder_adopt_live_config() -> None:
    """Alinea archivo/firma con el cfg del maestro (NVS). No sobrescribe el ESP32."""
    if not _serial_port("prefeeder"):
        return
    st = _prefeeder_refresh_cache()
    if not st:
        return
    try:
        tune = _load_prefeeder_tune()
    except (OSError, json.JSONDecodeError):
        tune = _canonical_prefeeder_tune({})
    changed = False
    for side in ("L", "R"):
        live = _prefeeder_cfg_from_status_side(st, side)
        if live is None:
            continue
        if _prefeeder_cfg_signature(tune[side]) != _prefeeder_cfg_signature(live):
            tune[side] = live
            changed = True
    if changed:
        try:
            _save_prefeeder_tune(tune)
        except OSError:
            pass


def _prefeeder_required() -> bool:
    return bool(_serial_port("prefeeder"))


def _prefeeder_cmd(payload: dict) -> dict:
    return _send_prefeeder_serial([_compact(payload)])


def _prefeeder_reply_ok(sent: dict, op: str) -> bool:
    if not sent.get("sent"):
        return False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return bool(msg.get("ok", True))
    return False


def _prefeeder_set_feed_lock(locked: bool) -> bool:
    return _prefeeder_reply_ok(
        _prefeeder_cmd({"op": "set_feed_lock", "locked": bool(locked)}),
        "set_feed_lock",
    )


def _prefeeder_set_force(side: str, on: bool) -> bool:
    side = str(side or "").upper()
    if side not in ("L", "R"):
        return False
    return _prefeeder_reply_ok(
        _prefeeder_cmd({"op": "set_force_feed", "side": side, "on": bool(on)}),
        "set_force_feed",
    )


def _prefeeder_set_omit(side: str, on: bool) -> bool:
    side = str(side or "").upper()
    if side not in ("L", "R"):
        return False
    return _prefeeder_reply_ok(
        _prefeeder_cmd({"op": "set_omit", "side": side, "on": bool(on)}),
        "set_omit",
    )


def _prefeeder_sync_omit_flags() -> None:
    if not _prefeeder_required():
        return
    ol, or_ = _holgura_omit_effective()
    _prefeeder_set_omit("L", ol)
    _prefeeder_set_omit("R", or_)


def _prefeeder_side_idle_ok(block: dict | None, *, omit_holgura: bool) -> bool:
    if not isinstance(block, dict) or not block.get("online"):
        return False
    try:
        fault = int(block.get("fault") or 0)
    except (TypeError, ValueError):
        fault = 0
    if fault == 6 and omit_holgura:
        fault = 0
    if fault:
        return False
    state = str(block.get("state") or "").strip().lower()
    if omit_holgura and state == "error":
        return True
    return state == "idle"


def _prefeeder_status_ready(status: dict | None) -> bool:
    """Listo según lados de feed_sides (el maestro viejo exige L+R siempre)."""
    if not status:
        return False
    if not status.get("feed_lock"):
        return False
    if _prefeeder_fault_codes_active(status):
        return False
    omit_l, omit_r = _holgura_omit_effective()
    for side in _active_feed_sides():
        omit = omit_l if side == "L" else omit_r
        if not _prefeeder_side_idle_ok(status.get(side), omit_holgura=omit):
            return False
    return True


def _prefeeder_stop_now() -> None:
    """Frena alimentación manual y bloquea de inmediato."""
    if not _prefeeder_required():
        return
    _prefeeder_set_force("L", False)
    _prefeeder_set_force("R", False)
    _prefeeder_set_feed_lock(True)
    _prefeeder_refresh_cache()


def _prefeeder_refresh_cache() -> dict | None:
    global _PF_CACHED_STATUS
    st = _prefeeder_status_get()
    if st:
        _PF_CACHED_STATUS = st
        _prefeeder_reconcile_pushed_from_status(st)
    return st


def _prefeeder_comm_active() -> bool:
    """Poll al maestro si hay COM (evita banner ‘sin enlace’ por no consultar en idle)."""
    if not _serial_port("prefeeder"):
        return False
    return True


def _prefeeder_fault_sig(st: dict | None) -> tuple[str, ...]:
    return tuple(sorted(_prefeeder_fault_codes_active(st)))


def _machine_fault_message(codes: tuple[str, ...]) -> tuple[str, str]:
    primary = codes[0]
    if len(codes) == 1:
        return primary, error_codes.format_message(primary)
    msg = " · ".join(error_codes.format_message(c) for c in codes)
    return primary, msg


def _recovery_blank() -> dict:
    return {
        "active": False,
        "step": "",
        "code": "",
        "mensaje": "",
    }


def _recovery_get() -> dict:
    with _STATE_LOCK:
        rec = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
    for key, default in _recovery_blank().items():
        rec.setdefault(key, default)
    return rec


def _recovery_set(**fields) -> None:
    with _STATE_LOCK:
        rec = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
        rec.update(fields)
        _MACHINE_STATE["recovery"] = rec


def _recovery_begin(code: str, msg: str) -> None:
    _recovery_set(
        active=True,
        step="error",
        code=str(code or "").strip(),
        mensaje=str(msg or "").strip(),
    )


def _recovery_clear() -> None:
    with _STATE_LOCK:
        _MACHINE_STATE["recovery"] = _recovery_blank()


def _recovery_cutters_clear_now() -> bool:
    """True si ambos cortadores estan apagados (válvula y sensor de falla)."""
    st = _plc_status_get()
    if st is None:
        return False
    if st.get("cutter_r") is True or st.get("cutter_l") is True:
        return False
    if st.get("fault_cutter") is True or st.get("sens_cutter") is True:
        return False
    return True


def _enter_machine_error(primary: str, msg: str, *, en_curso: bool | None = None) -> None:
    """Un solo evento al entrar en error; el primer código queda congelado hasta Reset."""
    with _STATE_LOCK:
        in_error = _MACHINE_STATE.get("estado") == "error"
        had_code = bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
        existing_code = str(_MACHINE_STATE.get("ciclo_error") or "").strip()
        existing_msg = str(_MACHINE_STATE.get("mensaje") or "")
        if en_curso is None:
            en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    if in_error or had_code:
        # Mismo stop: no pisar código/mensaje del popup con fallos posteriores.
        if not _recovery_get().get("active"):
            _recovery_begin(existing_code or primary, existing_msg or msg)
        if not in_error:
            _set_machine_ui(estado="error", estado_label="Error")
        return
    _push_event(msg, "error")
    with _STATE_LOCK:
        _MACHINE_STATE["post_error_purga"] = True
    _recovery_begin(primary, msg)
    _set_machine_ui(
        estado="error",
        estado_label="Error",
        mensaje=msg,
        ciclo_error=primary,
        en_curso=en_curso,
    )


def _prefeeder_maybe_clear_idle_error() -> None:
    """Ya no limpia sola: tras cualquier error el operador debe Reset y luego purga."""
    return


def _prefeeder_raise_fault(sig: tuple[str, ...]) -> None:
    if not sig:
        return
    primary, msg = _machine_fault_message(sig)
    with _STATE_LOCK:
        active = (
            bool(_MACHINE_STATE.get("ciclo_en_curso"))
            or _MACHINE_STATE.get("estado") == "running"
            or bool(_MACHINE_STATE.get("rutina"))
        )
    if active:
        _cycle_fail(primary, mensaje=msg)
        return
    _enter_machine_error(primary, msg, en_curso=False)


def _prefeeder_drain_pf_fault_lines() -> None:
    global _PF_ERROR_LATCH
    if not _serial_port("prefeeder"):
        return
    watched = set(_active_feed_sides())
    try:
        with _prefeeder_serial_lock:
            ser = _prefeeder_serial
            if ser is None or not ser.is_open:
                return
            for _ in range(12):
                line = _read_serial_line(ser, 0.04)
                if not line:
                    break
                try:
                    msg = json.loads(line)
                except (TypeError, json.JSONDecodeError):
                    continue
                if msg.get("op") != "pf_fault":
                    continue
                side = str(msg.get("side") or "").strip().upper()
                if side and side not in watched:
                    continue
                try:
                    fault = int(msg.get("fault") or 0)
                except (TypeError, ValueError):
                    fault = 0
                if fault == 0:
                    continue
                _PF_ERROR_LATCH = True
    except Exception:
        pass


def _prefeeder_watch_once() -> None:
    global _pf_last_fault_sig, _PF_ERROR_LATCH
    if not _serial_port("prefeeder"):
        _pf_last_fault_sig = ()
        _PF_ERROR_LATCH = False
        return
    st = _prefeeder_refresh_cache()
    _prefeeder_drain_pf_fault_lines()
    if not st:
        return
    sig = _prefeeder_fault_sig(st)
    with _STATE_LOCK:
        in_error = _MACHINE_STATE.get("estado") == "error"
    if sig != _pf_last_fault_sig:
        if sig:
            _PF_ERROR_LATCH = True
            _prefeeder_raise_fault(sig)
        else:
            _PF_ERROR_LATCH = False
            if in_error:
                _prefeeder_maybe_clear_idle_error()
        _pf_last_fault_sig = sig
    elif sig and not in_error:
        _PF_ERROR_LATCH = True
        _prefeeder_raise_fault(sig)


def _prefeeder_watch_loop() -> None:
    while True:
        try:
            if _prefeeder_required():
                _prefeeder_watch_once()
        except Exception:
            pass
        time.sleep(0.75)


def _prefeeder_sides_online(status: dict | None) -> bool:
    """Lados activos (feed_sides) reportados online por el maestro."""
    if not isinstance(status, dict):
        return False
    for side in _active_feed_sides():
        block = status.get(side)
        if not isinstance(block, dict) or not block.get("online"):
            return False
    return True


def _prefeeder_stabilize_and_lock() -> tuple[bool, str]:
    """Desbloquea, espera condición segura y vuelve a bloquear listo.

    Con holgura omitida y sensor OFF el esclavo puede quedarse en busy
    alimentando mientras está desbloqueado; en ese caso no hay que esperar
    stable=true (nunca llega): basta online + sin fallas activas, bloquear
    (frena la alimentación) y confirmar ready.
    Solo cuenta el lado de feed_sides (el contrario no bloquea).
    """
    if not _prefeeder_required():
        return True, ""
    _prefeeder_sync_omit_flags()
    _prefeeder_set_feed_lock(False)
    deadline = time.time() + _PF_STABILIZE_TIMEOUT_S
    can_lock = False
    while time.time() < deadline:
        st = _prefeeder_refresh_cache()
        if not st:
            time.sleep(0.3)
            continue
        sig = _prefeeder_fault_sig(st)
        if sig:
            _prefeeder_set_feed_lock(True)
            _, msg = _machine_fault_message(sig)
            return False, msg
        # No usar solo stable del maestro: con un lado omitido exige L+R.
        if _prefeeder_sides_online(st) and not _prefeeder_fault_codes_active(st):
            can_lock = True
            break
        time.sleep(0.3)
    _prefeeder_set_feed_lock(True)
    # Tras bloquear, el busy por holgura omitida debe pasar a idle/ready.
    ready_deadline = time.time() + 8.0
    st = None
    while time.time() < ready_deadline:
        st = _prefeeder_refresh_cache()
        if st and _prefeeder_status_ready(st):
            return True, ""
        time.sleep(0.25)
    if not st:
        return False, error_codes.format_message("E092")
    if not can_lock or not _prefeeder_status_ready(st):
        return False, error_codes.format_message("E099")
    return True, ""


def _prefeeder_unlock_for_cycle() -> None:
    global _PF_ERROR_LATCH
    if _prefeeder_required():
        _prefeeder_set_feed_lock(False)
        st = _PF_CACHED_STATUS
        if st and not _prefeeder_fault_sig(st):
            _PF_ERROR_LATCH = False


def _prefeeder_lock_after_cycle() -> None:
    if not _prefeeder_required():
        return
    ok, msg = _prefeeder_stabilize_and_lock()
    if not ok:
        _push_event(msg, "error")
        _set_machine_ui(mensaje=msg)


def _prefeeder_lock_after_cycle_async() -> None:
    _prefeeder_process_halt_async()


def _prefeeder_process_halt(*, stabilize: bool = True) -> None:
    """Bloquea alimentación; opcionalmente corre estabilización (misma lógica que Iniciar)."""
    if not _prefeeder_required():
        return
    _prefeeder_stop_now()
    if not stabilize:
        return
    st = _PF_CACHED_STATUS
    if st and _prefeeder_fault_sig(st):
        return
    ok, msg = _prefeeder_stabilize_and_lock()
    _prefeeder_refresh_cache()
    if not ok:
        _push_event(msg, "error")


def _prefeeder_process_halt_async(*, stabilize: bool = True) -> None:
    threading.Thread(
        target=_prefeeder_process_halt, kwargs={"stabilize": stabilize}, daemon=True
    ).start()


def _prefeeder_fault_clear_now() -> bool:
    """Fallas de campo (cilindro/holgura). Sin status ≠ falla activa."""
    if not _prefeeder_required():
        return True
    st = _prefeeder_refresh_cache() or _PF_CACHED_STATUS
    if not st:
        return True
    return not _prefeeder_fault_codes_active(st)


def _error_reset_blocked_message() -> str:
    """Motivo concreto si el reset no puede validar condiciones físicas."""
    if _serial_port("plc"):
        st = _plc_status_get()
        if st is None:
            return error_codes.format_message("E019")
        faults = error_codes.plc_fault_codes(st)
        if faults:
            return error_codes.format_message(faults[0])
    if _serial_port("motion"):
        mst = _motion_status_get()
        if mst is None:
            return error_codes.format_message("E043")
        if mst.get("safety") is True:
            return error_codes.format_message("E039")
    if _prefeeder_required():
        st = _prefeeder_refresh_cache() or _PF_CACHED_STATUS
        if st:
            faults = _prefeeder_fault_codes_active(st)
            if faults:
                return error_codes.format_message(faults[0])
    return error_codes.format_message("E046")


def _prefeeder_public_summary(
    *,
    omit_l: bool | None = None,
    omit_r: bool | None = None,
) -> dict:
    """Resumen para el HMI (sin consultar COM si no hay caché)."""
    if not _prefeeder_required():
        return {
            "com": False,
            "ready": None,
            "stable": None,
            "feed_lock": None,
            "label": "Sin puerto COM",
            "tone": "muted",
        }
    st = _PF_CACHED_STATUS
    if not st:
        return {
            "com": True,
            "ready": False,
            "stable": False,
            "feed_lock": True,
            "label": "Sin enlace al maestro",
            "tone": "warn",
        }
    faults = _prefeeder_fault_codes_active(st, omit_l=omit_l, omit_r=omit_r)
    feed_lock = bool(st.get("feed_lock"))
    ready = _prefeeder_status_ready(st)
    stable = _prefeeder_sides_online(st) and not faults
    if faults:
        tone = "warn"
        label = "Bloqueado"
    elif ready:
        tone = "ok"
        label = "Listo (bloqueado)"
    elif not feed_lock:
        tone = "active"
        label = "Desbloqueado — alimentación activa"
    elif not stable:
        tone = "warn"
        label = "Estabilizando…"
    else:
        tone = "warn"
        label = "Bloqueado — pulsa Iniciar"
    return {
        "com": True,
        "ready": ready,
        "stable": stable,
        "feed_lock": feed_lock,
        "label": label,
        "tone": tone,
    }


def _prefeeder_maybe_refresh_ui() -> None:
    """Refuerzo en /api/state si el hilo de vigilancia va retrasado."""
    global _PF_UI_REFRESH_AT
    if not _prefeeder_required():
        return
    now = time.time()
    if now - _PF_UI_REFRESH_AT < 0.6:
        return
    _PF_UI_REFRESH_AT = now
    _prefeeder_watch_once()


def _prefeeder_boot_lock() -> None:
    time.sleep(0.5)
    if _prefeeder_required():
        _prefeeder_set_feed_lock(True)
        _prefeeder_sync_omit_flags()
        _prefeeder_adopt_live_config()


def _build_plc_line(op: str, body: dict) -> dict:
    if op == "valve":
        name = str(body.get("name") or "")
        allowed = _PLC_VALVES + ("holder_encoder", "cutters")
        if name not in allowed:
            raise ValueError("Salida manual no reconocida.")
        if not isinstance(body.get("on"), bool):
            raise ValueError("Falta el estado on de la salida.")
        return {"op": "valve", "name": name, "on": body["on"]}
    if op == "blower_on":
        raw = body.get("ms")
        if raw is None or raw == "":
            ms = 2000
        else:
            ms = int(raw)
        if ms < 200:
            ms = 200
        if ms > 300000:
            ms = 300000
        return {"op": "blower_on", "ms": ms}
    if op == "blower_off":
        return {"op": "blower_off"}
    if op == "all_off":
        return {"op": "all_off"}
    if op == "reset":
        return {"op": "reset"}
    raise ValueError("Orden PLC no reconocida.")


def _plc_status_from_replies(replies):
    for raw in reversed(replies or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "status":
            return msg
    return None


def _status_module(replies) -> str:
    """Qué MCU contestó, según los campos de su status."""
    for raw in reversed(replies or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") != "status":
            continue
        if msg.get("mcu") == "motion":
            return "motion"
        if msg.get("mcu") == "prefeeder":
            return "prefeeder"
        if msg.get("mcu") == "andon" or (
            "state" in msg and "cutter_r" not in msg and "feeder_r_ready" not in msg
        ):
            return "andon"
        if "pulse_busy" in msg or "cutter_r" in msg:
            return "plc"
        if "feeder_r_ready" in msg or "lm_servo_on" in msg or "enc_r" in msg:
            return "motion"
    return ""


def _check_module_link(which: str) -> tuple[bool, str]:
    """True si el COM elegido responde y es el módulo pedido."""
    if which == "motion":
        missing_port, wrong, silent = "E041", "E042", "E043"
        if not _serial_port("motion"):
            return False, missing_port
        # En arranque el Motion puede tardar en init CAN y enviar muchas líneas antes del status al pedirlo.
        sent = _send_motion_serial(
            ['{"op":"status"}'],
            line_timeout_s=90.0,
            reply_until_op="status",
        )
    elif which == "plc":
        missing_port, wrong, silent = "E017", "E018", "E019"
        if not _serial_port("plc"):
            return False, missing_port
        sent = _send_plc_serial(['{"op":"status"}'])
    elif which == "andon":
        missing_port, wrong, silent = "E070", "E071", "E072"
        if not _serial_port("andon"):
            return False, missing_port
        sent = _send_andon_serial(['{"op":"status"}'])
    elif which == "prefeeder":
        missing_port, wrong, silent = "E090", "E091", "E092"
        if not _serial_port("prefeeder"):
            return False, missing_port
        # Misma recuperación que _prefeeder_status_get: reopen si el Monitor
        # dejó el COM en un estado donde write “pasa” pero no hay respuesta.
        st = _prefeeder_status_get()
        if st and st.get("mcu") == "prefeeder":
            return True, ""
        if st:
            return False, wrong
        return False, silent
    else:
        return False, "E070"
    if not sent.get("sent"):
        return False, silent
    found = _status_module(sent.get("replies"))
    if found == which:
        return True, ""
    if found:
        return False, wrong
    return False, silent


def _links_ready_for_cycle() -> tuple[bool, str]:
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR: dejar solo ("motion", "plc", "andon") ---
    modules: tuple[str, ...] = (
        ("motion", "plc")
        if _andon_skipped_effective()
        else ("motion", "plc", "andon")
    )
    messages = []
    for which in modules:
        ok, code = _check_module_link(which)
        if not ok:
            messages.append(error_codes.format_message(code))
    if messages:
        return False, " ".join(messages)
    return True, ""


def _plc_serial_configured() -> bool:
    return bool(_serial_port("plc"))


def _plc_status_get() -> dict | None:
    """GET status al MCU PLC (una consulta, sin polling en la UI)."""
    sent = _send_plc_serial(['{"op":"status"}'])
    if not sent.get("sent"):
        return None
    return _plc_status_from_replies(sent.get("replies"))


def _plc_status_after_command(sent: dict) -> dict | None:
    """Tras POST al PLC: status en la reply o GET adicional."""
    status = _plc_status_from_replies(sent.get("replies"))
    if status is not None:
        return status
    if not sent.get("sent"):
        return None
    return _plc_status_get()


def _plc_wait_ready(timeout_s: float = 3.0) -> dict | None:
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        # Un Stop de una rutina deja el flag del ciclo puesto. No es falta de status.
        if _cycle_should_stop() and _cycle_thread_alive():
            return last
        last = _plc_status_get()
        if last is not None and not last.get("pulse_busy"):
            return last
        time.sleep(0.05)
    if last is not None and last.get("pulse_busy"):
        return last
    return last


def _plc_status_valve_matches(st: dict | None, name: str, want_on: bool) -> bool:
    """True si el status del MCU ya refleja el estado pedido (válvulas KEEP/RES)."""
    keys = _PLC_VALVE_STATUS_KEYS.get(name)
    if not st or not keys:
        return False
    for key in keys:
        if st.get(key) is not want_on:
            return False
    return True


def _plc_valve_skip_pulse(name: str, want_on: bool) -> bool:
    """No enviar pulso si la válvula ya está en ese estado (evita toggle accidental)."""
    if not _serial_port("plc"):
        return False
    st = _plc_wait_ready(1.5)
    if st is None or st.get("pulse_busy"):
        return False
    return _plc_status_valve_matches(st, name, want_on)


def _plc_send_payload(payload: dict, *, push_log: bool = False) -> dict:
    """Envía una orden al MCU PLC; en válvulas omite el pulso si ya está en el estado."""
    op = str(payload.get("op") or "")
    name = str(payload.get("name") or "")
    want_on = bool(payload.get("on"))
    if op == "valve" and name and _plc_valve_skip_pulse(name, want_on):
        try:
            line = _compact(_build_plc_line(op, payload))
        except (TypeError, ValueError):
            line = _compact(payload)
        sent = {
            "sent": True,
            "skipped": True,
            "replies": [_compact({"op": "valve", "ok": True})],
            "last_payload": payload,
        }
        if push_log:
            _push_event(f"[PLC] {line} (sin pulso, ya en estado)")
        return sent
    try:
        if op in ("valve", "blower_on", "blower_off", "all_off", "reset"):
            line = _compact(_build_plc_line(op, payload))
        else:
            line = _compact(payload)
    except (TypeError, ValueError):
        line = _compact(payload)
    sent = _send_plc_serial([line])
    sent["last_payload"] = payload
    if not sent.get("sent"):
        sent["code"] = "E019" if _serial_port("plc") else "E017"
        if push_log:
            _push_event(error_codes.format_message(sent["code"]), "error")
    elif push_log:
        _push_event(f"[PLC] {line}")
    return sent


def _plc_verify_logical(st: dict, expected: dict) -> tuple[bool, str | None]:
    for key, want in expected.items():
        if st.get(key) is not want:
            if key == "holder":
                return False, "E011" if want else "E014"
            if key == "encoder":
                return False, "E012" if want else "E015"
            if key == "gripper":
                return False, "E013"
            return False, "E010"
    return True, None


def _plc_confirm(expected: dict, timeout_s: float = 3.0) -> tuple[bool, str | None]:
    """Confirma salidas lógicas. Sin COM no se da por bueno."""
    if not _plc_serial_configured():
        return False, "E017"
    st = _plc_wait_ready(timeout_s)
    if st is None:
        if _cycle_should_stop() and _cycle_thread_alive():
            return False, ""
        return False, "E010"
    if st.get("pulse_busy"):
        return False, "E016"
    faults = error_codes.plc_fault_codes(st)
    if faults:
        return False, faults[0]
    return _plc_verify_logical(st, expected)


def _plc_active_errors(status: dict | None) -> list[dict]:
    if not status:
        return []
    items = []
    for code in error_codes.plc_fault_codes(status):
        items.append({"code": code, "message": error_codes.format_message(code)})
    return items


def _send_motion_serial(
    lines,
    line_timeout_s=None,
    reply_until_op: str | None = None,
    nonblocking: bool = False,
    inject=None,
    on_line=None,
):
    port = _serial_port("motion")
    if not port:
        return {
            "sent": False,
            "reason": "Sin puerto de Motion. Elígelo en Parámetros, pestaña Conexión.",
            "replies": [],
        }
    try:
        import serial
    except ImportError:
        return {
            "sent": False,
            "reason": "Falta pyserial para abrir el puerto.",
            "replies": [],
        }

    replies = []
    global _motion_serial
    acquired = _serial_lock.acquire(blocking=not nonblocking)
    if not acquired:
        return {"sent": False, "reason": "Motion ocupado.", "replies": [], "busy": True}
    try:
        if (
            _motion_serial is None
            or not _motion_serial.is_open
            or getattr(_motion_serial, "port", None) != port
        ):
            if _motion_serial is not None:
                try:
                    _motion_serial.close()
                except Exception:
                    pass
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = int(os.environ.get("MOTION_SERIAL_BAUD", "115200"))
            ser.timeout = 0.05
            ser.write_timeout = 2
            ser.dtr = False
            ser.rts = False
            ser.open()
            time.sleep(0.05)
            _serial_rx_flush(ser)
            _motion_serial = ser
        ser = _motion_serial
        for index, line in enumerate(lines):
            trace_motion = '"op":"status"' not in line
            if trace_motion:
                print(f"[Motion TX] {line}", flush=True)
            with _motion_write_lock:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
            try:
                expected_op = str(json.loads(line).get("op") or "")
            except (TypeError, json.JSONDecodeError):
                expected_op = ""
            if reply_until_op and index == len(lines) - 1:
                expected_op = reply_until_op
            wait_s = line_timeout_s if line_timeout_s is not None else _reply_wait_s(line)
            if expected_op:
                replies.extend(
                    _read_lines_until_op(
                        ser,
                        expected_op,
                        wait_s,
                        trace_prefix="[Motion RX]" if trace_motion else None,
                        inject=inject if index == len(lines) - 1 else None,
                        on_line=on_line if index == len(lines) - 1 else None,
                    )
                )
            else:
                reply = _read_serial_line(ser, wait_s)
                if reply:
                    replies.append(reply)
                    if trace_motion:
                        print(f"[Motion RX] {reply}", flush=True)
    except Exception as exc:
        # Tras Monitor Serial / USB, el handle puede quedar abierto pero muerto.
        try:
            _invalidate_serial_handle(_motion_serial)
        except Exception:
            pass
        _motion_serial = None
        return {"sent": False, "reason": str(exc), "replies": replies}
    finally:
        if _serial_lock.locked():
            _serial_lock.release()
    return {"sent": True, "reason": "", "replies": replies}


def _send_motion_urgent(lines):
    """Escribe paros sin esperar el lock ocupado por una orden larga."""
    global _motion_serial
    if not _serial_lock.locked():
        return _send_motion_serial(lines, line_timeout_s=2.0)
    ser = _motion_serial
    if ser is None or not ser.is_open:
        return {"sent": False, "reason": "Motion ocupado sin puerto abierto.", "replies": []}
    try:
        with _motion_write_lock:
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
            ser.flush()
    except Exception as exc:
        return {"sent": False, "reason": str(exc), "replies": []}
    return {"sent": True, "reason": "", "replies": [], "urgent": True}


def _motion_data_flat(data: dict) -> dict:
    return {
        "op": "set_data",
        "target_length_mm": data.get("target_length_mm"),
        "tolerance_mm": data.get("tolerance_mm", 2.0),
        "modelo_id": data.get("modelo_id") or "",
        "measure_comp_mm": data.get("measure_comp_mm", 0.0),
    }


def _deep_merge_motion_tune(base: dict, patch: dict) -> None:
    for key, val in patch.items():
        if isinstance(val, dict) and isinstance(base.get(key), dict):
            _deep_merge_motion_tune(base[key], val)
        else:
            base[key] = val


def _load_modelos():
    with MODELOS_PATH.open(encoding="utf-8") as f:
        data = json.load(f)
    return data.get("modelos", [])


def _find_modelo(modelo_id: str):
    for item in _load_modelos():
        if item.get("modelo") == modelo_id:
            return item
    return None


def _load_cycle_params():
    params = dict(_DEFAULT_CYCLE_PARAMS)
    if not CYCLE_PATH.exists():
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        CYCLE_PATH.write_text(
            json.dumps(params, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return params
    try:
        with CYCLE_PATH.open(encoding="utf-8") as f:
            raw = json.load(f)
    except (OSError, json.JSONDecodeError):
        return params
    if isinstance(raw, dict):
        for key in params:
            if key in raw:
                params[key] = raw[key]
        for key in _HMI_CYCLE_EXTRA_KEYS:
            if key in raw:
                params[key] = raw[key]
    return params


def _save_hmi_cycle_extras() -> None:
    """Cantidad, modelo y compensación general en cycle.json."""
    try:
        params = _load_cycle_params()
        comps = _cut_comp_params_of(params)
        params["cut_comp_mode"] = comps["cut_comp_mode"]
        params["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
        params["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
        params["cut_offset_mm"] = _measure_comp_mm()
        try:
            params["cantidad"] = max(1, int(_MACHINE_STATE.get("cantidad") or 1))
        except (TypeError, ValueError):
            params["cantidad"] = 1
        params["numero_modelo"] = str(_MACHINE_STATE.get("numero_modelo") or "").strip()
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        CYCLE_PATH.write_text(
            json.dumps(params, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    except OSError:
        pass


def _hydrate_hmi_from_cycle() -> None:
    """Restaura pantalla principal desde cycle.json al arrancar app.py."""
    params = _load_cycle_params()
    try:
        qty = int(params.get("cantidad", 1))
    except (TypeError, ValueError):
        qty = 1
    _MACHINE_STATE["cantidad"] = max(1, qty)
    comps = _cut_comp_params_of(params)
    params["cut_comp_mode"] = comps["cut_comp_mode"]
    params["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
    params["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
    comp = _num(params.get("cut_offset_mm"), 0.0)
    _MACHINE_STATE["compensacion_medicion_mm"] = comp
    _MOTION_DATA["measure_comp_mm"] = comp
    _MOTION_DATA["tolerance_mm"] = _num(params.get("tolerance_mm"), 2.0)
    modelo_id = str(params.get("numero_modelo") or "").strip()
    if not modelo_id:
        return
    try:
        entry = _find_modelo(modelo_id)
    except (OSError, json.JSONDecodeError):
        entry = None
    if not entry:
        return
    _MACHINE_STATE["numero_modelo"] = entry["modelo"]
    _MACHINE_STATE["longitud_modelo_mm"] = entry.get("longitud_mm")
    _MOTION_DATA["modelo_id"] = entry["modelo"]
    _MOTION_DATA["target_length_mm"] = entry.get("longitud_mm")


def _cycle_step_meta(step_id: str):
    for i, step in enumerate(CYCLE_STEPS):
        if step["id"] == step_id:
            return i, step
    return -1, None


def _cycle_progress_pct(step_index: int) -> int:
    n = len(CYCLE_STEPS)
    if n <= 0:
        return 0
    return int(round(min(100, max(0, (step_index / n) * 100))))


def _lot_progress_pct(step_index: int | None = None) -> int:
    """Avance del lote (ciclos hechos / total). Fracción del paso actual si hay pieza en curso."""
    total = int(_LOT.get("cycles_total") or 0)
    if total <= 0:
        if step_index is not None:
            return _cycle_progress_pct(int(step_index))
        return 0
    done = int(_LOT.get("cycles_done") or 0)
    if done >= total:
        return 100
    frac = 0.0
    if (_LOT.get("active") or _LOT.get("tray_wait")) and step_index is not None:
        n = len(CYCLE_STEPS)
        if n > 0:
            frac = min(1.0, max(0.0, float(step_index) / float(n)))
    return int(round(min(100, max(0, ((done + frac) / total) * 100))))


def _now_hms() -> str:
    return time.strftime("%H:%M:%S")


_EVENTS_REV = 0


def _push_event(texto: str, tipo: str = "info") -> None:
    global _EVENTS_REV
    entry = {"texto": texto, "tipo": tipo, "hora": _now_hms()}
    with _STATE_LOCK:
        ev = _MACHINE_STATE.setdefault("eventos", [])
        if ev and tipo == "error" and ev[0].get("tipo") == "error" and ev[0].get("texto") == texto:
            return
        ev.insert(0, entry)
        del ev[300:]
        _EVENTS_REV += 1


def _clear_event_log() -> int:
    global _EVENTS_REV
    with _STATE_LOCK:
        _MACHINE_STATE["eventos"] = []
        _EVENTS_REV += 1
        return _EVENTS_REV


def _andon_desired(machine: dict) -> str:
    code = str(machine.get("ciclo_error") or "")
    if code in ("E033", "E034"):
        return "falta_material"
    if code or machine.get("estado") == "error":
        return "error"
    if machine.get("estado") == "running" or (
        machine.get("ciclo_en_curso") and machine.get("estado") == "paused"
    ):
        return "trabajando"
    if (
        not machine.get("ciclo_en_curso")
        and int(machine.get("ciclo_paso_indice") or 0) >= len(CYCLE_STEPS)
    ):
        return "terminado"
    return "simple"


def _andon_publish_from_state() -> None:
    global _andon_state_sent, _andon_attempted
    if not _serial_port("andon"):
        return
    with _STATE_LOCK:
        desired = _andon_desired(_MACHINE_STATE)
    with _andon_serial_lock:
        if desired == _andon_state_sent or desired == _andon_attempted:
            return
        _andon_attempted = desired
    sent = _send_andon_serial([_compact({"op": "set_state", "state": desired})])
    accepted = False
    if sent.get("sent"):
        for raw in sent.get("replies") or []:
            try:
                msg = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                continue
            if (
                msg.get("op") == "set_state"
                and msg.get("ok") is True
                and msg.get("state") == desired
            ):
                accepted = True
                break
    if accepted:
        _andon_state_sent = desired
        return
    _push_event(error_codes.format_message("E072"), "error")


def _public_machine_state() -> dict:
    estimado_s, corriendo_s = _lot_times_now()
    with _STATE_LOCK:
        state = dict(_MACHINE_STATE)
        state["checklist"] = dict(_MACHINE_STATE.get("checklist") or {})
        state["checklist_bypass"] = _checklist_bypassed()
        state["eventos"] = list(_MACHINE_STATE.get("eventos") or [])
        state["eventos_rev"] = _EVENTS_REV
        state["ciclo_feed"] = dict(_MACHINE_STATE.get("ciclo_feed") or {})
        state["charola_llena"] = bool(_LOT["tray_wait"])
        state["charola_ciclos"] = int(_LOT["deposits_per_tray"] or 0) if _LOT["tray_wait"] else 0
        state["lote_activo"] = bool(_LOT["active"] or _LOT["tray_wait"])
        done_c = int(_LOT["cycles_done"] or 0)
        total_c = int(_LOT["cycles_total"] or 0)
        state["lote_ciclos_hechos"] = done_c
        state["lote_ciclos_total"] = total_c
        state["lote_lados"] = _LOT["sides"]
        mult = _lot_piece_mult()
        lote_vivo = bool(_LOT["active"] or _LOT["tray_wait"])
        lote_terminado = (not lote_vivo) and total_c > 0 and done_c >= total_c
        if lote_vivo or lote_terminado:
            state["piezas_producidas"] = done_c * mult
            state["piezas_total"] = total_c * mult
        else:
            state["piezas_producidas"] = 0
            state["piezas_total"] = int(state.get("cantidad") or 0)
        if total_c > 0:
            state["progreso_pct"] = _lot_progress_pct(state.get("ciclo_paso_indice"))
        if estimado_s is not None:
            state["tiempo_estimado_s"] = estimado_s
        elif lote_terminado:
            state["tiempo_estimado_s"] = 0
        elif not (lote_vivo or state.get("ciclo_en_curso")):
            state["tiempo_estimado_s"] = None
        if lote_vivo or state.get("ciclo_en_curso"):
            state["tiempo_corriendo_s"] = corriendo_s
        params = _load_cycle_params()
        # Incluir compensación general actual para el plan (evita leer estado otra vez).
        params = dict(params)
        params["cut_offset_mm"] = _num(
            state.get("compensacion_medicion_mm"),
            _num(params.get("cut_offset_mm"), 0.0),
        )
        comps = _cut_comp_params_of(params)
        state["feed_sides"] = _feed_sides_of(params)
        state["cut_comp_mode"] = comps["cut_comp_mode"]
        state["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
        state["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
        length = _num(state.get("longitud_modelo_mm"), 0.0)
        if length > 0:
            plan = _build_cut_plan(length, params, state["feed_sides"])
            state["cut_base_mm"] = plan["base_mm"]
            if comps["cut_comp_mode"]:
                state["cut_mm_r"] = plan["base_mm"] + comps["cut_comp_r_mm"]
                state["cut_mm_l"] = plan["base_mm"] + comps["cut_comp_l_mm"]
            else:
                state["cut_mm_r"] = plan["last_mm"]
                state["cut_mm_l"] = plan["last_mm"]
            state["cut_mm_last"] = plan["last_mm"]
            state["cut_comp_sequential"] = bool(plan["sequential"])
        else:
            state["cut_base_mm"] = None
            state["cut_mm_r"] = None
            state["cut_mm_l"] = None
            state["cut_mm_last"] = None
            state["cut_comp_sequential"] = False
        # Leer omit flags aquí; armar resumen fuera del candado (evita reentrar
        # _STATE_LOCK cuando hay E095/E098 y congela toda la HMI).
        omit_l = bool(_MACHINE_STATE.get("omitir_holgura_l"))
        omit_r = bool(_MACHINE_STATE.get("omitir_holgura_r"))
        state["ciclo_paso_a_paso"] = bool(_CYCLE_CTRL.get("step_mode"))
        state["recovery"] = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
    # Efectivo = manual + auto (lado contrario a feed_sides).
    omit_l, omit_r = _holgura_omit_effective(
        state.get("feed_sides"), omit_l=omit_l, omit_r=omit_r
    )
    state["omitir_holgura_l"] = omit_l
    state["omitir_holgura_r"] = omit_r
    state["prefeeder"] = _prefeeder_public_summary(omit_l=omit_l, omit_r=omit_r)
    return state


def _set_machine_ui(
    *,
    estado=None,
    estado_label=None,
    mensaje=None,
    step_id=None,
    step_index=None,
    paralelo_id=None,
    en_curso=None,
    ciclo_error=None,
    progreso_pct=None,
    tiempo_corriendo_s=None,
    tiempo_ciclo_s=None,
    feed=None,
) -> None:
    with _STATE_LOCK:
        if estado is not None:
            _MACHINE_STATE["estado"] = estado
        if estado_label is not None:
            _MACHINE_STATE["estado_label"] = estado_label
        if mensaje is not None:
            _MACHINE_STATE["mensaje"] = mensaje
        if step_id is not None:
            _MACHINE_STATE["ciclo_paso_id"] = step_id
            _, meta = _cycle_step_meta(step_id)
            _MACHINE_STATE["ciclo_paso_label"] = meta["label"] if meta else step_id
        if step_index is not None:
            _MACHINE_STATE["ciclo_paso_indice"] = step_index
            if progreso_pct is None:
                if int(_LOT.get("cycles_total") or 0) > 0:
                    _MACHINE_STATE["progreso_pct"] = _lot_progress_pct(step_index)
                else:
                    _MACHINE_STATE["progreso_pct"] = _cycle_progress_pct(step_index)
        if paralelo_id is not None:
            _MACHINE_STATE["ciclo_paralelo_id"] = paralelo_id
        if progreso_pct is not None:
            _MACHINE_STATE["progreso_pct"] = progreso_pct
        if en_curso is not None:
            _MACHINE_STATE["ciclo_en_curso"] = en_curso
        if ciclo_error is not None:
            _MACHINE_STATE["ciclo_error"] = ciclo_error
        if tiempo_corriendo_s is not None:
            _MACHINE_STATE["tiempo_corriendo_s"] = tiempo_corriendo_s
        if tiempo_ciclo_s is not None:
            _MACHINE_STATE["tiempo_ciclo_s"] = tiempo_ciclo_s
        if feed is not None:
            _MACHINE_STATE["ciclo_feed"] = feed
    _andon_publish_from_state()


def _cycle_should_stop() -> bool:
    return _CYCLE_CTRL["stop"].is_set()


def _cycle_error_active() -> bool:
    """Error de ciclo latched. Sigue activo aunque el hilo ya haya muerto tras el fallo."""
    with _STATE_LOCK:
        has_err = bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
        en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    if not has_err:
        return False
    # Idle con fallo de módulo (sin lote) no debe abortar envíos ajenos al ciclo.
    return en_curso or bool(_LOT.get("active") or _LOT.get("tray_wait")) or _cycle_thread_alive()


def _lot_piece_mult(sides: str | None = None) -> int:
    s = (sides if sides is not None else _LOT.get("sides") or "R").upper()
    return 2 if s in ("LR", "RL", "BOTH", "AMBOS") else 1


def _lot_avg_cycle_s() -> float | None:
    if _PROD_CADENCE_SAMPLES:
        return sum(_PROD_CADENCE_SAMPLES) / len(_PROD_CADENCE_SAMPLES)
    if _LAST_PIECE_S and _LAST_PIECE_S > 0:
        return float(_LAST_PIECE_S)
    return None


def _lot_times_now() -> tuple[float | None, float]:
    """(tiempo_estimado_s restante del lote, tiempo_corriendo_s del lote o de la pieza)."""
    now = time.time()
    with _STATE_LOCK:
        estado = _MACHINE_STATE.get("estado")
        en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
        frozen = float(_MACHINE_STATE.get("tiempo_corriendo_s") or 0)
    lote_vivo = bool(_LOT.get("active") or _LOT.get("tray_wait"))
    corriendo = 0.0
    if lote_vivo and _LOT_T0 is not None:
        # Reloj del lote completo (no se reinicia por pieza).
        corriendo = max(0.0, now - _LOT_T0)
    elif en_curso:
        if estado in ("paused", "error"):
            # Congelado en pausa/error; no sigue sumando.
            if _PIECE_TICK0 is not None and frozen <= 0:
                corriendo = max(0.0, now - _PIECE_TICK0)
            else:
                corriendo = max(0.0, frozen)
        elif _PIECE_TICK0 is not None:
            corriendo = max(0.0, now - _PIECE_TICK0)
    estimado = None
    if lote_vivo:
        avg = _lot_avg_cycle_s()
        if avg is not None and avg > 0:
            total = int(_LOT.get("cycles_total") or 0)
            done = int(_LOT.get("cycles_done") or 0)
            remaining = max(0, total - done)
            if remaining <= 0:
                estimado = 0.0
            else:
                piece_elapsed = 0.0
                if _PIECE_TICK0 is not None and en_curso:
                    piece_elapsed = max(0.0, now - _PIECE_TICK0)
                if en_curso:
                    rest_current = max(0.0, avg - piece_elapsed)
                    estimado = rest_current + max(0, remaining - 1) * avg
                else:
                    estimado = remaining * avg
    return (
        round(estimado, 1) if estimado is not None else None,
        round(corriendo, 1),
    )


def _cycle_worker_is_current(gen: int) -> bool:
    """False si este hilo ya no es el worker oficial (Stop / nuevo Start / huérfano)."""
    return int(_CYCLE_CTRL.get("gen") or 0) == int(gen)


def _cycle_should_abort() -> bool:
    if _cycle_should_stop() or _cycle_error_active():
        return True
    # Hilo de ciclo (o auxiliar) marcado con gen: si ya no es el oficial, abortar.
    my_gen = getattr(_CYCLE_WORKER_TLS, "gen", None)
    if my_gen is not None and not _cycle_worker_is_current(my_gen):
        return True
    return False


def _cycle_motion_halt_for_error() -> None:
    """Para Motion y blower. No toca pinzas: si hay pieza sujeta, se retiene."""
    if _serial_port("motion"):
        _send_motion_urgent(['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}'])
    if _serial_port("plc"):
        _send_plc_serial([_compact({"op": "blower_off"})])


def _gripper_holding_set(holding: bool) -> None:
    global _GRIPPER_HOLDING
    _GRIPPER_HOLDING = bool(holding)


def _gripper_holding() -> bool:
    return bool(_GRIPPER_HOLDING)


def _cycle_thread_alive() -> bool:
    thread = _CYCLE_CTRL["thread"]
    return thread is not None and thread.is_alive()


def _cycle_spawn_worker(start_index: int, *, locked: bool = False) -> tuple[bool, str]:
    """Arranca exactamente un worker. Rechaza si ya hay uno vivo."""

    def _do() -> tuple[bool, str]:
        if _cycle_thread_alive():
            return False, "Hay un ciclo en ejecución."
        _CYCLE_CTRL["gen"] = int(_CYCLE_CTRL.get("gen") or 0) + 1
        gen = int(_CYCLE_CTRL["gen"])
        th = threading.Thread(
            target=_cycle_worker,
            args=(start_index, gen),
            daemon=True,
            name=f"cycle-worker-{gen}",
        )
        _CYCLE_CTRL["thread"] = th
        th.start()
        return True, ""

    if locked:
        return _do()
    with _CYCLE_SPAWN_LOCK:
        return _do()


def _motion_owner_busy() -> str:
    cycle = _CYCLE_CTRL["thread"]
    if cycle is not None and cycle.is_alive():
        return "Hay un ciclo en ejecución."
    routine = _ROUTINE["thread"]
    if routine is not None and routine.is_alive():
        return "Hay una rutina en curso."
    return ""


def _cycle_wait_if_paused() -> bool:
    """True si hay Stop brusco o error de ciclo (abortar de inmediato)."""
    return _cycle_should_abort()


def _cycle_sleep_ms(ms: int) -> bool:
    """True si se completó la espera; False si hubo Stop o error."""
    deadline = time.time() + (max(0, int(ms)) / 1000.0)
    while time.time() < deadline:
        if _cycle_wait_if_paused():
            return False
        time.sleep(0.02)
    return not _cycle_should_abort()


def _cycle_plc_send(payload: dict):
    return _plc_send_payload(payload, push_log=True)


def _cycle_motion_send(payload: dict, timeout_s: float = 2.0, inject=None, on_line=None):
    line = _compact(payload)
    sent = _send_motion_serial(
        [line], line_timeout_s=timeout_s, inject=inject, on_line=on_line
    )
    sent["last_payload"] = payload
    if sent.get("sent"):
        _push_event(f"[Motion] {line}")
    else:
        sent["code"] = "E043" if _serial_port("motion") else "E041"
        _push_event(error_codes.format_message(sent["code"]), "error")
    return sent


def _serial_reply_ok(sent: dict, op: str, fallback: str) -> bool:
    """True si hubo envío y la última respuesta JSON con ese op tiene ok (por defecto True)."""
    # Si el ciclo ya fue pausado por otro hilo, no reportar falso E043 de COM.
    if _cycle_error_active():
        return False
    if not sent.get("sent"):
        if sent.get("busy"):
            return False
        _cycle_fail(sent.get("code") or fallback)
        return False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            if not bool(msg.get("ok", True)):
                _cycle_fail("E044" if op in ("lmguide_move", "lmguide_home", "set_tune", "set_data") else fallback)
                return False
            return True
    # Espera cortada por Stop/error: no confundir con COM mudo.
    if _cycle_should_abort():
        return False
    _cycle_fail(fallback)
    return False


def _motion_status_get() -> dict | None:
    sent = _send_motion_serial(
        ['{"op":"status"}'],
        line_timeout_s=_MOTION_STATUS_TIMEOUT_S,
        reply_until_op="status",
    )
    if not sent.get("sent"):
        return None
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "status":
            return msg
    return None


def _motion_ready_for_cycle() -> tuple[bool, str]:
    status = _motion_status_get()
    if status is None:
        return False, error_codes.format_message("E043")
    if status.get("safety") is True:
        return False, error_codes.format_message("E039")
    if status.get("feeder_r_ready") is not True or status.get("feeder_l_ready") is not True:
        return False, error_codes.format_message("E032")
    # lm_servo_on es el último On enviado por la HMI, no el estado del drive.
    # Un home válido ya demuestra que el servo ejecutó el movimiento a cero.
    if status.get("lm_homed") is not True:
        return False, "LM Guide sin home. Llévalo a cero antes de Start."
    return True, ""


def _plc_cmd_ok(sent: dict, fallback: str) -> bool:
    op = str((sent.get("last_payload") or {}).get("op") or "")
    if not op:
        for raw in reversed(sent.get("replies") or []):
            try:
                msg = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                continue
            if isinstance(msg, dict) and msg.get("op"):
                op = str(msg.get("op"))
                break
    if not op:
        if sent.get("sent"):
            return True
        _cycle_fail(sent.get("code") or fallback)
        return False
    return _serial_reply_ok(sent, op, fallback)


def _motion_cmd_serial_ok(sent: dict, fallback: str) -> bool:
    op = str((sent.get("last_payload") or {}).get("op") or "")
    if not op:
        return _sent_ok(sent, fallback)
    return _serial_reply_ok(sent, op, fallback)


def _sent_ok(sent: dict, fallback: str) -> bool:
    if sent.get("sent"):
        return True
    _cycle_fail(sent.get("code") or fallback)
    return False


def _feed_dict_length_mm(feed: dict | None) -> Optional[float]:
    """Promedio de mm leídos en alimentación (L/R); None si aún no hay medición."""
    feed = feed or {}
    mm_vals = []
    for side in ("R", "L"):
        block = feed.get(side)
        if not isinstance(block, dict):
            continue
        raw = block.get("mm")
        if raw is None:
            continue
        try:
            mm_vals.append(abs(float(raw)))
        except (TypeError, ValueError):
            continue
    if not mm_vals:
        return None
    return sum(mm_vals) / len(mm_vals)


def _measured_feed_length_mm() -> Optional[float]:
    return _feed_dict_length_mm(_MACHINE_STATE.get("ciclo_feed") or {})


def _cycle_blower_length_mm() -> float:
    # Reglas §4.8: duración del blower = longitud programada de la pieza.
    # La medición de encoder no define el soplo (puede ser parcial o con validación off).
    prog = _num(_MOTION_DATA.get("target_length_mm"), 0.0)
    if prog > 0:
        return prog
    snap = _num(_PIECE_BLOWER_MM, 0.0)
    if snap > 0:
        return snap
    measured = _measured_feed_length_mm()
    return _num(measured, 0.0)


def _cycle_blower_duration_ms(params: dict) -> int:
    length_f = _cycle_blower_length_mm()
    per_mm = float(params.get("blower_ms_per_mm") or 8.0)
    minimum = int(params.get("blower_ms_min") or 200)
    ms = int(round(length_f * per_mm))
    return max(minimum, min(ms, 300000))


def _cycle_enter_step(step_id: str, step_index: int) -> None:
    labels = {
        "1": ("running", "Ciclo en marcha", "Secuencia automática iniciada."),
        "2": ("running", "Ciclo en marcha", "Activando holder y encoder."),
        "3": ("running", "Ciclo en marcha", "Alimentación L/R — láser y encoder."),
        "4": ("running", "Ciclo en marcha", "Pinzas y liberación de holder."),
        "5": ("running", "Ciclo en marcha", "LM Guide hacia corte."),
        "6": ("running", "Ciclo en marcha", "Corte neumático R/L."),
        "7": ("running", "Ciclo en marcha", "Descarga de pieza."),
        "8": ("running", "Ciclo en marcha", "Regreso a home y blower."),
    }
    estado, elabel, msg = labels.get(step_id, ("running", "Ciclo en marcha", step_id))
    msg = _cycle_piece_prefix() + msg
    global _STEP4_CLOSES
    if step_id == "4" and _STEP4_CLOSES:
        _STEP4_CLOSES = False
        msg = _cycle_piece_prefix() + "Home con pinzas abiertas. Cierra pinzas y sigue."
    elif step_id == "8" and _OVERLAP["feed_done"] and _LOT["active"]:
        msg = _cycle_piece_prefix() + "Pasos 1 a 3 listos. Esperando home con pinzas abiertas para cerrar."
    if step_id == "8" and _OVERLAP["on"] and not _OVERLAP["feed_done"] and _LOT["active"]:
        nxt = int(_LOT["cycles_done"]) + 2
        total = int(_LOT["cycles_total"])
        msg = msg + f" Ciclo {nxt} de {total} alimenta en paralelo."
    with _STATE_LOCK:
        keep_error = bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
        err_estado = _MACHINE_STATE.get("estado") == "error"
    if keep_error:
        # No pisar el estado de error: la secuencia queda pausada hasta Reset.
        _set_machine_ui(
            step_id=step_id,
            step_index=step_index,
            en_curso=True,
            estado="error" if err_estado or keep_error else estado,
            estado_label="Error",
        )
        return
    _set_machine_ui(
        estado=estado,
        estado_label=elabel,
        mensaje=msg,
        step_id=step_id,
        step_index=step_index,
        en_curso=True,
        ciclo_error="",
    )


def _cycle_fail(code: str, *, mensaje: str | None = None, **fmt) -> None:
    msg = mensaje if mensaje is not None else error_codes.format_message(code, **fmt)
    with _STATE_LOCK:
        in_error = _MACHINE_STATE.get("estado") == "error"
        had_code = bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
        was_en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    # Error ya latched: no re-disparar eventos ni pisar el código (aunque el UI
    # se haya movido a running por un hilo huérfano).
    if had_code:
        if not in_error:
            _set_machine_ui(
                estado="error",
                estado_label="Error",
                en_curso=True,
            )
        # Si el error quedó sin popup de recuperación, abrirlo.
        if not _recovery_get().get("active"):
            existing = str(_MACHINE_STATE.get("ciclo_error") or code or "").strip()
            _recovery_begin(existing, str(_MACHINE_STATE.get("mensaje") or msg))
        return
    if not in_error:
        _push_event(msg, "error")
    # Lote / ciclo en curso no se borran por error: solo Stop (o fin normal) lo hace.
    keep_lot = bool(_LOT.get("active") or _LOT.get("tray_wait") or was_en_curso)
    freeze_s = None
    if keep_lot and _PIECE_TICK0 is not None:
        freeze_s = round(max(0.0, time.time() - _PIECE_TICK0), 1)
    with _STATE_LOCK:
        _MACHINE_STATE["post_error_purga"] = True
    _recovery_begin(code, msg)
    _set_machine_ui(
        estado="error",
        estado_label="Error",
        mensaje=msg,
        en_curso=True if keep_lot else was_en_curso,
        ciclo_error=code,
        tiempo_corriendo_s=freeze_s,
    )
    if _MACHINE_STATE.get("ciclo_en_curso") or _cycle_thread_alive():
        try:
            _prod_log_error(code, msg)
        except Exception:
            pass
    _cycle_motion_halt_for_error()
    if _gripper_holding():
        _push_event(
            "Pinzas retenidas — pieza sujeta hasta Reset o Posición inicial.",
            "warn",
        )
    if _OVERLAP.get("on"):
        _OVERLAP["feed_error"] = True
    if _prefeeder_required() and (
        _MACHINE_STATE.get("ciclo_en_curso")
        or _MACHINE_STATE.get("estado") in ("running", "error")
    ):
        _prefeeder_process_halt_async()


def _cycle_latch_reset() -> None:
    _gripper_holding_set(False)
    with _STATE_LOCK:
        for key in _CYCLE_LATCH:
            _CYCLE_LATCH[key] = False


def _cycle_latch_after_completed_index(completed_idx: int) -> None:
    """Reconstruye interlocks según pasos ya terminados (índice en CYCLE_STEPS, -1 = ninguno)."""
    _cycle_latch_reset()
    if completed_idx >= 1:
        _CYCLE_LATCH["holder_encoder_active"] = True
    if completed_idx >= 2:
        _CYCLE_LATCH["feed_validated"] = True
    if completed_idx >= 3:
        # Tras pinzas: solo se libera el holder; el encoder permanece cerrado.
        _CYCLE_LATCH["holder_encoder_active"] = False
        _CYCLE_LATCH["holder_encoder_released"] = True
        _CYCLE_LATCH["gripper_active"] = True
    if completed_idx >= 4:
        _CYCLE_LATCH["lm_at_cut"] = True
    if completed_idx >= 5:
        _CYCLE_LATCH["cut_done"] = True
    if completed_idx >= 6:
        _CYCLE_LATCH["discharge_done"] = True


def _cycle_assert_preconditions(step_index: int) -> bool:
    """False → pausa con error; no avanzar sin la secuencia lógica previa."""
    _cycle_latch_after_completed_index(step_index - 1)
    step_id = CYCLE_STEPS[step_index]["id"]

    if step_id == "3":
        if not _CYCLE_LATCH["holder_encoder_active"]:
            _cycle_fail("E001")
            return False
        ok_hw, code = _plc_confirm({"holder": True, "encoder": True})
        if not ok_hw:
            if code:
                _cycle_fail(code)
            return False

    if step_id == "4":
        if not _CYCLE_LATCH["feed_validated"]:
            _cycle_fail("E002")
            return False

    if step_id == "5":
        if not _CYCLE_LATCH["gripper_active"] or not _CYCLE_LATCH["holder_encoder_released"]:
            _cycle_fail("E003")
            return False
        if _CYCLE_LATCH["holder_encoder_active"]:
            _cycle_fail("E003")
            return False
        ok_hw, code = _plc_confirm(
            {"gripper": True, "holder": False, "encoder": True}
        )
        if not ok_hw:
            if code:
                _cycle_fail(code)
            return False

    if step_id == "6":
        if not _CYCLE_LATCH["lm_at_cut"]:
            _cycle_fail("E004")
            return False

    if step_id == "7":
        if not _CYCLE_LATCH["cut_done"]:
            _cycle_fail("E005")
            return False

    if step_id == "8":
        if not _CYCLE_LATCH["discharge_done"]:
            _cycle_fail("E006")
            return False

    return True


def _cycle_latch_mark_step_done(step_index: int) -> None:
    _cycle_latch_after_completed_index(step_index)


def _read_feed_stage(sent: dict):
    """Respuesta de feed_stage: encontrada, ok, nombre de error, lectura L/R."""
    feed = {"R": None, "L": None}
    found = False
    ok = False
    error_name = ""
    for raw in sent.get("replies") or []:
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") != "feed_stage":
            continue
        found = True
        ok = bool(msg.get("ok"))
        error_name = str(msg.get("error") or "")
        for side in ("R", "L"):
            block = msg.get(side)
            if isinstance(block, dict):
                feed[side] = {
                    "laser": block.get("laser"),
                    "enc_ok": block.get("enc_ok"),
                    "mm": block.get("mm"),
                }
    return found, ok, error_name, feed


def _fail_feed_stage(error_name: str) -> None:
    if _OVERLAP.get("feed_done"):
        return
    if _cycle_should_stop():
        return
    err = (error_name or "").strip() or "feed_no_reply"
    if err == "ok":
        err = "unknown"
    code = error_codes.feed_stage_code(err)
    if code == "E038":
        _cycle_fail("E038", detail=err)
    else:
        _cycle_fail(code)


def _cycle_move_lm_mm(mm: float, inject=None, on_line=None) -> bool:
    tune = _load_motion_tune()
    puu = _mm_to_puu(mm)
    return _motion_cmd_serial_ok(
        _cycle_motion_send(
            {
                "op": "lmguide_move",
                "puu": puu,
                "lm_pr_speed_rpm": tune["lmguide"]["pr_speed_rpm"],
            },
            timeout_s=_MOTION_LM_MOVE_TIMEOUT_S,
            inject=inject,
            on_line=on_line,
        ),
        "E043",
    )


def _cycle_lm_home_absolute() -> bool:
    """Mueve el LM Guide a PUU 0 absoluto (no homing mecánico)."""
    tune = _load_motion_tune()
    return _motion_cmd_serial_ok(
        _cycle_motion_send(
            {
                "op": "lmguide_move",
                "target": "home",
                "lm_pr_speed_rpm": tune["lmguide"]["pr_speed_rpm"],
            },
            timeout_s=_MOTION_LM_MOVE_TIMEOUT_S,
        ),
        "E043",
    )


def _cycle_encoder_reset() -> bool:
    """Pide reset de encoder R+L al Motion. Usar solo con holder/encoder ya cerrados."""
    return _motion_cmd_serial_ok(
        _cycle_motion_send({"op": "encoder_reset"}, timeout_s=2.0),
        "E043",
    )


def _cycle_after_feed_encoder_reset(params: dict) -> bool:
    """Tras FeedStage: espera de estabilización y segundo reset de encoder."""
    if not _cycle_sleep_ms(int(params["ms_after_feed"])):
        return False
    return _cycle_encoder_reset()


def _cycle_run_step(step_id: str, params: dict) -> bool:
    # Step 1. Start — publicar datos de ciclo (modelo, longitud, tolerancia) al MCU Motion.
    if step_id == "1":
        flat = _motion_data_flat(_MOTION_DATA)
        return _motion_cmd_serial_ok(_cycle_motion_send(flat, timeout_s=2.0), "E043")

    # Step 2. Cerrar holder+encoder, estabilizar, reset encoder (antes de alimentar).
    if step_id == "2":
        if not _plc_cmd_ok(_cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": True}), "E019"):
            return False
        if not _cycle_sleep_ms(int(params["ms_holder_enc_stable"])):
            return False
        ok_hw, code = _plc_confirm({"holder": True, "encoder": True})
        if not ok_hw:
            if code:
                _cycle_fail(code)
            return False
        return _cycle_encoder_reset()

    # Step 3. Alimentar; al terminar FeedStage, esperar y resetear encoder otra vez.
    if step_id == "3":
        sides = _run_sides()
        if not _motion_cmd_serial_ok(_push_saved_motion_tune(), "E043"):
            return False
        payload = _feed_stage_payload(sides if sides in ("R", "L") else None)
        side_name = {"R": "derecho", "L": "izquierdo", "LR": "L y R"}[sides]
        _set_machine_ui(mensaje=_cycle_piece_prefix() + f"Alimentación {side_name}.")
        sent = _cycle_motion_send(payload, timeout_s=65.0)
        found, ok, error_name, feed = _read_feed_stage(sent)
        if not sent.get("sent"):
            _cycle_fail(sent.get("code") or "E043")
            return False
        if not found or not ok:
            _fail_feed_stage(error_name if found else "feed_no_reply")
            return False
        _set_machine_ui(feed=feed)
        global _PIECE_BLOWER_MM, _PIECE_FEED
        prog = _target_length_mm()
        _PIECE_BLOWER_MM = prog if prog > 0 else _feed_dict_length_mm(feed)
        _PIECE_FEED = {"R": feed.get("R"), "L": feed.get("L")}
        return _cycle_after_feed_encoder_reset(params)

    # Step 4. Activar pinzas; liberar holder; encoder permanece cerrado.
    if step_id == "4":
        if not _plc_cmd_ok(_cycle_plc_send({"op": "valve", "name": "gripper", "on": True}), "E019"):
            return False
        # Desde el cierre: si falla el resto del ciclo, retener (no abrir).
        _gripper_holding_set(True)
        if not _plc_cmd_ok(_cycle_plc_send({"op": "valve", "name": "holder", "on": False}), "E019"):
            return False
        if not _cycle_sleep_ms(int(params["ms_after_grip_release"])):
            return False
        ok_hw, code = _plc_confirm(
            {"gripper": True, "holder": False, "encoder": True}
        )
        if not ok_hw:
            if code:
                _cycle_fail(code)
            return False
        return True

    # Step 5. LM Guide a primera (o única) posición de corte; esperar.
    if step_id == "5":
        plan = _active_cut_plan()
        if plan is None:
            _cycle_fail("E007")
            return False
        cut_mm = float(plan["first_mm"])
        if plan.get("sequential"):
            _push_event(
                f"LM Guide a corte 1/{plan['first_label']}: {cut_mm:.2f} mm "
                f"(comp R {plan['comp_r_mm']:+.2f} / L {plan['comp_l_mm']:+.2f})."
            )
        else:
            _push_event(f"LM Guide a corte: {cut_mm:.2f} mm.")
        if not _cycle_move_lm_mm(cut_mm):
            return False
        return _cycle_sleep_ms(int(params["ms_lm_at_cut"]))

    # Step 6. Cortar en la posición actual; si hay compensación dual aplicable,
    # mover a la segunda posición (offset mayor) y cortar el otro lado.
    if step_id == "6":
        plan = _active_cut_plan()
        if plan is None:
            _cycle_fail("E007")
            return False

        def pulse_cutter(valve: str, label: str) -> bool:
            _set_machine_ui(mensaje=_cycle_piece_prefix() + f"Corte {label}.")
            if not _plc_cmd_ok(
                _cycle_plc_send({"op": "valve", "name": valve, "on": True}), "E019"
            ):
                return False
            if not _cycle_sleep_ms(int(params["ms_cutter_on"])):
                return False
            if not _plc_cmd_ok(
                _cycle_plc_send({"op": "valve", "name": valve, "on": False}), "E019"
            ):
                return False
            return _cycle_sleep_ms(int(params["ms_after_cutter"]))

        if not pulse_cutter(str(plan["first_valve"]), str(plan["first_label"])):
            return False
        if not plan.get("sequential"):
            return True

        second_mm = float(plan["second_mm"])
        _push_event(
            f"LM Guide a corte 2/{plan['second_label']}: {second_mm:.2f} mm."
        )
        if not _cycle_move_lm_mm(second_mm):
            return False
        if not _cycle_sleep_ms(int(params["ms_lm_at_cut"])):
            return False
        return pulse_cutter(str(plan["second_valve"]), str(plan["second_label"]))

    # Step 7. LM Guide al depósito del montón; liberar pinzas; esperar.
    if step_id == "7":
        pos = _active_cut_deposit()
        if pos is None:
            _cycle_fail("E007")
            return False
        _cut_mm, disc_mm, stack = pos
        _push_event(f"Depósito montón {stack}: {disc_mm:.2f} mm.")
        if not _cycle_move_lm_mm(disc_mm):
            return False
        if not _cycle_sleep_ms(int(params["ms_lm_at_discharge"])):
            return False
        # Error en el tramo con pieza sujeta: no abrir pinzas (retención).
        if _cycle_should_abort():
            return False
        if not _plc_cmd_ok(_cycle_plc_send({"op": "valve", "name": "gripper", "on": False}), "E019"):
            return False
        _gripper_holding_set(False)
        # La suelta de pinzas es el arranque del siguiente ciclo, si lo hay.
        if (
            _next_piece_follows()
            and not _OVERLAP["on"]
            and not _OVERLAP["feed_done"]
            and not _CYCLE_CTRL["pause_after_step"].is_set()
            and not _cycle_should_stop()
            and not _cycle_error_active()
        ):
            if not _overlap_arm(params):
                return False
        if not _cycle_sleep_ms(int(params["ms_after_grip_release"])):
            if _cycle_should_stop():
                _overlap_reset()
            return False
        return True

    # Step 8. Despeje, regreso a home y blower en ese regreso.
    if step_id == "8":
        pos = _active_cut_deposit()
        if pos is None:
            _cycle_fail("E007")
            return False
        _cut_mm, deposit, _stack = pos
        clearance = (
            _LOT["clearance_mm"]
            if _LOT["active"]
            else abs(_num(params.get("gripper_clearance_mm"), 0.0))
        )
        clear_at = _clear_of(deposit, clearance)
        if (
            _next_piece_follows()
            and not _OVERLAP["on"]
            and not _OVERLAP["feed_done"]
            and not _cycle_should_stop()
            and not _cycle_error_active()
        ):
            if not _overlap_arm(params):
                return False
        if abs(clear_at - deposit) >= 0.05:
            _push_event(f"Despeje de pinzas: {clear_at:.2f} mm.")
            feed_inject, feed_line = _overlap_feed_hook()
            if not _cycle_move_lm_mm(clear_at, inject=feed_inject, on_line=feed_line):
                return False
        home_result: dict = {}

        def send_home() -> None:
            feed_inject, feed_line = _overlap_feed_hook()
            home_result["sent"] = _cycle_motion_send(
                {"op": "lmguide_move", "target": "home"},
                timeout_s=_MOTION_LM_MOVE_TIMEOUT_S,
                inject=feed_inject,
                on_line=feed_line,
            )

        home_thread = threading.Thread(target=send_home, daemon=True)
        home_thread.start()
        if not _cycle_sleep_ms(int(params["ms_blower_delay"])):
            _send_motion_urgent(['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}'])
            return False

        def _lm_already_home() -> bool:
            if home_thread.is_alive():
                return False
            mst = _motion_status_get()
            return bool(mst and mst.get("lm_homed") is True)

        # Soplo solo en el regreso. Si ya está en home, no enciende.
        blower_started = False
        if not _lm_already_home():
            blower_ms = _cycle_blower_duration_ms(params)
            _push_event(
                f"Blower ON {blower_ms} ms "
                f"(longitud programada {_cycle_blower_length_mm():.1f} mm; "
                f"corte al llegar a home)."
            )
            if not _plc_cmd_ok(_cycle_plc_send({"op": "blower_on", "ms": blower_ms}), "E019"):
                _send_motion_urgent(['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}'])
                return False
            blower_started = True
        else:
            _push_event("LM Guide ya en home — blower omitido.")

        while home_thread.is_alive():
            if _cycle_wait_if_paused():
                _send_motion_urgent(['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}'])
                _cycle_plc_send({"op": "blower_off"})
                return False
            home_thread.join(0.05)

        if _cycle_should_stop():
            return False
        home_sent = home_result.get("sent") or {}
        replies = home_sent.get("replies") or []
        _overlap_ensure_feed_collected(replies, budget_s=4.0)
        home_ok = _motion_cmd_serial_ok(home_sent, "E043")
        if home_ok and _OVERLAP["on"] and not _OVERLAP["feed_done"] and not _OVERLAP["feed_sent"]:
            _overlap_start_feed_task()
        if not home_ok:
            return False

        t_home_ok = time.time()
        home_confirmed = False
        while time.time() - t_home_ok < 3.0:
            if _cycle_wait_if_paused():
                _cycle_plc_send({"op": "blower_off"})
                return False
            _overlap_poll_feed(0.15)
            mst = _motion_status_get()
            if mst and mst.get("lm_homed") is True:
                home_confirmed = True
                break
            time.sleep(0.05)
        if not home_confirmed:
            _cycle_fail("E044")
            return False

        # Límite: en home se corta el soplo (no sigue soplando parado en origen).
        if blower_started:
            _cycle_plc_send({"op": "blower_off"})
            _push_event("Blower OFF — LM Guide en home.")

        home_settle_deadline = time.time() + (int(params["ms_at_home_end"]) / 1000.0)
        while time.time() < home_settle_deadline:
            if _cycle_wait_if_paused():
                _cycle_plc_send({"op": "blower_off"})
                return False
            _overlap_poll_feed(0.15)
            if not _OVERLAP["feed_sent"] and _OVERLAP["on"] and not _OVERLAP["feed_done"]:
                _overlap_start_feed_task()
            time.sleep(0.05)
        if _OVERLAP["feed_error"]:
            return False
        return True

    return True


def _next_piece_follows() -> bool:
    """Hay otra pieza del lote después de la que se está cerrando."""
    if _CYCLE_CTRL.get("step_mode"):
        return False
    if not _LOT["active"] or _LOT["tray_wait"]:
        return False
    done_after = int(_LOT["cycles_done"]) + 1
    if done_after >= int(_LOT["cycles_total"]):
        return False
    deposits = int(_LOT["deposits_per_tray"] or 0)
    if deposits and done_after % deposits == 0:
        return False
    return True


def _overlap_reset() -> None:
    _OVERLAP["gen"] += 1
    _OVERLAP["on"] = False
    _OVERLAP["feed_sent"] = False
    _OVERLAP["feed_done"] = False
    _OVERLAP["post_enc_reset_done"] = False
    _OVERLAP["holder_error"] = ""
    _OVERLAP["feed"] = None
    _OVERLAP["blower_mm"] = None
    _OVERLAP["t0"] = None
    _OVERLAP["feed_error"] = False
    _OVERLAP["feed_task_started"] = False
    _OVERLAP["holder_ready"].clear()
    global _STEP4_CLOSES
    _STEP4_CLOSES = False
    if _MACHINE_STATE.get("ciclo_paralelo_id"):
        _set_machine_ui(paralelo_id="")


def _plc_accepted(sent: dict, op: str) -> bool:
    if not sent.get("sent"):
        return False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return bool(msg.get("ok", True))
    return False


def _overlap_fail_holder(code: str) -> None:
    """Falla del holder en solape: aborta el ciclo al instante (no espera al paso 8)."""
    if not code or code == "stop":
        _OVERLAP["holder_error"] = "stop"
        return
    _OVERLAP["holder_error"] = code
    _OVERLAP["feed_error"] = True
    _cycle_fail(code)


def _overlap_start_holder(params: dict) -> None:
    gen = int(_OVERLAP["gen"])
    worker_gen = getattr(_CYCLE_WORKER_TLS, "gen", None)

    def run() -> None:
        if worker_gen is not None:
            _CYCLE_WORKER_TLS.gen = worker_gen
        if _OVERLAP["gen"] != gen or _cycle_should_abort():
            return
        t_holder = time.perf_counter()
        nxt = int(_LOT["cycles_done"]) + 2
        _set_machine_ui(paralelo_id="2")
        sent = _cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": True})
        if _OVERLAP["gen"] != gen:
            return
        if not _plc_accepted(sent, "valve"):
            _overlap_fail_holder(sent.get("code") or "E019")
            return
        if not _cycle_sleep_ms(int(params["ms_holder_enc_stable"])):
            if _OVERLAP["gen"] == gen:
                _OVERLAP["holder_error"] = "stop"
            return
        ok_hw, code = _plc_confirm({"holder": True, "encoder": True})
        if _OVERLAP["gen"] != gen:
            return
        if not ok_hw:
            _overlap_fail_holder(code or "E010")
            return
        # Holder/encoder cerrados: reset antes de alimentar (mismo orden que el paso 2).
        sent_enc = _cycle_motion_send({"op": "encoder_reset"}, timeout_s=2.0)
        if _OVERLAP["gen"] != gen:
            return
        if not _motion_cmd_ok(sent_enc, "encoder_reset"):
            _overlap_fail_holder(sent_enc.get("code") or "E043")
            return
        _OVERLAP["holder_ready"].set()
        _prod_log_op(
            "2",
            time.perf_counter() - t_holder,
            piece=nxt,
            parallel=True,
        )

    threading.Thread(target=run, daemon=True).start()


def _overlap_arm(params: dict) -> bool:
    """Paso 1 del siguiente ciclo: datos y ajuste. El holder sale en paralelo."""
    if _OVERLAP["on"] or _OVERLAP["feed_done"]:
        return True
    t_arm = time.perf_counter()
    if not _motion_cmd_serial_ok(
        _cycle_motion_send(_motion_data_flat(_MOTION_DATA), timeout_s=2.0),
        "E043",
    ):
        return False
    if not _motion_cmd_serial_ok(_push_saved_motion_tune(), "E043"):
        return False
    _OVERLAP["on"] = True
    _OVERLAP["t0"] = time.time()
    _OVERLAP["feed_sent"] = False
    _OVERLAP["feed_done"] = False
    _OVERLAP["post_enc_reset_done"] = False
    _OVERLAP["holder_error"] = ""
    _OVERLAP["feed"] = None
    _OVERLAP["blower_mm"] = None
    _OVERLAP["feed_error"] = False
    _OVERLAP["feed_task_started"] = False
    _OVERLAP["holder_ready"].clear()
    nxt = int(_LOT["cycles_done"]) + 2
    total = int(_LOT["cycles_total"])
    _set_machine_ui(
        paralelo_id="1",
        mensaje=(
            _cycle_piece_prefix()
            + f"Pinzas libres. Ciclo {nxt} de {total} inicia en paralelo."
        ),
    )
    _push_event(f"Ciclo {nxt} de {total}: paso 1 en paralelo con la descarga.")
    _prod_log_op(
        "1",
        time.perf_counter() - t_arm,
        piece=nxt,
        parallel=True,
    )
    _overlap_start_holder(params)
    return True


def _overlap_feed_hook():
    """Alimentación del siguiente ciclo mientras el eje todavía se mueve."""
    if not _OVERLAP["on"] or _OVERLAP["feed_done"]:
        return None, None
    return _overlap_feed_inject, _overlap_on_feed_line


def _overlap_on_feed_line(msg: dict) -> None:
    if msg.get("op") != "feed_stage" or _OVERLAP["feed_done"]:
        return
    if not _overlap_accept_feed([_compact(msg)]):
        return
    _set_machine_ui(
        mensaje=(
            _cycle_piece_prefix()
            + "Pasos 1 a 3 listos. Esperando home con pinzas abiertas para cerrar."
        ),
    )


def _overlap_feed_inject():
    """None sigue esperando al holder. [] no escribe. Una lista se manda una vez."""
    if not _OVERLAP["on"] or _OVERLAP["feed_done"] or _OVERLAP["feed_sent"]:
        return []
    if _cycle_should_abort() or _OVERLAP["holder_error"]:
        return []
    if not _OVERLAP["holder_ready"].is_set():
        return None
    _OVERLAP["feed_sent"] = True
    _OVERLAP["feed_t0"] = time.perf_counter()
    nxt = int(_LOT["cycles_done"]) + 2
    total = int(_LOT["cycles_total"])
    _set_machine_ui(
        paralelo_id="3",
        mensaje=_cycle_piece_prefix() + f"Home y soplo. Ciclo {nxt} de {total} alimenta.",
    )
    _push_event(f"Ciclo {nxt} de {total}: alimentación en paralelo con el regreso a home.")
    sides = _run_sides()
    payload = _feed_stage_payload(sides if sides in ("R", "L") else None)
    return [_compact(payload)]


def _replies_have_op(replies, op: str) -> bool:
    for raw in replies or []:
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return True
    return False


def _overlap_accept_feed(replies) -> bool:
    if _OVERLAP["feed_done"]:
        return True
    found, ok, error_name, feed = _read_feed_stage({"replies": replies})
    if _cycle_should_stop() and (not found or not ok):
        return False
    if not found or not ok:
        _OVERLAP["feed_error"] = True
        if not _MACHINE_STATE.get("ciclo_error"):
            _fail_feed_stage(error_name if found else "feed_no_reply")
        return False
    _OVERLAP["feed_done"] = True
    _OVERLAP["feed"] = feed
    prog = _target_length_mm()
    _OVERLAP["blower_mm"] = prog if prog > 0 else _feed_dict_length_mm(feed)
    _set_machine_ui(feed=feed, paralelo_id="")
    nxt = int(_LOT["cycles_done"]) + 2
    total = int(_LOT["cycles_total"])
    _push_event(f"Ciclo {nxt} de {total}: pasos 1 a 3 listos.")
    t0 = _OVERLAP.get("feed_t0")
    if t0 is not None:
        _prod_log_op(
            "3",
            time.perf_counter() - float(t0),
            piece=nxt,
            parallel=True,
            feed=feed,
        )
    return True


def _motion_drain_replies(
    want_op: str, budget_s: float, *, blocking: bool = True
) -> list[str]:
    """Lee respuestas pendientes en Motion sin enviar órdenes nuevas."""
    port = _serial_port("motion")
    if not port:
        return []
    try:
        import serial
    except ImportError:
        return []
    global _motion_serial
    wait_s = max(0.05, min(budget_s, 5.0))
    if blocking:
        if not _serial_lock.acquire(timeout=wait_s):
            return []
    elif not _serial_lock.acquire(blocking=False):
        return []
    try:
        ser = _motion_serial
        if ser is None or not ser.is_open or getattr(ser, "port", None) != port:
            return []
        return _read_lines_until_op(ser, want_op, budget_s, trace_prefix=None)
    finally:
        _serial_lock.release()


def _overlap_try_accept_from_replies(replies) -> None:
    if _OVERLAP["feed_done"]:
        return
    if replies and _replies_have_op(replies, "feed_stage"):
        _overlap_accept_feed(replies)


def _overlap_poll_feed(budget_s: float = 0.35) -> None:
    if not _OVERLAP["on"] or _OVERLAP["feed_done"] or not _OVERLAP["feed_sent"]:
        return
    lines = _motion_drain_replies("feed_stage", budget_s, blocking=budget_s >= 0.35)
    if lines:
        _overlap_accept_feed(lines)


def _overlap_ensure_feed_collected(replies, budget_s: float = 3.0) -> None:
    """Recolecta feed_stage si la respuesta llegó tras lmguide_move/home."""
    if not _OVERLAP["on"] or _OVERLAP["feed_done"]:
        return
    _overlap_try_accept_from_replies(replies)
    if _OVERLAP["feed_done"]:
        return
    if _OVERLAP["feed_sent"]:
        _overlap_poll_feed(budget_s)


def _overlap_send_feed_blocking() -> bool:
    """Envía feed_stage y procesa respuesta (hilo o espera antes del paso 4)."""
    if _OVERLAP["feed_done"]:
        return _overlap_post_feed_encoder_reset()
    if _cycle_should_stop():
        return False
    if _OVERLAP["holder_error"] and not _OVERLAP["holder_ready"].is_set():
        _cycle_fail(_OVERLAP["holder_error"] if _OVERLAP["holder_error"] != "stop" else "E010")
        return False
    if not _OVERLAP["holder_ready"].wait(3.0):
        if _cycle_should_stop():
            return False
        _cycle_fail(_OVERLAP["holder_error"] or "E010")
        return False
    if _OVERLAP["feed_sent"]:
        _overlap_poll_feed(2.0)
        if not _OVERLAP["feed_done"]:
            return False
        return _overlap_post_feed_encoder_reset()
    _OVERLAP["feed_sent"] = True
    _OVERLAP["feed_t0"] = time.perf_counter()
    _set_machine_ui(paralelo_id="3")
    sides = _run_sides()
    payload = _feed_stage_payload(sides if sides in ("R", "L") else None)
    sent = _cycle_motion_send(payload, timeout_s=65.0)
    if _cycle_should_stop():
        return False
    if not sent.get("sent"):
        _cycle_fail(sent.get("code") or "E043")
        return False
    if not _overlap_accept_feed(sent.get("replies") or []):
        return False
    return _overlap_post_feed_encoder_reset()


def _overlap_start_feed_task() -> None:
    """Alimentación del siguiente ciclo sin bloquear el paso 8 en curso."""
    if (
        not _OVERLAP["on"]
        or _OVERLAP["feed_done"]
        or _OVERLAP["feed_sent"]
        or _OVERLAP["feed_task_started"]
    ):
        return
    _OVERLAP["feed_task_started"] = True
    gen = int(_OVERLAP["gen"])
    worker_gen = getattr(_CYCLE_WORKER_TLS, "gen", None)

    def run() -> None:
        if worker_gen is not None:
            _CYCLE_WORKER_TLS.gen = worker_gen
        try:
            if _OVERLAP["gen"] != gen or _cycle_should_abort():
                return
            _overlap_send_feed_blocking()
        finally:
            if not _OVERLAP["feed_sent"] and not _OVERLAP["feed_done"]:
                _OVERLAP["feed_task_started"] = False

    threading.Thread(target=run, daemon=True).start()


def _overlap_post_feed_encoder_reset() -> bool:
    """Tras FeedStage en paralelo: espera ms_after_feed y resetea encoder (una vez)."""
    if _OVERLAP.get("post_enc_reset_done"):
        return True
    if not _cycle_after_feed_encoder_reset(_load_cycle_params()):
        return False
    _OVERLAP["post_enc_reset_done"] = True
    return True


def _overlap_wait_feed_done(timeout_s: float = 120.0) -> bool:
    """Espera la alimentación solapada antes de entregar el paso 4 al siguiente ciclo."""
    if not _OVERLAP["on"]:
        return True
    if _OVERLAP["feed_error"]:
        return False
    if _OVERLAP["feed_done"]:
        return _overlap_post_feed_encoder_reset()
    total = int(_LOT["cycles_total"])
    nxt = int(_LOT["cycles_done"]) + 1
    _set_machine_ui(
        estado="running",
        estado_label="Ciclo en marcha",
        mensaje=(
            _cycle_piece_prefix()
            + f"Pieza {nxt} de {total}: esperando alimentación (paso 3 en paralelo)…"
        ),
        en_curso=True,
    )
    if not _OVERLAP["feed_sent"] and not _OVERLAP["feed_task_started"]:
        _overlap_start_feed_task()
    deadline = time.time() + timeout_s
    blocking_after = time.time() + 0.75
    while time.time() < deadline:
        if _cycle_should_abort():
            return False
        if _OVERLAP["feed_done"]:
            return _overlap_post_feed_encoder_reset()
        if _OVERLAP["feed_error"]:
            return False
        _overlap_poll_feed(1.0)
        if not _OVERLAP["feed_done"] and not _OVERLAP["feed_sent"]:
            if not _OVERLAP["feed_task_started"]:
                _overlap_start_feed_task()
            elif time.time() >= blocking_after:
                if not _overlap_send_feed_blocking():
                    if _OVERLAP["feed_error"] or _MACHINE_STATE.get("ciclo_error"):
                        return False
                    if _cycle_should_abort():
                        return False
                blocking_after = time.time() + 2.0
        time.sleep(0.05)
    _cycle_fail("E038", detail="feed_overlap_timeout")
    return False


def _overlap_handoff() -> tuple[int, float | None]:
    """Paso 8 ya acabó (home, pinzas abiertas) y los pasos 1 a 3 también."""
    if not _OVERLAP["feed_done"]:
        return 0, None
    global _PIECE_BLOWER_MM, _PIECE_FEED, _STEP4_CLOSES
    mm = _OVERLAP["blower_mm"]
    t0 = _OVERLAP["t0"]
    feed = _OVERLAP["feed"]
    _overlap_reset()
    _STEP4_CLOSES = True
    if mm is not None:
        _PIECE_BLOWER_MM = mm
    if feed is not None:
        _PIECE_FEED = {"R": feed.get("R"), "L": feed.get("L")}
        _set_machine_ui(feed=feed)
    _push_event("Home con pinzas abiertas. El paso 4 cierra y sigue.")
    return 3, t0


def _finish_piece_idle(mensaje: str) -> None:
    global _LAST_PIECE_S, _PIECE_TICK0, _LOT_T0
    lot_elapsed = 0.0
    if _LOT_T0 is not None:
        lot_elapsed = max(0.0, time.time() - _LOT_T0)
    elif _LAST_PIECE_S:
        lot_elapsed = float(_LAST_PIECE_S)
    total_c = int(_LOT.get("cycles_total") or 0)
    done_c = int(_LOT.get("cycles_done") or 0)
    lot_done = total_c > 0 and done_c >= total_c
    _LOT["active"] = False
    _LOT["tray_wait"] = False
    _LOT["home_before"] = False
    _CYCLE_CTRL["step_mode"] = False
    _PIECE_TICK0 = None
    _LOT_T0 = None
    _set_machine_ui(
        estado="idle",
        estado_label="En espera",
        mensaje=mensaje,
        en_curso=False,
        ciclo_error="",
        step_id="",
        step_index=len(CYCLE_STEPS) if lot_done else 0,
        progreso_pct=100 if lot_done else _lot_progress_pct(0),
        tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
        tiempo_corriendo_s=round(lot_elapsed, 1) if lot_done else 0,
        paralelo_id="",
    )
    with _STATE_LOCK:
        _MACHINE_STATE["tiempo_estimado_s"] = 0 if lot_done else None
    _overlap_reset()
    _cycle_latch_reset()
    _prefeeder_lock_after_cycle_async()


def _enter_tray_wait(deposits: int) -> None:
    _LOT["tray_wait"] = True
    lot_elapsed = 0.0
    if _LOT_T0 is not None:
        lot_elapsed = max(0.0, time.time() - _LOT_T0)
    _set_machine_ui(
        estado="paused",
        estado_label="Charola llena",
        mensaje=f"Charola llena — {deposits} ciclos. Vacía la charola y confirma.",
        en_curso=True,
        step_id="",
        step_index=0,
        progreso_pct=_lot_progress_pct(0),
        tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
        tiempo_corriendo_s=round(lot_elapsed, 1),
        paralelo_id="",
    )
    _push_event(f"Charola llena — {deposits} ciclos.", "warn")
    _overlap_reset()
    _cycle_latch_reset()


def _home_after_tray() -> bool:
    _LOT["home_before"] = False
    _set_machine_ui(
        estado="running",
        estado_label="Ciclo en marcha",
        mensaje="Charola vacía. Regresando el eje a cero.",
        en_curso=True,
    )
    sent = _cycle_motion_send(
        {"op": "lmguide_move", "target": "home"},
        timeout_s=_MOTION_LM_MOVE_TIMEOUT_S,
    )
    if _motion_cmd_serial_ok(sent, "E043"):
        return True
    _LOT["tray_wait"] = True
    _push_event("El eje no volvió a cero. Confirma otra vez cuando pueda moverse.", "error")
    _set_machine_ui(
        estado="paused",
        estado_label="Charola llena",
        mensaje="El eje no volvió a cero. Vacía la charola y confirma otra vez.",
        en_curso=True,
    )
    return False


def _piece_log_name() -> str:
    if _LOT["active"]:
        n = int(_LOT["cycles_done"]) + 1
        total = int(_LOT["cycles_total"])
        return f"Pieza {n} de {total}"
    msg = str(_MACHINE_STATE.get("mensaje") or "").rstrip(".")
    if msg.startswith("Pieza de prueba"):
        return msg
    return "Pieza"


def _run_one_piece(start_index: int, started_at: float | None = None) -> bool:
    global _LAST_PIECE_S, _PREV_PIECE_END_T, _PROD_CTX, _PIECE_TICK0
    params = _load_cycle_params()
    regime = bool(started_at is not None and start_index > 0 and _PREV_PIECE_END_T is not None)
    if regime:
        tick0 = _PREV_PIECE_END_T
    else:
        tick0 = time.time()
    _PIECE_TICK0 = tick0
    fresh = start_index <= 0 and not regime
    piece_num = int(_LOT["cycles_done"]) + 1 if _LOT.get("active") else 1
    name = _piece_log_name()
    if fresh:
        _push_event(f"{name} inicia.")
        # Arranque fresco: primero PUU 0 absoluto. Si el eje quedó a media
        # carrera tras un abort, no se debe tratar el corte como offset relativo.
        _set_machine_ui(
            estado="running",
            estado_label="Ciclo en marcha",
            mensaje=_cycle_piece_prefix() + "LM Guide a home (absoluto).",
            en_curso=True,
            step_id="1",
            step_index=0,
        )
        if not _cycle_lm_home_absolute():
            return False
    for idx in range(start_index, len(CYCLE_STEPS)):
        step = CYCLE_STEPS[idx]
        step_id = step["id"]
        t_step = time.perf_counter()
        _PROD_CTX = {
            "piece": piece_num,
            "step_id": step_id,
            "t0": t_step,
            "parallel": False,
        }
        if _cycle_should_abort():
            return False
        _cycle_enter_step(step_id, idx)
        if not _cycle_assert_preconditions(idx):
            return False
        ok = _cycle_run_step(step_id, params)
        if _cycle_error_active():
            return False
        step_s = time.perf_counter() - t_step
        if ok:
            _prod_log_op(step_id, step_s, piece=piece_num, parallel=False)
        elif not _MACHINE_STATE.get("ciclo_error"):
            detail = str(_MACHINE_STATE.get("mensaje") or "falló")
            _prod_log_op(
                step_id,
                step_s,
                piece=piece_num,
                status="ERROR",
                detail=detail,
            )
        if not ok:
            if _MACHINE_STATE.get("ciclo_error"):
                return False
            return False
        _cycle_latch_mark_step_done(idx)
        elapsed = time.time() - tick0
        _set_machine_ui(tiempo_corriendo_s=round(elapsed, 1))
        if _CYCLE_CTRL["pause_after_step"].is_set():
            _CYCLE_CTRL["pause_after_step"].clear()
            next_idx = idx + 1
            # En paso a paso, el último paso cierra la pieza sin pedir otro avance.
            if next_idx < len(CYCLE_STEPS):
                next_step = CYCLE_STEPS[next_idx]["id"]
                step_mode = bool(_CYCLE_CTRL.get("step_mode"))
                if step_mode:
                    nxt = CYCLE_STEPS[next_idx]
                    msg = (
                        f"Paso {step_id} listo. Siguiente: {nxt['id']} — {nxt['label']}. "
                        "Pulsa «Siguiente paso»."
                    )
                    label = "Paso a paso"
                    event = f"Paso a paso — paso {step_id} completado."
                else:
                    msg = "Ciclo en pausa. Reanudar para continuar."
                    label = "Pausa"
                    event = "Pausa — paso actual completado."
                pause_elapsed = (
                    round(time.time() - tick0, 1) if tick0 else round(elapsed, 1)
                )
                _set_machine_ui(
                    estado="paused",
                    estado_label=label,
                    mensaje=msg,
                    en_curso=True,
                    step_id=next_step,
                    step_index=next_idx,
                    tiempo_corriendo_s=pause_elapsed,
                )
                _push_event(event, "warn")
                _prefeeder_process_halt_async()
                return False
    t_end = time.time()
    _LAST_PIECE_S = t_end - tick0
    _PREV_PIECE_END_T = t_end
    if regime:
        kind = "HANDOFF"
        global _PROD_CADENCE_SAMPLES
        _PROD_CADENCE_SAMPLES.append(_LAST_PIECE_S)
    elif fresh:
        kind = "FIRST"
    else:
        kind = "NORMAL"
        _PROD_CADENCE_SAMPLES.append(_LAST_PIECE_S)
    _prod_log_complete(_LAST_PIECE_S, kind)
    if fresh or regime:
        tag = " (régimen)" if regime else ""
        _push_event(f"{name} termina en {_LAST_PIECE_S:.1f} s{tag}.")
    else:
        _push_event(f"{name} termina.")
    return True


def _cycle_worker(start_index: int = 0, gen: int = 0) -> None:
    _CYCLE_WORKER_TLS.gen = int(gen)
    lot_mode = bool(_LOT["active"])
    index = start_index
    started_at = None
    try:
        while True:
            if not _cycle_worker_is_current(gen) or _cycle_should_abort():
                return
            if _LOT["home_before"] and not _home_after_tray():
                return
            if not _run_one_piece(index, started_at):
                return
            _mant_add_cycle()
            if not lot_mode:
                if _cycle_worker_is_current(gen):
                    _finish_piece_idle("Ciclo completado. Listo para siguiente Start.")
                return
            if not _cycle_worker_is_current(gen) or _cycle_should_abort() or not _LOT["active"]:
                return
            _LOT["cycles_done"] += 1
            done = _LOT["cycles_done"]
            total = _LOT["cycles_total"]
            deposits = int(_LOT["deposits_per_tray"] or 0)
            _set_machine_ui(
                progreso_pct=_lot_progress_pct(0),
                tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
            )
            if done >= total:
                avg_cadence = (
                    sum(_PROD_CADENCE_SAMPLES) / len(_PROD_CADENCE_SAMPLES)
                    if _PROD_CADENCE_SAMPLES
                    else 0.0
                )
                _prod_log_summary(done, total, avg_cadence)
                if _CYCLE_CTRL.get("step_mode"):
                    _push_event("Prueba paso a paso terminada.")
                    _finish_piece_idle("Prueba paso a paso terminada.")
                else:
                    _push_event(f"Lote terminado. {done} ciclos.")
                    _finish_piece_idle(f"Lote terminado. {done} ciclos.")
                return
            if deposits and done % deposits == 0:
                _enter_tray_wait(deposits)
                return
            _cycle_latch_reset()
            if not _overlap_wait_feed_done():
                return
            if not _cycle_worker_is_current(gen) or _cycle_should_abort():
                return
            index, started_at = _overlap_handoff()
            if index == 0:
                _set_machine_ui(
                    feed={"R": None, "L": None},
                    tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
                    tiempo_corriendo_s=0,
                    paralelo_id="",
                )
            else:
                _set_machine_ui(
                    tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
                    tiempo_corriendo_s=0,
                    paralelo_id="",
                )
    except Exception as exc:
        if _cycle_worker_is_current(gen):
            _cycle_motion_halt_for_error()
            if not _cycle_error_active():
                _cycle_fail(
                    "E000",
                    mensaje=f"Ciclo abortado por excepción interna: {exc}",
                )
    finally:
        # Solo el worker oficial limpia stop/lote. Cualquier hilo se quita de
        # thread si es el registrado; un huerfano no toca el control del nuevo.
        with _CYCLE_SPAWN_LOCK:
            is_self = _CYCLE_CTRL.get("thread") is threading.current_thread()
            is_owner = _cycle_worker_is_current(gen) and is_self
            if is_self:
                _CYCLE_CTRL["thread"] = None
                _CYCLE_CTRL["pause_after_step"].clear()
            if not is_owner:
                return
            stop_req = _CYCLE_CTRL["stop"].is_set()
            if stop_req:
                _CYCLE_CTRL["stop"].clear()
                _CYCLE_CTRL["step_mode"] = False
        if stop_req:
            _cycle_stop_idle()
        else:
            # Pausa o error: el lote permanece. Si murio en running sin error, latch E043.
            with _STATE_LOCK:
                stuck_running = (
                    bool(_MACHINE_STATE.get("ciclo_en_curso") or _LOT.get("active"))
                    and _MACHINE_STATE.get("estado") == "running"
                    and not str(_MACHINE_STATE.get("ciclo_error") or "").strip()
                )
            if stuck_running and _cycle_worker_is_current(gen):
                _cycle_fail(
                    "E043",
                    mensaje="Ciclo detenido sin respuesta del Motion. Revisa COM o el firmware.",
                )


def _cycle_blocked_by_routine() -> str:
    if not _checklist_bypassed() and not (_MACHINE_STATE.get("checklist") or {}).get("ok"):
        return "Completa la revisión de arranque antes de Iniciar."
    if _MACHINE_STATE.get("post_error_purga"):
        return "Tras un error: Reset y completa la purga antes de Iniciar o Reanudar."
    if _MACHINE_STATE.get("rutina") == "purga":
        return "Hay una purga en curso."
    return _motion_owner_busy()


def _cycle_start(*, step_mode: bool = False) -> tuple[bool, str]:
    blocked = _cycle_blocked_by_routine()
    if blocked:
        return False, blocked
    if _LOT["tray_wait"]:
        return False, "La charola está llena. Vacía y confirma, o pulsa Stop."
    if not _MOTION_DATA.get("modelo_id") or _MOTION_DATA.get("target_length_mm") is None:
        return False, "Selecciona un modelo con longitud antes de Start."
    length = _target_length_mm()
    if length <= 0:
        return False, "Selecciona un modelo con longitud antes de Start."
    params = _load_cycle_params()
    sides = _feed_sides_of(params)
    if step_mode:
        # Una sola pieza: con ambos lados basta cantidad 2 para validar el pedido.
        qty = 2 if sides == "LR" else 1
        cycles, qty_err = _order_cycles(qty, sides)
    else:
        cycles, qty_err = _order_cycles(_MACHINE_STATE.get("cantidad"), sides)
    if qty_err:
        return False, qty_err
    if step_mode:
        cycles = 1
    plan, plan_err = _tray_plan(length, params)
    if plan_err:
        return False, plan_err
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    ready, motion_err = _motion_ready_for_cycle()
    if not ready:
        return False, motion_err
    if _prefeeder_required():
        st = _prefeeder_refresh_cache()
        if not st:
            return False, error_codes.format_message("E092")
        if not _prefeeder_status_ready(st):
            return False, error_codes.format_message("E099")
    if not step_mode:
        _announce_trays(cycles, plan)
    global _LOT_SEQ, _PROD_CADENCE_SAMPLES, _PREV_PIECE_END_T, _PIECE_TICK0, _LOT_T0
    with _CYCLE_SPAWN_LOCK:
        if _cycle_thread_alive():
            return False, "Hay un ciclo en ejecución."
        if _LOT["active"]:
            return False, "Hay un lote en curso. Reanuda o pulsa Stop."
        _LOT_SEQ += 1
        _PROD_CADENCE_SAMPLES = []
        _LOT["active"] = True
        _LOT["log_id"] = _LOT_SEQ
        _LOT["tray_wait"] = False
        _LOT["home_before"] = False
        _LOT["cycles_total"] = cycles
        _LOT["cycles_done"] = 0
        _LOT["stacks"] = plan["stacks"]
        _LOT["deposits_per_tray"] = plan["deposits"]
        _LOT["batch_size"] = plan["batch_size"]
        _LOT["cut_mm"] = plan["cut_mm"]
        cut_comps = _cut_comp_params_of(params)
        _LOT["cut_offset_mm"] = _measure_comp_mm()
        _LOT["cut_comp_mode"] = cut_comps["cut_comp_mode"]
        _LOT["cut_comp_r_mm"] = cut_comps["cut_comp_r_mm"]
        _LOT["cut_comp_l_mm"] = cut_comps["cut_comp_l_mm"]
        _LOT["length_mm"] = plan["length_mm"]
        _LOT["gap_mm"] = plan["gap_mm"]
        _LOT["clearance_mm"] = plan["clearance_mm"]
        _LOT["sides"] = sides
        _CYCLE_CTRL["stop"].clear()
        _CYCLE_CTRL["step_mode"] = bool(step_mode)
        if step_mode:
            _CYCLE_CTRL["pause_after_step"].set()
        else:
            _CYCLE_CTRL["pause_after_step"].clear()
        _overlap_reset()
        _cycle_latch_reset()
        _PREV_PIECE_END_T = None
        _LOT_T0 = time.time()
        _PIECE_TICK0 = _LOT_T0
        if step_mode:
            ui_estado = "running"
            ui_label = "Paso a paso"
            ui_msg = "Prueba paso a paso — ejecutando paso 1."
            event_msg = "Prueba paso a paso iniciada."
        else:
            ui_estado = "running"
            ui_label = "Ciclo en marcha"
            ui_msg = "Secuencia automática iniciada."
            event_msg = None
        est0 = None
        avg0 = _lot_avg_cycle_s()
        if avg0 is not None and cycles > 0:
            est0 = round(cycles * avg0, 1)
        _set_machine_ui(
            estado=ui_estado,
            estado_label=ui_label,
            mensaje=ui_msg,
            en_curso=True,
            ciclo_error="",
            feed={"R": None, "L": None},
            step_id="1",
            step_index=0,
            progreso_pct=0,
            tiempo_ciclo_s=None,
            tiempo_corriendo_s=0,
        )
        with _STATE_LOCK:
            _MACHINE_STATE["tiempo_estimado_s"] = est0
        ok, err = _cycle_spawn_worker(0, locked=True)
        if not ok:
            _LOT["active"] = False
            _PIECE_TICK0 = None
            _LOT_T0 = None
            return False, err
    _prefeeder_unlock_for_cycle()
    if event_msg:
        _push_event(event_msg)
    return True, ""


def _cycle_pause() -> None:
    if _MACHINE_STATE.get("rutina"):
        _push_event("No se puede pausar durante una rutina.", "warn")
        return
    if not (_CYCLE_CTRL["thread"] and _CYCLE_CTRL["thread"].is_alive()):
        _push_event("No hay ciclo en marcha para pausar.", "warn")
        return
    _CYCLE_CTRL["pause_after_step"].set()
    _set_machine_ui(
        estado="running",
        estado_label="Paso a paso" if _CYCLE_CTRL.get("step_mode") else "Ciclo en marcha",
        mensaje="Pausa solicitada. Terminando el paso en curso…",
    )
    _push_event("Pausa solicitada — se detendrá al completar el paso.", "warn")


def _cycle_resume() -> tuple[bool, str]:
    if _LOT["tray_wait"]:
        return False, "La charola está llena. Vacía y confirma."
    if _MACHINE_STATE.get("ciclo_error"):
        return False, "Hay un error activo. Usa Reset cuando la condición se haya corregido."
    if _MACHINE_STATE.get("post_error_purga"):
        return False, "Tras un error debes completar la purga antes de Reanudar."
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "Hay una purga en curso."
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    idx = int(_MACHINE_STATE.get("ciclo_paso_indice") or 0)
    lot_open = bool(_LOT["active"])
    en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    # Pausa limpia: reanuda el paso guardado. Tras error+purga, _rearm_piece
    # ya dejó índice 0; cycles_done no se toca aquí.
    if not lot_open and idx <= 0 and not en_curso:
        return _cycle_start()
    if lot_open and not en_curso:
        # Defensa: el lote sigue, aunque el flag en_curso se haya perdido.
        en_curso = True
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    if idx >= len(CYCLE_STEPS):
        idx = len(CYCLE_STEPS) - 1
    if idx < 0:
        idx = 0
    step_mode = bool(_CYCLE_CTRL.get("step_mode"))
    global _PIECE_TICK0
    # Tras error+purga el reloj de pieza arranca en 0 (reintento).
    # En pausa limpia se conserva el tiempo congelado.
    frozen = float(_MACHINE_STATE.get("tiempo_corriendo_s") or 0)
    with _CYCLE_SPAWN_LOCK:
        if _cycle_thread_alive():
            return False, "Hay un ciclo en ejecución."
        _CYCLE_CTRL["stop"].clear()
        if step_mode:
            _CYCLE_CTRL["pause_after_step"].set()
        else:
            _CYCLE_CTRL["pause_after_step"].clear()
        _PIECE_TICK0 = time.time() - max(0.0, frozen)
        ok, err = _cycle_spawn_worker(idx, locked=True)
        if not ok:
            return False, err
    _prefeeder_unlock_for_cycle()
    if step_mode:
        step = CYCLE_STEPS[idx] if 0 <= idx < len(CYCLE_STEPS) else None
        sid = step["id"] if step else "?"
        label = step["label"] if step else ""
        _set_machine_ui(
            estado="running",
            estado_label="Paso a paso",
            mensaje=f"Ejecutando paso {sid}" + (f" — {label}." if label else "."),
            en_curso=True,
        )
        _push_event(f"Paso a paso — ejecutando paso {sid}.")
    else:
        _set_machine_ui(
            estado="running",
            estado_label="Ciclo en marcha",
            mensaje="Reanudando secuencia.",
            en_curso=True,
        )
        _push_event("Ciclo reanudado.")
    return True, ""


def _cycle_step_start() -> tuple[bool, str]:
    """Inicia una pieza en modo prueba: pausa al completar cada paso."""
    if _cycle_thread_alive() or _MACHINE_STATE.get("ciclo_en_curso"):
        return False, "Hay un ciclo en curso. Deténlo antes de la prueba paso a paso."
    ok, err = _cycle_start(step_mode=True)
    if not ok:
        return False, err
    return True, "Prueba paso a paso iniciada."


def _cycle_step_next() -> tuple[bool, str]:
    """Avanza un paso en la prueba paso a paso (equivalente a Reanudar con pausa armada)."""
    if not _CYCLE_CTRL.get("step_mode"):
        return False, "No hay una prueba paso a paso activa."
    if _MACHINE_STATE.get("estado") != "paused" or not _MACHINE_STATE.get("ciclo_en_curso"):
        return False, "Espera a que termine el paso en curso."
    idx = int(_MACHINE_STATE.get("ciclo_paso_indice") or 0)
    if idx >= len(CYCLE_STEPS):
        _CYCLE_CTRL["step_mode"] = False
        _finish_piece_idle("Prueba paso a paso terminada.")
        return True, "Prueba paso a paso terminada."
    ok, err = _cycle_resume()
    if not ok:
        return False, err
    return True, "Siguiente paso."


def _charola_vacia(_data=None) -> tuple[bool, str]:
    if not _LOT["tray_wait"] or not _LOT["active"]:
        return False, "La charola no está en espera."
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    with _CYCLE_SPAWN_LOCK:
        if _cycle_thread_alive():
            return False, "Hay un ciclo en ejecución."
        _LOT["tray_wait"] = False
        _LOT["home_before"] = True
        _CYCLE_CTRL["stop"].clear()
        _CYCLE_CTRL["pause_after_step"].clear()
        _cycle_latch_reset()
        _set_machine_ui(
            estado="running",
            estado_label="Ciclo en marcha",
            mensaje="Charola vacía. Regresando el eje a cero.",
            en_curso=True,
        )
        ok, err = _cycle_spawn_worker(0, locked=True)
        if not ok:
            _LOT["tray_wait"] = True
            _LOT["home_before"] = False
            return False, err
    _prefeeder_unlock_for_cycle()
    return True, "Charola vacía. El eje vuelve a cero y el siguiente corte sale del primer montón."


def _cycle_stop_idle(mensaje: str = "Stop — ciclo detenido. Listo para Iniciar.") -> None:
    """Solo Stop (o el finally del hilo tras Stop) debe llegar aquí y borrar el lote."""
    global _STEP4_CLOSES, _PIECE_TICK0, _LOT_T0
    _STEP4_CLOSES = False
    _LOT["active"] = False
    _LOT["tray_wait"] = False
    _LOT["home_before"] = False
    _LOT["cycles_done"] = 0
    _LOT["cycles_total"] = 0
    _CYCLE_CTRL["step_mode"] = False
    _PIECE_TICK0 = None
    _LOT_T0 = None
    _overlap_reset()
    _cycle_latch_reset()
    with _STATE_LOCK:
        _MACHINE_STATE["post_error_purga"] = False
        _MACHINE_STATE["recovery"] = _recovery_blank()
    _set_machine_ui(
        estado="idle",
        estado_label="En espera",
        mensaje=mensaje,
        en_curso=False,
        ciclo_error="",
        step_id="",
        step_index=0,
        progreso_pct=0,
        tiempo_corriendo_s=0,
        paralelo_id="",
        feed={"R": None, "L": None},
    )
    with _STATE_LOCK:
        _MACHINE_STATE["tiempo_estimado_s"] = None


def _cycle_stop() -> None:
    """Única acción de operador que cancela el lote en curso."""
    with _CYCLE_SPAWN_LOCK:
        th = _CYCLE_CTRL.get("thread")
        running = th is not None and th.is_alive()
        # Invalida workers oficiales y huerfanos: gen cambia → abort inmediato.
        _CYCLE_CTRL["gen"] = int(_CYCLE_CTRL.get("gen") or 0) + 1
        _CYCLE_CTRL["stop"].set()
        _CYCLE_CTRL["pause_after_step"].clear()
        _CYCLE_CTRL["step_mode"] = False
    _ROUTINE["stop"].set()
    if _serial_port("motion"):
        _send_motion_urgent(['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}'])
    if _serial_port("plc"):
        _send_plc_serial([_compact({"op": "blower_off"})])
    if _MACHINE_STATE.get("rutina") == "purga" and not (
        _ROUTINE["thread"] and _ROUTINE["thread"].is_alive()
    ):
        _purga_finish(True)
    _cycle_stop_idle()
    if _prefeeder_required():
        _prefeeder_stop_now()
    _push_event("Stop — ciclo abortado.", "warn")
    if running and th is not None:
        th.join(timeout=2.0)
    with _CYCLE_SPAWN_LOCK:
        if not _cycle_thread_alive():
            _CYCLE_CTRL["stop"].clear()
            if _CYCLE_CTRL.get("thread") is th:
                _CYCLE_CTRL["thread"] = None


def _error_physical_clear_now() -> bool:
    """True si no hay fallas PLC ni seguridad Motion activas (lectura puntual)."""
    plc_port = bool(_serial_port("plc"))
    motion_port = bool(_serial_port("motion"))
    if plc_port:
        st = _plc_status_get()
        if st is None:
            return False
        if error_codes.plc_fault_codes(st):
            return False
    if motion_port:
        mst = _motion_status_get()
        if mst is None:
            return False
        if mst.get("safety") is True:
            return False
    if not _prefeeder_fault_clear_now():
        return False
    return True


def _wait_error_physical_clear_stable() -> bool:
    """Condiciones despejadas durante _ERROR_RESET_STABLE_MS seguidos."""
    need_s = max(0.05, _ERROR_RESET_STABLE_MS / 1000.0)
    clear_since = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if not _error_physical_clear_now():
            clear_since = None
        else:
            now = time.time()
            if clear_since is None:
                clear_since = now
            elif now - clear_since >= need_s:
                return True
        time.sleep(0.05)
    return False


def _cycle_reset_error() -> tuple[bool, str]:
    global _pf_last_fault_sig, _PF_ERROR_LATCH
    with _STATE_LOCK:
        code = str(_MACHINE_STATE.get("ciclo_error") or "").strip()
        in_err = _MACHINE_STATE.get("estado") == "error" or bool(code)
        msg_now = str(_MACHINE_STATE.get("mensaje") or "")
    if not in_err:
        return True, ""
    def _keep_lot() -> bool:
        return bool(
            _MACHINE_STATE.get("ciclo_en_curso")
            or _LOT.get("active")
            or _LOT.get("tray_wait")
        )

    def _finish_reset_to_purge(keep: bool) -> tuple[bool, str]:
        # Ruta especial: tras Reset la purga es obligatoria antes de Reanudar/Start.
        # El reset PLC apaga válvulas lógicas: libera holder/pinzas (suelta la pieza).
        _gripper_holding_set(False)
        with _STATE_LOCK:
            _MACHINE_STATE["post_error_purga"] = True
            rec = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
            if rec.get("active"):
                rec["step"] = "piezas"
                rec["code"] = str(rec.get("code") or code or "").strip()
                if not rec.get("mensaje"):
                    rec["mensaje"] = msg_now
                _MACHINE_STATE["recovery"] = rec
            else:
                prev = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
                _MACHINE_STATE["recovery"] = {
                    "active": True,
                    "step": "piezas",
                    "code": str(code or "").strip(),
                    "mensaje": msg_now or str(prev.get("mensaje") or ""),
                }
        _set_machine_ui(
            estado="paused" if keep else "idle",
            estado_label="Recuperación",
            ciclo_error="",
            mensaje="Error limpiado. Confirma las piezas y completa la recuperación.",
            en_curso=keep,
        )
        _push_event("Reset — holder y pieza liberados; sigue la recuperación.")
        return True, ""

    # Error de COM/enlace: no pedir pulso/status al módulo caído.
    if (code in _RECOVERY_LINK_SOFT_RESET) or (not code and _MACHINE_STATE.get("estado") == "error"):
        _pf_last_fault_sig = ()
        _PF_ERROR_LATCH = False
        if code in _RECOVERY_LINK_SOFT_RESET:
            _push_event("Reset — error de enlace limpiado; sigue la recuperación.")
        return _finish_reset_to_purge(_keep_lot())

    # Pulso GPIO Reset PLC (borrar fallos) antes de validar condiciones físicas.
    if _serial_port("plc"):
        sent = _cycle_plc_send({"op": "reset"})
        if not sent.get("sent"):
            fail_code = str(sent.get("code") or "E019")
            # Si el pulso falla por COM, no dejar al operador atrapado.
            if fail_code in _RECOVERY_LINK_SOFT_RESET:
                _pf_last_fault_sig = ()
                _PF_ERROR_LATCH = False
                _push_event("Reset — sin respuesta PLC; se continúa la recuperación.")
                return _finish_reset_to_purge(_keep_lot())
            return False, error_codes.format_message(fail_code)
        _plc_wait_ready(1.5)

    if not _wait_error_physical_clear_stable():
        return False, _error_reset_blocked_message()
    if _prefeeder_required():
        _prefeeder_stop_now()
        # Holgura omitida: no exigir estabilización física de ese sensor.
        soft_pf = code in _PF_SOFT_ERRORS or _prefeeder_holgura_omitted_for_code(code)
        if not soft_pf:
            ok_pf, msg_pf = _prefeeder_stabilize_and_lock()
            if not ok_pf:
                return False, msg_pf
            _prefeeder_refresh_cache()
        elif _prefeeder_holgura_omitted_for_code(code):
            _prefeeder_set_feed_lock(True)
            _prefeeder_refresh_cache()
    st = _PF_CACHED_STATUS
    _pf_last_fault_sig = _prefeeder_fault_sig(st) if st else ()
    _PF_ERROR_LATCH = False
    # El lote solo lo borra Stop: tras un error se queda para Reanudar (después de purga).
    en_curso = bool(
        _MACHINE_STATE.get("ciclo_en_curso") or _LOT.get("active") or _LOT.get("tray_wait")
    )
    return _finish_reset_to_purge(en_curso)


# ── Vías especiales (rutinas de operador) ───────────────────────────────────
# Purga, posición inicial, revisión, carga y avance de un lado.
# No agregan pasos a CYCLE_STEPS ni alteran la secuencia del ciclo.


def _checklist_now() -> dict:
    with _STATE_LOCK:
        return dict(_MACHINE_STATE.get("checklist") or {})


def _checklist_update(**fields) -> None:
    with _STATE_LOCK:
        block = dict(_MACHINE_STATE.get("checklist") or {})
        block.update(fields)
        _MACHINE_STATE["checklist"] = block


def _checklist_blank() -> dict:
    return {
        "ok": False,
        "paso": 1,
        "empleado": "",
        "aire_ok": False,
        "eje_ok": False,
        "recorrido_ok": False,
        "alimentador_ok": False,
        "purga_ok": False,
        "piezas_cortadas": False,
    }


def _set_routine_ui(*, rutina=None, rutina_fase=None, rutina_manos=None, mensaje=None, estado_label=None) -> None:
    with _STATE_LOCK:
        if rutina is not None:
            _MACHINE_STATE["rutina"] = rutina
        if rutina_fase is not None:
            _MACHINE_STATE["rutina_fase"] = rutina_fase
        if rutina_manos is not None:
            _MACHINE_STATE["rutina_manos"] = bool(rutina_manos)
    if mensaje is not None or estado_label is not None:
        _set_machine_ui(mensaje=mensaje, estado_label=estado_label)


def _empleado(value) -> str:
    text = str(value or "").strip()
    if len(text) == 5 and text.isdigit():
        return text
    return ""


def _target_length_mm() -> float:
    try:
        return float(_MOTION_DATA.get("target_length_mm") or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _mant_write_file(n: int) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    MANT_PATH.write_text(json.dumps({"ciclos": int(n)}) + "\n", encoding="utf-8")


def _mant_load() -> None:
    n = 0
    if MANT_PATH.exists():
        try:
            data = json.loads(MANT_PATH.read_text(encoding="utf-8"))
            n = max(0, int(data.get("ciclos") or 0))
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            n = 0
    with _STATE_LOCK:
        _MACHINE_STATE["ciclos_mant"] = n


def _mant_add_cycle() -> None:
    with _STATE_LOCK:
        n = int(_MACHINE_STATE.get("ciclos_mant") or 0) + 1
        _MACHINE_STATE["ciclos_mant"] = n
    try:
        _mant_write_file(n)
    except OSError:
        _push_event("No se pudo guardar el contador de mantenimiento.", "warn")


def _mant_reset(data) -> tuple[bool, str]:
    empleado = _empleado(data.get("empleado"))
    if not empleado:
        return False, "Hace falta el número de empleado, cinco dígitos."
    with _STATE_LOCK:
        _MACHINE_STATE["ciclos_mant"] = 0
    try:
        _mant_write_file(0)
    except OSError:
        return False, "No se pudo guardar el contador."
    _push_event(f"Contador de mantenimiento en cero. Empleado {empleado}.")
    return True, "Contador en cero."


def _checklist_log(empleado: str) -> None:
    entry = {
        "empleado": empleado,
        "fecha": time.strftime("%Y-%m-%d %H:%M:%S"),
        "resultado": "ok",
        "ciclos_mant": int(_MACHINE_STATE.get("ciclos_mant") or 0),
    }
    rows = []
    if CHECKLIST_LOG_PATH.exists():
        try:
            saved = json.loads(CHECKLIST_LOG_PATH.read_text(encoding="utf-8"))
            if isinstance(saved, list):
                rows = saved
        except (OSError, json.JSONDecodeError):
            rows = []
    rows.append(entry)
    del rows[:-50]
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    CHECKLIST_LOG_PATH.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")


def _rearm_piece(mensaje: str) -> None:
    """Vuelve la pieza al paso 1. No toca cycles_done ni el lote activo."""
    global _PIECE_FEED, _PIECE_BLOWER_MM, _PIECE_TICK0
    _overlap_reset()
    _cycle_latch_reset()
    _PIECE_FEED = {"R": None, "L": None}
    _PIECE_BLOWER_MM = None
    _PIECE_TICK0 = None
    _set_machine_ui(
        estado="paused",
        estado_label="Pausa",
        mensaje=mensaje,
        en_curso=True,
        ciclo_error="",
        step_id="1",
        step_index=0,
        progreso_pct=_lot_progress_pct(0) if int(_LOT.get("cycles_total") or 0) > 0 else 0,
        tiempo_corriendo_s=0,
        paralelo_id="",
        feed={"R": None, "L": None},
    )


def _routine_sleep_ms(ms: int) -> bool:
    deadline = time.time() + (max(0, int(ms)) / 1000.0)
    while time.time() < deadline:
        if _ROUTINE["stop"].is_set():
            return False
        time.sleep(0.02)
    return not _ROUTINE["stop"].is_set()


def _run_routine(fn) -> None:
    try:
        fn()
    except Exception as exc:
        _ROUTINE["from_lote"] = False
        _push_event(f"La rutina se detuvo: {exc}", "error")
        _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False, mensaje="La rutina se detuvo.")
    finally:
        _ROUTINE["thread"] = None


def _start_routine(fn) -> tuple[bool, str]:
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    if not _cycle_thread_alive():
        _CYCLE_CTRL["stop"].clear()
    _ROUTINE["stop"].clear()
    th = threading.Thread(target=_run_routine, args=(fn,), daemon=True)
    _ROUTINE["thread"] = th
    th.start()
    return True, ""


def _motion_cmd_ok(sent: dict, op: str) -> bool:
    if not sent.get("sent"):
        return False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return bool(msg.get("ok", True))
    return False


def _require_links() -> tuple[bool, str]:
    ready, err = _links_ready_for_cycle()
    if not ready:
        return False, err
    return True, ""


def _routine_try_reset_fault() -> bool:
    if not _serial_port("plc"):
        return True
    faults = _plc_active_errors(_plc_status_get())
    if not faults and not _MACHINE_STATE.get("ciclo_error"):
        return True
    _cycle_plc_send({"op": "reset"})
    faults = _plc_active_errors(_plc_status_get())
    if faults:
        _push_event(faults[0]["message"], "error")
        _set_machine_ui(mensaje="La falla sigue presente.")
        return False
    if _MACHINE_STATE.get("ciclo_error"):
        _set_machine_ui(ciclo_error="")
    return True


def _feed_stage_payload(side: str | None = None) -> dict:
    """feed_stage con el botón de validación tal como está guardado."""
    tune = _load_motion_tune()
    payload = {
        "op": "feed_stage",
        "feed_validate_encoder": bool(tune["feed_stage"]["validate_encoder"]),
    }
    if side in ("R", "L"):
        payload["side"] = side
    return payload


def _push_saved_motion_tune() -> dict:
    """Publica Motion.json. El MCU no conserva el botón entre reinicios."""
    return _cycle_motion_send(_motion_tune_flat(_load_motion_tune()), timeout_s=2.0)


def _feed_once(side: str | None):
    tune_sent = _push_saved_motion_tune()
    if not tune_sent.get("sent") or not _motion_cmd_ok(tune_sent, "set_tune"):
        return False, tune_sent.get("code") or "E043", {"R": None, "L": None}
    payload = _feed_stage_payload(side)
    sent = _cycle_motion_send(payload, timeout_s=65.0)
    found, ok, error_name, feed = _read_feed_stage(sent)
    _set_machine_ui(feed=feed)
    if not sent.get("sent"):
        return False, sent.get("code") or "E043", feed
    if not found or not ok:
        err = error_name or "unknown"
        return False, error_codes.feed_stage_code(err), feed
    return True, "", feed


def _feeder_test(side: str, tune: dict) -> dict:
    """Cierra holder/encoder y ejecuta FeedStage de un solo lado."""
    busy = _motion_owner_busy()
    if busy:
        return {"ok": False, "error": busy}

    for module in ("motion", "plc"):
        linked, code = _check_module_link(module)
        if not linked:
            return {"ok": False, "error": error_codes.format_message(code)}

    plc_sent = _cycle_plc_send(
        {"op": "valve", "name": "holder_encoder", "on": True}
    )
    if not plc_sent.get("sent"):
        return {
            "ok": False,
            "error": plc_sent.get("reason")
            or error_codes.format_message(plc_sent.get("code") or "E019"),
        }

    params = _load_cycle_params()
    time.sleep(max(0, int(params["ms_holder_enc_stable"])) / 1000.0)
    holder_ok, code = _plc_confirm({"holder": True, "encoder": True})
    if not holder_ok:
        return {
            "ok": False,
            "error": error_codes.format_message(code) if code else "Detenido.",
        }

    tune_payload = _motion_tune_flat(tune)
    tune_sent = _cycle_motion_send(tune_payload, timeout_s=2.0)
    if not _motion_cmd_ok(tune_sent, "set_tune"):
        return {
            "ok": False,
            "error": tune_sent.get("reason") or "Motion no aceptó los parámetros.",
        }

    feed_payload = _feed_stage_payload(side)
    feed_sent = _cycle_motion_send(feed_payload, timeout_s=65.0)
    found, feed_ok, error_name, feed = _read_feed_stage(feed_sent)
    _set_machine_ui(feed=feed)
    replies = (tune_sent.get("replies") or []) + (feed_sent.get("replies") or [])
    if not feed_sent.get("sent"):
        return {
            "ok": False,
            "error": feed_sent.get("reason") or error_codes.format_message("E043"),
            "lines": [_compact(tune_payload), _compact(feed_payload)],
            "replies": replies,
        }
    return {
        "ok": True,
        "sent": True,
        "command_ok": bool(found and feed_ok),
        "error": "" if found and feed_ok else error_codes.feed_stage_message(error_name or "unknown"),
        "lines": [_compact(tune_payload), _compact(feed_payload)],
        "replies": replies,
        "feed": feed,
    }


def _purge_feed_side() -> str | None:
    """Lado de purga según feed_sides. None = ambos (LR)."""
    sides = _run_sides()
    return sides if sides in ("R", "L") else None


def _feed_laser():
    """Purga: lados activos (feed_sides) hasta su láser. Sin encoder ni compensación."""
    payload = {"op": "feed_stage", "mode": "laser"}
    side = _purge_feed_side()
    if side:
        payload["side"] = side
    sent = _cycle_motion_send(payload, timeout_s=20.0)
    found, ok, error_name, feed = _read_feed_stage(sent)
    _set_machine_ui(feed=feed)
    if not sent.get("sent"):
        return False, sent.get("code") or "E043", feed
    if not found or not ok:
        err = error_name or "unknown"
        return False, error_codes.feed_stage_code(err), feed
    return True, "", feed


def _feed_clean():
    payload = {"op": "feed_stage", "mode": "clean", "timeout_ms": PURGE_CLEAN_MS}
    side = _purge_feed_side()
    if side:
        payload["side"] = side
    sent = _cycle_motion_send(payload, timeout_s=8.0)
    found, ok, error_name, feed = _read_feed_stage(sent)
    _set_machine_ui(feed=feed)
    if not sent.get("sent"):
        return False, sent.get("code") or "E043", feed
    if not found or not ok:
        err = error_name or "unknown"
        return False, error_codes.feed_stage_code(err), feed
    return True, "", feed


def _routine_move_lm_mm(mm: float) -> bool:
    tune = _load_motion_tune()
    if _ROUTINE["stop"].is_set():
        return False
    move = _cycle_motion_send(
        {
            "op": "lmguide_move",
            "puu": _mm_to_puu(mm),
            "lm_pr_speed_rpm": tune["lmguide"]["pr_speed_rpm"],
        },
        timeout_s=40.0,
    )
    return _motion_cmd_ok(move, "lmguide_move")


def _purga_finish(cancelled: bool) -> None:
    with _STATE_LOCK:
        if _MACHINE_STATE.get("rutina") != "purga":
            return
    _cycle_plc_send({"op": "all_off"})
    with _STATE_LOCK:
        paso = (_MACHINE_STATE.get("checklist") or {}).get("paso")
        _MACHINE_STATE["rutina"] = ""
        _MACHINE_STATE["rutina_fase"] = ""
        _MACHINE_STATE["rutina_manos"] = False
        from_lote = bool(_ROUTINE.get("from_lote"))
        _ROUTINE["from_lote"] = False
        recovery = bool(_MACHINE_STATE.get("post_error_purga"))
        keep = bool(
            _MACHINE_STATE.get("ciclo_en_curso")
            or _LOT.get("active")
            or _LOT.get("tray_wait")
        )
    if cancelled:
        _push_event("Purga cancelada.", "warn")
        msg = "Purga cancelada."
        if recovery:
            _set_machine_ui(
                estado="paused" if keep else "idle",
                estado_label="Purga pendiente",
                mensaje="Purga cancelada. Completa la purga para continuar.",
                en_curso=keep,
            )
            return
    else:
        if paso == 5:
            _checklist_update(purga_ok=True)
        _push_event("Purga terminada.")
        msg = "Purga terminada."
        if recovery:
            with _STATE_LOCK:
                _MACHINE_STATE["post_error_purga"] = False
                rec = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
                if rec.get("active"):
                    rec["step"] = "listo"
                    _MACHINE_STATE["recovery"] = rec
            if keep:
                if _LOT.get("tray_wait"):
                    deposits = int(_LOT.get("deposits_per_tray") or 0)
                    _set_machine_ui(
                        estado="paused",
                        estado_label="Charola llena",
                        mensaje=(
                            f"Purga terminada. Charola llena — {deposits} ciclos. "
                            "Vacía la charola y confirma."
                        ),
                        en_curso=True,
                        ciclo_error="",
                    )
                else:
                    # Pieza incompleta: se reintenta desde el 1. cycles_done intacto.
                    if _LOT.get("active"):
                        piece = int(_LOT.get("cycles_done") or 0) + 1
                        total = int(_LOT.get("cycles_total") or 0)
                        rearm_msg = (
                            f"Listo. Pieza {piece} de {total} se reintenta desde el inicio. "
                            "Puedes Reanudar."
                        )
                    else:
                        rearm_msg = (
                            "Listo. La pieza se reintenta desde el inicio. Puedes Reanudar."
                        )
                    _rearm_piece(rearm_msg)
            else:
                _set_machine_ui(
                    estado="idle",
                    estado_label="En espera",
                    mensaje="Listo para operar.",
                    en_curso=False,
                )
            return
    if from_lote:
        _rearm_piece(msg + " El ciclo queda en pausa y la pieza vuelve al paso 1.")
        return
    _set_machine_ui(estado="idle", estado_label="En espera", mensaje=msg, en_curso=False)


def _purga_prepare() -> None:
    _set_routine_ui(
        rutina="purga",
        rutina_fase="segura",
        rutina_manos=False,
        estado_label="Purga",
        mensaje=f"Purga: llevando el eje a parque ({PURGE_PARK_MM:.0f} mm).",
    )
    if not _routine_try_reset_fault() or _ROUTINE["stop"].is_set():
        _purga_finish(True)
        return
    if not _cycle_plc_send({"op": "all_off"}).get("sent"):
        _purga_finish(True)
        return
    _gripper_holding_set(False)
    if _ROUTINE["stop"].is_set() or not _routine_move_lm_mm(PURGE_PARK_MM):
        _purga_finish(True)
        return
    _set_routine_ui(
        rutina_fase="espera",
        mensaje=(
            f"Eje en parque ({PURGE_PARK_MM:.0f} mm). "
            "Alimentar y Limpiar cierran holder y encoder. "
            "Continuar: home, corte, depósito a 100 mm y home. "
            "Terminar cierra la purga."
        ),
    )


def _purga_start(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "Ya hay una purga en curso."
    if _MACHINE_STATE.get("estado") == "error" or _MACHINE_STATE.get("ciclo_error"):
        return False, "Limpia el error con Reset antes de la purga."
    if _MACHINE_STATE.get("estado") == "running":
        return False, "Pausa o detén el ciclo antes de la purga."
    ready, err = _require_links()
    if not ready:
        return False, err
    # En ruta post-error se conserva el paso pausado; no rearmar a paso 1 al terminar.
    recovery = bool(_MACHINE_STATE.get("post_error_purga"))
    _ROUTINE["from_lote"] = bool(_MACHINE_STATE.get("ciclo_en_curso")) and not recovery
    _set_routine_ui(rutina="purga", rutina_fase="segura", rutina_manos=False, estado_label="Purga")
    ok, err = _start_routine(_purga_prepare)
    if not ok:
        _ROUTINE["from_lote"] = False
        _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False)
        return False, err
    return True, "Purga iniciada."


def _purga_close_holder() -> bool:
    params = _load_cycle_params()
    sent = _cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": True})
    if not sent.get("sent") or _ROUTINE["stop"].is_set():
        return False
    if not _routine_sleep_ms(int(params["ms_holder_enc_stable"])):
        return False
    ok_hw, code = _plc_confirm({"holder": True, "encoder": True})
    if not ok_hw:
        if code:
            _push_event(error_codes.format_message(code), "error")
        return False
    return True


def _purga_alimentar_run() -> None:
    _set_routine_ui(rutina_fase="alimentar", mensaje="Cerrando holder y encoder.")
    if not _purga_close_holder():
        if _ROUTINE["stop"].is_set():
            _purga_finish(True)
        else:
            _set_routine_ui(rutina_fase="espera", mensaje="No se cerró holder y encoder.")
        return
    side = _purge_feed_side()
    side_name = {"R": "derecho", "L": "izquierdo"}.get(side or "", "ambos lados")
    _set_routine_ui(mensaje=f"Alimentación: {side_name} hasta el láser.")
    ok, code, _feed = _feed_laser()
    if _ROUTINE["stop"].is_set():
        _purga_finish(True)
        return
    if not ok:
        _push_event(error_codes.format_message(code), "error")
        _set_routine_ui(
            rutina_fase="espera",
            mensaje="No llegó al láser. Puedes repetir, continuar o terminar.",
        )
        return
    _set_routine_ui(
        rutina_fase="espera",
        mensaje="Alimentación lista. Continuar: home, corte, depósito a 100 mm y home.",
    )


def _purge_cut() -> bool:
    """Solo purga, al pulsar Continuar: home, corte con pinzas, depósito fijo, home."""
    params = _load_cycle_params()
    _set_routine_ui(
        rutina_fase="corte",
        rutina_manos=True,
        estado_label="Purga",
        mensaje="NO METAS LAS MANOS. Regresando a 0.",
    )

    def sent_ok(sent) -> bool:
        op = str((sent.get("last_payload") or {}).get("op") or "")
        return bool(op and _motion_cmd_ok(sent, op)) and not _ROUTINE["stop"].is_set()

    if not _routine_move_lm_mm(0.0) or _ROUTINE["stop"].is_set():
        return False

    _set_routine_ui(mensaje="NO METAS LAS MANOS. Corte y pinzas en 0.")
    if not sent_ok(_cycle_plc_send({"op": "valve", "name": "gripper", "on": True})):
        return False
    if not sent_ok(_cycle_plc_send({"op": "valve", "name": "holder", "on": False})):
        return False
    if not sent_ok(_cycle_plc_send({"op": "valve", "name": "cutters", "on": True})):
        return False
    if not _routine_sleep_ms(int(params["ms_cutter_on"])):
        return False
    if not sent_ok(_cycle_plc_send({"op": "valve", "name": "cutters", "on": False})):
        return False
    if not _routine_sleep_ms(PURGE_AFTER_CUT_MS):
        return False

    _set_routine_ui(
        mensaje=f"NO METAS LAS MANOS. Depósito a {PURGE_DEPOSIT_MM:.0f} mm.",
    )
    if not _routine_move_lm_mm(PURGE_DEPOSIT_MM):
        return False
    if not _routine_sleep_ms(int(params["ms_lm_at_discharge"])):
        return False
    if not sent_ok(_cycle_plc_send({"op": "valve", "name": "gripper", "on": False})):
        return False
    if not _routine_sleep_ms(int(params["ms_after_grip_release"])):
        return False

    _set_routine_ui(mensaje="NO METAS LAS MANOS. Regresando a home.")
    home_result: dict = {}

    def send_home() -> None:
        home_result["sent"] = _cycle_motion_send(
            {"op": "lmguide_move", "target": "home"}, timeout_s=40.0
        )

    home_thread = threading.Thread(target=send_home, daemon=True)
    home_thread.start()
    if not _routine_sleep_ms(int(params["ms_blower_delay"])):
        _cycle_plc_send({"op": "blower_off"})
        return False
    blower_started = False
    if home_thread.is_alive():
        blower_ms = _cycle_blower_duration_ms(params)
        if not sent_ok(_cycle_plc_send({"op": "blower_on", "ms": blower_ms})):
            return False
        blower_started = True
    while home_thread.is_alive():
        if _ROUTINE["stop"].is_set():
            _cycle_plc_send({"op": "blower_off"})
            return False
        home_thread.join(0.05)
    home = home_result.get("sent") or {}
    if not _motion_cmd_ok(home, "lmguide_move") or _ROUTINE["stop"].is_set():
        _cycle_plc_send({"op": "blower_off"})
        return False
    # Límite: en home se corta el soplo.
    if blower_started:
        _cycle_plc_send({"op": "blower_off"})
    if not _routine_sleep_ms(int(params["ms_at_home_end"])):
        return False
    _cycle_plc_send({"op": "all_off"})
    return not _ROUTINE["stop"].is_set()


def _purga_cut_run() -> None:
    ok = False
    try:
        ok = _purge_cut()
    finally:
        if not ok or _ROUTINE["stop"].is_set():
            _cycle_plc_send({"op": "all_off"})
            _purga_finish(True)
        else:
            _purga_finish(False)


def _purga_limpiar_run() -> None:
    _set_routine_ui(rutina_fase="limpiar", mensaje="Cerrando holder y encoder.")
    if not _purga_close_holder():
        if _ROUTINE["stop"].is_set():
            _purga_finish(True)
        else:
            _set_routine_ui(rutina_fase="espera", mensaje="No se cerró holder y encoder.")
        return
    side = _purge_feed_side()
    side_name = {"R": "derecho", "L": "izquierdo"}.get(side or "", "ambos lados")
    _set_routine_ui(mensaje=f"Limpiar ({side_name}): alimentando 3 s sin mirar el láser.")
    ok, code, _feed = _feed_clean()
    if _ROUTINE["stop"].is_set():
        _purga_finish(True)
        return
    if not ok:
        _push_event(error_codes.format_message(code), "error")
        _set_routine_ui(
            rutina_fase="espera",
            mensaje="La limpieza no completó los 3 s. Puedes repetir, continuar o terminar.",
        )
        return
    _set_routine_ui(
        rutina_fase="espera",
        mensaje="Limpieza lista. Continuar: home, corte, depósito a 100 mm y home.",
    )


def _purga_limpiar_action(_data=None) -> tuple[bool, str]:
    if not _purga_claim("limpiar"):
        return False, "La purga no está en espera."
    ok, err = _start_routine(_purga_limpiar_run)
    if not ok:
        _set_routine_ui(rutina_fase="espera", rutina_manos=False)
        return False, err
    return True, "Limpiando."


def _purga_continuar_action(_data=None) -> tuple[bool, str]:
    if not _purga_claim("corte"):
        return False, "La purga no está en espera."
    ok, err = _start_routine(_purga_cut_run)
    if not ok:
        _set_routine_ui(rutina_fase="espera", rutina_manos=False)
        return False, err
    return True, "Purga en corte."


def _purga_terminar_action(_data=None) -> tuple[bool, str]:
    if not _purga_claim("terminar"):
        return False, "La purga no está en espera."
    _purga_finish(False)
    return True, "Purga terminada."


def _posicion_inicial_run() -> None:
    in_purge = _MACHINE_STATE.get("rutina") == "purga"
    if not in_purge:
        _set_routine_ui(rutina="posicion_inicial", rutina_fase="", rutina_manos=False, mensaje="Posición inicial.")
    if not _routine_try_reset_fault() or _ROUTINE["stop"].is_set():
        if in_purge and _ROUTINE["stop"].is_set():
            _purga_finish(True)
        elif not in_purge:
            _set_routine_ui(rutina="", mensaje="La falla sigue presente.")
        return
    home = _cycle_motion_send({"op": "lmguide_move", "target": "home"}, timeout_s=40.0)
    if _ROUTINE["stop"].is_set() or not _motion_cmd_ok(home, "lmguide_move"):
        if in_purge:
            _purga_finish(True)
        else:
            _set_routine_ui(rutina="", mensaje="No se llegó a cero.")
        return
    enc = _cycle_motion_send({"op": "encoder_set0", "side": "both"}, timeout_s=2.0)
    _cycle_plc_send({"op": "all_off"})
    _gripper_holding_set(False)
    if not _motion_cmd_ok(enc, "encoder_set0"):
        _push_event("El eje llegó a cero. Los encoders no confirmaron el cero.", "warn")
    if in_purge:
        if _ROUTINE["stop"].is_set():
            _purga_finish(True)
        else:
            _set_routine_ui(
                rutina_fase="espera",
                mensaje="Purga en espera. Eje en cero, encoders en cero y aire apagado.",
            )
        return
    if _MACHINE_STATE.get("ciclo_en_curso"):
        _rearm_piece("Posición inicial. El ciclo queda en pausa y la pieza vuelve al paso 1.")
    else:
        _set_machine_ui(
            estado="idle",
            estado_label="En espera",
            mensaje="Posición inicial lista. Eje en cero, encoders en cero y aire apagado.",
            en_curso=False,
        )
    _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False)


def _posicion_inicial(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina_manos"):
        return False, "Espera a que termine el tramo de purga."
    rutina = _MACHINE_STATE.get("rutina") or ""
    if rutina == "purga" and _MACHINE_STATE.get("rutina_fase") != "espera":
        return False, "La purga todavía se está moviendo."
    if rutina not in ("", "purga"):
        return False, "Hay otra rutina en curso."
    if _MACHINE_STATE.get("estado") == "running":
        return False, "Detén el ciclo antes de la posición inicial."
    ready, err = _require_links()
    if not ready:
        return False, err
    ok, err = _start_routine(_posicion_inicial_run)
    if not ok:
        return False, err
    return True, "Posición inicial en curso."


_SIDE_NAME = {"L": "izquierda", "R": "derecha"}


def _alimentar_run(side: str) -> None:
    _set_routine_ui(rutina="alimentar", rutina_fase=side, mensaje=f"Alimentando lado {_SIDE_NAME[side]}.")
    ok, code, feed = _feed_once(side)
    block = feed.get(side) or {}
    if ok:
        mm = block.get("mm")
        text = f"Lado {_SIDE_NAME[side]} alimentado"
        if mm is not None:
            text += f" ({mm} mm)"
        _set_machine_ui(mensaje=text + ".")
        _push_event(text + ".")
    else:
        msg = error_codes.format_message(code, detail=side)
        _push_event(msg, "error")
        _set_machine_ui(mensaje=msg)
    _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False)


def _alimentar_izquierda(data=None):
    return False, "Mantén pulsado JOG - L en el panel."


def _alimentar_derecha(data=None):
    return False, "Mantén pulsado JOG - R en el panel."


def _feeder_init_now() -> tuple[bool, str]:
    ok_link, code = _check_module_link("motion")
    if not ok_link:
        return False, error_codes.format_message(code)
    tune = _load_motion_tune()
    lines = _build_motion_lines("motion_hw_init", tune, {})[1]
    sent = _send_motion_serial(
        lines,
        line_timeout_s=120.0,
        reply_until_op="motion_hw_init",
    )
    sent["last_payload"] = {"op": "motion_hw_init"}
    if not _motion_cmd_ok(sent, "motion_hw_init"):
        return False, error_codes.format_message(sent.get("code") or "E043")
    _push_event("Hardware Motion inicializado (CAN + LM home).")
    return True, "Hardware Motion inicializado (CAN + LM home)."


def _prealimentador_iniciar(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina"):
        return False, "Hay una rutina en curso."
    if not _prefeeder_required():
        return True, "PreAlimentador sin COM (omite estabilización)."
    ok, msg = _prefeeder_stabilize_and_lock()
    _prefeeder_refresh_cache()
    if not ok:
        st = _PF_CACHED_STATUS
        sig = _prefeeder_fault_sig(st) if st else ()
        if sig:
            _prefeeder_raise_fault(sig)
        else:
            _enter_machine_error("E099", msg, en_curso=False)
        return False, msg
    global _pf_last_fault_sig, _PF_ERROR_LATCH
    _pf_last_fault_sig = ()
    _PF_ERROR_LATCH = False
    with _STATE_LOCK:
        if _MACHINE_STATE.get("estado") == "error" and not _MACHINE_STATE.get("ciclo_en_curso"):
            _set_machine_ui(
                estado="idle",
                estado_label="En espera",
                ciclo_error="",
                mensaje="PreAlimentador estabilizado y bloqueado.",
            )
    _push_event("PreAlimentador estabilizado y bloqueado.")
    return True, "PreAlimentador estabilizado y bloqueado."


def _omitir_holgura(side: str, data) -> tuple[bool, str]:
    global _pf_last_fault_sig, _PF_ERROR_LATCH
    empleado = _empleado(data.get("empleado"))
    if not empleado:
        return False, "Hace falta el número de empleado, cinco dígitos."
    key = "omitir_holgura_l" if side == "L" else "omitir_holgura_r"
    with _STATE_LOCK:
        nxt = not bool(_MACHINE_STATE.get(key))
        _MACHINE_STATE[key] = nxt
    nombre = _SIDE_NAME[side]
    if _prefeeder_required():
        _prefeeder_set_omit(side, nxt)
        st = _prefeeder_refresh_cache()
        # Tras omitir, la E095/E098 de ese lado deja de bloquear Reset.
        _pf_last_fault_sig = _prefeeder_fault_sig(st)
        if not _pf_last_fault_sig:
            _PF_ERROR_LATCH = False
    if nxt:
        _push_event(
            f"Holgura {nombre} omitida. Empleado {empleado}. "
            f"Pulsa Reset para limpiar el error."
        )
        return True, f"Holgura {nombre} omitida. Pulsa Reset."
    _push_event(f"Holgura {nombre} restaurada. Empleado {empleado}.")
    return True, f"Holgura {nombre} restaurada."


def _omitir_holgura_izquierda(data):
    return _omitir_holgura("L", data)


def _omitir_holgura_derecha(data):
    return _omitir_holgura("R", data)


def _checklist_motion_free() -> str:
    if _MACHINE_STATE.get("ciclo_en_curso"):
        return "Termina el ciclo antes de este paso de la revisión."
    if _MACHINE_STATE.get("rutina"):
        return "Hay una rutina en curso."
    return _motion_owner_busy()


def _checklist_empezar(data) -> tuple[bool, str]:
    if _checklist_now().get("ok"):
        return False, "La revisión ya está completa. Usa Nueva revisión para repetirla."
    empleado = _empleado(data.get("empleado"))
    if not empleado:
        return False, "Hace falta el número de empleado, cinco dígitos."
    if int(_MACHINE_STATE.get("ciclos_mant") or 0) >= CHECKLIST_MAX_CYCLES:
        return False, "El contador llegó a 10000 ciclos. Resetealo en Mantenimiento."
    _checklist_update(empleado=empleado, paso=2)
    _push_event(f"Revisión de arranque. Empleado {empleado}.")
    return True, "Revisión iniciada."


def _checklist_nueva(_data=None) -> tuple[bool, str]:
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    _checklist_update(**_checklist_blank())
    _set_machine_ui(mensaje="Revisión de arranque reiniciada.")
    return True, "Revisión reiniciada."


def _checklist_aire(_data=None) -> tuple[bool, str]:
    if _checklist_now().get("paso") != 2:
        return False, "La prueba de aire no corresponde a este paso."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    ok_link, code = _check_module_link("plc")
    if not ok_link:
        return False, error_codes.format_message(code)
    params = _load_cycle_params()
    sent = _cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": True})
    if not sent.get("sent"):
        return False, error_codes.format_message(sent.get("code") or "E019")
    time.sleep(int(params["ms_holder_enc_stable"]) / 1000.0)
    faults = _plc_active_errors(_plc_status_get())
    _cycle_plc_send({"op": "all_off"})
    if faults:
        return False, faults[0]["message"]
    _checklist_update(aire_ok=True, paso=3)
    _push_event("Prueba de aire correcta.")
    return True, "Aire correcto."


def _checklist_holgura(_data=None) -> tuple[bool, str]:
    if _checklist_now().get("paso") != 3:
        return False, "La confirmación de holgura no corresponde a este paso."
    _checklist_update(paso=4)
    _push_event("Holgura confirmada por el operador.")
    return True, "Holgura confirmada."


def _checklist_home_run() -> None:
    _set_routine_ui(rutina="checklist", mensaje="Buscando la referencia mecánica del eje.")
    sent = _cycle_motion_send({"op": "lmguide_home"}, timeout_s=40.0)
    if _ROUTINE["stop"].is_set() or not _motion_cmd_ok(sent, "lmguide_home"):
        _set_routine_ui(rutina="", mensaje="La referencia del eje no se completó.")
        return
    _checklist_update(eje_ok=True)
    _set_routine_ui(rutina="", mensaje="Eje en referencia. Haz el recorrido y confirma que regresó.")


def _checklist_home(_data=None) -> tuple[bool, str]:
    if _checklist_now().get("paso") != 4:
        return False, "La referencia del eje no corresponde a este paso."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    ok_link, code = _check_module_link("motion")
    if not ok_link:
        return False, error_codes.format_message(code)
    ok, err = _start_routine(_checklist_home_run)
    if not ok:
        return False, err
    return True, "Buscando referencia del eje."


def _checklist_recorrido_run() -> None:
    _set_routine_ui(rutina="checklist", mensaje="Recorrido del eje.")
    params = _load_cycle_params()
    pos = _lm_cut_and_discharge(params)
    if pos is not None:
        _cut_mm, disc = pos
        tune = _load_motion_tune()
        puu = _mm_to_puu(disc)
        move = _cycle_motion_send(
            {
                "op": "lmguide_move",
                "puu": puu,
                "lm_pr_speed_rpm": tune["lmguide"]["pr_speed_rpm"],
            },
            timeout_s=40.0,
        )
        if _ROUTINE["stop"].is_set() or not _motion_cmd_ok(move, "lmguide_move"):
            _set_routine_ui(rutina="", mensaje="El recorrido no se completó.")
            return
        if not _routine_sleep_ms(int(params["ms_lm_at_discharge"])):
            _set_routine_ui(rutina="", mensaje="Recorrido detenido.")
            return
    else:
        _push_event("Sin longitud suficiente para la descarga. El recorrido solo regresa a cero.", "warn")
    home = _cycle_motion_send({"op": "lmguide_move", "target": "home"}, timeout_s=40.0)
    if _ROUTINE["stop"].is_set() or not _motion_cmd_ok(home, "lmguide_move"):
        _set_routine_ui(rutina="", mensaje="El eje no regresó a cero.")
        return
    _checklist_update(recorrido_ok=True)
    _set_routine_ui(rutina="", mensaje="Recorrido listo. Confirma que el eje regresó.")


def _checklist_recorrido(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 4 or not ch.get("eje_ok"):
        return False, "Primero lleva el eje a la referencia."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    ok, err = _start_routine(_checklist_recorrido_run)
    if not ok:
        return False, err
    return True, "Recorrido en curso."


def _checklist_eje_listo(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 4:
        return False, "Este paso no está activo."
    if not ch.get("eje_ok") or not ch.get("recorrido_ok"):
        return False, "Falta la referencia o el recorrido."
    _checklist_update(paso=5)
    return True, "Eje confirmado."


def _checklist_preparar(_data=None) -> tuple[bool, str]:
    if _checklist_now().get("paso") != 5:
        return False, "Preparar el alimentador no corresponde a este paso."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    ok, err = _feeder_init_now()
    if not ok:
        return False, err
    if _prefeeder_required():
        ok_pf, err_pf = _prealimentador_iniciar()
        if not ok_pf:
            return False, err_pf
        _checklist_update(alimentador_ok=True)
        return True, "Motion y PreAlimentador listos."
    _checklist_update(alimentador_ok=True)
    return True, err


def _checklist_purga(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 5 or not ch.get("alimentador_ok"):
        return False, "Primero prepara el alimentador."
    return _purga_start()


def _checklist_purga_hecha(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 5:
        return False, "Este paso no está activo."
    if not ch.get("purga_ok"):
        return False, "Primero termina la purga."
    _checklist_update(paso=6)
    return True, "Purga de revisión lista."


def _checklist_piezas_run() -> None:
    for n in (1, 2):
        if _ROUTINE["stop"].is_set():
            _set_routine_ui(rutina="", mensaje="Piezas de prueba detenidas.")
            return
        _set_routine_ui(rutina="checklist", rutina_fase="piezas", mensaje=f"Pieza de prueba {n} de 2.")
        _cycle_latch_reset()
        _CYCLE_CTRL["stop"].clear()
        _CYCLE_CTRL["pause_after_step"].clear()
        _set_machine_ui(ciclo_error="", feed={"R": None, "L": None})
        _cycle_worker(0)
        if _MACHINE_STATE.get("ciclo_error") or _MACHINE_STATE.get("estado") != "idle":
            failed = bool(_MACHINE_STATE.get("ciclo_error"))
            _cycle_latch_reset()
            _set_machine_ui(en_curso=False, step_id="", step_index=0, progreso_pct=0)
            _set_routine_ui(
                rutina="",
                rutina_fase="",
                mensaje="La pieza de prueba no se completó." if failed else "Piezas de prueba detenidas.",
            )
            return
    _checklist_update(piezas_cortadas=True)
    _set_routine_ui(
        rutina="",
        rutina_fase="",
        mensaje="Piezas cortadas. Anota la longitud medida de cada lado.",
    )


def _checklist_piezas(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 6:
        return False, "Las piezas de prueba no corresponden a este paso."
    if ch.get("piezas_cortadas"):
        return False, "Las piezas ya se cortaron. Anota la medida."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    if not _MOTION_DATA.get("modelo_id") or _target_length_mm() <= 0:
        return False, "Selecciona un modelo con longitud."
    ready, err = _require_links()
    if not ready:
        return False, err
    ok, err = _start_routine(_checklist_piezas_run)
    if not ok:
        return False, err
    return True, "Cortando dos piezas de prueba."


def _checklist_medicion(data) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 6 or not ch.get("piezas_cortadas"):
        return False, "Primero corta las dos piezas."
    try:
        mm_l = float(data.get("mm_l"))
        mm_r = float(data.get("mm_r"))
    except (TypeError, ValueError):
        return False, "Anota las dos longitudes en milímetros."
    expected = _target_length_mm()
    tol = float(_load_cycle_params().get("tolerance_mm") or 2.0)
    if abs(mm_l - expected) > tol or abs(mm_r - expected) > tol:
        return False, f"Fuera de tolerancia. Se espera {expected:.1f} mm ±{tol:.0f}."
    try:
        _checklist_log(ch.get("empleado") or "")
    except OSError:
        _push_event("La revisión pasó, pero no se pudo guardar el registro.", "warn")
    _checklist_update(ok=True)
    _set_machine_ui(mensaje="Revisión de arranque completa. Ya se puede iniciar.")
    _push_event("Revisión de arranque completa.")
    return True, "Revisión completa."


def _purga_claim(fase: str) -> bool:
    with _STATE_LOCK:
        if _MACHINE_STATE.get("rutina") != "purga" or _MACHINE_STATE.get("rutina_fase") != "espera":
            return False
        _MACHINE_STATE["rutina_fase"] = fase
        return True


def _purga_alimentar_action(data=None) -> tuple[bool, str]:
    if not _purga_claim("alimentar"):
        return False, "La purga no está esperando alimentación."
    ok, err = _start_routine(_purga_alimentar_run)
    if not ok:
        _set_routine_ui(rutina_fase="espera")
        return False, err
    return True, "Alimentando."


def _recovery_confirm_piezas(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return False, "No hay una recuperación en curso."
    if rec.get("step") != "piezas":
        return False, "Primero limpia el error con Reset."
    if _MACHINE_STATE.get("estado") == "error" or _MACHINE_STATE.get("ciclo_error"):
        return False, "Todavía hay un error activo. Pulsa Reset."
    _recovery_set(step="manguera")
    _set_machine_ui(
        mensaje="Mueve la manguera hacia atrás hasta apagar ambos cortadores.",
        estado_label="Recuperación",
    )
    return True, "Confirma la manguera."


def _recovery_auto_purga_run() -> None:
    """Purga automática post-error: parque + corte (reusa _purge_cut / _purga_finish)."""
    _ROUTINE["from_lote"] = False
    _recovery_set(step="purga")
    _set_routine_ui(
        rutina="purga",
        rutina_fase="segura",
        rutina_manos=True,
        estado_label="Purga",
        mensaje="NO INTRODUCIR MANOS. Proceso de limpieza en curso, espera.",
    )
    _set_machine_ui(mensaje="NO INTRODUCIR MANOS. Proceso de limpieza en curso, espera.")
    # Tiempo para leer el aviso antes de mover.
    if not _routine_sleep_ms(500):
        _purga_finish(True)
        _recovery_set(step="manguera")
        return
    if not _routine_try_reset_fault() or _ROUTINE["stop"].is_set():
        _purga_finish(True)
        _recovery_set(step="manguera")
        return
    if not _cycle_plc_send({"op": "all_off"}).get("sent"):
        _purga_finish(True)
        _recovery_set(step="manguera")
        return
    _gripper_holding_set(False)
    _set_routine_ui(
        mensaje=f"NO METAS LAS MANOS. Llevando el eje a parque ({PURGE_PARK_MM:.0f} mm).",
    )
    if _ROUTINE["stop"].is_set() or not _routine_move_lm_mm(PURGE_PARK_MM):
        _purga_finish(True)
        _recovery_set(step="manguera")
        return
    ok = False
    try:
        ok = _purge_cut()
    finally:
        if not ok or _ROUTINE["stop"].is_set():
            _cycle_plc_send({"op": "all_off"})
            _purga_finish(True)
            _recovery_set(step="manguera")
        else:
            _purga_finish(False)


def _recovery_confirm_manguera(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return False, "No hay una recuperación en curso."
    if rec.get("step") != "manguera":
        return False, "Primero confirma que las piezas están bien."
    if _MACHINE_STATE.get("estado") == "error" or _MACHINE_STATE.get("ciclo_error"):
        return False, "Todavía hay un error activo."
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "Ya hay una purga en curso."
    if not _recovery_cutters_clear_now():
        return False, "Los cortadores aún no están apagados. Mueve la manguera hacia atrás."
    ready, err = _require_links()
    if not ready:
        return False, err
    # Mostrar el aviso en el popup de inmediato; la rutina espera 500 ms para leer.
    _recovery_set(step="purga")
    _set_routine_ui(
        rutina="purga",
        rutina_fase="segura",
        rutina_manos=True,
        estado_label="Purga",
        mensaje="NO INTRODUCIR MANOS. Proceso de limpieza en curso, espera.",
    )
    _set_machine_ui(mensaje="NO INTRODUCIR MANOS. Proceso de limpieza en curso, espera.")
    ok, err = _start_routine(_recovery_auto_purga_run)
    if not ok:
        _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False)
        _recovery_set(step="manguera")
        return False, err
    return True, "Purga automática iniciada."


def _recovery_done(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return True, ""
    if rec.get("step") == "purga" or _MACHINE_STATE.get("rutina") == "purga":
        return False, "Espera a que termine la purga."
    if rec.get("step") not in ("listo", "piezas", "manguera", "error"):
        return False, "La recuperación aún no termina."
    if rec.get("step") != "listo":
        # Salir del popup está permitido; no borra la recuperación pendiente.
        return True, "Puedes volver cuando quieras."
    _recovery_clear()
    return True, "Listo."


_ROUTINE_ACTIONS = {
    "purga": _purga_start,
    "purga_alimentar": _purga_alimentar_action,
    "purga_limpiar": _purga_limpiar_action,
    "purga_continuar": _purga_continuar_action,
    "purga_terminar": _purga_terminar_action,
    "posicion_inicial": _posicion_inicial,
    "iniciar": _prealimentador_iniciar,
    "omitir_holgura_izquierda": _omitir_holgura_izquierda,
    "omitir_holgura_derecha": _omitir_holgura_derecha,
    "checklist_empezar": _checklist_empezar,
    "checklist_nueva": _checklist_nueva,
    "checklist_aire": _checklist_aire,
    "checklist_holgura": _checklist_holgura,
    "checklist_home": _checklist_home,
    "checklist_recorrido": _checklist_recorrido,
    "checklist_eje_listo": _checklist_eje_listo,
    "checklist_preparar": _checklist_preparar,
    "checklist_purga": _checklist_purga,
    "checklist_purga_hecha": _checklist_purga_hecha,
    "checklist_piezas": _checklist_piezas,
    "checklist_medicion": _checklist_medicion,
    "mantenimiento_reset": _mant_reset,
    "charola_vacia": _charola_vacia,
    "recovery_confirm_piezas": _recovery_confirm_piezas,
    "recovery_confirm_manguera": _recovery_confirm_manguera,
    "recovery_done": _recovery_done,
}

_mant_load()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/modelos", methods=["GET"])
def get_modelos():
    try:
        modelos = _load_modelos()
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "modelos": [], "error": "No se pudo leer modelos.json"}), 500
    return jsonify({"ok": True, "modelos": modelos})


@app.route("/api/modelo", methods=["POST"])
def set_modelo():
    data = request.get_json(silent=True) or {}
    modelo_id = (data.get("modelo") or "").strip()
    if not modelo_id:
        _MACHINE_STATE["numero_modelo"] = ""
        _MACHINE_STATE["longitud_modelo_mm"] = None
        _MOTION_DATA["modelo_id"] = ""
        _MOTION_DATA["target_length_mm"] = None
        _save_hmi_cycle_extras()
        return jsonify({"ok": True, "state": _public_machine_state(), "motion_data": dict(_MOTION_DATA)})

    entry = _find_modelo(modelo_id)
    if not entry:
        return jsonify({"ok": False, "error": "Modelo no encontrado"}), 404

    _MACHINE_STATE["numero_modelo"] = entry["modelo"]
    _MACHINE_STATE["longitud_modelo_mm"] = entry.get("longitud_mm")
    _MOTION_DATA["modelo_id"] = entry["modelo"]
    _MOTION_DATA["target_length_mm"] = entry.get("longitud_mm")
    _save_hmi_cycle_extras()
    return jsonify({"ok": True, "state": _public_machine_state(), "motion_data": dict(_MOTION_DATA)})


@app.route("/api/compensacion", methods=["POST"])
def set_compensacion():
    data = request.get_json(silent=True) or {}
    try:
        mm = float(data.get("mm"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "Compensación inválida"}), 400
    _MACHINE_STATE["compensacion_medicion_mm"] = mm
    _MOTION_DATA["measure_comp_mm"] = mm
    _mirror_comp_to_cycle_json(mm)
    flat = _motion_data_flat(_MOTION_DATA)
    sent = _send_motion_serial([_compact(flat)])
    return jsonify(
        {
            "ok": True,
            "state": _public_machine_state(),
            "motion_data": dict(_MOTION_DATA),
            "esp32_line": flat,
            **sent,
        }
    )


@app.route("/api/cantidad", methods=["POST"])
def set_cantidad():
    data = request.get_json(silent=True) or {}
    try:
        n = int(data.get("cantidad"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "La cantidad tiene que ser un entero."}), 400
    if n < 1:
        return jsonify({"ok": False, "error": "La cantidad tiene que ser al menos 1."}), 400
    if _LOT["active"] or _LOT["tray_wait"]:
        return jsonify({"ok": False, "error": "No se cambia la cantidad con un lote en curso."}), 400
    _MACHINE_STATE["cantidad"] = n
    _save_hmi_cycle_extras()
    return jsonify({"ok": True, "state": _public_machine_state()})


@app.route("/api/state", methods=["GET"])
def get_state():
    _prefeeder_maybe_refresh_ui()
    return jsonify({"ok": True, **_public_machine_state()})


@app.route("/api/events/clear", methods=["POST"])
def clear_events():
    rev = _clear_event_log()
    return jsonify({"ok": True, "eventos_rev": rev})


@app.route("/api/errors", methods=["GET"])
def api_errors():
    return jsonify({"ok": True, "catalog": error_codes.catalog_for_api()})


@app.route("/api/cycle", methods=["GET", "POST"])
def cycle_config():
    if request.method == "GET":
        params = _load_cycle_params()
        comps = _cut_comp_params_of(params)
        params["cut_comp_mode"] = comps["cut_comp_mode"]
        params["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
        params["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
        params["cut_offset_mm"] = _measure_comp_mm()
        return jsonify(
            {
                "ok": True,
                "steps": list(CYCLE_STEPS),
                "params": params,
                "state": _public_machine_state(),
            }
        )

    data = request.get_json(silent=True) or {}
    patch = data.get("params") or data
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "JSON inválido"}), 400
    params = _load_cycle_params()
    for key in _DEFAULT_CYCLE_PARAMS:
        if key in patch and patch[key] is not None:
            params[key] = patch[key]
    params["feed_sides"] = _feed_sides_of(params)
    if "cut_comp_mode" in patch:
        params["cut_comp_mode"] = _parse_bool(patch.get("cut_comp_mode"), False)
    for key in ("cut_comp_r_mm", "cut_comp_l_mm"):
        if key in patch and patch[key] is not None:
            params[key] = _num(patch.get(key), 0.0)
    comps = _cut_comp_params_of(params)
    params["cut_comp_mode"] = comps["cut_comp_mode"]
    params["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
    params["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
    # La compensación general se edita en pantalla principal; no se pisa aquí.
    params["cut_offset_mm"] = _measure_comp_mm()
    for key in (
        "deposit_stack_gap_mm",
        "deposit_max_travel_mm",
        "gripper_clearance_mm",
        "tolerance_mm",
        "blower_ms_per_mm",
        "cut_comp_r_mm",
        "cut_comp_l_mm",
    ):
        params[key] = _num(params.get(key), _DEFAULT_CYCLE_PARAMS[key])
    params["deposit_batch_size"] = int(_num(params.get("deposit_batch_size"), 50))
    _MOTION_DATA["tolerance_mm"] = _num(params.get("tolerance_mm"), 2.0)
    try:
        params["cantidad"] = max(1, int(_MACHINE_STATE.get("cantidad") or 1))
    except (TypeError, ValueError):
        params["cantidad"] = 1
    params["numero_modelo"] = str(_MACHINE_STATE.get("numero_modelo") or "").strip()
    try:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        CYCLE_PATH.write_text(
            json.dumps(params, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar cycle.json"}), 500
    # feed_sides L/R → omitir holgura del contrario en el PreAlimentador.
    _prefeeder_sync_omit_flags()
    return jsonify(
        {
            "ok": True,
            "steps": list(CYCLE_STEPS),
            "params": params,
            "state": _public_machine_state(),
        }
    )


@app.route("/api/motion/tune", methods=["GET", "POST"])
def motion_tune():
    try:
        tune = _load_motion_tune()
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "error": "No se pudo leer Motion.json"}), 500

    if request.method == "GET":
        return jsonify(
            {
                "ok": True,
                "tune": tune,
                **_motion_scale(),
                "esp32_line": _motion_tune_flat(tune),
            }
        )

    data = request.get_json(silent=True) or {}
    patch = data.get("tune") or data
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "JSON inválido"}), 400
    _deep_merge_motion_tune(tune, patch)
    tune = _canonical_motion_tune(tune)
    try:
        _save_motion_tune(tune)
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar Motion.json"}), 500
    return jsonify(
        {
            "ok": True,
            "tune": tune,
            **_motion_scale(),
            "esp32_line": _motion_tune_flat(tune),
        }
    )


def _encoder_from_status(replies):
    reading = {"R": None, "L": None}
    for raw in replies or []:
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") != "status":
            continue
        for side in ("R", "L"):
            block = msg.get("enc_" + side)
            if not isinstance(block, dict):
                continue
            item = {}
            live = block.get("mm_live")
            if live is None:
                live = block.get("mm_off")
            if live is not None:
                item["mm"] = live
            if "laser" in block:
                item["laser"] = bool(block.get("laser"))
            if "joint" in block:
                item["joint"] = bool(block.get("joint"))
            if item:
                reading[side] = item
    return reading


@app.route("/api/motion/status", methods=["GET"])
def motion_status():
    sent = _send_motion_serial(
        ['{"op":"status"}'],
        line_timeout_s=2.0,
        reply_until_op="status",
        nonblocking=True,
    )
    lm_servo_on = None
    lm_homed = None
    feeder_ready = {"R": None, "L": None}
    safety = None
    for raw in reversed(sent.get("replies") or []):
        try:
            status_reply = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if status_reply.get("op") == "status":
            lm_servo_on = status_reply.get("lm_servo_on")
            lm_homed = status_reply.get("lm_homed")
            feeder_ready["R"] = status_reply.get("feeder_r_ready")
            feeder_ready["L"] = status_reply.get("feeder_l_ready")
            safety = status_reply.get("safety")
            break
    return jsonify(
        {
            "ok": True,
            "sent": bool(sent.get("sent")),
            "encoder": _encoder_from_status(sent.get("replies")),
            "lm_servo_on": lm_servo_on,
            "lm_homed": lm_homed,
            "feeder_ready": feeder_ready,
            "safety": safety,
        }
    )


@app.route("/api/motion/cmd", methods=["POST"])
def motion_cmd():
    body = request.get_json(silent=True) or {}
    op = (body.get("op") or "").strip()
    try:
        tune = _load_motion_tune()
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "error": "No se pudo leer Motion.json"}), 500

    patch = body.get("tune")
    if isinstance(patch, dict):
        _deep_merge_motion_tune(tune, patch)
        tune = _canonical_motion_tune(tune)

    if op == "feeder_test":
        side = str(body.get("side") or "").upper()
        if side not in ("R", "L"):
            return jsonify({"ok": False, "error": "El Test pide lado R o L."}), 400
        try:
            _save_motion_tune(tune)
        except OSError:
            return jsonify({"ok": False, "error": "No se pudo guardar Motion.json"}), 500
        result = _feeder_test(side, tune)
        result.update({"op": op, "tune": tune, **_motion_scale()})
        return jsonify(result)

    try:
        tune, lines = _build_motion_lines(op, tune, body)
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400

    if isinstance(patch, dict) or op in ("set_tune", "lmguide_move"):
        try:
            _save_motion_tune(tune)
        except OSError:
            return jsonify({"ok": False, "error": "No se pudo guardar Motion.json"}), 500

    if op == "lmguide_stop":
        sent = _send_motion_urgent(lines)
    elif op == "motion_hw_init":
        sent = _send_motion_serial(
            lines,
            line_timeout_s=120.0,
            reply_until_op="motion_hw_init",
            nonblocking=True,
        )
    else:
        sent = _send_motion_serial(lines, nonblocking=True)
    try:
        response_op = str(json.loads(lines[-1]).get("op") or op)
    except (IndexError, TypeError, json.JSONDecodeError):
        response_op = op
    command_ok = None
    for raw in reversed(sent.get("replies") or []):
        try:
            reply = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if reply.get("op") == response_op:
            command_ok = bool(reply.get("ok", True))
            break
    return jsonify(
        {
            "ok": True,
            "op": op,
            "tune": tune,
            "lines": lines,
            "command_ok": command_ok,
            **_motion_scale(),
            **sent,
        }
    )


@app.route("/api/motion/data", methods=["GET", "POST"])
def motion_data():
    global _MOTION_DATA
    if request.method == "GET":
        return jsonify(
            {
                "ok": True,
                "data": dict(_MOTION_DATA),
                "esp32_line": _motion_data_flat(_MOTION_DATA),
            }
        )

    data = request.get_json(silent=True) or {}
    patch = data.get("data") or data
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "JSON inválido"}), 400
    for key in ("target_length_mm", "tolerance_mm", "modelo_id", "measure_comp_mm"):
        if key in patch:
            _MOTION_DATA[key] = patch[key]
    if "measure_comp_mm" in patch:
        try:
            comp = float(patch["measure_comp_mm"])
        except (TypeError, ValueError):
            comp = 0.0
        _MACHINE_STATE["compensacion_medicion_mm"] = comp
        _MOTION_DATA["measure_comp_mm"] = comp
        _mirror_comp_to_cycle_json(comp)
    flat = _motion_data_flat(_MOTION_DATA)
    return jsonify({"ok": True, "data": dict(_MOTION_DATA), "esp32_line": flat})


@app.route("/api/plc/status", methods=["GET"])
def plc_status():
    status = _plc_status_get()
    return jsonify(
        {
            "ok": True,
            "sent": bool(_plc_serial_configured() and status is not None),
            "status": status,
            "active_errors": _plc_active_errors(status),
        }
    )


@app.route("/api/plc/cmd", methods=["POST"])
def plc_cmd():
    body = request.get_json(silent=True) or {}
    op = (body.get("op") or "").strip()
    try:
        line = _build_plc_line(op, body)
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    sent = _plc_send_payload(line, push_log=False)
    packed = _compact(line)
    command_ok = None
    for raw in reversed(sent.get("replies") or []):
        try:
            reply = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if reply.get("op") == op:
            command_ok = bool(reply.get("ok", True))
            break
    status = _plc_status_after_command(sent)
    if sent.get("sent") and op in ("valve", "blower_on", "blower_off", "all_off", "reset"):
        status = _plc_wait_ready(2.5) or status
    return jsonify(
        {
            "ok": True,
            "op": op,
            "lines": [packed],
            "command_ok": command_ok,
            "status": status,
            "active_errors": _plc_active_errors(status),
            **sent,
        }
    )


@app.route("/api/prefeeder/tune", methods=["GET", "POST"])
def prefeeder_tune():
    try:
        tune = _load_prefeeder_tune()
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "error": "No se pudo leer PreFeeder.json"}), 500

    if request.method == "GET":
        # Leer actuales del maestro (NVS). PreFeeder.json solo es espejo local.
        st = None
        if _serial_port("prefeeder"):
            st = _prefeeder_status_get() or _PF_CACHED_STATUS
            if st:
                _prefeeder_reconcile_pushed_from_status(st)
        changed = False
        for side in ("L", "R"):
            live = _prefeeder_cfg_from_status_side(st, side)
            if live is not None:
                if _prefeeder_cfg_signature(tune[side]) != _prefeeder_cfg_signature(live):
                    changed = True
                tune[side] = live
        if changed:
            try:
                _save_prefeeder_tune(tune)
            except OSError:
                pass
        return jsonify({"ok": True, "tune": tune, "live": bool(st)})

    data = request.get_json(silent=True) or {}
    patch = data.get("tune") or data
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "JSON inválido"}), 400
    merged = dict(tune)
    for side in ("L", "R"):
        if isinstance(patch.get(side), dict):
            merged[side] = {**(tune.get(side) or {}), **patch[side]}
    next_tune = _canonical_prefeeder_tune(merged)

    # Guardar = escribir al maestro. El JSON local solo espeja lo confirmado.
    push = {"L": False, "R": False}
    applied: dict[str, bool | None] = {"L": None, "R": None}
    if not _serial_port("prefeeder"):
        return jsonify(
            {
                "ok": False,
                "error": "Sin COM del maestro; no se pueden guardar parámetros en el ESP32.",
                "tune": next_tune,
                "pushed": push,
                "applied": applied,
            }
        ), 400

    if _PF_CACHED_STATUS:
        _prefeeder_reconcile_pushed_from_status(_PF_CACHED_STATUS)
    for side in ("L", "R"):
        if not isinstance(patch.get(side), dict):
            continue
        ok, _changed = _prefeeder_set_config_side(side, next_tune[side])
        push[side] = ok
    st = _prefeeder_refresh_cache()
    for side in ("L", "R"):
        live = _prefeeder_cfg_from_status_side(st, side)
        if isinstance(patch.get(side), dict):
            if live is None:
                applied[side] = False
            else:
                applied[side] = (
                    _prefeeder_cfg_signature(live)
                    == _prefeeder_cfg_signature(next_tune[side])
                )
        if live is not None:
            next_tune[side] = live
    # Espejo local = lo que quedó en el maestro.
    try:
        _save_prefeeder_tune(next_tune)
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar PreFeeder.json"}), 500
    return jsonify(
        {"ok": True, "tune": next_tune, "pushed": push, "applied": applied}
    )


@app.route("/api/prefeeder/status", methods=["GET"])
def prefeeder_status():
    global _PF_CACHED_STATUS
    port_ok = bool(_serial_port("prefeeder"))
    force = request.args.get("force") == "1"
    status = None
    if port_ok and (force or _prefeeder_comm_active()):
        status = _prefeeder_refresh_cache()
        _prefeeder_watch_once()
    elif _PF_CACHED_STATUS:
        status = _PF_CACHED_STATUS
    return jsonify(
        {
            "ok": True,
            "sent": bool(port_ok and status is not None),
            "status": status,
            "active_errors": _prefeeder_active_errors(status),
        }
    )


@app.route("/api/prefeeder/cmd", methods=["POST"])
def prefeeder_cmd():
    body = request.get_json(silent=True) or {}
    op = str(body.get("op") or "").strip()
    if op == "set_force_feed":
        side = str(body.get("side") or "").upper()
        on = bool(body.get("on"))
        if side not in ("L", "R"):
            return jsonify({"ok": False, "error": "Lado no reconocido."}), 400
        if not _prefeeder_required():
            return jsonify({"ok": False, "error": "Sin COM del PreAlimentador."}), 400
        ok = _prefeeder_set_force(side, on)
        if ok:
            with _PF_MANUAL_HOLD_LOCK:
                _PF_MANUAL_HOLD[side] = on
                other = "R" if side == "L" else "L"
                any_hold = _PF_MANUAL_HOLD["L"] or _PF_MANUAL_HOLD["R"]
            if not on and not any_hold:
                with _STATE_LOCK:
                    in_cycle = bool(_MACHINE_STATE.get("ciclo_en_curso"))
                    running = _MACHINE_STATE.get("estado") == "running"
                    rutina = bool(_MACHINE_STATE.get("rutina"))
                if not in_cycle and not running and not rutina:
                    _prefeeder_set_feed_lock(True)
        return jsonify({"ok": ok, "op": op, "side": side, "on": on})
    return jsonify({"ok": False, "error": "Orden no reconocida."}), 400


@app.route("/api/serial", methods=["GET", "POST"])
def serial_ports():
    """Puertos COM elegidos en la pantalla."""
    if request.method == "GET":
        # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR (GET: ad, andon_skipped, andon_none_value) ---
        with _serial_cfg_lock:
            ad = bool(_serial_cfg.get("andon_disabled"))
        return jsonify(
            {
                "ok": True,
                "ports": _list_com_ports(),
                "motion": _serial_port("motion"),
                "plc": _serial_port("plc"),
                "andon": _serial_port("andon"),
                "prefeeder": _serial_port("prefeeder"),
                "andon_disabled": ad,
                "andon_skipped": _andon_skipped_effective(),
                "andon_none_value": _ANDON_NONE_TOKEN,
            }
        )

    body = request.get_json(silent=True) or {}
    listed = _list_com_ports()
    motion_raw = str(body.get("motion") or "").strip()
    plc_raw = str(body.get("plc") or "").strip()
    andon_raw = str(body.get("andon") or "").strip()
    prefeeder_raw = str(body.get("prefeeder") or "").strip()
    motion = _match_listed_port(motion_raw, listed)
    plc = _match_listed_port(plc_raw, listed)
    prefeeder = _match_listed_port(prefeeder_raw, listed)
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR (POST: token __none__ y andon_disable_ui) ---
    andon_disable_ui = andon_raw == _ANDON_NONE_TOKEN
    if andon_disable_ui:
        andon = ""
    else:
        andon = _match_listed_port(andon_raw, listed)
    if motion_raw and not motion:
        return jsonify({"ok": False, "error": "El puerto de Motion no está conectado."}), 400
    if plc_raw and not plc:
        return jsonify({"ok": False, "error": "El puerto del PLC no está conectado."}), 400
    if andon_raw and not andon_disable_ui and not andon:
        return jsonify({"ok": False, "error": "El puerto del Andon no está conectado."}), 400
    if prefeeder_raw and not prefeeder:
        return jsonify({"ok": False, "error": "El puerto del PreAlimentador no está conectado."}), 400
    chosen = [port for port in (motion, plc, andon, prefeeder) if port]
    if len(chosen) != len({port.upper() for port in chosen}):
        return jsonify({"ok": False, "error": "Cada módulo tiene que usar un COM distinto."}), 400

    with _serial_cfg_lock:
        checklist_bypass = bool(_serial_cfg.get("checklist_bypass"))
    try:
        _save_serial_cfg(motion, plc, andon, andon_disable_ui, checklist_bypass, prefeeder)
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar el puerto."}), 500
    if prefeeder:
        _prefeeder_adopt_live_config()

    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR (respuesta POST: campos extras) ---
    return jsonify(
        {
            "ok": True,
            "motion": motion,
            "plc": plc,
            "andon": andon,
            "prefeeder": prefeeder,
            "andon_disabled": andon_disable_ui,
            "andon_skipped": _andon_skipped_effective(),
            "andon_none_value": _ANDON_NONE_TOKEN,
        }
    )


@app.route("/api/settings/checklist-bypass", methods=["GET", "POST"])
def checklist_bypass_setting():
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        enabled = bool(body.get("enabled"))
        with _serial_cfg_lock:
            motion = str(_serial_cfg.get("motion") or "")
            plc = str(_serial_cfg.get("plc") or "")
            andon = str(_serial_cfg.get("andon") or "")
            prefeeder = str(_serial_cfg.get("prefeeder") or "")
            andon_disabled = bool(_serial_cfg.get("andon_disabled"))
        try:
            _save_serial_cfg(motion, plc, andon, andon_disabled, enabled, prefeeder)
        except OSError:
            return jsonify({"ok": False, "error": "No se pudo guardar la configuración."}), 500
        _push_event("Bypass de checklist activo." if enabled else "Checklist obligatorio.")
    return jsonify({"ok": True, "enabled": _checklist_bypassed()})


@app.route("/api/serial/test", methods=["POST"])
def serial_module_test():
    """Comprueba que el COM elegido responde y es el MCU pedido. Andon incluido."""
    body = request.get_json(silent=True) or {}
    which = str(body.get("module") or "").strip().lower()
    if which not in ("motion", "plc", "andon", "prefeeder"):
        return jsonify({"ok": False, "error": "Módulo no reconocido."}), 400
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR (early return si Andon omitido) ---
    if which == "andon" and _andon_skipped_effective():
        return jsonify(
            {
                "ok": True,
                "module": which,
                "skipped": True,
                "message": "Andon no usado (Conexión o HMI_ANDON_SKIP=1). No se prueba COM.",
            }
        )
    # ---
    ok, code = _check_module_link(which)
    if not ok:
        return jsonify(
            {
                "ok": False,
                "module": which,
                "code": code,
                "error": error_codes.format_message(code),
            }
        )
    return jsonify(
        {
            "ok": True,
            "module": which,
            "port": _serial_port(which),
            "message": f"{which.upper()} en {_serial_port(which)} respondió como {which.upper()}.",
        }
    )


def _action_rejected(err: str):
    """El rechazo queda en el letrero. El panel de eventos puede estar cerrado."""
    if err:
        _push_event(err, "error")
        with _STATE_LOCK:
            already_error = (
                _MACHINE_STATE.get("estado") == "error"
                or bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
            )
        # Si ya hay error/recuperación, no relatchear otro código (p. ej. Reset fallido
        # por COM que devolvía E019 y dejaba al operador atrapado).
        if already_error:
            _set_machine_ui(mensaje=err)
        else:
            head = (err.split(maxsplit=1)[0] if err else "") or ""
            if len(head) >= 4 and head[0] == "E" and head[1:].isdigit():
                code = head
                _recovery_begin(code, err)
                _set_machine_ui(
                    estado="error",
                    estado_label="Error",
                    mensaje=err,
                    ciclo_error=code,
                )
            else:
                _set_machine_ui(mensaje=err)
    return jsonify({"ok": False, "error": err, "state": _public_machine_state()}), 400


@app.route("/api/action", methods=["POST"])
def action():
    """Orquestador: acciones de proceso (Start / Pausa / …) y registro del resto."""
    data = request.get_json(silent=True) or {}
    block = data.get("block", "")
    action_name = (data.get("action") or "").strip()
    message = ""

    if action_name in _ROUTINE_ACTIONS:
        ok, err = _ROUTINE_ACTIONS[action_name](data)
        if not ok:
            return _action_rejected(err)
        message = err or "Listo."
    elif block == "proceso":
        if action_name == "start":
            ok, err = _cycle_start()
            if not ok:
                return _action_rejected(err)
            message = "Lote iniciado."
        elif action_name == "step_start":
            ok, err = _cycle_step_start()
            if not ok:
                return _action_rejected(err)
            message = err or "Prueba paso a paso iniciada."
        elif action_name == "step_next":
            ok, err = _cycle_step_next()
            if not ok:
                return _action_rejected(err)
            message = err or "Siguiente paso."
        elif action_name == "pause":
            _cycle_pause()
            message = "Pausa solicitada."
        elif action_name == "resume":
            ok, err = _cycle_resume()
            if not ok:
                return _action_rejected(err)
            message = "Reanudar."
        elif action_name == "stop":
            _cycle_stop()
            message = "Stop solicitado."
        elif action_name == "reset":
            ok, err = _cycle_reset_error()
            if not ok:
                return _action_rejected(err)
            message = "Reset."
        else:
            message = f"Acción proceso: {action_name}"
    else:
        message = f"Acción registrada: {block} / {action_name}"

    return jsonify(
        {
            "ok": True,
            "block": block,
            "action": action_name,
            "message": message,
            "state": _public_machine_state(),
        }
    )


_hydrate_hmi_from_cycle()

threading.Thread(target=_prefeeder_watch_loop, daemon=True, name="prefeeder_watch").start()
threading.Thread(target=_prefeeder_boot_lock, daemon=True, name="prefeeder_boot").start()

if __name__ == "__main__":
    # Sin una línea por cada GET de sondeo del HMI (~400–800 ms).
    # use_reloader=False: el reloader mata el hilo del ciclo a mitad de un
    # lmguide_move / feed_stage y al reiniciar deja la HMI en «En espera»
    # sin código de error (estado en RAM se pierde).
    app.run(
        debug=True,
        use_reloader=False,
        host="0.0.0.0",
        port=5000,
        request_handler=_QuietRequestHandler,
    )

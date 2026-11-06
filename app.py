import json
import math
import os
import sys
import threading
import time
from pathlib import Path

from flask import Flask, jsonify, render_template, request

_CLASS_DIR = Path(__file__).resolve().parent / "Class"
if str(_CLASS_DIR) not in sys.path:
    sys.path.insert(0, str(_CLASS_DIR))

import error_codes  # noqa: E402

app = Flask(__name__)

DATA_DIR = Path(__file__).resolve().parent / "data"
MODELOS_PATH = DATA_DIR / "modelos.json"
MOTION_PATH = DATA_DIR / "Motion.json"
CYCLE_PATH = DATA_DIR / "cycle.json"
SERIAL_PATH = DATA_DIR / "serial.json"
MANT_PATH = DATA_DIR / "mantenimiento.json"
CHECKLIST_LOG_PATH = DATA_DIR / "checklist.json"

# Vías de operador. No son parámetros del ciclo automático.
PURGE_PARK_MM = 250.0
PURGE_CLEAN_MS = 3000
PURGE_AFTER_CUT_MS = 500
CHECKLIST_EXTRA_MM = 55.0
CHECKLIST_MAX_CYCLES = 10000

# ServoLMGuide.h [7]: 770 PUU en 45 mm. El sentido de trabajo es PUU negativo.
LMGUIDE_STEPS_PER_MM = 770.0 / 45.0
LMGUIDE_PUU_SIGN = -1

# ServoFeeder.h [8]: 3600 counts = 55 mm de cinta. Vale para mm/s → 6081 y mm → 607A.
FEED_STEPS_PER_MM = 3600.0 / 55.0

_serial_lock = threading.Lock()
_motion_serial = None
_serial_cfg_lock = threading.Lock()
_serial_cfg = {"motion": "", "plc": "", "andon": ""}

# Orquestador del ciclo automático (rules.md §4.1–4.8) — app.py coordina Motion + PLC.
_STATE_LOCK = threading.Lock()
_CYCLE_CTRL = {
    "thread": None,
    "stop": threading.Event(),
    "pause_after_step": threading.Event(),
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
    "length_mm": 0.0,
    "gap_mm": 0.0,
    "clearance_mm": 0.0,
    "sides": "R",
}
_LAST_PIECE_S = 0.0

# Pasos ejecutables en orden (1 = arranque / set_data al Motion).
CYCLE_STEPS = (
    {"id": "1", "label": "Start — inicio de secuencia", "actor": "coordinator"},
    {"id": "2", "label": "Holder dual + encoder dual", "actor": "plc"},
    {"id": "3", "label": "Alimentación y medición L/R", "actor": "motion"},
    {"id": "4", "label": "Pinzas — liberar holder/encoder", "actor": "plc"},
    {"id": "5", "label": "LM Guide — posición de corte", "actor": "motion"},
    {"id": "6", "label": "Corte R/L", "actor": "plc"},
    {"id": "7", "label": "Descarga de pieza", "actor": "motion+plc"},
    {"id": "8", "label": "Home LM Guide + blower", "actor": "motion+plc"},
)

_DEFAULT_CYCLE_PARAMS = {
    "ms_holder_enc_stable": 100,
    "ms_lm_at_cut": 100,
    "ms_lm_at_discharge": 100,
    "ms_cutter_on": 50,
    "ms_after_cutter": 100,
    "ms_after_grip_release": 100,
    "ms_blower_delay": 200,
    "ms_at_home_end": 50,
    "tolerance_mm": 2.0,
    "cut_offset_mm": 0.0,
    "deposit_batch_size": 50,
    "deposit_stack_gap_mm": 100.0,
    "deposit_max_travel_mm": 1500.0,
    "gripper_clearance_mm": 20.0,
    "feed_sides": "R",
    "blower_ms_per_mm": 8.0,
    "blower_ms_min": 200,
}

# Estado inicial (orquestador + UI)
_MACHINE_STATE = {
    "estado": "idle",
    "estado_label": "En espera",
    "mensaje": "Sistema listo. Sin errores activos.",
    "progreso_pct": 0,
    "ciclo_paso_id": "",
    "ciclo_paso_label": "",
    "ciclo_paso_indice": 0,
    "ciclo_en_curso": False,
    "ciclo_error": "",
    "ciclo_feed": {"R": None, "L": None},
    "numero_modelo": "",
    "cantidad": 1,
    "longitud_modelo_mm": None,
    "compensacion_medicion_mm": 0.0,
    "tiempo_ciclo_s": None,
    "tiempo_estimado_s": 0,
    "tiempo_corriendo_s": 0,
    "eventos": [],
    "rutina": "",
    "rutina_fase": "",
    "rutina_manos": False,
    "carga_material": False,
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
            "pr_speed_rpm": _as_float(src, ("lmguide", "pr_speed_rpm"), 1200.0),
            "pos_puu": _as_int(src, ("lmguide", "pos_puu"), 0),
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


def _cut_mm_of(length: float, params: dict) -> float:
    return abs(length) + _num(params.get("cut_offset_mm"), 0.0)


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
    """Corte = |L| + offset. Descarga = depósito del primer montón."""
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
        side = str(body.get("side") or "R").upper()
        if side not in ("R", "L"):
            raise ValueError("El reset del encoder pide lado R o L.")
        return tune, [_compact({"op": "encoder_reset", "side": side})]
    if op == "feeder_init":
        return tune, [tune_line, '{"op":"feeder_init","side":"both"}']
    if op == "lmguide_init":
        return tune, [tune_line, '{"op":"lmguide_servo_on"}']
    if op == "lmguide_servo":
        if body.get("on"):
            return tune, [tune_line, '{"op":"lmguide_servo_on"}']
        return tune, ['{"op":"lmguide_servo_off"}']
    if op == "lmguide_home":
        return tune, ['{"op":"lmguide_home"}']
    if op == "lmguide_move_home":
        return tune, [tune_line, '{"op":"lmguide_move","target":"home"}']
    if op == "lmguide_move":
        if body.get("mm") is None or body.get("mm") == "":
            raise ValueError("Falta la posición en mm.")
        puu = _mm_to_puu(body.get("mm"))
        tune["lmguide"]["pos_puu"] = puu
        tune_line = _compact(_motion_tune_flat(tune))
        return tune, [tune_line, _compact({"op": "lmguide_move", "puu": puu})]
    if op == "lmguide_jog":
        if body.get("jog_mm") is None or body.get("jog_mm") == "":
            raise ValueError("Falta el jog en mm.")
        jog_mm = float(body.get("jog_mm"))
        if jog_mm == 0:
            raise ValueError("El jog en mm tiene que ser distinto de cero.")
        direction = 1 if int(body.get("dir") or 1) >= 0 else -1
        delta = _mm_to_puu(direction * abs(jog_mm))
        return tune, [tune_line, _compact({"op": "lmguide_jog", "delta_puu": delta})]
    if op == "lmguide_stop":
        return tune, ['{"op":"lmguide_stop"}']
    raise ValueError("Orden Motion no reconocida.")


def _reply_wait_s(line: str) -> float:
    if "lmguide_home" in line:
        return 40.0
    if "feeder_init" in line or "lmguide_servo_on" in line:
        return 8.0
    return 2.0


def _load_serial_cfg() -> None:
    motion = os.environ.get("MOTION_SERIAL", "").strip()
    plc = os.environ.get("PLC_SERIAL", "").strip()
    andon = os.environ.get("ANDON_SERIAL", "").strip()
    if SERIAL_PATH.exists():
        try:
            saved = json.loads(SERIAL_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            saved = None
        if isinstance(saved, dict):
            motion = str(saved.get("motion") or "").strip()
            plc = str(saved.get("plc") or "").strip()
            andon = str(saved.get("andon") or "").strip()
    with _serial_cfg_lock:
        _serial_cfg["motion"] = motion
        _serial_cfg["plc"] = plc
        _serial_cfg["andon"] = andon


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


def _save_serial_cfg(motion: str, plc: str, andon: str) -> None:
    motion = str(motion or "").strip()
    plc = str(plc or "").strip()
    andon = str(andon or "").strip()
    with _serial_cfg_lock:
        prev_motion = _serial_cfg["motion"]
        prev_plc = _serial_cfg["plc"]
        prev_andon = _serial_cfg["andon"]
        _serial_cfg["motion"] = motion
        _serial_cfg["plc"] = plc
        _serial_cfg["andon"] = andon
    SERIAL_PATH.parent.mkdir(parents=True, exist_ok=True)
    SERIAL_PATH.write_text(
        json.dumps({"motion": motion, "plc": plc, "andon": andon}, indent=2) + "\n",
        encoding="utf-8",
    )
    if motion != prev_motion:
        _close_motion_serial()
    if plc != prev_plc:
        _close_plc_serial()
    if andon != prev_andon:
        _close_andon_serial()


def _read_serial_line(ser, timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if b"\n" in buf:
                line, _, _rest = buf.partition(b"\n")
                return line.decode("utf-8", "replace").strip()
        else:
            time.sleep(0.02)
    return buf.decode("utf-8", "replace").strip()


_plc_serial = None
_plc_serial_lock = threading.Lock()
_andon_serial = None
_andon_serial_lock = threading.Lock()
_andon_state_sent = ""
_andon_attempted = ""
_PLC_VALVES = ("holder", "encoder", "cutter_r", "cutter_l", "gripper")
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
                _plc_serial = ser
            ser = _plc_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                reply = _read_serial_line(ser, 2.0)
                if reply:
                    replies.append(reply)
    except Exception as exc:
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
                _andon_serial = ser
            ser = _andon_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                reply = _read_serial_line(ser, 2.0)
                if reply:
                    replies.append(reply)
    except Exception as exc:
        return {"sent": False, "reason": str(exc), "replies": replies}
    return {"sent": True, "reason": "", "replies": replies}


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
        sent = _send_motion_serial(['{"op":"status"}'], line_timeout_s=2.0)
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
    messages = []
    for which in ("motion", "plc", "andon"):
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
        if _cycle_should_stop():
            return last
        last = _plc_status_get()
        if last is not None and not last.get("pulse_busy"):
            return last
        time.sleep(0.05)
    if last is not None and last.get("pulse_busy"):
        return last
    return last


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


def _send_motion_serial(lines, line_timeout_s=None):
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
    try:
        with _serial_lock:
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
                _motion_serial = ser
            ser = _motion_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                reply = _read_serial_line(
                    ser,
                    line_timeout_s if line_timeout_s is not None else _reply_wait_s(line),
                )
                if reply:
                    replies.append(reply)
    except Exception as exc:
        return {"sent": False, "reason": str(exc), "replies": replies}
    return {"sent": True, "reason": "", "replies": replies}


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
    return params


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


def _now_hms() -> str:
    return time.strftime("%H:%M:%S")


def _push_event(texto: str, tipo: str = "info") -> None:
    entry = {"texto": texto, "tipo": tipo, "hora": _now_hms()}
    with _STATE_LOCK:
        ev = _MACHINE_STATE.setdefault("eventos", [])
        ev.insert(0, entry)
        del ev[20:]


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
    with _STATE_LOCK:
        state = dict(_MACHINE_STATE)
        state["checklist"] = dict(_MACHINE_STATE.get("checklist") or {})
        state["eventos"] = list(_MACHINE_STATE.get("eventos") or [])
        state["ciclo_feed"] = dict(_MACHINE_STATE.get("ciclo_feed") or {})
        state["charola_llena"] = bool(_LOT["tray_wait"])
        state["charola_ciclos"] = int(_LOT["deposits_per_tray"] or 0) if _LOT["tray_wait"] else 0
        state["lote_ciclos_hechos"] = int(_LOT["cycles_done"] or 0)
        state["lote_ciclos_total"] = int(_LOT["cycles_total"] or 0)
        state["lote_lados"] = _LOT["sides"]
        return state


def _set_machine_ui(
    *,
    estado=None,
    estado_label=None,
    mensaje=None,
    step_id=None,
    step_index=None,
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
                _MACHINE_STATE["progreso_pct"] = _cycle_progress_pct(step_index)
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


def _motion_owner_busy() -> str:
    cycle = _CYCLE_CTRL["thread"]
    if cycle is not None and cycle.is_alive():
        return "Hay un ciclo en ejecución."
    routine = _ROUTINE["thread"]
    if routine is not None and routine.is_alive():
        return "Hay una rutina en curso."
    return ""


def _cycle_wait_if_paused() -> bool:
    """True si hay Stop brusco (abortar de inmediato)."""
    return _cycle_should_stop()


def _cycle_sleep_ms(ms: int) -> bool:
    """True si se completó la espera; False si hubo Stop."""
    deadline = time.time() + (max(0, int(ms)) / 1000.0)
    while time.time() < deadline:
        if _cycle_wait_if_paused():
            return False
        time.sleep(0.02)
    return not _cycle_should_stop()


def _cycle_plc_send(payload: dict):
    op = str(payload.get("op") or "")
    try:
        if op in ("valve", "blower_on", "blower_off", "all_off", "reset"):
            line = _compact(_build_plc_line(op, payload))
        else:
            line = _compact(payload)
    except (TypeError, ValueError):
        line = _compact(payload)
    sent = _send_plc_serial([line])
    if not sent.get("sent"):
        sent["code"] = "E019" if _serial_port("plc") else "E017"
        _push_event(error_codes.format_message(sent["code"]), "error")
    else:
        _push_event(f"[PLC] {line}")
    return sent


def _cycle_motion_send(payload: dict, timeout_s: float = 2.0):
    line = _compact(payload)
    sent = _send_motion_serial([line], line_timeout_s=timeout_s)
    if sent.get("sent"):
        _push_event(f"[Motion] {line}")
    else:
        sent["code"] = "E043" if _serial_port("motion") else "E041"
        _push_event(error_codes.format_message(sent["code"]), "error")
    return sent


def _sent_ok(sent: dict, fallback: str) -> bool:
    if sent.get("sent"):
        return True
    _cycle_fail(sent.get("code") or fallback)
    return False


def _cycle_blower_duration_ms(params: dict) -> int:
    length = _MOTION_DATA.get("target_length_mm")
    try:
        length_f = float(length) if length is not None else 0.0
    except (TypeError, ValueError):
        length_f = 0.0
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
    _set_machine_ui(
        estado=estado,
        estado_label=elabel,
        mensaje=msg,
        step_id=step_id,
        step_index=step_index,
        en_curso=True,
        ciclo_error="",
    )
    _, meta = _cycle_step_meta(step_id)
    if meta:
        _push_event(f"Paso {step_id} — {meta['label']}")


def _cycle_fail(code: str, **fmt) -> None:
    msg = error_codes.format_message(code, **fmt)
    _push_event(msg, "error")
    _set_machine_ui(
        estado="error",
        estado_label="Error",
        mensaje=msg,
        en_curso=True,
        ciclo_error=code,
    )


def _cycle_latch_reset() -> None:
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
            _cycle_fail(code or "E010")
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
            {"gripper": True, "holder": False, "encoder": False}
        )
        if not ok_hw:
            _cycle_fail(code or "E010")
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
    err = error_name or "unknown"
    if err == "ok":
        err = "unknown"
    code = error_codes.feed_stage_code(err)
    if code == "E038":
        _cycle_fail("E038", detail=err)
    else:
        _cycle_fail(code)


def _cycle_move_lm_mm(mm: float) -> bool:
    tune = _load_motion_tune()
    puu = _mm_to_puu(mm)
    if not _sent_ok(_cycle_motion_send(_motion_tune_flat(tune), timeout_s=2.0), "E043"):
        return False
    return _sent_ok(_cycle_motion_send({"op": "lmguide_move", "puu": puu}, timeout_s=40.0), "E043")


def _cycle_run_step(step_id: str, params: dict) -> bool:
    # Step 1. Start — publicar datos de ciclo (modelo, longitud, tolerancia) al MCU Motion.
    if step_id == "1":
        flat = _motion_data_flat(_MOTION_DATA)
        return _sent_ok(_cycle_motion_send(flat, timeout_s=2.0), "E043")

    # Step 2. Activar holder dual y encoder dual juntos; esperar estabilización.
    if step_id == "2":
        if not _sent_ok(_cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": True}), "E019"):
            return False
        if not _cycle_sleep_ms(int(params["ms_holder_enc_stable"])):
            return False
        ok_hw, code = _plc_confirm({"holder": True, "encoder": True})
        if not ok_hw:
            _cycle_fail(code or "E010")
            return False
        return True

    # Step 3. Alimentar el lado configurado hasta el láser y validar el encoder.
    if step_id == "3":
        sides = _run_sides()
        payload = {"op": "feed_stage"}
        if sides in ("R", "L"):
            payload["side"] = sides
        side_name = {"R": "derecho", "L": "izquierdo", "LR": "L y R"}[sides]
        _set_machine_ui(mensaje=_cycle_piece_prefix() + f"Alimentación {side_name}.")
        sent = _cycle_motion_send(payload, timeout_s=65.0)
        found, ok, error_name, feed = _read_feed_stage(sent)
        if not sent.get("sent"):
            _cycle_fail(sent.get("code") or "E043")
            return False
        if not found or not ok:
            _fail_feed_stage(error_name)
            return False
        _set_machine_ui(feed=feed)
        return True

    # Step 4. Activar pinzas; desactivar holder y encoder; esperar.
    if step_id == "4":
        if not _sent_ok(_cycle_plc_send({"op": "valve", "name": "gripper", "on": True}), "E019"):
            return False
        if not _sent_ok(_cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": False}), "E019"):
            return False
        if not _cycle_sleep_ms(int(params["ms_after_grip_release"])):
            return False
        ok_hw, code = _plc_confirm(
            {"gripper": True, "holder": False, "encoder": False}
        )
        if not ok_hw:
            _cycle_fail(code or "E010")
            return False
        return True

    # Step 5. LM Guide a |L| + offset de corte; esperar en corte.
    if step_id == "5":
        pos = _active_cut_deposit()
        if pos is None:
            _cycle_fail("E007")
            return False
        cut_mm, _disc_mm, _stack = pos
        _push_event(f"LM Guide a corte: {cut_mm:.2f} mm.")
        if not _cycle_move_lm_mm(cut_mm):
            return False
        return _cycle_sleep_ms(int(params["ms_lm_at_cut"]))

    # Step 6. Pulsar el cortador del lado que alimenta. Ambos lados pulsan los dos.
    if step_id == "6":
        sides = _run_sides()
        valve = {"R": "cutter_r", "L": "cutter_l", "LR": "cutters"}[sides]
        side_name = {"R": "derecho", "L": "izquierdo", "LR": "L y R"}[sides]
        _set_machine_ui(mensaje=_cycle_piece_prefix() + f"Corte {side_name}.")
        if not _sent_ok(_cycle_plc_send({"op": "valve", "name": valve, "on": True}), "E019"):
            return False
        if not _cycle_sleep_ms(int(params["ms_cutter_on"])):
            return False
        if not _sent_ok(_cycle_plc_send({"op": "valve", "name": valve, "on": False}), "E019"):
            return False
        return _cycle_sleep_ms(int(params["ms_after_cutter"]))

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
        if not _sent_ok(_cycle_plc_send({"op": "valve", "name": "gripper", "on": False}), "E019"):
            return False
        return _cycle_sleep_ms(int(params["ms_after_grip_release"]))

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
        if abs(clear_at - deposit) >= 0.05:
            _push_event(f"Despeje de pinzas: {clear_at:.2f} mm.")
            if not _cycle_move_lm_mm(clear_at):
                return False
        if not _sent_ok(_cycle_motion_send({"op": "lmguide_move", "target": "home"}, timeout_s=40.0), "E043"):
            return False
        if not _cycle_sleep_ms(int(params["ms_blower_delay"])):
            return False
        blower_ms = _cycle_blower_duration_ms(params)
        if not _sent_ok(_cycle_plc_send({"op": "blower_on", "ms": blower_ms}), "E019"):
            return False
        blower_t0 = time.time()
        home_deadline = time.time() + 30.0
        while True:
            if _cycle_wait_if_paused():
                return False
            blower_done = (time.time() - blower_t0) >= (blower_ms / 1000.0)
            home_done = time.time() >= home_deadline
            if blower_done and home_done:
                break
            time.sleep(0.05)
        return _cycle_sleep_ms(int(params["ms_at_home_end"]))

    return True


def _finish_piece_idle(mensaje: str) -> None:
    global _LAST_PIECE_S
    _LOT["active"] = False
    _LOT["tray_wait"] = False
    _LOT["home_before"] = False
    _set_machine_ui(
        estado="idle",
        estado_label="En espera",
        mensaje=mensaje,
        en_curso=False,
        ciclo_error="",
        step_id="",
        step_index=len(CYCLE_STEPS),
        progreso_pct=100,
        tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
        tiempo_corriendo_s=0,
    )
    _cycle_latch_reset()


def _enter_tray_wait(deposits: int) -> None:
    _LOT["tray_wait"] = True
    _set_machine_ui(
        estado="paused",
        estado_label="Charola llena",
        mensaje=f"Charola llena — {deposits} ciclos. Vacía la charola y confirma.",
        en_curso=True,
        step_id="",
        step_index=0,
        progreso_pct=100,
        tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
        tiempo_corriendo_s=0,
    )
    _push_event(f"Charola llena — {deposits} ciclos.", "warn")
    _cycle_latch_reset()


def _home_after_tray() -> bool:
    _LOT["home_before"] = False
    _set_machine_ui(
        estado="running",
        estado_label="Ciclo en marcha",
        mensaje="Charola vacía. Regresando el eje a cero.",
        en_curso=True,
    )
    sent = _cycle_motion_send({"op": "lmguide_move", "target": "home"}, timeout_s=40.0)
    if sent.get("sent"):
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


def _run_one_piece(start_index: int) -> bool:
    global _LAST_PIECE_S
    params = _load_cycle_params()
    t0 = time.time()
    for idx in range(start_index, len(CYCLE_STEPS)):
        step = CYCLE_STEPS[idx]
        if _cycle_should_stop():
            _set_machine_ui(
                estado="stopped",
                estado_label="Detenido",
                mensaje="Stop — detenido en el paso actual.",
                en_curso=True,
                step_id=step["id"],
                step_index=idx,
            )
            _push_event("Ciclo detenido (Stop).", "warn")
            return False
        _cycle_enter_step(step["id"], idx)
        if not _cycle_assert_preconditions(idx):
            return False
        ok = _cycle_run_step(step["id"], params)
        if not ok:
            if _MACHINE_STATE.get("ciclo_error"):
                return False
            if _cycle_should_stop():
                _set_machine_ui(
                    estado="stopped",
                    estado_label="Detenido",
                    mensaje="Stop — detenido en el paso actual.",
                    en_curso=True,
                    step_id=step["id"],
                    step_index=idx,
                )
                _push_event("Ciclo detenido (Stop).", "warn")
            return False
        _cycle_latch_mark_step_done(idx)
        elapsed = time.time() - t0
        _set_machine_ui(tiempo_corriendo_s=round(elapsed, 1))
        if _CYCLE_CTRL["pause_after_step"].is_set():
            _CYCLE_CTRL["pause_after_step"].clear()
            next_idx = idx + 1
            next_step = CYCLE_STEPS[next_idx]["id"] if next_idx < len(CYCLE_STEPS) else ""
            _set_machine_ui(
                estado="paused",
                estado_label="Pausa",
                mensaje="Ciclo en pausa. Reanudar para continuar.",
                en_curso=True,
                step_id=next_step,
                step_index=next_idx,
            )
            _push_event("Pausa — paso actual completado.", "warn")
            return False
    _LAST_PIECE_S = time.time() - t0
    return True


def _cycle_worker(start_index: int = 0) -> None:
    lot_mode = bool(_LOT["active"])
    index = start_index
    try:
        while True:
            if _LOT["home_before"] and not _home_after_tray():
                return
            if not _run_one_piece(index):
                return
            _mant_add_cycle()
            if not lot_mode:
                _push_event(f"Ciclo terminado en {_LAST_PIECE_S:.1f} s.")
                _finish_piece_idle("Ciclo completado. Listo para siguiente Start.")
                return
            if _cycle_should_stop() or not _LOT["active"]:
                if _cycle_should_stop():
                    _set_machine_ui(
                        estado="stopped",
                        estado_label="Detenido",
                        mensaje="Stop — lote detenido.",
                        en_curso=False,
                    )
                return
            _LOT["cycles_done"] += 1
            done = _LOT["cycles_done"]
            total = _LOT["cycles_total"]
            deposits = int(_LOT["deposits_per_tray"] or 0)
            if done >= total:
                _push_event(f"Lote terminado. {done} ciclos.")
                _finish_piece_idle(f"Lote terminado. {done} ciclos.")
                return
            if deposits and done % deposits == 0:
                _enter_tray_wait(deposits)
                return
            _cycle_latch_reset()
            _set_machine_ui(
                feed={"R": None, "L": None},
                tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
                tiempo_corriendo_s=0,
            )
            index = 0
    finally:
        _CYCLE_CTRL["pause_after_step"].clear()
        _CYCLE_CTRL["stop"].clear()
        _CYCLE_CTRL["thread"] = None


def _cycle_blocked_by_routine() -> str:
    if not (_MACHINE_STATE.get("checklist") or {}).get("ok"):
        return "Completa la revisión de arranque antes de Iniciar."
    if _MACHINE_STATE.get("carga_material"):
        return "Apaga la carga de material antes de Iniciar."
    if _MACHINE_STATE.get("rutina") == "purga":
        return "Hay una purga en curso."
    return _motion_owner_busy()


def _cycle_start() -> tuple[bool, str]:
    blocked = _cycle_blocked_by_routine()
    if blocked:
        return False, blocked
    if _LOT["tray_wait"]:
        return False, "La charola está llena. Vacía y confirma, o pulsa Stop."
    if _LOT["active"]:
        return False, "Hay un lote en curso. Reanuda o pulsa Stop."
    if not _MOTION_DATA.get("modelo_id") or _MOTION_DATA.get("target_length_mm") is None:
        return False, "Selecciona un modelo con longitud antes de Start."
    length = _target_length_mm()
    if length <= 0:
        return False, "Selecciona un modelo con longitud antes de Start."
    params = _load_cycle_params()
    sides = _feed_sides_of(params)
    cycles, qty_err = _order_cycles(_MACHINE_STATE.get("cantidad"), sides)
    if qty_err:
        return False, qty_err
    plan, plan_err = _tray_plan(length, params)
    if plan_err:
        _push_event(plan_err, "error")
        _set_machine_ui(mensaje=plan_err)
        return False, plan_err
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    _announce_trays(cycles, plan)
    _LOT["active"] = True
    _LOT["tray_wait"] = False
    _LOT["home_before"] = False
    _LOT["cycles_total"] = cycles
    _LOT["cycles_done"] = 0
    _LOT["stacks"] = plan["stacks"]
    _LOT["deposits_per_tray"] = plan["deposits"]
    _LOT["batch_size"] = plan["batch_size"]
    _LOT["cut_mm"] = plan["cut_mm"]
    _LOT["length_mm"] = plan["length_mm"]
    _LOT["gap_mm"] = plan["gap_mm"]
    _LOT["clearance_mm"] = plan["clearance_mm"]
    _LOT["sides"] = sides
    _CYCLE_CTRL["stop"].clear()
    _CYCLE_CTRL["pause_after_step"].clear()
    _cycle_latch_reset()
    _set_machine_ui(
        ciclo_error="",
        feed={"R": None, "L": None},
        step_index=0,
        progreso_pct=0,
        tiempo_ciclo_s=None,
        tiempo_corriendo_s=0,
    )
    th = threading.Thread(target=_cycle_worker, args=(0,), daemon=True)
    _CYCLE_CTRL["thread"] = th
    th.start()
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
        estado_label="Ciclo en marcha",
        mensaje="Pausa solicitada. Terminando el paso en curso…",
    )
    _push_event("Pausa solicitada — se detendrá al completar el paso.", "warn")


def _cycle_resume() -> tuple[bool, str]:
    if _LOT["tray_wait"]:
        return False, "La charola está llena. Vacía y confirma."
    if _MACHINE_STATE.get("ciclo_error"):
        return False, "Hay un error activo. Usa Reset cuando la condición se haya corregido."
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "Hay una purga en curso."
    if _MACHINE_STATE.get("carga_material"):
        return False, "Apaga la carga de material antes de reanudar."
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    idx = int(_MACHINE_STATE.get("ciclo_paso_indice") or 0)
    if idx <= 0 and not _MACHINE_STATE.get("ciclo_en_curso"):
        return _cycle_start()
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    if idx >= len(CYCLE_STEPS):
        idx = len(CYCLE_STEPS) - 1
    _CYCLE_CTRL["stop"].clear()
    _CYCLE_CTRL["pause_after_step"].clear()
    th = threading.Thread(target=_cycle_worker, args=(idx,), daemon=True)
    _CYCLE_CTRL["thread"] = th
    th.start()
    _set_machine_ui(estado="running", estado_label="Ciclo en marcha", mensaje="Reanudando secuencia.")
    _push_event("Ciclo reanudado.")
    return True, ""


def _charola_vacia(_data=None) -> tuple[bool, str]:
    if not _LOT["tray_wait"] or not _LOT["active"]:
        return False, "La charola no está en espera."
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    _LOT["tray_wait"] = False
    _LOT["home_before"] = True
    _CYCLE_CTRL["stop"].clear()
    _CYCLE_CTRL["pause_after_step"].clear()
    _cycle_latch_reset()
    th = threading.Thread(target=_cycle_worker, args=(0,), daemon=True)
    _CYCLE_CTRL["thread"] = th
    th.start()
    return True, "Charola vacía. El eje vuelve a cero y el siguiente corte sale del primer montón."


def _cycle_stop() -> None:
    was_tray = bool(_LOT["tray_wait"])
    running = bool(_CYCLE_CTRL["thread"] and _CYCLE_CTRL["thread"].is_alive())
    _LOT["active"] = False
    _LOT["tray_wait"] = False
    _LOT["home_before"] = False
    _CYCLE_CTRL["stop"].set()
    _CYCLE_CTRL["pause_after_step"].clear()
    _ROUTINE["stop"].set()
    if was_tray and not running:
        _set_machine_ui(
            estado="stopped",
            estado_label="Detenido",
            mensaje="Stop — lote detenido.",
            en_curso=False,
            step_id="",
            step_index=0,
        )
    elif _MACHINE_STATE.get("estado") in ("running", "error") or _MACHINE_STATE.get("rutina"):
        _set_machine_ui(mensaje="Stop — deteniendo de inmediato…")
    if _MACHINE_STATE.get("rutina") == "purga" and not (
        _ROUTINE["thread"] and _ROUTINE["thread"].is_alive()
    ):
        _purga_finish(True)
    _push_event("Stop solicitado — detención inmediata.", "warn")


def _cycle_reset_error() -> tuple[bool, str]:
    if not _MACHINE_STATE.get("ciclo_error"):
        return True, ""
    en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    _set_machine_ui(
        ciclo_error="",
        estado="paused" if en_curso else "idle",
        estado_label="Pausa" if en_curso else "En espera",
        mensaje="Error limpiado. Reanudar para continuar."
        if en_curso
        else "Sistema listo. Sin errores activos.",
    )
    _push_event("Reset de error — listo para reanudar." if en_curso else "Reset de error.")
    return True, ""


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
    _cycle_latch_reset()
    _set_machine_ui(
        estado="paused",
        estado_label="Pausa",
        mensaje=mensaje,
        en_curso=True,
        ciclo_error="",
        step_id="1",
        step_index=0,
        progreso_pct=0,
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


def _feed_once(side: str | None):
    payload = {"op": "feed_stage"}
    if side in ("R", "L"):
        payload["side"] = side
    sent = _cycle_motion_send(payload, timeout_s=65.0)
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
    if not _cycle_motion_send(_motion_tune_flat(tune), timeout_s=2.0).get("sent"):
        return False
    if _ROUTINE["stop"].is_set():
        return False
    move = _cycle_motion_send({"op": "lmguide_move", "puu": _mm_to_puu(mm)}, timeout_s=40.0)
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
    if cancelled:
        _push_event("Purga cancelada.", "warn")
        msg = "Purga cancelada."
    else:
        if paso == 5:
            _checklist_update(purga_ok=True)
        _push_event("Purga terminada.")
        msg = "Purga terminada."
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
    if _ROUTINE["stop"].is_set() or not _routine_move_lm_mm(PURGE_PARK_MM):
        _purga_finish(True)
        return
    _set_routine_ui(
        rutina_fase="espera",
        mensaje=(
            f"Eje en parque ({PURGE_PARK_MM:.0f} mm). "
            "Alimentar y Limpiar cierran holder y encoder. "
            "Continuar corta. Terminar cierra la purga."
        ),
    )


def _purga_start(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "Ya hay una purga en curso."
    if _MACHINE_STATE.get("estado") == "running":
        return False, "Pausa o detén el ciclo antes de la purga."
    ready, err = _require_links()
    if not ready:
        return False, err
    _ROUTINE["from_lote"] = bool(_MACHINE_STATE.get("ciclo_en_curso"))
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
        _push_event(error_codes.format_message(code or "E010"), "error")
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
    _set_routine_ui(mensaje="Alimentación: láser y encoder.")
    ok, code, _feed = _feed_once(None)
    if _ROUTINE["stop"].is_set():
        _purga_finish(True)
        return
    if not ok:
        _push_event(error_codes.format_message(code), "error")
        _set_routine_ui(
            rutina_fase="espera",
            mensaje="La alimentación no validó. Puedes repetir, continuar o terminar.",
        )
        return
    _set_routine_ui(
        rutina_fase="espera",
        mensaje="Alimentación lista. Continuar corta. Terminar cierra la purga.",
    )


def _purge_cut() -> bool:
    params = _load_cycle_params()
    length = _target_length_mm()
    _set_routine_ui(
        rutina_fase="corte",
        rutina_manos=True,
        estado_label="Purga",
        mensaje="NO METAS LAS MANOS",
    )

    def sent_ok(sent) -> bool:
        return bool(sent.get("sent")) and not _ROUTINE["stop"].is_set()

    if not sent_ok(_cycle_plc_send({"op": "valve", "name": "gripper", "on": True})):
        return False
    if not sent_ok(_cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": False})):
        return False

    if length >= 1.0:
        if not sent_ok(_cycle_plc_send({"op": "valve", "name": "cutters", "on": True})):
            return False
        if not _routine_sleep_ms(int(params["ms_cutter_on"])):
            return False
        if not sent_ok(_cycle_plc_send({"op": "valve", "name": "cutters", "on": False})):
            return False
        if not _routine_sleep_ms(PURGE_AFTER_CUT_MS):
            return False
        pos = _lm_cut_and_discharge(params)
        if pos is not None:
            _cut_mm, disc = pos
            tune = _load_motion_tune()
            puu = _mm_to_puu(disc)
            if not sent_ok(_cycle_motion_send(_motion_tune_flat(tune), timeout_s=2.0)):
                return False
            move = _cycle_motion_send({"op": "lmguide_move", "puu": puu}, timeout_s=40.0)
            if not _motion_cmd_ok(move, "lmguide_move"):
                return False
            if not _routine_sleep_ms(int(params["ms_lm_at_discharge"])):
                return False
        else:
            _push_event("Sin longitud suficiente para el depósito. El eje se queda en cero.", "warn")
        if not sent_ok(_cycle_plc_send({"op": "valve", "name": "gripper", "on": False})):
            return False
        if not _routine_sleep_ms(int(params["ms_after_grip_release"])):
            return False
    else:
        _push_event("Sin longitud de pieza: la purga no corta.", "warn")

    home = _cycle_motion_send({"op": "lmguide_move", "target": "home"}, timeout_s=40.0)
    if not _motion_cmd_ok(home, "lmguide_move"):
        return False
    if length >= 1.0:
        if not _routine_sleep_ms(int(params["ms_blower_delay"])):
            return False
        blower_ms = _cycle_blower_duration_ms(params)
        if not sent_ok(_cycle_plc_send({"op": "blower_on", "ms": blower_ms})):
            return False
        if not _routine_sleep_ms(blower_ms):
            return False
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
    _set_routine_ui(mensaje="Limpiar: alimentando 3 s sin mirar el láser.")
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
        mensaje="Limpieza lista. Continuar corta. Terminar cierra la purga.",
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


def _alimentar_lado(side: str):
    def run():
        _alimentar_run(side)

    if _MACHINE_STATE.get("rutina"):
        return False, "Hay una rutina en curso."
    if _MACHINE_STATE.get("estado") == "running":
        return False, "Hay un ciclo en marcha."
    if _target_length_mm() <= 0:
        return False, "Selecciona un modelo con longitud."
    ready, err = _require_links()
    if not ready:
        return False, err
    ok, err = _start_routine(run)
    if not ok:
        return False, err
    return True, f"Alimentando lado {_SIDE_NAME[side]}."


def _alimentar_izquierda(data=None):
    return _alimentar_lado("L")


def _alimentar_derecha(data=None):
    return _alimentar_lado("R")


def _feeder_init_now() -> tuple[bool, str]:
    ok_link, code = _check_module_link("motion")
    if not ok_link:
        return False, error_codes.format_message(code)
    sent = _cycle_motion_send({"op": "feeder_init", "side": "both"}, timeout_s=8.0)
    if not _motion_cmd_ok(sent, "feeder_init"):
        return False, error_codes.format_message(sent.get("code") or "E043")
    _push_event("Alimentadores inicializados.")
    return True, "Alimentadores inicializados."


def _prealimentador_iniciar(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina"):
        return False, "Hay una rutina en curso."
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    return _feeder_init_now()


def _carga_material(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "No se puede cargar material con una purga en curso."
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    turning_on = not bool(_MACHINE_STATE.get("carga_material"))
    with _STATE_LOCK:
        _MACHINE_STATE["carga_material"] = turning_on
    if turning_on and _MACHINE_STATE.get("ciclo_en_curso"):
        _rearm_piece("Carga de material. El ciclo queda en pausa y la pieza vuelve al paso 1.")
    elif turning_on:
        _set_machine_ui(mensaje="Carga de material activa. Iniciar y Reanudar quedan bloqueados.")
    else:
        _set_machine_ui(mensaje="Carga de material apagada.")
    _push_event("Carga de material activa." if turning_on else "Carga de material apagada.")
    return True, "Carga de material activa." if turning_on else "Carga de material apagada."


def _omitir_holgura(side: str, data) -> tuple[bool, str]:
    empleado = _empleado(data.get("empleado"))
    if not empleado:
        return False, "Hace falta el número de empleado, cinco dígitos."
    key = "omitir_holgura_l" if side == "L" else "omitir_holgura_r"
    with _STATE_LOCK:
        nxt = not bool(_MACHINE_STATE.get(key))
        _MACHINE_STATE[key] = nxt
    nombre = _SIDE_NAME[side]
    if nxt:
        _push_event(f"Holgura {nombre} omitida. Empleado {empleado}.")
        return True, f"Holgura {nombre} omitida."
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
        tune_sent = _cycle_motion_send(_motion_tune_flat(tune), timeout_s=2.0)
        move = _cycle_motion_send({"op": "lmguide_move", "puu": puu}, timeout_s=40.0)
        if _ROUTINE["stop"].is_set() or not tune_sent.get("sent") or not _motion_cmd_ok(move, "lmguide_move"):
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
    expected = _target_length_mm() + CHECKLIST_EXTRA_MM
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


_ROUTINE_ACTIONS = {
    "purga": _purga_start,
    "purga_alimentar": _purga_alimentar_action,
    "purga_limpiar": _purga_limpiar_action,
    "purga_continuar": _purga_continuar_action,
    "purga_terminar": _purga_terminar_action,
    "posicion_inicial": _posicion_inicial,
    "carga_material": _carga_material,
    "iniciar": _prealimentador_iniciar,
    "alimentar_izquierda": _alimentar_izquierda,
    "alimentar_derecha": _alimentar_derecha,
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
        return jsonify({"ok": True, "state": _public_machine_state(), "motion_data": dict(_MOTION_DATA)})

    entry = _find_modelo(modelo_id)
    if not entry:
        return jsonify({"ok": False, "error": "Modelo no encontrado"}), 404

    _MACHINE_STATE["numero_modelo"] = entry["modelo"]
    _MACHINE_STATE["longitud_modelo_mm"] = entry.get("longitud_mm")
    _MOTION_DATA["modelo_id"] = entry["modelo"]
    _MOTION_DATA["target_length_mm"] = entry.get("longitud_mm")
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
    return jsonify({"ok": True, "state": _public_machine_state()})


@app.route("/api/state", methods=["GET"])
def get_state():
    return jsonify({"ok": True, **_public_machine_state()})


@app.route("/api/errors", methods=["GET"])
def api_errors():
    return jsonify({"ok": True, "catalog": error_codes.catalog_for_api()})


@app.route("/api/cycle", methods=["GET", "POST"])
def cycle_config():
    if request.method == "GET":
        params = _load_cycle_params()
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
    for key in (
        "cut_offset_mm",
        "deposit_stack_gap_mm",
        "deposit_max_travel_mm",
        "gripper_clearance_mm",
        "tolerance_mm",
        "blower_ms_per_mm",
    ):
        params[key] = _num(params.get(key), _DEFAULT_CYCLE_PARAMS[key])
    params["deposit_batch_size"] = int(_num(params.get("deposit_batch_size"), 50))
    try:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        CYCLE_PATH.write_text(
            json.dumps(params, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar cycle.json"}), 500
    return jsonify({"ok": True, "steps": list(CYCLE_STEPS), "params": params})


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
            if block.get("mm_off") is not None:
                item["mm"] = block.get("mm_off")
            if "laser" in block:
                item["laser"] = bool(block.get("laser"))
            if item:
                reading[side] = item
    return reading


@app.route("/api/motion/status", methods=["GET"])
def motion_status():
    sent = _send_motion_serial(['{"op":"status"}'])
    return jsonify(
        {
            "ok": True,
            "sent": bool(sent.get("sent")),
            "encoder": _encoder_from_status(sent.get("replies")),
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

    try:
        tune, lines = _build_motion_lines(op, tune, body)
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400

    if isinstance(patch, dict) or op in ("set_tune", "lmguide_move"):
        try:
            _save_motion_tune(tune)
        except OSError:
            return jsonify({"ok": False, "error": "No se pudo guardar Motion.json"}), 500

    sent = _send_motion_serial(lines)
    return jsonify(
        {
            "ok": True,
            "op": op,
            "tune": tune,
            "lines": lines,
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
            _MACHINE_STATE["compensacion_medicion_mm"] = float(patch["measure_comp_mm"])
        except (TypeError, ValueError):
            _MACHINE_STATE["compensacion_medicion_mm"] = 0.0
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
    packed = _compact(line)
    sent = _send_plc_serial([packed])
    status = _plc_status_after_command(sent)
    if sent.get("sent") and op in ("valve", "blower_on", "blower_off", "all_off", "reset"):
        status = _plc_wait_ready(2.5) or status
    return jsonify(
        {
            "ok": True,
            "op": op,
            "lines": [packed],
            "status": status,
            "active_errors": _plc_active_errors(status),
            **sent,
        }
    )


@app.route("/api/serial", methods=["GET", "POST"])
def serial_ports():
    """Puertos COM elegidos en la pantalla."""
    if request.method == "GET":
        return jsonify(
            {
                "ok": True,
                "ports": _list_com_ports(),
                "motion": _serial_port("motion"),
                "plc": _serial_port("plc"),
                "andon": _serial_port("andon"),
            }
        )

    body = request.get_json(silent=True) or {}
    listed = _list_com_ports()
    motion_raw = str(body.get("motion") or "").strip()
    plc_raw = str(body.get("plc") or "").strip()
    andon_raw = str(body.get("andon") or "").strip()
    motion = _match_listed_port(motion_raw, listed)
    plc = _match_listed_port(plc_raw, listed)
    andon = _match_listed_port(andon_raw, listed)
    if motion_raw and not motion:
        return jsonify({"ok": False, "error": "El puerto de Motion no está conectado."}), 400
    if plc_raw and not plc:
        return jsonify({"ok": False, "error": "El puerto del PLC no está conectado."}), 400
    if andon_raw and not andon:
        return jsonify({"ok": False, "error": "El puerto del Andon no está conectado."}), 400
    chosen = [port for port in (motion, plc, andon) if port]
    if len(chosen) != len({port.upper() for port in chosen}):
        return jsonify({"ok": False, "error": "Cada módulo tiene que usar un COM distinto."}), 400

    try:
        _save_serial_cfg(motion, plc, andon)
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar el puerto."}), 500

    return jsonify({"ok": True, "motion": motion, "plc": plc, "andon": andon})


@app.route("/api/serial/test", methods=["POST"])
def serial_module_test():
    """Comprueba que el COM elegido responde y es el MCU pedido. Andon incluido."""
    body = request.get_json(silent=True) or {}
    which = str(body.get("module") or "").strip().lower()
    if which not in ("motion", "plc", "andon"):
        return jsonify({"ok": False, "error": "Módulo no reconocido."}), 400
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
            if err:
                _set_machine_ui(mensaje=err)
            return jsonify({"ok": False, "error": err, "state": _public_machine_state()}), 400
        message = err or "Listo."
    elif block == "proceso":
        if action_name == "start":
            ok, err = _cycle_start()
            if not ok:
                return jsonify({"ok": False, "error": err, "state": _public_machine_state()}), 400
            message = "Lote iniciado."
        elif action_name == "pause":
            _cycle_pause()
            message = "Pausa solicitada."
        elif action_name == "resume":
            ok, err = _cycle_resume()
            if not ok:
                return jsonify({"ok": False, "error": err, "state": _public_machine_state()}), 400
            message = "Reanudar."
        elif action_name == "stop":
            _cycle_stop()
            message = "Stop solicitado."
        elif action_name == "reset":
            ok, err = _cycle_reset_error()
            if not ok:
                return jsonify({"ok": False, "error": err, "state": _public_machine_state()}), 400
            message = "Reset de error."
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


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=5000)

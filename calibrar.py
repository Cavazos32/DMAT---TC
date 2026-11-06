"""Calibración de banco para encoder, feeder y LM Guide.

Habla con el MCU Motion por el mismo serial JSON que usa la aplicación.
No escribe Parameters.h ni app.py: el resultado queda en data/calibracion.json.

Escalas de fábrica que este archivo invierte o compara. Tienen que coincidir
con el firmware actual:

- Encoder: PARAM_ENC_COUNTS_PER_100MM = 1273. El status no trae cuentas;
  trae mm_live, ya convertido. Las cuentas se recuperan con esa escala.
  mm_off no sirve: está redondeado a 0.5 mm.
- Feeder: cada jog son PARAM_FEED_JOG_STEP_COUNTS = 5000 cuentas.
  La escala de fábrica es 3600 / 55 pasos por mm.
- LM Guide: la escala de fábrica es 770 / 45 PUU por mm. El signo de trabajo
  sigue en PUU negativo hacia el corte; aquí solo se calibra la magnitud.

Uso:
  python calibrar.py
  python calibrar.py --cli
  python calibrar.py --check
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

_HERE = Path(__file__).resolve().parent
ROOT = _HERE if (_HERE / "data").is_dir() else _HERE.parent
OUT_PATH = ROOT / "data" / "calibracion.json"

FACTORY_ENC_COUNTS_PER_100MM = 1273
FACTORY_FEED_STEPS = 3600.0
FACTORY_FEED_MM = 55.0
FACTORY_FEED_STEPS_PER_MM = FACTORY_FEED_STEPS / FACTORY_FEED_MM
FEED_JOG_COUNTS = 5000
FACTORY_LM_PUU = 770.0
FACTORY_LM_MM = 45.0
FACTORY_LM_STEPS_PER_MM = FACTORY_LM_PUU / FACTORY_LM_MM
LM_PUU_SIGN = -1

# Misma conversión que app.py: LMGUIDE_STEPS_PER_MM, LMGUIDE_PUU_SIGN, FEED_STEPS_PER_MM.
APP_LM_STEPS_PER_MM = 770.0 / 45.0
APP_LM_PUU_SIGN = -1
APP_FEED_STEPS_PER_MM = 3600.0 / 55.0

LINK = None


class MotionError(Exception):
    pass


def counts_from_mm_live(mm_live, counts_per_100mm=FACTORY_ENC_COUNTS_PER_100MM):
    """Invierte mm_live del firmware. Solo vale con la escala que el MCU tiene ahora."""
    return float(mm_live) * (float(counts_per_100mm) / 100.0)


def pool_ratio(samples, unit_key):
    units = 0.0
    mm = 0.0
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        try:
            unit = abs(float(sample[unit_key]))
            length = float(sample["measured_mm"])
        except (KeyError, TypeError, ValueError):
            continue
        if unit == 0 or length <= 0:
            continue
        units += unit
        mm += length
    if mm <= 0:
        return None
    return units / mm


def encoder_scale(samples):
    per_mm = pool_ratio(samples, "counts")
    if per_mm is None:
        return None
    return per_mm * 100.0


def mm_to_lm_puu(mm, steps_per_mm=None):
    """Igual que app.py _mm_to_puu: signo negativo hacia el corte."""
    scale = APP_LM_STEPS_PER_MM if steps_per_mm is None else float(steps_per_mm)
    return int(round(APP_LM_PUU_SIGN * float(mm) * scale))


def mm_to_feed_counts(mm, steps_per_mm=None):
    scale = APP_FEED_STEPS_PER_MM if steps_per_mm is None else float(steps_per_mm)
    return int(round(float(mm) * scale))


def point_units(point):
    """PUU o cuentas que corresponden al mm real. Si el drive marcó otro valor, manda ese."""
    if not isinstance(point, dict):
        return None
    marked = point.get("marked_units")
    if marked not in (None, ""):
        try:
            return abs(float(marked))
        except (TypeError, ValueError):
            return None
    try:
        return abs(float(point["sent_units"]))
    except (KeyError, TypeError, ValueError):
        return None


def fit_steps_per_mm(points):
    """|unidades| = pasos_por_mm × mm reales, con el cero en home."""
    num = 0.0
    den = 0.0
    ratios = []
    for point in points or []:
        try:
            mm = float(point["measured_mm"])
        except (KeyError, TypeError, ValueError):
            continue
        units = point_units(point)
        if units is None or mm <= 0 or units <= 0:
            continue
        num += units * mm
        den += mm * mm
        ratios.append(units / mm)
    if den <= 0:
        return None
    return {"steps_per_mm": num / den, "n": len(ratios), "ratios": ratios}


def feeder_scale(samples):
    return pool_ratio(samples, "commanded_counts")


def lm_scale(samples):
    return pool_ratio(samples, "delta_puu")


def fmt(value):
    text = f"{float(value):.4f}".rstrip("0").rstrip(".")
    return text if text else "0"


def fmt_pct(new, old):
    return f"{((float(new) - float(old)) / float(old)) * 100.0:+.1f} %"


def now_stamp():
    return datetime.now().isoformat(timespec="seconds")


def confirm(text, default=False):
    hint = "S/n" if default else "s/N"
    raw = input(f"{text} ({hint}): ").strip().lower()
    if not raw:
        return default
    return raw in {"s", "si", "sí", "y", "yes"}


def ask_float(text, default=None):
    while True:
        raw = input(text).strip().replace(",", ".")
        if not raw and default is not None:
            return float(default)
        try:
            value = float(raw)
        except ValueError:
            print("Escribe un número.")
            continue
        if value != value:
            print("Escribe un número.")
            continue
        return value


def ask_side():
    while True:
        raw = input("Lado (R/L): ").strip().upper()
        if raw in {"R", "L"}:
            return raw
        print("Escribe R o L.")


def ask_measured():
    while True:
        raw = input("Milímetros medidos con la cinta (c cancela): ").strip().lower().replace(",", ".")
        if raw in {"c", "cancelar"}:
            return None
        try:
            mm = float(raw)
        except ValueError:
            print("Escribe un número.")
            continue
        if mm != mm or mm <= 0:
            print("La medida tiene que ser mayor que cero.")
            continue
        if mm < 50 and not confirm("Menos de 50 mm deja mucha incertidumbre. ¿La uso igual?", default=False):
            continue
        return mm


def empty_store():
    return {
        "updated": None,
        "firmware_no_modificado": True,
        "nota": "Registro de banco. Estas escalas no están escritas en el firmware ni en app.py.",
        "encoder": {
            "factory_counts_per_100mm": FACTORY_ENC_COUNTS_PER_100MM,
            "R": {"samples": []},
            "L": {"samples": []},
        },
        "feeder": {
            "factory_steps_per_mm": round(FACTORY_FEED_STEPS_PER_MM, 6),
            "jog_step_counts": FEED_JOG_COUNTS,
            "points": [],
            "R": {"samples": []},
            "L": {"samples": []},
        },
        "lmguide": {
            "factory_steps_per_mm": round(FACTORY_LM_STEPS_PER_MM, 6),
            "puu_sign": LM_PUU_SIGN,
            "points": [],
            "samples": [],
        },
    }


def load_store():
    base = empty_store()
    if not OUT_PATH.exists():
        return base
    try:
        raw = json.loads(OUT_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        print("No pude leer data/calibracion.json. Empiezo un registro nuevo.")
        return base
    if not isinstance(raw, dict):
        return base
    for section in ("encoder", "feeder"):
        block = raw.get(section)
        if not isinstance(block, dict):
            continue
        for side in ("R", "L"):
            side_block = block.get(side)
            if isinstance(side_block, dict) and isinstance(side_block.get("samples"), list):
                base[section][side]["samples"] = side_block["samples"]
    feed = raw.get("feeder")
    if isinstance(feed, dict) and isinstance(feed.get("points"), list):
        base["feeder"]["points"] = feed["points"]
    lm = raw.get("lmguide")
    if isinstance(lm, dict) and isinstance(lm.get("samples"), list):
        base["lmguide"]["samples"] = lm["samples"]
    if isinstance(lm, dict) and isinstance(lm.get("points"), list):
        base["lmguide"]["points"] = lm["points"]
    if raw.get("updated"):
        base["updated"] = raw["updated"]
    return base


def refresh_derived(store):
    for side in ("R", "L"):
        enc_samples = store["encoder"][side].get("samples") or []
        enc_value = encoder_scale(enc_samples)
        store["encoder"][side]["counts_per_100mm"] = round(enc_value, 4) if enc_value is not None else None
        feed_samples = store["feeder"][side].get("samples") or []
        feed_value = feeder_scale(feed_samples)
        store["feeder"][side]["steps_per_mm"] = round(feed_value, 6) if feed_value is not None else None
    lm_samples = store["lmguide"].get("samples") or []
    lm_value = lm_scale(lm_samples)
    store["lmguide"]["steps_per_mm"] = round(lm_value, 6) if lm_value is not None else None


def save_store(store, out=print):
    refresh_derived(store)
    store["updated"] = now_stamp()
    store["firmware_no_modificado"] = True
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(store, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    out(f"Guardado en {OUT_PATH}")


def merge_samples(old, new):
    if not old:
        return list(new)
    print(f"Ya hay {len(old)} muestras guardadas.")
    raw = input("Enter agrega las nuevas, r reemplaza: ").strip().lower()
    if raw == "r":
        return list(new)
    return list(old) + list(new)


def warn_sides(left, right, name, out=print):
    if left is None or right is None:
        if (left is None) != (right is None):
            out(f"El firmware usa una sola escala de {name} para R y L. Falta el otro lado.")
        return
    base = max(abs(left), abs(right))
    if base <= 0:
        return
    rel = abs(left - right) / base
    if rel > 0.02:
        out(
            f"Aviso: {name} R y L difieren {rel * 100:.1f} %. "
            "El firmware usa una sola escala para los dos lados."
        )


def report_encoder(store, out=print):
    out("Encoder — cuentas por 100 mm")
    out(f"  Fábrica: {FACTORY_ENC_COUNTS_PER_100MM}")
    values = {}
    for side in ("R", "L"):
        block = store["encoder"][side]
        value = block.get("counts_per_100mm")
        count = len(block.get("samples") or [])
        values[side] = value
        if value is None:
            out(f"  {side}: sin muestras")
        else:
            out(
                f"  {side}: {fmt(value)}  "
                f"({count} muestras, {fmt_pct(value, FACTORY_ENC_COUNTS_PER_100MM)} vs fábrica)"
            )
    warn_sides(values["R"], values["L"], "encoder", out=out)
    out("  El firmware sigue con PARAM_ENC_COUNTS_PER_100MM = 1273 hasta que se autorice el cambio.")


def _report_point_fit(store, section, unit, current, out):
    fit = fit_steps_per_mm(store.get(section, {}).get("points"))
    if not fit:
        out("  Sin puntos de calibración.")
        return
    new = fit["steps_per_mm"]
    out(f"  Ajuste con {fit['n']} puntos: {fmt(new)} {unit}  ({fmt_pct(new, current)} vs la constante actual)")
    ratios = fit["ratios"]
    if len(ratios) >= 2 and min(ratios) > 0 and (max(ratios) - min(ratios)) / min(ratios) > 0.02:
        out("  Los puntos no caen en la misma recta. Conviene repetir el que se salga.")
    out("  Esta corrección no está escrita en app.py.")


def report_feeder(store, out=print):
    out("Feeder — cuentas por mm")
    out(f"  Constante actual: {fmt(APP_FEED_STEPS_PER_MM)}  (3600 / 55)")
    _report_point_fit(store, "feeder", "cuentas/mm", APP_FEED_STEPS_PER_MM, out)
    values = {}
    for side in ("R", "L"):
        block = store["feeder"][side]
        value = block.get("steps_per_mm")
        count = len(block.get("samples") or [])
        values[side] = value
        if value is None:
            out(f"  {side}: sin muestras")
        else:
            out(
                f"  {side}: {fmt(value)}  "
                f"({count} muestras, {fmt_pct(value, FACTORY_FEED_STEPS_PER_MM)} vs fábrica)"
            )
    warn_sides(values["R"], values["L"], "feeder", out=out)
    out("  El firmware sigue con 3600 / 55 hasta que se autorice el cambio.")


def report_lm(store, out=print):
    out("LM Guide — PUU por mm")
    out(f"  Constante actual: {fmt(APP_LM_STEPS_PER_MM)}  (770 / 45), signo {APP_LM_PUU_SIGN}")
    _report_point_fit(store, "lmguide", "PUU/mm", APP_LM_STEPS_PER_MM, out)
    fit = fit_steps_per_mm(store.get("lmguide", {}).get("points"))
    if fit:
        mm_for_770 = FACTORY_LM_PUU / fit["steps_per_mm"]
        out(f"  Equivalente: 770 PUU en {fmt(mm_for_770)} mm.")
    out("  El signo hacia el corte sigue negativo.")


def report_all(store, out=print):
    when = store.get("updated") or "aún no hay registro"
    out(f"Registro: {when}")
    out(store.get("nota") or "")
    report_encoder(store, out=out)
    out("")
    report_feeder(store, out=out)
    out("")
    report_lm(store, out=out)


class MotionLink:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None

    def open(self):
        try:
            import serial
        except ImportError as exc:
            raise MotionError("Falta pyserial. Instálalo con: pip install pyserial") from exc
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baud
        ser.timeout = 0.05
        ser.write_timeout = 2
        ser.dtr = False
        ser.rts = False
        try:
            ser.open()
        except serial.SerialException as exc:
            raise MotionError(
                f"No pude abrir {self.port}. Si la aplicación web tiene ese puerto, ciérrala. {exc}"
            ) from exc
        time.sleep(0.15)
        ser.reset_input_buffer()
        self.ser = ser

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def _readline(self, deadline):
        buf = bytearray()
        while time.time() < deadline:
            chunk = self.ser.read(1)
            if not chunk:
                continue
            if chunk == b"\n":
                return buf.decode("utf-8", "replace").strip()
            if chunk != b"\r":
                buf.extend(chunk)
        return ""

    def command(self, payload, timeout):
        if self.ser is None or not self.ser.is_open:
            raise MotionError("El serial no está abierto.")
        line = json.dumps(payload, separators=(",", ":"))
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()
        want = payload["op"]
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self._readline(deadline)
            if not raw:
                continue
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if isinstance(msg, dict) and msg.get("op") in {want, "error"}:
                if msg.get("op") == "error":
                    raise MotionError(msg.get("msg") or "El MCU rechazó la orden.")
                return msg
        raise TimeoutError(f"Sin respuesta a {want}.")


def timeout_for(op):
    if op == "lmguide_home":
        return 40.0
    if op in {"feeder_init", "lmguide_servo_on"}:
        return 8.0
    return 2.0


def call(link, payload):
    return link.command(payload, timeout_for(payload["op"]))


def ensure_link(args):
    global LINK
    if LINK is not None and LINK.ser is not None and LINK.ser.is_open:
        return LINK
    port = (args.port or os.environ.get("MOTION_SERIAL", "")).strip()
    if not port:
        port = input("Puerto serial del MCU Motion (ejemplo COM3): ").strip()
    if not port:
        raise MotionError("Falta el puerto serial.")
    baud = int(os.environ.get("MOTION_SERIAL_BAUD", "115200"))
    link = MotionLink(port, baud)
    link.open()
    LINK = link
    print(f"Serial abierto en {port} a {baud} baud.")
    return link


def stop_axes():
    if LINK is None or LINK.ser is None or not LINK.ser.is_open:
        return
    for op in ("feeder_stop", "lmguide_stop"):
        try:
            LINK.command({"op": op}, 2.0)
        except Exception:
            pass


def read_mm_live(link, side):
    time.sleep(0.3)
    msg = call(link, {"op": "status"})
    block = msg.get("enc_" + side)
    if not isinstance(block, dict) or block.get("mm_live") is None:
        raise MotionError(f"El status no trajo mm_live del lado {side}.")
    return float(block["mm_live"])


def feeder_is_ready(link, side):
    msg = call(link, {"op": "status"})
    key = "feeder_r_ready" if side == "R" else "feeder_l_ready"
    return bool(msg.get(key))


def prepare_feeder(link, side):
    print("Inicializo el feeder de ese lado.")
    call(link, {"op": "feeder_init", "side": side})
    if not feeder_is_ready(link, side):
        raise MotionError("El feeder no quedó listo. Revisa el CAN antes de calibrar.")


def jog_feeder(link, side):
    print("Cada jog avanza 5000 cuentas en el sentido de alimentación de este lado.")
    print("Enter manda un jog. u quita el último si el tubo no se movió. f pasa a medir.")
    sent = 0
    while True:
        raw = input("> ").strip().lower()
        if raw in {"f", "fin", "q"}:
            break
        if raw in {"u", "deshacer"}:
            if sent > 0:
                sent -= 1
            print(f"Jogs contados: {sent}. Cuentas pedidas: {sent * FEED_JOG_COUNTS}.")
            continue
        if raw not in {"", "j", "s"}:
            print("Enter para otro jog, u para quitar el último, f para medir.")
            continue
        msg = call(link, {"op": "feeder_jog", "side": side, "dir": 1})
        if msg.get("ok") is False:
            print("El MCU no aceptó el jog.")
            continue
        sent += 1
        print(f"Jogs aceptados: {sent}. Cuentas pedidas: {sent * FEED_JOG_COUNTS}.")
    return sent * FEED_JOG_COUNTS


def take_encoder_serial(link, side):
    call(link, {"op": "encoder_set0", "side": side})
    input("Cero puesto. Avanza el material y pulsa Enter cuando esté quieto. ")
    mm_live = read_mm_live(link, side)
    print(f"mm_live del lado {side}: {fmt(mm_live)}")
    if abs(mm_live) < 0.05:
        print("El encoder no se movió. Esta muestra no sirve.")
        return None
    measured = ask_measured()
    if measured is None:
        return None
    counts = abs(counts_from_mm_live(mm_live))
    return {
        "at": now_stamp(),
        "source": "serial",
        "mm_live": round(mm_live, 4),
        "factory_counts_per_100mm": FACTORY_ENC_COUNTS_PER_100MM,
        "counts": round(counts, 4),
        "measured_mm": measured,
    }


def take_encoder_manual(side):
    print(f"Lado {side}. Escribe las cuentas que avanzó el encoder, no el milímetro ya convertido.")
    counts = ask_float("Cuentas: ")
    if counts == 0:
        print("Cero cuentas no calibran.")
        return None
    measured = ask_measured()
    if measured is None:
        return None
    return {
        "at": now_stamp(),
        "source": "manual",
        "counts": round(abs(counts), 4),
        "measured_mm": measured,
    }


def take_feeder_serial(link, side):
    print("Marca el tubo ahora, con el eje ya quieto.")
    input("Marca hecha. Enter para empezar los jog. ")
    commanded = jog_feeder(link, side)
    if commanded <= 0:
        print("No hubo avance comandado.")
        return None
    measured = ask_measured()
    if measured is None:
        return None
    return {
        "at": now_stamp(),
        "source": "serial",
        "jog_step_counts": FEED_JOG_COUNTS,
        "commanded_counts": commanded,
        "measured_mm": measured,
    }


def take_feeder_manual(side):
    print(f"Lado {side}. Escribe las cuentas que se le pidieron al servo, y lo que midió la cinta.")
    commanded = ask_float("Cuentas comandadas: ")
    if commanded == 0:
        print("Cero cuentas no calibran.")
        return None
    measured = ask_measured()
    if measured is None:
        return None
    return {
        "at": now_stamp(),
        "source": "manual",
        "commanded_counts": round(abs(commanded), 4),
        "measured_mm": measured,
    }


def take_lm_serial(link):
    print("Se mide el desplazamiento de este movimiento, no la posición absoluta.")
    print("Un delta negativo va hacia el corte.")
    if confirm("¿Enciendo el servo del LM Guide?", default=True):
        msg = call(link, {"op": "lmguide_servo_on"})
        if msg.get("ok") is False:
            raise MotionError("El servo del LM Guide no encendió.")
    elif not confirm("Sin servo ON el eje no se mueve. ¿Sigo de todos modos?", default=False):
        return None
    delta = ask_float("Delta PUU [-770]: ", default=-FACTORY_LM_PUU)
    if delta == 0:
        print("El delta no puede ser cero.")
        return None
    delta = int(round(delta))
    print(f"Voy a pedir {delta} PUU.")
    if not confirm("¿Envío el movimiento?", default=True):
        return None
    msg = call(link, {"op": "lmguide_jog", "delta_puu": delta})
    if msg.get("ok") is False:
        raise MotionError("El drive no aceptó el jog.")
    input("Espera a que el eje se detenga y pulsa Enter. ")
    measured = ask_measured()
    if measured is None:
        return None
    return {
        "at": now_stamp(),
        "source": "serial",
        "delta_puu": delta,
        "measured_mm": measured,
    }


def take_lm_manual():
    delta = ask_float("Delta PUU comandado: ")
    if delta == 0:
        print("El delta no puede ser cero.")
        return None
    measured = ask_measured()
    if measured is None:
        return None
    return {
        "at": now_stamp(),
        "source": "manual",
        "delta_puu": int(round(delta)),
        "measured_mm": measured,
    }


def collect(take_one):
    samples = []
    while True:
        sample = take_one()
        if sample is not None:
            samples.append(sample)
            print("Muestra anotada.")
        else:
            print("Muestra cancelada.")
        if not confirm("¿Otra muestra?", default=False):
            break
    return samples


def store_side(section, side, samples):
    if not samples:
        print("No hubo muestras nuevas.")
        return
    store = load_store()
    block = store[section][side]
    block["samples"] = merge_samples(block.get("samples") or [], samples)
    save_store(store)
    if section == "encoder":
        report_encoder(store)
    else:
        report_feeder(store)


def store_lm(samples):
    if not samples:
        print("No hubo muestras nuevas.")
        return
    store = load_store()
    store["lmguide"]["samples"] = merge_samples(store["lmguide"].get("samples") or [], samples)
    save_store(store)
    report_lm(store)


def run_encoder(args):
    side = ask_side()
    print(
        "Con serial, las cuentas se recuperan invirtiendo mm_live "
        f"con la escala de fábrica ({FACTORY_ENC_COUNTS_PER_100MM} cuentas = 100 mm). "
        "Si el firmware ya usa otra escala, captura las cuentas a mano."
    )
    if confirm("¿Lo hago con el serial del MCU?", default=True):
        print("La cinta manda. El milímetro del monitor todavía usa la escala vieja.")
        link = ensure_link(args)
        samples = collect(lambda: take_encoder_serial(link, side))
    else:
        samples = collect(lambda: take_encoder_manual(side))
    store_side("encoder", side, samples)


def run_feeder(args):
    side = ask_side()
    if confirm("¿Lo hago con el serial del MCU?", default=True):
        print("La máquina va a alimentar tubo. Deja libre la zona.")
        print("Inicializar puede mover el feeder hasta la posición guardada. Espera a que pare antes de marcar.")
        link = ensure_link(args)
        prepare_feeder(link, side)
        samples = collect(lambda: take_feeder_serial(link, side))
    else:
        samples = collect(lambda: take_feeder_manual(side))
    store_side("feeder", side, samples)


def run_avance(args):
    side = ask_side()
    print("La máquina va a alimentar tubo. Deja libre la zona.")
    print(
        "Un mismo avance calibra feeder y encoder: las cuentas del jog contra la cinta, "
        "y el encoder contra esa misma cinta."
    )
    link = ensure_link(args)
    print(
        "El encoder se recupera invirtiendo mm_live con "
        f"{FACTORY_ENC_COUNTS_PER_100MM} cuentas = 100 mm."
    )
    print("Inicializar puede mover el feeder hasta la posición guardada. El cero se pone después, con el eje quieto.")
    prepare_feeder(link, side)
    samples_feed = []
    samples_enc = []
    while True:
        call(link, {"op": "encoder_set0", "side": side})
        print("Cero del encoder puesto. Marca el tubo en esta posición.")
        input("Enter para empezar los jog. ")
        commanded = jog_feeder(link, side)
        if commanded <= 0:
            print("No hubo avance comandado.")
        else:
            mm_live = read_mm_live(link, side)
            print(f"mm_live del lado {side}: {fmt(mm_live)}")
            measured = ask_measured()
            if measured is None:
                print("Muestra cancelada.")
            else:
                stamp = now_stamp()
                samples_feed.append(
                    {
                        "at": stamp,
                        "source": "serial",
                        "jog_step_counts": FEED_JOG_COUNTS,
                        "commanded_counts": commanded,
                        "measured_mm": measured,
                    }
                )
                if abs(mm_live) < 0.05:
                    print("El feeder se anotó. El encoder no se movió, así que ese lado no entra.")
                else:
                    samples_enc.append(
                        {
                            "at": stamp,
                            "source": "serial",
                            "mm_live": round(mm_live, 4),
                            "factory_counts_per_100mm": FACTORY_ENC_COUNTS_PER_100MM,
                            "counts": round(abs(counts_from_mm_live(mm_live)), 4),
                            "measured_mm": measured,
                        }
                    )
                    print("Muestra de feeder y de encoder anotadas.")
        if not confirm("¿Otro avance?", default=False):
            break
    if not samples_feed and not samples_enc:
        print("No hubo muestras nuevas.")
        return
    store = load_store()
    if samples_feed:
        store["feeder"][side]["samples"] = merge_samples(
            store["feeder"][side].get("samples") or [], samples_feed
        )
    if samples_enc:
        store["encoder"][side]["samples"] = merge_samples(
            store["encoder"][side].get("samples") or [], samples_enc
        )
    save_store(store)
    report_feeder(store)
    print()
    report_encoder(store)


def run_lmguide(args):
    if confirm("¿Lo hago con el serial del MCU?", default=True):
        print("El LM Guide se va a mover. Deja libre la zona.")
        link = ensure_link(args)
        samples = collect(lambda: take_lm_serial(link))
    else:
        samples = collect(take_lm_manual)
    store_lm(samples)


def run_ver():
    store = load_store()
    refresh_derived(store)
    report_all(store)


def run_eje(name, args):
    if name == "encoder":
        run_encoder(args)
    elif name == "feeder":
        run_feeder(args)
    elif name == "avance":
        run_avance(args)
    elif name == "lmguide":
        run_lmguide(args)
    elif name == "ver":
        run_ver()


def menu(args):
    actions = {
        "1": "encoder",
        "2": "feeder",
        "3": "avance",
        "4": "lmguide",
        "5": "ver",
    }
    while True:
        print()
        print("Calibración de banco")
        print("  1  Encoder")
        print("  2  Feeder")
        print("  3  Mismo avance (encoder y feeder)")
        print("  4  LM Guide")
        print("  5  Ver resultado")
        print("  0  Salir")
        raw = input("> ").strip().lower()
        if raw in {"0", "q", "salir"}:
            return
        name = actions.get(raw)
        if name is None:
            print("Elige 1, 2, 3, 4, 5 o 0.")
            continue
        try:
            run_eje(name, args)
        except (MotionError, TimeoutError, OSError) as exc:
            print(f"No se completó: {exc}")


def run_check():
    got = counts_from_mm_live(100.0)
    if abs(got - 1273) > 1e-6:
        raise SystemExit(f"counts_from_mm_live devolvió {got}")
    if abs(counts_from_mm_live(50.0) - 636.5) > 1e-6:
        raise SystemExit("counts_from_mm_live de 50 mm falló")
    scale = encoder_scale(
        [
            {"counts": 1273, "measured_mm": 100},
            {"counts": 2546, "measured_mm": 200},
        ]
    )
    if scale is None or abs(scale - 1273) > 1e-6:
        raise SystemExit(f"encoder_scale devolvió {scale}")
    mixed = encoder_scale(
        [
            {"counts": 1000, "measured_mm": 100},
            {"counts": 3000, "measured_mm": 200},
        ]
    )
    if mixed is None or abs(mixed - (4000.0 / 300.0 * 100.0)) > 1e-6:
        raise SystemExit(f"promedio ponderado devolvió {mixed}")
    feed = feeder_scale([{"commanded_counts": 3600, "measured_mm": 55}])
    if feed is None or abs(feed - (3600.0 / 55.0)) > 1e-9:
        raise SystemExit(f"feeder_scale devolvió {feed}")
    guide = lm_scale([{"delta_puu": -770, "measured_mm": 45}])
    if guide is None or abs(guide - (770.0 / 45.0)) > 1e-9:
        raise SystemExit(f"lm_scale devolvió {guide}")
    if mm_to_lm_puu(45) != -770:
        raise SystemExit(f"mm_to_lm_puu(45) devolvió {mm_to_lm_puu(45)}")
    if mm_to_lm_puu(100) != -1711:
        raise SystemExit(f"mm_to_lm_puu(100) devolvió {mm_to_lm_puu(100)}")
    if mm_to_feed_counts(55) != 3600:
        raise SystemExit(f"mm_to_feed_counts(55) devolvió {mm_to_feed_counts(55)}")
    fitted = fit_steps_per_mm(
        [
            {"sent_units": -1711, "measured_mm": 100},
            {"sent_units": mm_to_lm_puu(500), "measured_mm": 500},
            {"sent_units": mm_to_lm_puu(1400), "measured_mm": 1400},
        ]
    )
    if fitted is None or abs(fitted["steps_per_mm"] - APP_LM_STEPS_PER_MM) > 0.05:
        raise SystemExit(f"fit_steps_per_mm devolvió {fitted}")
    short = fit_steps_per_mm([{"sent_units": -1711, "marked_units": -1711, "measured_mm": 97}])
    if short is None or abs(short["steps_per_mm"] - (1711 / 97)) > 1e-6:
        raise SystemExit(f"fit con marca real devolvió {short}")
    if encoder_scale([]) is not None:
        raise SystemExit("encoder_scale vacío debía ser None")
    print("check ok")


def list_com_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [item.device for item in list_ports.comports()]


class CalibApp:
    def __init__(self, root, args, tk, ttk, messagebox):
        self.root = root
        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.link = None
        self.connected = False
        self.busy = False
        self.serial_buttons = []
        self.feed_jogs = 0
        self.avance_jogs = 0
        self.lm_sent = None

        root.title("Calibrar milímetros")
        root.geometry("820x760")
        root.minsize(760, 680)
        root.calib = self

        style = ttk.Style()
        style.configure("TButton", padding=(8, 4))

        self.port_var = tk.StringVar(value=(args.port or os.environ.get("MOTION_SERIAL", "")).strip())
        self.baud_var = tk.StringVar(value=os.environ.get("MOTION_SERIAL_BAUD", "115200"))
        self.status_var = tk.StringVar(value="Sin conexión")
        self.scale_var = tk.StringVar(value="")
        self.enc_side = tk.StringVar(value="R")
        self.feed_side = tk.StringVar(value="R")
        self.avance_side = tk.StringVar(value="R")
        self.enc_mm = tk.StringVar()
        self.enc_manual_counts = tk.StringVar()
        self.enc_read_var = tk.StringVar(value="Sin lectura")
        self.enc_counts_var = tk.StringVar(value="")
        self.feed_mm = tk.StringVar()
        self.feed_jog_var = tk.StringVar()
        self.feed_manual_counts = tk.StringVar()
        self.avance_mm = tk.StringVar()
        self.avance_jog_var = tk.StringVar()
        self.avance_read_var = tk.StringVar(value="Sin lectura")
        self.lm_cmd_mm = tk.StringVar(value="100")
        self.lm_real_mm = tk.StringVar()
        self.lm_drive_puu = tk.StringVar()
        self.lm_convert_var = tk.StringVar()
        self.lm_fit_var = tk.StringVar(value="Sin puntos.")
        self.feed_cmd_mm = tk.StringVar(value="100")
        self.feed_real_mm = tk.StringVar()
        self.feed_drive_counts = tk.StringVar()
        self.feed_convert_var = tk.StringVar()
        self.feed_fit_var = tk.StringVar(value="Sin puntos.")

        self._build_top()
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        self._build_lm()
        self._build_feeder()
        self._build_log()
        self._refresh_ports()
        self._apply_connection_state()
        self.log("Conecta el puerto del MCU Motion. Esta ventana no cambia la pantalla principal.")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_top(self):
        tk = self.tk
        ttk = self.ttk
        top = ttk.Frame(self.root, padding=(8, 8, 8, 4))
        top.pack(fill="x")
        ttk.Label(top, text="Puerto").pack(side="left")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=12)
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="Puertos", command=self._refresh_ports).pack(side="left")
        ttk.Label(top, text="Baud").pack(side="left", padx=(10, 0))
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side="left", padx=6)
        self.connect_btn = ttk.Button(top, text="Conectar", command=self._connect)
        self.connect_btn.pack(side="left", padx=(8, 0))
        self.disconnect_btn = ttk.Button(top, text="Desconectar", command=self._disconnect)
        self.disconnect_btn.pack(side="left", padx=6)
        stop = ttk.Button(top, text="Paro", command=self._stop)
        stop.pack(side="left", padx=(4, 0))
        self.serial_buttons.append(stop)
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=12)

    def _tab(self, title):
        frame = self.ttk.Frame(self.notebook, padding=12)
        self.notebook.add(frame, text=title)
        return frame

    def _row(self, parent):
        row = self.ttk.Frame(parent)
        row.pack(anchor="w", pady=3)
        return row

    def _hint(self, parent, text):
        self.ttk.Label(parent, text=text, wraplength=760).pack(anchor="w", pady=(0, 8))

    def _step(self, parent, title, text):
        box = self.ttk.LabelFrame(parent, text=title, padding=10)
        box.pack(fill="x", pady=(0, 8))
        self.ttk.Label(box, text=text, wraplength=720, justify="left").pack(anchor="w", pady=(0, 6))
        return box

    def _side_row(self, parent, variable, on_change):
        row = self._row(parent)
        self.ttk.Label(row, text="Lado").pack(side="left", padx=(0, 8))
        for side in ("R", "L"):
            self.ttk.Radiobutton(
                row, text=side, value=side, variable=variable, command=on_change
            ).pack(side="left", padx=(0, 8))

    def _entry(self, parent, label, variable, width=12):
        self.ttk.Label(parent, text=label).pack(side="left")
        box = self.ttk.Entry(parent, textvariable=variable, width=width)
        box.pack(side="left", padx=6)
        return box

    def _serial_button(self, parent, text, command):
        btn = self.ttk.Button(parent, text=text, command=command)
        btn.pack(side="left", padx=(0, 6))
        self.serial_buttons.append(btn)
        return btn

    def _build_encoder(self):
        tab = self._tab("Encoder")
        self._hint(
            tab,
            "Pon el cero, avanza el material y escribe lo que midió la cinta. "
            "Al guardar se lee de nuevo mm_live y se invierte con 1273 cuentas = 100 mm.",
        )
        live = self.ttk.LabelFrame(tab, text="Con el MCU", padding=10)
        live.pack(fill="x", pady=(0, 10))
        self._side_row(live, self.enc_side, self._show_scales)
        row = self._row(live)
        self._serial_button(row, "Poner cero", self._enc_zero)
        self._serial_button(row, "Leer", self._enc_read)
        self.ttk.Label(live, textvariable=self.enc_read_var).pack(anchor="w")
        self.ttk.Label(live, textvariable=self.enc_counts_var).pack(anchor="w", pady=(0, 4))
        row = self._row(live)
        self._entry(row, "Cinta (mm)", self.enc_mm)
        self._serial_button(row, "Guardar muestra", self._enc_save_serial)
        manual = self.ttk.LabelFrame(tab, text="Anotación manual", padding=10)
        manual.pack(fill="x")
        row = self._row(manual)
        self._entry(row, "Cuentas", self.enc_manual_counts)
        self._entry(row, "Cinta (mm)", self.enc_mm)
        self.ttk.Button(row, text="Guardar cuentas", command=self._enc_save_manual).pack(side="left")

    def _build_feeder(self):
        tab = self._tab("Alimentador")
        step = self._step(
            tab,
            "1. Mira cuántas cuentas salen hoy",
            "Escribe los milímetros que pediste. Abajo aparece en cuántas cuentas los convierte la pantalla ahora. Desde aquí el alimentador no se mueve.",
        )
        row = self._row(step)
        self._entry(row, "Milímetros pedidos", self.feed_cmd_mm, width=8)
        self.ttk.Label(step, textvariable=self.feed_convert_var, wraplength=720).pack(anchor="w", pady=(4, 0))
        step = self._step(
            tab,
            "2. Anota lo que midió la cinta",
            "Si pediste 100 mm y la cinta midió 96, deja 100 arriba y escribe 96 aquí. Repite con otras distancias.",
        )
        row = self._row(step)
        self._entry(row, "Milímetros medidos", self.feed_real_mm, width=8)
        self.ttk.Button(row, text="Anotar medida", command=self._feed_add_point).pack(side="left", padx=(8, 0))
        self.ttk.Button(row, text="Quitar la seleccionada", command=self._feed_remove_point).pack(side="left", padx=6)
        self.feed_list = self.tk.Listbox(tab, height=5, font=("Segoe UI", 10))
        self.feed_list.pack(fill="x", pady=(0, 8))
        self.ttk.Label(tab, textvariable=self.feed_fit_var, wraplength=740, justify="left").pack(anchor="w")
        self.feed_cmd_mm.trace_add("write", lambda *_: self._show_feed_convert())
        self._show_feed_convert()
        self._reload_points("feeder", self.feed_list, self.feed_fit_var, "cuentas/mm", APP_FEED_STEPS_PER_MM)

    def _build_avance(self):
        tab = self._tab("Mismo avance")
        self._hint(
            tab,
            "Un recorrido anota el feeder y el encoder contra la misma cinta. "
            "El cero del encoder se pone después de inicializar, con el eje quieto.",
        )
        box = self.ttk.LabelFrame(tab, text="Con el MCU", padding=10)
        box.pack(fill="x")
        self._side_row(box, self.avance_side, self._avance_side_changed)
        row = self._row(box)
        self._serial_button(row, "Inicializar", self._avance_init)
        self._serial_button(row, "Cero encoder", self._avance_zero)
        row = self._row(box)
        self._serial_button(row, "Jog +5000", self._avance_jog)
        self.ttk.Button(row, text="Quitar último", command=self._avance_undo).pack(side="left")
        self.ttk.Label(box, textvariable=self.avance_jog_var).pack(anchor="w")
        self.ttk.Label(box, textvariable=self.avance_read_var).pack(anchor="w", pady=(0, 4))
        row = self._row(box)
        self._entry(row, "Cinta (mm)", self.avance_mm)
        self._serial_button(row, "Guardar muestra", self._avance_save)

    def _build_lm(self):
        tab = self._tab("Guía")
        step = self._step(
            tab,
            "1. Enciende y ve a cero",
            "Home es el cero. Las distancias de abajo se miden desde ahí, con una cinta.",
        )
        row = self._row(step)
        self._serial_button(row, "Encender servo", self._lm_servo)
        self._serial_button(row, "Ir a home", self._lm_home)
        step = self._step(
            tab,
            "2. Pide una distancia",
            "Elige 100, 500 o 1400 mm. El botón mueve la guía usando la cuenta de hoy. Espera a que pare.",
        )
        row = self._row(step)
        for preset in (100, 500, 1400):
            self.ttk.Button(
                row,
                text=f"{preset} mm",
                command=lambda value=preset: self.lm_cmd_mm.set(str(value)),
            ).pack(side="left", padx=(0, 6))
        self._serial_button(row, "Mover", self._lm_move_mm)
        self.ttk.Label(step, textvariable=self.lm_convert_var, wraplength=720).pack(anchor="w", pady=(4, 0))
        step = self._step(
            tab,
            "3. Anota lo que midió la cinta",
            "No copies el número que pediste. Escribe los milímetros que marcó la cinta desde home.",
        )
        row = self._row(step)
        self._entry(row, "Milímetros medidos", self.lm_real_mm, width=8)
        self.ttk.Button(row, text="Anotar medida", command=self._lm_add_point).pack(side="left", padx=(8, 0))
        self.ttk.Button(row, text="Quitar la seleccionada", command=self._lm_remove_point).pack(side="left", padx=6)
        self.lm_list = self.tk.Listbox(tab, height=4, font=("Segoe UI", 10))
        self.lm_list.pack(fill="x", pady=(0, 8))
        self.ttk.Label(tab, textvariable=self.lm_fit_var, wraplength=740, justify="left").pack(anchor="w")
        self.lm_cmd_mm.trace_add("write", lambda *_: self._show_lm_convert())
        self._show_lm_convert()
        self._reload_points("lmguide", self.lm_list, self.lm_fit_var, "PUU/mm", APP_LM_STEPS_PER_MM)

    def _build_result(self):
        tab = self._tab("Resultado")
        row = self._row(tab)
        self.ttk.Button(row, text="Actualizar", command=self._show_result).pack(side="left")
        self.result = self.tk.Text(tab, height=22, wrap="word", font=("Consolas", 10))
        self.result.pack(fill="both", expand=True, pady=(8, 0))

    def _build_log(self):
        frame = self.ttk.Frame(self.root, padding=(8, 0, 8, 8))
        frame.pack(fill="x")
        self.ttk.Label(frame, text="Avisos").pack(anchor="w")
        self.log_box = self.tk.Text(frame, height=4, wrap="word", font=("Segoe UI", 10))
        self.log_box.pack(fill="x")
        self.log_box.configure(state="disabled")

    def log(self, text):
        def write():
            self.log_box.configure(state="normal")
            self.log_box.insert("end", str(text) + "\n")
            self.log_box.see("end")
            self.log_box.configure(state="disabled")

        if threading.current_thread() is threading.main_thread():
            write()
        else:
            self.root.after(0, write)

    def _refresh_ports(self):
        ports = list_com_ports()
        self.port_box["values"] = ports
        if not self.port_var.get().strip() and ports:
            self.port_var.set(ports[0])

    def _apply_connection_state(self):
        serial_state = "normal" if self.connected and not self.busy else "disabled"
        for btn in self.serial_buttons:
            btn.configure(state=serial_state)
        self.connect_btn.configure(state="disabled" if self.connected or self.busy else "normal")
        self.disconnect_btn.configure(state="normal" if self.connected and not self.busy else "disabled")

    def _bg(self, work, done=None):
        if self.busy:
            self.log("Espera a que termine la orden anterior.")
            return

        self.busy = True
        self._apply_connection_state()

        def task():
            try:
                result = work()
            except Exception as exc:
                self.root.after(0, lambda error=exc: self._bg_fail(error))
            else:
                self.root.after(0, lambda value=result: self._bg_ok(value, done))

        threading.Thread(target=task, daemon=True).start()

    def _bg_fail(self, exc):
        self.busy = False
        self._apply_connection_state()
        self.log(exc)
        self.messagebox.showerror("Calibración", str(exc))

    def _bg_ok(self, result, done):
        self.busy = False
        self._apply_connection_state()
        if done is not None:
            done(result)

    def _need_link(self):
        if self.connected and self.link is not None:
            return True
        self.messagebox.showerror("Calibración", "Primero conecta el serial del MCU Motion.")
        return False

    def _baud(self):
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            raise MotionError("El baud tiene que ser un número.") from None
        if baud <= 0:
            raise MotionError("El baud tiene que ser mayor que cero.")
        return baud

    def _ask_mm(self, variable):
        raw = variable.get().strip().replace(",", ".")
        if not raw:
            self.messagebox.showerror("Calibración", "Falta la medida en mm.")
            return None
        try:
            mm = float(raw)
        except ValueError:
            self.messagebox.showerror("Calibración", "La medida tiene que ser un número.")
            return None
        if mm != mm or mm <= 0:
            self.messagebox.showerror("Calibración", "La medida tiene que ser mayor que cero.")
            return None
        if mm < 50 and not self.messagebox.askyesno(
            "Medida corta",
            "Menos de 50 mm deja mucha incertidumbre. ¿La uso igual?",
        ):
            return None
        return mm

    def _ask_number(self, variable, label):
        raw = variable.get().strip().replace(",", ".")
        if not raw:
            self.messagebox.showerror("Calibración", f"Falta {label}.")
            return None
        try:
            value = float(raw)
        except ValueError:
            self.messagebox.showerror("Calibración", f"{label} tiene que ser un número.")
            return None
        if value != value or value == 0:
            self.messagebox.showerror("Calibración", f"{label} no puede ser cero.")
            return None
        return value

    def _merge_decision(self, old_count, what):
        if old_count <= 0:
            return False
        return self.messagebox.askyesnocancel(
            "Muestras guardadas",
            f"Ya hay {old_count} muestras de {what}.\n\n"
            "Sí reemplaza las anteriores.\n"
            "No agrega estas.\n"
            "Cancelar no guarda.",
        )

    def _save(self, store):
        save_store(store, out=self.log)
        self._show_result()

    def _store_side(self, section, side, samples):
        store = load_store()
        old = list(store[section][side].get("samples") or [])
        decision = self._merge_decision(len(old), f"{section} lado {side}")
        if decision is None:
            return False
        store[section][side]["samples"] = list(samples) if decision else old + list(samples)
        self._save(store)
        return True

    def _show_scales(self):
        store = load_store()
        refresh_derived(store)
        parts = []
        for side in ("R", "L"):
            enc = store["encoder"][side].get("counts_per_100mm")
            feed = store["feeder"][side].get("steps_per_mm")
            enc_txt = "—" if enc is None else fmt(enc)
            feed_txt = "—" if feed is None else fmt(feed)
            parts.append(f"{side}  enc {enc_txt}   feed {feed_txt}")
        lm_fit = fit_steps_per_mm(store["lmguide"].get("points"))
        feed_fit = fit_steps_per_mm(store["feeder"].get("points"))
        lm_txt = fmt(lm_fit["steps_per_mm"]) + " ajustado" if lm_fit else "770/45"
        feed_txt = fmt(feed_fit["steps_per_mm"]) + " ajustado" if feed_fit else "3600/55"
        self.scale_var.set(
            "      ".join(parts) + f"      LM {lm_txt} PUU/mm      Feeder {feed_txt} cuentas/mm"
        )

    def _show_result(self):
        if not hasattr(self, "result"):
            return
        store = load_store()
        refresh_derived(store)
        lines = []
        report_all(store, out=lines.append)
        self.result.configure(state="normal")
        self.result.delete("1.0", "end")
        self.result.insert("1.0", "\n".join(lines))
        self.result.configure(state="disabled")
        self._show_scales()

    def _show_feed_jogs(self):
        counts = self.feed_jogs * FEED_JOG_COUNTS
        self.feed_jog_var.set(f"Jogs contados: {self.feed_jogs}.  Cuentas pedidas: {counts}.")

    def _show_avance_jogs(self):
        counts = self.avance_jogs * FEED_JOG_COUNTS
        self.avance_jog_var.set(f"Jogs contados: {self.avance_jogs}.  Cuentas pedidas: {counts}.")

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            self.messagebox.showerror("Calibración", "Escribe el puerto, por ejemplo COM3.")
            return
        try:
            baud = self._baud()
        except MotionError as exc:
            self.messagebox.showerror("Calibración", str(exc))
            return

        def work():
            link = MotionLink(port, baud)
            link.open()
            return link

        def done(link):
            global LINK
            self.link = link
            LINK = link
            self.connected = True
            self.status_var.set(f"Conectado a {port}")
            self._apply_connection_state()
            self.log(f"Serial abierto en {port} a {baud} baud.")

        self._bg(work, done)

    def _disconnect(self):
        global LINK
        if self.link is not None:
            try:
                self.link.close()
            except Exception as exc:
                self.log(exc)
        had_link = self.link is not None
        self.link = None
        LINK = None
        self.connected = False
        self.status_var.set("Sin conexión")
        self._apply_connection_state()
        if had_link:
            self.log("Serial cerrado.")

    def _on_close(self):
        if self.busy:
            self.messagebox.showinfo("Calibración", "Espera a que termine la orden en curso.")
            return
        self._disconnect()
        self.root.destroy()

    def _stop(self):
        if not self._need_link():
            return

        def work():
            call(self.link, {"op": "feeder_stop"})
            call(self.link, {"op": "lmguide_stop"})

        self._bg(work, lambda _result: self.log("Paro enviado al feeder y al LM Guide."))

    def _enc_zero(self):
        if not self._need_link():
            return
        side = self.enc_side.get()

        def work():
            call(self.link, {"op": "encoder_set0", "side": side})
            return side

        def done(done_side):
            self.enc_read_var.set("Cero puesto. Avanza el material y luego guarda la cinta.")
            self.enc_counts_var.set("")
            self.log(f"Cero del encoder {done_side}.")

        self._bg(work, done)

    def _enc_read(self):
        if not self._need_link():
            return
        side = self.enc_side.get()

        def work():
            return read_mm_live(self.link, side)

        def done(mm_live):
            self._show_encoder_read(mm_live)

        self._bg(work, done)

    def _show_encoder_read(self, mm_live):
        self.enc_read_var.set(f"mm_live {fmt(mm_live)}")
        if abs(mm_live) < 0.05:
            self.enc_counts_var.set("El encoder no se movió.")
            self.log("El encoder no se movió.")
            return None
        counts = abs(counts_from_mm_live(mm_live))
        self.enc_counts_var.set(f"{fmt(counts)} cuentas recuperadas")
        self.log(f"Encoder: mm_live {fmt(mm_live)}, cuentas {fmt(counts)}.")
        return counts

    def _enc_save_serial(self):
        if not self._need_link():
            return
        measured = self._ask_mm(self.enc_mm)
        if measured is None:
            return
        side = self.enc_side.get()

        def work():
            return read_mm_live(self.link, side)

        def done(mm_live):
            counts = self._show_encoder_read(mm_live)
            if counts is None:
                self.messagebox.showerror("Calibración", "El encoder no se movió. Esta muestra no sirve.")
                return
            sample = {
                "at": now_stamp(),
                "source": "serial",
                "mm_live": round(mm_live, 4),
                "factory_counts_per_100mm": FACTORY_ENC_COUNTS_PER_100MM,
                "counts": round(counts, 4),
                "measured_mm": measured,
            }
            if self._store_side("encoder", side, [sample]):
                self.enc_mm.set("")

        self._bg(work, done)

    def _enc_save_manual(self):
        counts = self._ask_number(self.enc_manual_counts, "Las cuentas")
        if counts is None:
            return
        measured = self._ask_mm(self.enc_mm)
        if measured is None:
            return
        sample = {
            "at": now_stamp(),
            "source": "manual",
            "counts": round(abs(counts), 4),
            "measured_mm": measured,
        }
        if self._store_side("encoder", self.enc_side.get(), [sample]):
            self.enc_mm.set("")
            self.enc_manual_counts.set("")

    def _feed_side_changed(self):
        if self.feed_jogs:
            self.log("Cambié de lado. El conteo de jogs del feeder volvió a cero.")
        self.feed_jogs = 0
        self._show_feed_jogs()
        self._show_scales()

    def _feed_init(self):
        if not self._need_link():
            return
        if not self.messagebox.askokcancel(
            "Feeder",
            "Inicializar puede mover el feeder hasta la posición guardada.\n"
            "Espera a que pare antes de marcar el tubo.",
        ):
            return
        side = self.feed_side.get()

        def work():
            prepare_feeder(self.link, side)

        def done(_result):
            self.feed_jogs = 0
            self._show_feed_jogs()
            self.log(f"Feeder {side} listo. Marca el tubo cuando esté quieto.")

        self._bg(work, done)

    def _feed_jog(self):
        if not self._need_link():
            return
        side = self.feed_side.get()

        def work():
            msg = call(self.link, {"op": "feeder_jog", "side": side, "dir": 1})
            if msg.get("ok") is False:
                raise MotionError("El MCU no aceptó el jog.")
            return side

        def done(done_side):
            if self.feed_side.get() != done_side:
                self.log(f"El jog de {done_side} terminó después de cambiar de lado. No lo conté.")
                return
            self.feed_jogs += 1
            self._show_feed_jogs()
            self.log(f"Jog {done_side}. Cuentas pedidas: {self.feed_jogs * FEED_JOG_COUNTS}.")

        self._bg(work, done)

    def _feed_undo(self):
        if self.feed_jogs > 0:
            self.feed_jogs -= 1
        self._show_feed_jogs()

    def _feed_save(self):
        if self.feed_jogs <= 0:
            self.messagebox.showerror("Calibración", "No hay jogs contados.")
            return
        measured = self._ask_mm(self.feed_mm)
        if measured is None:
            return
        sample = {
            "at": now_stamp(),
            "source": "serial",
            "jog_step_counts": FEED_JOG_COUNTS,
            "commanded_counts": self.feed_jogs * FEED_JOG_COUNTS,
            "measured_mm": measured,
        }
        if self._store_side("feeder", self.feed_side.get(), [sample]):
            self.feed_jogs = 0
            self._show_feed_jogs()
            self.feed_mm.set("")

    def _feed_save_manual(self):
        commanded = self._ask_number(self.feed_manual_counts, "Las cuentas comandadas")
        if commanded is None:
            return
        measured = self._ask_mm(self.feed_mm)
        if measured is None:
            return
        sample = {
            "at": now_stamp(),
            "source": "manual",
            "commanded_counts": round(abs(commanded), 4),
            "measured_mm": measured,
        }
        if self._store_side("feeder", self.feed_side.get(), [sample]):
            self.feed_mm.set("")
            self.feed_manual_counts.set("")

    def _avance_side_changed(self):
        if self.avance_jogs:
            self.log("Cambié de lado. El conteo de jogs de este avance volvió a cero.")
        self.avance_jogs = 0
        self._show_avance_jogs()
        self.avance_read_var.set("Sin lectura")

    def _avance_init(self):
        if not self._need_link():
            return
        if not self.messagebox.askokcancel(
            "Feeder",
            "Inicializar puede mover el feeder hasta la posición guardada.\n"
            "El cero del encoder se pone después, con el eje quieto.",
        ):
            return
        side = self.avance_side.get()

        def work():
            prepare_feeder(self.link, side)

        def done(_result):
            self.avance_jogs = 0
            self._show_avance_jogs()
            self.avance_read_var.set("Sin lectura")
            self.log(f"Feeder {side} listo. Cuando pare, pon el cero y marca el tubo.")

        self._bg(work, done)

    def _avance_zero(self):
        if not self._need_link():
            return
        side = self.avance_side.get()

        def work():
            call(self.link, {"op": "encoder_set0", "side": side})
            return side

        def done(done_side):
            self.avance_read_var.set("Cero puesto. Marca el tubo y empieza los jog.")
            self.log(f"Cero del encoder {done_side}.")

        self._bg(work, done)

    def _avance_jog(self):
        if not self._need_link():
            return
        side = self.avance_side.get()

        def work():
            msg = call(self.link, {"op": "feeder_jog", "side": side, "dir": 1})
            if msg.get("ok") is False:
                raise MotionError("El MCU no aceptó el jog.")
            return side

        def done(done_side):
            if self.avance_side.get() != done_side:
                self.log(f"El jog de {done_side} terminó después de cambiar de lado. No lo conté.")
                return
            self.avance_jogs += 1
            self._show_avance_jogs()

        self._bg(work, done)

    def _avance_undo(self):
        if self.avance_jogs > 0:
            self.avance_jogs -= 1
        self._show_avance_jogs()

    def _avance_save(self):
        if not self._need_link():
            return
        if self.avance_jogs <= 0:
            self.messagebox.showerror("Calibración", "No hay jogs contados.")
            return
        measured = self._ask_mm(self.avance_mm)
        if measured is None:
            return
        side = self.avance_side.get()
        commanded = self.avance_jogs * FEED_JOG_COUNTS

        def work():
            return read_mm_live(self.link, side)

        def done(mm_live):
            self._avance_commit(side, commanded, measured, mm_live)

        self._bg(work, done)

    def _avance_commit(self, side, commanded, measured, mm_live):
        self.avance_read_var.set(f"mm_live {fmt(mm_live)}")
        feed_sample = {
            "at": now_stamp(),
            "source": "serial",
            "jog_step_counts": FEED_JOG_COUNTS,
            "commanded_counts": commanded,
            "measured_mm": measured,
        }
        enc_sample = None
        if abs(mm_live) < 0.05:
            self.log("El encoder no se movió. Se guardará solo el feeder.")
        else:
            enc_sample = {
                "at": feed_sample["at"],
                "source": "serial",
                "mm_live": round(mm_live, 4),
                "factory_counts_per_100mm": FACTORY_ENC_COUNTS_PER_100MM,
                "counts": round(abs(counts_from_mm_live(mm_live)), 4),
                "measured_mm": measured,
            }
        store = load_store()
        old_feed = list(store["feeder"][side].get("samples") or [])
        old_enc = list(store["encoder"][side].get("samples") or [])
        old_count = len(old_feed) + (len(old_enc) if enc_sample else 0)
        what = f"feeder y encoder lado {side}" if enc_sample else f"feeder lado {side}"
        decision = self._merge_decision(old_count, what)
        if decision is None:
            return
        store["feeder"][side]["samples"] = [feed_sample] if decision else old_feed + [feed_sample]
        if enc_sample:
            store["encoder"][side]["samples"] = [enc_sample] if decision else old_enc + [enc_sample]
        self.avance_jogs = 0
        self._show_avance_jogs()
        self.avance_mm.set("")
        self._save(store)

    def _lm_servo(self):
        if not self._need_link():
            return
        if not self.messagebox.askokcancel("Guía", "Se va a encender el motor de la guía."):
            return

        def work():
            msg = call(self.link, {"op": "lmguide_servo_on"})
            if msg.get("ok") is False:
                raise MotionError("El servo del LM Guide no encendió.")

        self._bg(work, lambda _result: self.log("Motor de la guía encendido."))

    def _read_positive(self, variable, label):
        raw = variable.get().strip().replace(",", ".")
        if not raw:
            self.messagebox.showerror("Calibración", f"Falta {label}.")
            return None
        try:
            value = float(raw)
        except ValueError:
            self.messagebox.showerror("Calibración", f"{label} tiene que ser un número.")
            return None
        if value != value or value <= 0:
            self.messagebox.showerror("Calibración", f"{label} tiene que ser mayor que cero.")
            return None
        return value

    def _read_optional(self, variable, label):
        raw = variable.get().strip().replace(",", ".")
        if not raw:
            return None
        try:
            value = float(raw)
        except ValueError:
            self.messagebox.showerror("Calibración", f"{label} tiene que ser un número.")
            return False
        if value != value or value == 0:
            self.messagebox.showerror("Calibración", f"{label} no puede ser cero.")
            return False
        return value

    def _show_lm_convert(self):
        mm = self.lm_cmd_mm.get().strip().replace(",", ".")
        try:
            value = float(mm)
        except ValueError:
            self.lm_convert_var.set("Escribe los mm pedidos.")
            return
        if value <= 0:
            self.lm_convert_var.set("Los mm pedidos tienen que ser mayores que cero.")
            return
        puu = mm_to_lm_puu(value)
        self.lm_convert_var.set(
            f"Hoy {fmt(value)} mm se mandan como {abs(puu)} PUU hacia el corte."
        )

    def _show_feed_convert(self):
        mm = self.feed_cmd_mm.get().strip().replace(",", ".")
        try:
            value = float(mm)
        except ValueError:
            self.feed_convert_var.set("Escribe los mm pedidos.")
            return
        if value <= 0:
            self.feed_convert_var.set("Los mm pedidos tienen que ser mayores que cero.")
            return
        counts = mm_to_feed_counts(value)
        self.feed_convert_var.set(f"Hoy {fmt(value)} mm se convierten en {counts} cuentas.")

    def _point_line(self, point, unit_name):
        sent = abs(int(point["sent_units"]))
        measured = fmt(point["measured_mm"])
        asked = fmt(point["commanded_mm"])
        if unit_name.startswith("PUU"):
            return f"Pediste {asked} mm ({sent} PUU). La cinta midió {measured} mm."
        return f"Pediste {asked} mm ({sent} cuentas). La cinta midió {measured} mm."

    def _fit_sentence(self, section, fit, current):
        if section == "lmguide":
            old_100 = abs(mm_to_lm_puu(100, current))
            new_100 = abs(mm_to_lm_puu(100, fit["steps_per_mm"]))
            text = (
                f"Con {fit['n']} medidas, cada milímetro real vale {fmt(fit['steps_per_mm'])} PUU.\n"
                f"Hoy 100 mm se mandan como {old_100} PUU. Con este ajuste serían {new_100} PUU.\n"
                "La pantalla principal no cambia hasta que se autorice."
            )
        else:
            old_100 = mm_to_feed_counts(100, current)
            new_100 = mm_to_feed_counts(100, fit["steps_per_mm"])
            text = (
                f"Con {fit['n']} medidas, cada milímetro real vale {fmt(fit['steps_per_mm'])} cuentas.\n"
                f"Hoy 100 mm son {old_100} cuentas. Con este ajuste serían {new_100}.\n"
                "La pantalla principal no cambia hasta que se autorice."
            )
        ratios = fit["ratios"]
        if len(ratios) >= 2 and min(ratios) > 0 and (max(ratios) - min(ratios)) / min(ratios) > 0.02:
            text += "\nLas medidas no coinciden entre sí. Revisa la que se vea distinta."
        return text

    def _reload_points(self, section, listing, fit_var, unit_name, current):
        store = load_store()
        points = list(store.get(section, {}).get("points") or [])
        listing.delete(0, "end")
        for point in points:
            listing.insert("end", self._point_line(point, unit_name))
        fit = fit_steps_per_mm(points)
        if not fit:
            if section == "lmguide":
                fit_var.set(f"Todavía no hay medidas. Hoy 100 mm se mandan como {abs(mm_to_lm_puu(100))} PUU.")
            else:
                fit_var.set(f"Todavía no hay medidas. Hoy 100 mm son {mm_to_feed_counts(100)} cuentas.")
            return
        fit_var.set(self._fit_sentence(section, fit, current))

    def _append_point(self, section, point, listing, fit_var, unit_name, current):
        store = load_store()
        points = list(store[section].get("points") or [])
        points.append(point)
        store[section]["points"] = points
        self._save(store)
        self._reload_points(section, listing, fit_var, unit_name, current)

    def _drop_point(self, section, listing, fit_var, unit_name, current):
        selected = listing.curselection()
        if not selected:
            self.messagebox.showerror("Calibración", "Selecciona el punto que quieres quitar.")
            return
        store = load_store()
        points = list(store[section].get("points") or [])
        index = selected[0]
        if index >= len(points):
            return
        del points[index]
        store[section]["points"] = points
        self._save(store)
        self._reload_points(section, listing, fit_var, unit_name, current)

    def _lm_home(self):
        if not self._need_link():
            return
        if not self.messagebox.askokcancel(
            "Guía",
            "La guía va a buscar el cero. Espera a que termine antes de medir.",
        ):
            return

        def work():
            msg = call(self.link, {"op": "lmguide_home"})
            if msg.get("ok") is False:
                raise MotionError("El home del LM Guide no terminó.")

        self._bg(work, lambda _result: self.log("La guía está en cero. Ya puedes medir desde ahí."))

    def _lm_move_mm(self):
        if not self._need_link():
            return
        mm = self._read_positive(self.lm_cmd_mm, "Los mm pedidos")
        if mm is None:
            return
        puu = mm_to_lm_puu(mm)
        if not self.messagebox.askokcancel(
            "LM Guide",
            f"La guía va a ir a {fmt(mm)} mm desde home. Hoy eso son {abs(puu)} PUU.",
        ):
            return

        def work():
            msg = call(self.link, {"op": "lmguide_move", "puu": puu})
            if msg.get("ok") is False:
                raise MotionError("El drive no aceptó el movimiento.")
            return puu

        def done(sent):
            self.lm_sent = sent
            self.log(f"La guía va hacia {fmt(mm)} mm. Cuando pare, mide con la cinta y anota ese número.")

        self._bg(work, done)

    def _lm_add_point(self):
        commanded = self._read_positive(self.lm_cmd_mm, "Los mm pedidos")
        if commanded is None:
            return
        measured = self._read_positive(self.lm_real_mm, "Los mm reales")
        if measured is None:
            return
        marked = self._read_optional(self.lm_drive_puu, "El PUU del drive")
        if marked is False:
            return
        sent = mm_to_lm_puu(commanded)
        point = {
            "at": now_stamp(),
            "commanded_mm": commanded,
            "sent_units": sent,
            "marked_units": None if marked is None else int(round(marked)),
            "measured_mm": measured,
        }
        self.lm_real_mm.set("")
        self.lm_drive_puu.set("")
        self._append_point("lmguide", point, self.lm_list, self.lm_fit_var, "PUU/mm", APP_LM_STEPS_PER_MM)
        self.log(self._point_line(point, "PUU/mm"))

    def _lm_remove_point(self):
        self._drop_point("lmguide", self.lm_list, self.lm_fit_var, "PUU/mm", APP_LM_STEPS_PER_MM)

    def _feed_add_point(self):
        commanded = self._read_positive(self.feed_cmd_mm, "Los mm pedidos")
        if commanded is None:
            return
        measured = self._read_positive(self.feed_real_mm, "Los mm reales")
        if measured is None:
            return
        marked = self._read_optional(self.feed_drive_counts, "Las cuentas del servo")
        if marked is False:
            return
        sent = mm_to_feed_counts(commanded)
        point = {
            "at": now_stamp(),
            "commanded_mm": commanded,
            "sent_units": sent,
            "marked_units": None if marked is None else int(round(marked)),
            "measured_mm": measured,
        }
        self.feed_real_mm.set("")
        self.feed_drive_counts.set("")
        self._append_point("feeder", point, self.feed_list, self.feed_fit_var, "cuentas/mm", APP_FEED_STEPS_PER_MM)
        self.log(self._point_line(point, "cuentas/mm"))

    def _feed_remove_point(self):
        self._drop_point("feeder", self.feed_list, self.feed_fit_var, "cuentas/mm", APP_FEED_STEPS_PER_MM)


def open_ui(args):
    import tkinter as tk
    from tkinter import messagebox, ttk

    root = tk.Tk()
    CalibApp(root, args, tk, ttk, messagebox)
    return root


def main(argv=None):
    parser = argparse.ArgumentParser(description="Calibración de encoder, feeder y LM Guide.")
    parser.add_argument(
        "eje",
        nargs="?",
        choices=("encoder", "feeder", "avance", "lmguide", "ver"),
        help="Atajo de terminal. Sin argumentos abre la ventana.",
    )
    parser.add_argument("--port", help="Puerto del MCU Motion, por ejemplo COM3. También vale MOTION_SERIAL.")
    parser.add_argument("--cli", action="store_true", help="Abre el menú en la terminal.")
    parser.add_argument("--check", action="store_true", help="Prueba las fórmulas y termina.")
    args = parser.parse_args(argv)
    if args.check:
        run_check()
        return 0
    if not args.cli and not args.eje:
        try:
            open_ui(args).mainloop()
        except KeyboardInterrupt:
            print("\nDetenido.")
            stop_axes()
            return 130
        return 0
    try:
        if args.eje:
            try:
                run_eje(args.eje, args)
            except (MotionError, TimeoutError, OSError) as exc:
                print(f"No se completó: {exc}")
                return 1
        else:
            menu(args)
    except KeyboardInterrupt:
        print("\nDetenido.")
        stop_axes()
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

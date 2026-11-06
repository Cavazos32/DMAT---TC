"""
Catálogo único de errores de la aplicación / orquestador.

Formato visible: E000 Módulo: descripción
"""

from __future__ import annotations

from typing import Any

# code -> (module, description template)
CATALOG: dict[str, tuple[str, str]] = {
    "E000": ("Sistema", "Error desconocido."),
    "E001": ("Orquestador", "No se puede alimentar: holder y encoder deben estar activos (paso 2)."),
    "E002": ("Orquestador", "No se pueden accionar pinzas: falta alimentación L/R validada (paso 3)."),
    "E003": ("Orquestador", "No se puede ir a corte: pinzas activas y holder/encoder liberados (paso 4)."),
    "E004": ("Orquestador", "No se puede cortar: LM Guide debe estar en posición de corte (paso 5)."),
    "E005": ("Orquestador", "No se puede descargar: el corte debe haberse completado (paso 6)."),
    "E006": ("Orquestador", "No se puede ir a home/blower: falta descarga de pieza (paso 7)."),
    "E007": ("Orquestador", "Corte del LM Guide no válido: falta la longitud del modelo."),
    "E010": ("PLC", "Sin respuesta de status. Revisa el COM en Parámetros, pestaña Conexión."),
    "E011": ("PLC", "Holder no confirmado en estado lógico activo."),
    "E012": ("PLC", "Encoder no confirmado en estado lógico activo."),
    "E013": ("PLC", "Pinzas no confirmadas activas."),
    "E014": ("PLC", "Holder no confirmado liberado."),
    "E015": ("PLC", "Encoder no confirmado liberado."),
    "E016": ("PLC", "Cola de pulsos aún ocupada (pulse_busy)."),
    "E017": ("PLC", "No hay puerto COM seleccionado. Elígelo en Parámetros, pestaña Conexión."),
    "E018": ("PLC", "El COM seleccionado no es el MCU PLC."),
    "E019": ("PLC", "El COM no respondió. Revisa el cable, la placa o cierra el monitor serial."),
    "E020": ("PLC", "Fallo de holder desde el PLC físico."),
    "E021": ("PLC", "Fallo de encoder desde el PLC físico."),
    "E022": ("PLC", "Fallo de cortador desde el PLC físico."),
    "E023": ("PLC", "Fallo de pinzas desde el PLC físico."),
    "E030": ("Motion", "FeedStage ocupado (busy)."),
    "E031": ("Motion", "Longitud objetivo no definida (no_target)."),
    "E032": ("Motion", "Alimentadores CAN no listos (feeder_not_ready)."),
    "E033": ("Motion", "Timeout de láser lado R (laser_timeout_r)."),
    "E034": ("Motion", "Timeout de láser lado L (laser_timeout_l)."),
    "E035": ("Motion", "Encoder fuera de tolerancia lado R (encoder_tolerance_r)."),
    "E036": ("Motion", "Encoder fuera de tolerancia lado L (encoder_tolerance_l)."),
    "E037": ("Motion", "Fallo al aplicar compensación (comp_move)."),
    "E039": ("Motion", "Safety exhaust / E-stop activo."),
    "E038": ("Motion", "FeedStage falló ({detail})."),
    "E040": ("Motion", "Sin respuesta de status. Revisa el COM en Parámetros, pestaña Conexión."),
    "E041": ("Motion", "No hay puerto COM seleccionado. Elígelo en Parámetros, pestaña Conexión."),
    "E042": ("Motion", "El COM seleccionado no es el MCU Motion."),
    "E043": ("Motion", "El COM no respondió. Revisa el cable, la placa o cierra el monitor serial."),
    "E070": ("Andon", "No hay puerto COM seleccionado. Elígelo en Parámetros, pestaña Conexión."),
    "E071": ("Andon", "El COM seleccionado no es el MCU Andon."),
    "E072": ("Andon", "El COM no respondió. Revisa el cable, la placa o cierra el monitor serial."),
}

FEED_STAGE_TO_CODE: dict[str, str] = {
    "ok": "",
    "busy": "E030",
    "no_target": "E031",
    "feeder_not_ready": "E032",
    "laser_timeout_r": "E033",
    "laser_timeout_l": "E034",
    "encoder_tolerance_r": "E035",
    "encoder_tolerance_l": "E036",
    "comp_move": "E037",
    "safety": "E039",
    "unknown": "E038",
}

PLC_FAULT_FIELDS: tuple[tuple[str, str], ...] = (
    ("fault_holder", "E020"),
    ("fault_encoder", "E021"),
    ("fault_cutter", "E022"),
    ("fault_gripper", "E023"),
)


def format_message(code: str, **kwargs: Any) -> str:
    module, text = CATALOG.get(code, CATALOG["E000"])
    if kwargs:
        try:
            text = text.format(**kwargs)
        except (KeyError, ValueError):
            pass
    return f"{code} {module}: {text}"


def feed_stage_code(detail: str) -> str:
    key = (detail or "unknown").strip()
    code = FEED_STAGE_TO_CODE.get(key, "E038")
    if code == "E038":
        return code
    if code == "":
        return "E000"
    return code


def feed_stage_message(detail: str) -> str:
    code = feed_stage_code(detail)
    if code in ("E000", "E038"):
        return format_message(code, detail=detail or "unknown")
    return format_message(code)


def plc_fault_codes(status: dict | None) -> list[str]:
    if not status:
        return []
    out: list[str] = []
    for field, code in PLC_FAULT_FIELDS:
        if status.get(field):
            out.append(code)
    return out


def plc_fault_messages(status: dict | None) -> list[str]:
    return [format_message(c) for c in plc_fault_codes(status)]


def catalog_for_api() -> dict[str, dict[str, str]]:
    return {
        code: {"module": mod, "description": desc, "label": format_message(code)}
        for code, (mod, desc) in CATALOG.items()
    }

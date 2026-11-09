(function () {
  const STORAGE_EVENTS = "dmat_show_events_panel";
  const STORAGE_THEME = "dmat_theme";
  const STORAGE_MODEL = "dmat_modelo";
  const STORAGE_COMP = "dmat_comp_mm";
  const LM_PIECE_OFFSET_MM = 55;

  const els = {
    statusBanner: document.getElementById("statusBanner"),
    statusBannerLabel: document.getElementById("statusBannerLabel"),
    statusBannerDetail: document.getElementById("statusBannerDetail"),
    progressPct: document.getElementById("progressPct"),
    progressFill: document.getElementById("progressFill"),
    selectModelo: document.getElementById("selectModelo"),
    metricLongitud: document.getElementById("metricLongitud"),
    metricCantidad: document.getElementById("metricCantidad"),
    metricCompensacion: document.getElementById("metricCompensacion"),
    metricTiempoCiclo: document.getElementById("metricTiempoCiclo"),
    metricTiempoEstimado: document.getElementById("metricTiempoEstimado"),
    metricTiempoCorriendo: document.getElementById("metricTiempoCorriendo"),
    eventsPanel: document.getElementById("eventsPanel"),
    eventsLog: document.getElementById("eventsLog"),
    btnClearEvents: document.getElementById("btnClearEvents"),
    selectMotionPort: document.getElementById("selectMotionPort"),
    selectPlcPort: document.getElementById("selectPlcPort"),
    selectAndonPort: document.getElementById("selectAndonPort"),
    selectPrefeederPort: document.getElementById("selectPrefeederPort"),
    btnRefreshPorts: document.getElementById("btnRefreshPorts"),
    btnMotionTest: document.getElementById("btnMotionTest"),
    btnPlcTest: document.getElementById("btnPlcTest"),
    btnAndonTest: document.getElementById("btnAndonTest"),
    btnPrefeederTest: document.getElementById("btnPrefeederTest"),
    serialMotionStatus: document.getElementById("serialMotionStatus"),
    serialPlcStatus: document.getElementById("serialPlcStatus"),
    serialAndonStatus: document.getElementById("serialAndonStatus"),
    serialPrefeederStatus: document.getElementById("serialPrefeederStatus"),
    prefeederTuneForm: document.getElementById("prefeederTuneForm"),
    prefeederTuneReload: document.getElementById("prefeederTuneReload"),
    prefeederStatusRefresh: document.getElementById("prefeederStatusRefresh"),
    prefeederEsp32Hint: document.getElementById("prefeederEsp32Hint"),
    pfBanL: document.getElementById("pfBanL"),
    pfBanR: document.getElementById("pfBanR"),
    toggleEventsPanel: document.getElementById("toggleEventsPanel"),
    toggleChecklistBypass: document.getElementById("toggleChecklistBypass"),
    settingsModal: document.getElementById("settingsModal"),
    parametrosModal: document.getElementById("parametrosModal"),
    btnAppSettings: document.getElementById("btnAppSettings"),
    settingsModalClose: document.getElementById("settingsModalClose"),
    parametrosModalClose: document.getElementById("parametrosModalClose"),
    parametrosModalBack: document.getElementById("parametrosModalBack"),
    themeToggle: document.getElementById("themeToggle"),
    motionTuneForm: document.getElementById("motionTuneForm"),
    motionTuneReload: document.getElementById("motionTuneReload"),
    motionStatusRefresh: document.getElementById("motionStatusRefresh"),
    motionEsp32Hint: document.getElementById("motionEsp32Hint"),
    motionDataModelo: document.getElementById("motionDataModelo"),
    motionDataTargetMm: document.getElementById("motionDataTargetMm"),
    motionDataToleranceMm: document.getElementById("motionDataToleranceMm"),
    plcManualForm: document.getElementById("plcManualForm"),
    plcEsp32Hint: document.getElementById("plcEsp32Hint"),
    plcBlowerMs: document.getElementById("plcBlowerMs"),
    plcGraceNote: document.getElementById("plcGraceNote"),
    settingsTabs: document.querySelectorAll(".modal-tab[data-tab]"),
    settingsTabPanels: document.querySelectorAll("[data-tab-panel]"),
    cycleStepsList: document.getElementById("cycleStepsList"),
    cycleParamsForm: document.getElementById("cycleParamsForm"),
    cycleCutMm: document.getElementById("cycleCutMm"),
    cycleStepStart: document.getElementById("cycleStepStart"),
    cycleStepNext: document.getElementById("cycleStepNext"),
    cycleStepStop: document.getElementById("cycleStepStop"),
    cycleStepTestHint: document.getElementById("cycleStepTestHint"),
    statusBannerActions: document.getElementById("statusBannerActions"),
    checklistModal: document.getElementById("checklistModal"),
    checklistModalClose: document.getElementById("checklistModalClose"),
    checklistBody: document.getElementById("checklistBody"),
    mantModal: document.getElementById("mantModal"),
    mantModalClose: document.getElementById("mantModalClose"),
    mantCounter: document.getElementById("mantCounter"),
    mantEmpleado: document.getElementById("mantEmpleado"),
    mantReset: document.getElementById("mantReset"),
    empleadoModal: document.getElementById("empleadoModal"),
    empleadoModalClose: document.getElementById("empleadoModalClose"),
    empleadoInput: document.getElementById("empleadoInput"),
    empleadoConfirm: document.getElementById("empleadoConfirm"),
  };

  let eventsEnabled = false;
  let pendingEventos = [];
  let eventsRev = -1;
  let clearingLog = false;
  let lastProcessAction = null;
  let modelosCatalog = [];
  let syncingModelSelect = false;
  let motionStepsPerMm = 770 / 45;
  let motionPuuSign = -1;
  let motionFeedStepsPerMm = 3600 / 55;
  let motionEncPollTimer = null;
  let motionEncPollBusy = false;
  let motionCommandInFlight = false;
  let statePollTimer = null;
  let cycleStepsCatalog = [];
  let plcPollTimer = null;
  let pfPollTimer = null;
  let checklistViewKey = "";
  let empleadoWait = null;

  const PLC_OUTPUTS = [
    ["holder", "holder"],
    ["encoder", "encoder"],
    ["cutter_r", "cutter_r"],
    ["cutter_l", "cutter_l"],
    ["gripper", "gripper"],
  ];

  const PLC_FAULTS = [
    ["plcFaultHolder", "fault_holder"],
    ["plcFaultEncoder", "fault_encoder"],
    ["plcFaultCutter", "fault_cutter"],
    ["plcFaultGripper", "fault_gripper"],
  ];

  const PROCESS_ACTIONS = new Set([
    "start",
    "step_start",
    "step_next",
    "resume",
    "pause",
    "stop",
    "reset",
  ]);

  const bannerClassByEstado = {
    idle: "status-banner--idle",
    running: "status-banner--running",
    paused: "status-banner--paused",
    stopped: "status-banner--stopped",
    error: "status-banner--error",
    manual: "status-banner--manual",
  };

  function formatTime(date) {
    return date.toLocaleTimeString("es-MX", {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
    });
  }

  function formatSeconds(total) {
    if (total == null || total < 0) return "—";
    const s = Math.floor(total);
    const m = Math.floor(s / 60);
    const r = s % 60;
    if (m > 0) {
      return m + " min " + String(r).padStart(2, "0") + " s";
    }
    return r.toFixed(1) + " s";
  }

  function formatMm(value) {
    if (value == null) return "—";
    return Number(value).toFixed(2);
  }

  function setCantidadInput(n, locked) {
    if (!els.metricCantidad) return;
    els.metricCantidad.disabled = !!locked;
    if (document.activeElement === els.metricCantidad) return;
    const v = parseInt(n, 10);
    els.metricCantidad.value = Number.isFinite(v) && v > 0 ? String(v) : "1";
  }

  async function saveCantidad(n) {
    try {
      const res = await fetch("/api/cantidad", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ cantidad: n }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se guardó la cantidad.", "error");
        if (data.state) applyState(data.state);
        return;
      }
      if (data.state) applyState(data.state);
    } catch (err) {
      pushLocalEvent("Sin conexión al guardar la cantidad.", "error");
    }
  }

  function setCompInput(mm) {
    if (!els.metricCompensacion) return;
    if (document.activeElement === els.metricCompensacion) return;
    const n = Number(mm);
    els.metricCompensacion.value = Number.isFinite(n) ? String(n) : "0";
  }

  async function saveMeasureComp(mm, quiet) {
    try {
      const res = await fetch("/api/compensacion", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mm: mm }),
      });
      const data = await res.json();
      if (!data.ok) {
        if (!quiet) pushLocalEvent(data.error || "No se guardó la compensación.", "error");
        return;
      }
      persistCompLocal(mm);
      if (data.state) applyState(data.state);
    } catch (err) {
      if (!quiet) pushLocalEvent("No se guardó la compensación.", "error");
    }
  }

  function persistCompLocal(mm) {
    try {
      localStorage.setItem(STORAGE_COMP, String(mm));
    } catch (e) {
      /* ignore */
    }
  }

  function syncUiFromPersistedState(state) {
    if (!state) return;
    const comp = Number(state.compensacion_medicion_mm);
    if (Number.isFinite(comp)) {
      persistCompLocal(comp);
      setCompInput(comp);
    }
    const modelo = state.numero_modelo || "";
    if (modelo) {
      try {
        localStorage.setItem(STORAGE_MODEL, modelo);
      } catch (e) {
        /* ignore */
      }
      syncModelSelect(modelo);
    }
  }

  async function maybeRestoreModelFromLocal() {
    if (!els.selectModelo || els.selectModelo.value) return;
    let saved = "";
    try {
      saved = localStorage.getItem(STORAGE_MODEL) || "";
    } catch (e) {
      /* ignore */
    }
    if (saved && modelosCatalog.some(function (m) { return m.modelo === saved; })) {
      await selectModelo(saved, true);
    }
  }

  function setEventsPanelVisible(visible) {
    eventsEnabled = visible;
    els.eventsPanel.hidden = !visible;
    els.toggleEventsPanel.checked = visible;
    try {
      localStorage.setItem(STORAGE_EVENTS, visible ? "1" : "0");
    } catch (e) {
      /* ignore */
    }
    if (visible && pendingEventos.length) {
      renderEvents(pendingEventos);
    }
  }

  function getTheme() {
    const t = document.documentElement.getAttribute("data-theme");
    return t === "light" ? "light" : "dark";
  }

  function applyTheme(theme) {
    const next = theme === "light" ? "light" : "dark";
    document.documentElement.setAttribute("data-theme", next);
    try {
      localStorage.setItem(STORAGE_THEME, next);
    } catch (e) {
      /* ignore */
    }
    if (els.themeToggle) {
      els.themeToggle.setAttribute(
        "aria-label",
        next === "dark" ? "Activar modo claro" : "Activar modo oscuro"
      );
    }
  }

  function toggleTheme() {
    applyTheme(getTheme() === "dark" ? "light" : "dark");
  }

  function loadThemePreference() {
    try {
      const saved = localStorage.getItem(STORAGE_THEME);
      if (saved === "light" || saved === "dark") {
        applyTheme(saved);
        return;
      }
    } catch (e) {
      /* ignore */
    }
    applyTheme("dark");
  }

  function loadEventsPreference() {
    try {
      const v = localStorage.getItem(STORAGE_EVENTS);
      setEventsPanelVisible(v === "1");
    } catch (e) {
      setEventsPanelVisible(false);
    }
  }

  function applyState(state) {
    if (!state) return;

    const estado = state.estado || "idle";
    let bannerEstado = estado;
    if (state.rutina_manos) bannerEstado = "stopped";
    else if (state.rutina) bannerEstado = "manual";
    els.statusBanner.className =
      "status-banner status-banner--full " +
      (bannerClassByEstado[bannerEstado] || bannerClassByEstado.idle);
    els.statusBannerLabel.textContent =
      estado === "error" && state.ciclo_error
        ? (state.estado_label || "Error") + " · " + state.ciclo_error
        : state.estado_label || "En espera";
    els.statusBannerDetail.textContent = state.mensaje || "";

    const pct = Math.min(100, Math.max(0, Number(state.progreso_pct) || 0));
    els.progressPct.textContent = Math.round(pct) + " %";
    els.progressFill.style.width = pct + "%";

    syncModelSelect(state.numero_modelo || "");
    els.metricLongitud.textContent = formatMm(state.longitud_modelo_mm);
    if (els.cycleCutMm) {
      const len = Number(state.longitud_modelo_mm);
      const off = Number(state.compensacion_medicion_mm);
      const cut = len - LM_PIECE_OFFSET_MM + (Number.isFinite(off) ? off : 0);
      els.cycleCutMm.value = Number.isFinite(len) && len > 0 ? cut.toFixed(2) : "";
    }
    setCantidadInput(state.cantidad, !!state.ciclo_en_curso || !!state.charola_llena);
    setCompInput(state.compensacion_medicion_mm);
    els.metricTiempoCiclo.textContent = formatSeconds(state.tiempo_ciclo_s);
    els.metricTiempoEstimado.textContent = formatSeconds(state.tiempo_estimado_s);
    els.metricTiempoCorriendo.textContent = formatSeconds(state.tiempo_corriendo_s);

    applyServerEvents(state, estado);

    syncProcessControls(state);
    syncCycleStepTestControls(state);
    syncRoutine(state);
    renderCycleSteps(
      state.ciclo_paso_id || "",
      state.ciclo_paso_indice,
      state.ciclo_paralelo_id || ""
    );
    const pfCom = !!(state.prefeeder && state.prefeeder.com);
    setStatePoll(
      estado === "running" ||
        estado === "paused" ||
        estado === "error" ||
        !!state.rutina ||
        pfCom
    );
  }

  function renderCycleSteps(activeId, activeIndex, parallelId) {
    if (!els.cycleStepsList) return;
    els.cycleStepsList.innerHTML = "";
    const steps = cycleStepsCatalog.length ? cycleStepsCatalog : [];
    steps.forEach(function (step, idx) {
      const li = document.createElement("li");
      li.className = "cycle-steps__item";
      li.dataset.stepId = step.id;
      if (step.id === activeId) {
        li.classList.add("is-active");
      }
      if (parallelId && step.id === parallelId) {
        li.classList.add("is-parallel");
        li.title = "Siguiente ciclo, en paralelo";
      }
      if (activeIndex != null && idx < activeIndex) {
        li.classList.add("is-done");
      }
      const idSpan = document.createElement("span");
      idSpan.className = "cycle-steps__id";
      idSpan.textContent = "Paso " + step.id;
      const labelSpan = document.createElement("span");
      labelSpan.className = "cycle-steps__label";
      labelSpan.textContent = step.label;
      const actorSpan = document.createElement("span");
      actorSpan.className = "cycle-steps__actor";
      actorSpan.textContent = step.actor;
      li.appendChild(idSpan);
      li.appendChild(labelSpan);
      li.appendChild(actorSpan);
      els.cycleStepsList.appendChild(li);
    });
  }

  function fillCycleParamsForm(params) {
    if (!els.cycleParamsForm || !params) return;
    els.cycleParamsForm.querySelectorAll("[data-cycle-param]").forEach(function (input) {
      const key = input.dataset.cycleParam;
      const val = params[key];
      if (val == null || val === "") {
        input.value = "";
        return;
      }
      input.value = String(val);
    });
  }

  async function loadCycleConfig() {
    try {
      const res = await fetch("/api/cycle");
      const data = await res.json();
      if (!data.ok) return;
      cycleStepsCatalog = data.steps || [];
      fillCycleParamsForm(data.params);
      if (data.state) applyState(data.state);
      else renderCycleSteps("", 0);
    } catch (err) {
      pushLocalEvent("No se pudo cargar la secuencia de ciclo.", "error");
    }
  }

  async function saveCycleParams(ev) {
    if (ev) ev.preventDefault();
    if (!els.cycleParamsForm) return;
    const params = {};
    els.cycleParamsForm.querySelectorAll("[data-cycle-param]").forEach(function (input) {
      const key = input.dataset.cycleParam;
      const raw = input.value.trim();
      if (raw === "") {
        params[key] = null;
        return;
      }
      if (key === "feed_sides") {
        params[key] = raw.toUpperCase();
      } else if (key.indexOf("_mm") >= 0 || key.indexOf("per_mm") >= 0) {
        params[key] = parseFloat(raw);
      } else {
        params[key] = parseInt(raw, 10);
      }
    });
    try {
      const res = await fetch("/api/cycle", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ params: params }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent("No se guardaron parámetros de ciclo.", "error");
        return;
      }
      fillCycleParamsForm(data.params);
      pushLocalEvent("Parámetros de ciclo guardados.");
    } catch (err) {
      pushLocalEvent("Sin conexión al guardar ciclo.", "error");
    }
  }

  function setStatePoll(on) {
    if (statePollTimer) {
      window.clearInterval(statePollTimer);
      statePollTimer = null;
    }
    if (!on) return;
    statePollTimer = window.setInterval(fetchState, 400);
  }

  function getProcessPanel() {
    return document.querySelector('[data-block="proceso"]');
  }

  function setToggleLabel(btn, on) {
    if (!btn) return;
    if (btn.getAttribute("data-motion-op") === "lmguide_servo") {
      btn.textContent = on ? "Encendido" : "Apagado";
      return;
    }
    if (btn.hasAttribute("data-plc-op")) {
      btn.textContent = on ? "Encendido" : "Apagado";
    }
  }

  function syncProcessControls(state) {
    const panel = getProcessPanel();
    if (!panel) return;

    const estado = state.estado || "idle";
    const enCurso = !!state.ciclo_en_curso;
    const enError = estado === "error" || !!state.ciclo_error;
    const running = estado === "running";
    const paused = estado === "paused";
    const rutina = !!state.rutina;
    const purga = state.rutina === "purga";
    const ch = state.checklist || {};
    const checklistReady = !!ch.ok || !!state.checklist_bypass;

    const sidesEl = document.querySelector("[data-cycle-param='feed_sides']");
    const sidesRaw =
      state.feed_sides ||
      (sidesEl ? String(sidesEl.value || "").trim() : "") ||
      "";
    const sides = String(sidesRaw).trim().toUpperCase();
    const piezas = parseInt(String(state.cantidad), 10);
    const ambos =
      sides === "LR" || sides === "RL" || sides === "BOTH" || sides === "AMBOS";
    const cantidadImpar =
      ambos && (!Number.isFinite(piezas) || piezas < 2 || piezas % 2 !== 0);
    const pfCom = !!(state.prefeeder && state.prefeeder.com);

    const start = panel.querySelector('[data-action="start"]');
    const resume = panel.querySelector('[data-action="resume"]');
    const pause = panel.querySelector('[data-action="pause"]');
    const stop = panel.querySelector('[data-action="stop"]');
    const reset = panel.querySelector('[data-action="reset"]');

    panel.querySelectorAll("[data-action]").forEach(function (btn) {
      btn.classList.remove("is-active");
      btn.removeAttribute("aria-pressed");
    });

    if (start) {
      start.disabled =
        !checklistReady ||
        purga ||
        running ||
        (paused && enCurso) ||
        cantidadImpar ||
        enError;
      if (!checklistReady) {
        start.title = "Completa la revisión de arranque.";
      } else if (purga) {
        start.title = "Hay una purga en curso.";
      } else if (cantidadImpar) {
        start.title =
          "Con ambos lados la cantidad tiene que ser par y de al menos 2 piezas.";
      } else if (running) {
        start.title = "El ciclo ya está en marcha.";
      } else if (enError) {
        start.title = "Corrige el error y pulsa Reset.";
      } else {
        start.title = "";
      }
    }
    if (resume) {
      resume.disabled = !enCurso || !paused || purga || enError;
      resume.title = resume.disabled
        ? paused && !enCurso
          ? "No hay un ciclo pausado para reanudar."
          : ""
        : "Continúa el ciclo desde donde quedó.";
    }
    if (pause) {
      pause.disabled = (!running && !(enError && enCurso)) || rutina;
      pause.title = pause.disabled ? "Solo disponible con el ciclo en marcha." : "";
    }
    if (stop) {
      stop.disabled = !(running || paused || enCurso || rutina || enError || pfCom);
      stop.title = stop.disabled
        ? ""
        : "Detiene el ciclo, la rutina o bloquea el PreAlimentador.";
    }
    if (reset) {
      reset.disabled = !enError;
      reset.title = enError
        ? "Cuando PLC, Motion y PreAlimentador estén bien: estabiliza y limpia el error."
        : "Solo aplica cuando hay un error activo.";
    }

    if (enError && reset && !reset.disabled) {
      reset.classList.add("is-active");
      reset.setAttribute("aria-pressed", "true");
    } else if (paused && enCurso && resume && !resume.disabled) {
      resume.classList.add("is-active");
      resume.setAttribute("aria-pressed", "true");
    }

    if (els.statusBannerDetail && !enError && (estado === "idle" || estado === "paused")) {
      let hint = "";
      if (start && start.disabled) {
        if (!checklistReady) {
          hint = "Completa la revisión de arranque para poder iniciar el ciclo.";
        } else if (cantidadImpar) {
          hint =
            "Lote en ambos lados (L+R): pon cantidad par de al menos 2 piezas, o cambia el ciclo a un solo lado.";
        }
      }
      if (hint) {
        els.statusBannerDetail.textContent = hint;
        els.statusBannerDetail.classList.add("is-warn");
      } else {
        els.statusBannerDetail.textContent = state.mensaje || "";
        els.statusBannerDetail.classList.remove("is-warn");
      }
    }
  }

  function flashButton(btn) {
    btn.classList.remove("btn--flash");
    void btn.offsetWidth;
    btn.classList.add("btn--flash");

    const latched =
      btn.classList.contains("is-active") ||
      btn.getAttribute("aria-pressed") === "true";

    if (!latched) {
      btn.classList.add("is-vivid");
      window.setTimeout(function () {
        if (
          !btn.classList.contains("is-active") &&
          btn.getAttribute("aria-pressed") !== "true"
        ) {
          btn.classList.remove("is-vivid");
        }
      }, 450);
    }

    btn.addEventListener(
      "animationend",
      function onEnd() {
        btn.classList.remove("btn--flash");
        btn.removeEventListener("animationend", onEnd);
      },
      { once: true }
    );
  }

  function applyServerEvents(state, estado) {
    if (!Array.isArray(state.eventos) || clearingLog) return;
    const rev = Number(state.eventos_rev);
    const known = Number.isFinite(rev);
    if (known && rev < eventsRev) return;
    if (known && rev === eventsRev && !state.eventos.length) return;
    if (!known && !state.eventos.length) return;
    if (known) eventsRev = rev;
    pendingEventos = state.eventos;
    if (eventsEnabled) renderEvents(pendingEventos);
    else if (estado === "error" && state.eventos[0]) {
      setEventsPanelVisible(true);
      if (els.toggleEventsPanel) els.toggleEventsPanel.checked = true;
      renderEvents(pendingEventos);
    }
  }

  async function clearEventsLog() {
    if (!els.btnClearEvents || clearingLog) return;
    clearingLog = true;
    els.btnClearEvents.disabled = true;
    pendingEventos = [];
    if (els.eventsLog) els.eventsLog.value = "";
    try {
      const res = await fetch("/api/events/clear", { method: "POST" });
      const data = await res.json().catch(function () { return {}; });
      if (!res.ok || !data.ok) {
        throw new Error(data.error || "No se pudo borrar el registro.");
      }
      if (Number.isFinite(Number(data.eventos_rev))) eventsRev = Number(data.eventos_rev);
      pendingEventos = [];
      if (els.eventsLog) els.eventsLog.value = "";
    } catch (err) {
      clearingLog = false;
      pushLocalEvent(err.message || "No se pudo borrar el registro.", "error");
    } finally {
      clearingLog = false;
      if (els.btnClearEvents) els.btnClearEvents.disabled = false;
    }
  }

  function formatEventLine(ev) {
    const mark =
      ev.tipo === "error"
        ? "ERROR  "
        : ev.tipo === "warn"
          ? "AVISO  "
          : ev.tipo === "prod"
            ? "PROD   "
            : "";
    const text = mark + (ev.texto || "");
    return ev.hora ? ev.hora + "  " + text : text;
  }

  function renderEvents(eventos) {
    if (!eventsEnabled || !els.eventsLog) return;
    const next = (eventos || []).slice().reverse().map(formatEventLine).join("\n");
    const box = els.eventsLog;
    if (box.value === next) return;
    const prev = box.value;
    const grewAtEnd = next.startsWith(prev);
    const stick = box.scrollHeight - box.scrollTop - box.clientHeight < 24;
    const start = box.selectionStart;
    const end = box.selectionEnd;
    const selecting = document.activeElement === box && start !== end;
    const scrollTop = box.scrollTop;
    box.value = next;
    if (selecting && grewAtEnd) {
      box.setSelectionRange(start, end);
      box.scrollTop = scrollTop;
    } else if (stick) {
      box.scrollTop = box.scrollHeight;
    } else {
      box.scrollTop = scrollTop;
    }
  }

  function pushLocalEvent(texto, tipo) {
    const entry = { texto: texto, tipo: tipo || "info", hora: formatTime(new Date()) };
    pendingEventos.unshift(entry);
    pendingEventos = pendingEventos.slice(0, 300);
    renderEvents(pendingEventos);
  }

  function syncModelSelect(modeloId) {
    if (!els.selectModelo) return;
    syncingModelSelect = true;
    els.selectModelo.value = modeloId || "";
    syncingModelSelect = false;
  }

  function populateModelSelect(modelos) {
    if (!els.selectModelo) return;
    modelosCatalog = modelos || [];
    const current = els.selectModelo.value;
    els.selectModelo.innerHTML = "";
    const empty = document.createElement("option");
    empty.value = "";
    empty.textContent = "— Seleccionar —";
    els.selectModelo.appendChild(empty);
    modelosCatalog.forEach(function (item) {
      const opt = document.createElement("option");
      opt.value = item.modelo;
      opt.textContent = item.modelo;
      opt.dataset.longitud = String(item.longitud_mm ?? "");
      els.selectModelo.appendChild(opt);
    });
    if (current) syncModelSelect(current);
  }

  async function loadModelos() {
    try {
      const res = await fetch("/api/modelos");
      const data = await res.json();
      if (!data.ok) return;
      populateModelSelect(data.modelos);
    } catch (err) {
      pushLocalEvent("No se pudo cargar el catálogo de modelos.", "error");
    }
  }

  async function selectModelo(modeloId, persist) {
    if (persist === undefined) persist = true;
    try {
      const res = await fetch("/api/modelo", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ modelo: modeloId }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se pudo seleccionar el modelo.", "error");
        return;
      }
      if (persist) {
        try {
          if (modeloId) localStorage.setItem(STORAGE_MODEL, modeloId);
          else localStorage.removeItem(STORAGE_MODEL);
        } catch (e) {
          /* ignore */
        }
      }
      if (data.state) applyState(data.state);
      if (data.motion_data) motionApplyDataFields(data.motion_data);
    } catch (err) {
      pushLocalEvent("Error al guardar el modelo.", "error");
    }
  }

  function setByPath(obj, path, value) {
    const parts = path.split(".");
    let cur = obj;
    for (let i = 0; i < parts.length - 1; i++) {
      const key = parts[i];
      if (!cur[key] || typeof cur[key] !== "object") cur[key] = {};
      cur = cur[key];
    }
    cur[parts[parts.length - 1]] = value;
  }

  function getByPath(obj, path) {
    const parts = path.split(".");
    let cur = obj;
    for (let i = 0; i < parts.length; i++) {
      if (cur == null) return undefined;
      cur = cur[parts[i]];
    }
    return cur;
  }

  function motionApplyScale(data) {
    if (!data) return;
    const steps = Number(data.steps_per_mm);
    if (Number.isFinite(steps) && steps > 0) motionStepsPerMm = steps;
    if (data.puu_sign === 1 || data.puu_sign === -1) motionPuuSign = data.puu_sign;
    const feed = Number(data.feed_steps_per_mm);
    if (Number.isFinite(feed) && feed > 0) motionFeedStepsPerMm = feed;
  }

  function feedMmToCounts(mm) {
    return Math.round(Number(mm) * motionFeedStepsPerMm);
  }

  function feedCountsToMmText(counts) {
    const mm = Number(counts) / (motionFeedStepsPerMm || 1);
    const target = Math.round(Number(counts));
    for (let d = 1; d <= 4; d++) {
      const f = Math.pow(10, d);
      const shown = Math.round(mm * f) / f;
      if (Math.round(shown * motionFeedStepsPerMm) === target) return String(shown);
    }
    return String(Math.round(mm * 10000) / 10000);
  }

  function motionMmToPuu(mm) {
    return Math.round(motionPuuSign * mm * motionStepsPerMm);
  }

  function motionRefreshPuuReadout() {
    const input = document.getElementById("lmMoveMm");
    const out = document.getElementById("lmMovePuu");
    if (!input || !out) return;
    const raw = input.value.trim();
    const mm = Number(raw);
    if (raw === "" || !Number.isFinite(mm)) {
      out.textContent = "—";
      return;
    }
    out.textContent = String(motionMmToPuu(mm));
  }

  function motionFormatMm(puu) {
    const mm = Math.abs(Number(puu)) / (motionStepsPerMm || 1);
    if (!Number.isFinite(mm)) return "";
    return String(Math.round(mm * 1000) / 1000);
  }

  function motionFormFromTune(tune) {
    if (!els.motionTuneForm || !tune) return;
    els.motionTuneForm.querySelectorAll("[data-motion-path]").forEach(function (input) {
      const path = input.getAttribute("data-motion-path");
      const val = getByPath(tune, path);
      if (val == null) return;
      if (input.type === "checkbox") {
        input.checked = !!val;
        return;
      }
      const unit = input.getAttribute("data-motion-unit");
      if (unit === "mm") {
        input.value = motionFormatMm(val);
        return;
      }
      if (unit === "feed_mms" || unit === "feed_mm") {
        input.value = feedCountsToMmText(val);
        return;
      }
      input.value = String(val);
    });
    motionRefreshPuuReadout();
  }

  function motionTuneFromForm() {
    const tune = {
      feeder: { R: {}, L: {} },
      lmguide: {},
    };
    if (!els.motionTuneForm) return tune;
    els.motionTuneForm.querySelectorAll("[data-motion-path]").forEach(function (input) {
      const path = input.getAttribute("data-motion-path");
      if (input.type === "checkbox") {
        setByPath(tune, path, input.checked);
        return;
      }
      const raw = input.value.trim();
      if (raw === "") return;
      let num = Number(raw);
      if (!Number.isFinite(num)) {
        setByPath(tune, path, raw);
        return;
      }
      const unit = input.getAttribute("data-motion-unit");
      if (unit === "mm") num = motionMmToPuu(num);
      if (unit === "feed_mms" || unit === "feed_mm") num = feedMmToCounts(num);
      setByPath(tune, path, num);
    });
    return tune;
  }

  function motionApplyDataFields(data) {
    if (!data) return;
    if (els.motionDataModelo) els.motionDataModelo.value = data.modelo_id || "";
    if (els.motionDataTargetMm) {
      els.motionDataTargetMm.value =
        data.target_length_mm != null ? String(data.target_length_mm) : "";
    }
    if (els.motionDataToleranceMm) {
      els.motionDataToleranceMm.value =
        data.tolerance_mm != null ? String(data.tolerance_mm) : "2";
    }
  }

  async function loadMotionTune() {
    try {
      const res = await fetch("/api/motion/tune");
      const data = await res.json();
      if (!data.ok) return;
      motionApplyScale(data);
      if (data.tune) motionFormFromTune(data.tune);
    } catch (err) {
      pushLocalEvent("No se pudo cargar ajustes Motion.", "error");
    }
    try {
      const resData = await fetch("/api/motion/data");
      const payload = await resData.json();
      if (payload.ok && payload.data) motionApplyDataFields(payload.data);
    } catch (err) {
      /* ignore */
    }
  }

  function setMotionHint(text) {
    if (!els.motionEsp32Hint) return;
    els.motionEsp32Hint.hidden = !text;
    els.motionEsp32Hint.textContent = text || "";
  }

  function latchLmServo(on) {
    const btn = document.getElementById("lmServoToggle");
    if (!btn) return;
    btn.setAttribute("aria-pressed", on ? "true" : "false");
    btn.classList.toggle("is-active", !!on);
    setToggleLabel(btn, on);
  }

  async function sendMotionOp(op, extra) {
    const urgent = op === "lmguide_stop";
    if (motionCommandInFlight && !urgent) {
      pushLocalEvent("Motion está ejecutando otra orden. Usa Paro o espera a que termine.", "error");
      return null;
    }
    if (!urgent) motionCommandInFlight = true;
    const body = Object.assign({ op: op, tune: motionTuneFromForm() }, extra || {});
    try {
      const res = await fetch("/api/motion/cmd", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se armó la orden Motion.", "error");
        return null;
      }
      if (!data.sent) {
        if (op !== "set_tune") {
          pushLocalEvent(data.reason || "No se envió la orden Motion.", "error");
          return null;
        }
        return data;
      }
      if (data.command_ok === false) {
        pushLocalEvent(data.error || "El MCU Motion rechazó la orden: " + op, "error");
        return null;
      }
      motionApplyScale(data);
      if (data.tune) motionFormFromTune(data.tune);
      if (data.sent) {
        pushLocalEvent("Motion: orden enviada.");
        setMotionHint("Listo.");
      } else {
        const reason = data.reason || "No se envió al puerto.";
        pushLocalEvent(reason, "error");
        setMotionHint(reason);
      }
      return data;
    } catch (err) {
      pushLocalEvent("Error al enviar la orden Motion.", "error");
      return null;
    } finally {
      if (!urgent) motionCommandInFlight = false;
    }
  }

  async function saveMotionTune(ev) {
    if (ev && ev.preventDefault) ev.preventDefault();
    const tune = motionTuneFromForm();
    try {
      const res = await fetch("/api/motion/tune", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ tune: tune }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se guardaron ajustes Motion.", "error");
        return;
      }
      motionApplyScale(data);
      if (data.tune) motionFormFromTune(data.tune);
      pushLocalEvent("Ajustes Motion guardados.");
      const pushed = await sendMotionOp("set_tune", { tune: data.tune });
      if (pushed && pushed.sent === false) {
        setMotionHint(pushed.reason || "Guardado en disco; sin puerto Motion para enviar al MCU.");
      }
    } catch (err) {
      pushLocalEvent("Sin conexión al guardar ajustes Motion.", "error");
    }
  }

  async function runMotionButton(btn) {
    const op = btn.getAttribute("data-motion-op");
    if (!op) return;
    flashButton(btn);
    const extra = {};
    if (op === "lmguide_servo") {
      const on = btn.getAttribute("aria-pressed") !== "true";
      extra.on = on;
      const data = await sendMotionOp(op, extra);
      if (data && data.command_ok === true) latchLmServo(on);
      return;
    }
    if (op === "lmguide_init") {
      const data = await sendMotionOp(op);
      if (data && data.command_ok === true) latchLmServo(true);
      return;
    }
    if (op === "motion_hw_init") {
      const data = await sendMotionOp(op);
      if (data && data.command_ok === true) latchLmServo(true);
      return;
    }
    if (op === "lmguide_move") {
      const raw = (document.getElementById("lmMoveMm") || {}).value || "";
      const mm = Number(String(raw).trim());
      if (!Number.isFinite(mm)) {
        pushLocalEvent("Escribe la posición del LM Guide en mm.", "error");
        return;
      }
      extra.mm = mm;
    }
    if (op === "encoder_reset") {
      extra.side = btn.getAttribute("data-motion-side") || "R";
      const data = await sendMotionOp(op, extra);
      if (data) pollEncoder();
      return;
    }
    if (op === "feeder_test") {
      extra.side = btn.getAttribute("data-motion-side") || "R";
      const data = await sendMotionOp(op, extra);
      if (data) pollEncoder();
      return;
    }
    if (op === "lmguide_jog") {
      const raw = (document.getElementById("lmJogDeltaMm") || {}).value || "";
      const mm = Number(String(raw).trim());
      if (!Number.isFinite(mm) || mm === 0) {
        pushLocalEvent("Escribe un jog en mm distinto de cero.", "error");
        return;
      }
      extra.jog_mm = Math.abs(mm);
      extra.dir = Number(btn.getAttribute("data-motion-dir") || 1);
    }
    await sendMotionOp(op, extra);
  }

  function motionShowEncoder(encoder) {
    ["R", "L"].forEach(function (side) {
      const el = document.getElementById(side === "R" ? "encReadR" : "encReadL");
      const led = document.getElementById(side === "R" ? "laserLedR" : "laserLedL");
      const sideData = encoder && encoder[side];
      if (el) {
        const mm = sideData && sideData.mm;
        el.textContent = mm == null || !Number.isFinite(Number(mm)) ? "—" : Number(mm).toFixed(2) + " mm";
      }
      if (led) {
        const on = !!(sideData && sideData.laser === true);
        const state = led.querySelector(".motion-laser__state");
        const label = on ? "Detectado" : "Sin detección";
        led.classList.toggle("is-on", on);
        if (state && state.textContent !== label) state.textContent = label;
        led.setAttribute("aria-label", "Láser lado " + side + (on ? " detectado" : " sin detección"));
      }
    });
  }

  async function pollEncoder() {
    if (motionEncPollBusy || motionCommandInFlight) return;
    motionEncPollBusy = true;
    try {
      const res = await fetch("/api/motion/status");
      const data = await res.json();
      if (data.ok && data.sent) {
        motionShowEncoder(data.encoder);
        if (typeof data.lm_servo_on === "boolean") latchLmServo(data.lm_servo_on);
      }
    } catch (err) {
      motionShowEncoder(null);
    } finally {
      motionEncPollBusy = false;
    }
  }

  function setEncoderPoll(on) {
    if (motionEncPollTimer) {
      window.clearInterval(motionEncPollTimer);
      motionEncPollTimer = null;
    }
    if (!on) return;
    pollEncoder();
    motionEncPollTimer = window.setInterval(pollEncoder, 400);
  }

  function activateSettingsTab(tabId) {
    els.settingsTabs.forEach(function (btn) {
      const active = btn.dataset.tab === tabId;
      btn.classList.toggle("is-active", active);
      btn.setAttribute("aria-selected", active ? "true" : "false");
    });
    els.settingsTabPanels.forEach(function (panel) {
      const active = panel.dataset.tabPanel === tabId;
      panel.classList.toggle("is-active", active);
      panel.hidden = !active;
    });
    setEncoderPoll(tabId === "motion");
    setPlcPoll(tabId === "plc");
    setPrefeederPoll(tabId === "prefeeder");
    if (tabId === "conexion") {
      loadSerialPorts();
    }
    if (tabId === "cycle") {
      loadCycleConfig();
    }
    if (tabId === "motion") {
      loadMotionTune();
    }
    if (tabId === "prefeeder") {
      loadPrefeederTune();
    }
  }

  async function fetchState() {
    try {
      const res = await fetch("/api/state");
      const data = await res.json();
      if (data.ok) {
        applyState(data);
        return data;
      }
    } catch (err) {
      pushLocalEvent("No se pudo leer el estado de la máquina.", "error");
    }
    return null;
  }

  function labelForAction(action) {
    const labels = {
      start: "Iniciar",
      step_start: "Prueba paso a paso",
      step_next: "Siguiente paso",
      resume: "Reanudar",
      pause: "Pausa",
      stop: "Stop",
      reset: "Reset",
      parametros: "Parámetros",
      mantenimiento: "Mantenimiento",
      checklist: "Checklist",
      purga: "Purga",
      purga_alimentar: "Alimentar",
      purga_limpiar: "Limpiar",
      purga_continuar: "Continuar",
      purga_terminar: "Terminar",
      posicion_inicial: "Posición inicial",
      charola_vacia: "Charola vacía",
      iniciar: "Iniciar (PreAlimentador)",
      alimentar_izquierda: "Alimentar izquierda",
      alimentar_derecha: "Alimentar derecha",
      omitir_holgura_izquierda: "Omitir holgura izquierda",
      omitir_holgura_derecha: "Omitir holgura derecha",
    };
    return labels[action] || action;
  }

  function syncCycleStepTestControls(state) {
    const stepMode = !!state.ciclo_paso_a_paso;
    const estado = state.estado || "idle";
    const enCurso = !!state.ciclo_en_curso;
    const enError = estado === "error" || !!state.ciclo_error;
    const running = estado === "running";
    const paused = estado === "paused";
    const rutina = !!state.rutina;
    const ch = state.checklist || {};
    const checklistReady = !!ch.ok || !!state.checklist_bypass;

    if (els.cycleStepStart) {
      els.cycleStepStart.disabled =
        !checklistReady || rutina || running || (paused && enCurso) || enError;
      els.cycleStepStart.title = !checklistReady
        ? "Completa la revisión de arranque."
        : running || (paused && enCurso)
          ? "Hay un ciclo en curso."
          : enError
            ? "Corrige el error y pulsa Reset."
            : "Inicia una pieza y pausa al terminar cada paso.";
    }
    if (els.cycleStepNext) {
      const canNext = stepMode && paused && enCurso && !enError && !rutina;
      els.cycleStepNext.disabled = !canNext;
      els.cycleStepNext.title = canNext
        ? "Ejecuta el siguiente paso y vuelve a pausar."
        : stepMode && running
          ? "Espera a que termine el paso en curso."
          : "Solo en pausa durante la prueba paso a paso.";
    }
    if (els.cycleStepStop) {
      els.cycleStepStop.disabled = !(stepMode && (running || paused || enCurso || enError));
      els.cycleStepStop.title = els.cycleStepStop.disabled
        ? ""
        : "Detiene la prueba paso a paso.";
    }
    if (els.cycleStepTestHint) {
      if (stepMode && running) {
        els.cycleStepTestHint.textContent =
          state.mensaje || "Ejecutando paso…";
      } else if (stepMode && paused && enCurso) {
        els.cycleStepTestHint.textContent =
          state.mensaje || "Paso listo. Pulsa «Siguiente paso».";
      } else if (stepMode && enError) {
        els.cycleStepTestHint.textContent =
          state.ciclo_error || state.mensaje || "Error en la prueba.";
      } else {
        els.cycleStepTestHint.textContent =
          "Ejecuta una pieza y se detiene al completar cada paso. Usa «Siguiente paso» para continuar.";
      }
    }
  }

  async function sendAction(block, action, extra) {
    try {
      const res = await fetch("/api/action", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(Object.assign({ block: block, action: action }, extra || {})),
      });
      const data = await res.json();
      if (!data.ok) {
        const errorText = data.error || "Error al registrar acción.";
        pushLocalEvent(errorText, "error");
        if (data.state) applyState(data.state);
        if (els.statusBannerDetail && els.statusBannerDetail.textContent !== errorText) {
          els.statusBannerDetail.textContent = errorText;
        }
        return;
      }
      pushLocalEvent("[" + block + "] " + labelForAction(action));
      if (block === "proceso" && PROCESS_ACTIONS.has(action)) {
        lastProcessAction = action;
      }
      if (data.state) applyState(data.state);
      if (data.motion_data) motionApplyDataFields(data.motion_data);
    } catch (err) {
      pushLocalEvent("Sin conexión con el servidor.", "error");
    }
  }

  function openParametros() {
    els.settingsModal.hidden = true;
    els.parametrosModal.hidden = false;
    activateSettingsTab("cycle");
    els.parametrosModalClose.focus();
  }

  function closeParametros(reopenSettings) {
    setEncoderPoll(false);
    setPlcPoll(false);
    els.parametrosModal.hidden = true;
    if (reopenSettings) {
      els.settingsModal.hidden = false;
      els.settingsModalClose.focus();
    } else {
      els.btnAppSettings.focus();
    }
  }

  let serialSyncing = false;

  /* ==========================================================================
   * ANDON_SKIP_OPCIONAL — TODO_BORRAR
   * Puente de banco: opción «No usar Andon», mensajes y fillPortSelect(extraNone).
   * Buscar este tag en app.py y rutinas-operador.md para eliminar todo junto.
   * ========================================================================== */
  /** True cuando Andon no se usa (Conexión «No usar» o HMI_ANDON_SKIP=1). */
  let serialAndonSkipped = false;
  /** Valor enviado al guardar cuando se elige «No usar Andon» (coincide con app.py). */
  let andonNoneToken = "__none__";

  function fillPortSelect(select, ports, selected, extraNone) {
    const current = selected || "";
    const seen = {};
    select.innerHTML = "";
    if (extraNone && extraNone.value) {
      const optNone = document.createElement("option");
      optNone.value = extraNone.value;
      optNone.textContent = extraNone.label || "—";
      select.appendChild(optNone);
    }
    const none = document.createElement("option");
    none.value = "";
    none.textContent = "Sin puerto";
    select.appendChild(none);
    (ports || []).forEach(function (port) {
      if (!port || !port.device || seen[port.device]) return;
      seen[port.device] = true;
      const opt = document.createElement("option");
      opt.value = port.device;
      opt.textContent = port.description ? port.device + " — " + port.description : port.device;
      select.appendChild(opt);
    });
    if (current && !seen[current]) {
      const missing = document.createElement("option");
      missing.value = current;
      missing.textContent = current + " — no está conectado";
      select.appendChild(missing);
    }
    select.value = current;
  }

  function setLinkStatus(el, text, ok) {
    if (!el) return;
    el.textContent = text;
    el.classList.toggle("is-error", ok === false);
    el.classList.toggle("is-ok", ok === true);
  }

  function showComErrorInBanner(message) {
    if (!message || !els.statusBanner) return;
    els.statusBanner.className =
      "status-banner status-banner--full status-banner--error";
    if (els.statusBannerLabel) els.statusBannerLabel.textContent = "Error";
    if (els.statusBannerDetail) els.statusBannerDetail.textContent = message;
  }

  function serialLinkText(name, port) {
    if (!port) return name + ": sin puerto. El ciclo no arranca.";
    return name + ": " + port + " guardado. Pulsa Probar COM.";
  }

  function andonConnectionStatusText(data) {
    if (!data) return "";
    if (data.andon_disabled) {
      return "Andon: no usado (puedes conectarlo más tarde en este menú).";
    }
    if (data.andon_skipped && !data.andon_disabled) {
      return "Andon: omitido en esta sesión (variable HMI_ANDON_SKIP=1).";
    }
    return "";
  }

  async function loadSerialPorts() {
    if (!els.selectMotionPort || !els.selectPlcPort || !els.selectAndonPort) return;
    serialSyncing = true;
    try {
      const res = await fetch("/api/serial");
      const data = await res.json();
      if (data.andon_none_value) {
        andonNoneToken = data.andon_none_value;
      }
      fillPortSelect(els.selectMotionPort, data.ports, data.motion);
      fillPortSelect(els.selectPlcPort, data.ports, data.plc);
      const andonSel =
        data.andon_disabled && data.andon_none_value ? data.andon_none_value : data.andon;
      fillPortSelect(els.selectAndonPort, data.ports, andonSel, {
        value: andonNoneToken,
        label: "No usar Andon",
      });
      if (els.selectPrefeederPort) {
        fillPortSelect(els.selectPrefeederPort, data.ports, data.prefeeder || "");
      }
      serialAndonSkipped = !!data.andon_skipped;
      if (!data.motion) setLinkStatus(els.serialMotionStatus, serialLinkText("Motion", ""), false);
      if (!data.plc) setLinkStatus(els.serialPlcStatus, serialLinkText("PLC", ""), false);
      const andonHint = andonConnectionStatusText(data);
      if (andonHint && els.serialAndonStatus) {
        setLinkStatus(els.serialAndonStatus, andonHint, null);
      } else if (!data.andon && !data.andon_disabled) {
        setLinkStatus(els.serialAndonStatus, serialLinkText("Andon", ""), false);
      }
      if (els.serialPrefeederStatus && !data.prefeeder) {
        setLinkStatus(
          els.serialPrefeederStatus,
          "PreAlimentador: sin puerto (opcional).",
          null
        );
      }
    } catch (err) {
      if (els.serialMotionStatus) els.serialMotionStatus.textContent = "No se pudo leer la lista de puertos.";
    } finally {
      serialSyncing = false;
    }
  }

  async function saveSerialPorts() {
    if (serialSyncing || !els.selectMotionPort || !els.selectPlcPort || !els.selectAndonPort) return;
    if (els.serialMotionStatus) els.serialMotionStatus.textContent = "Guardando puertos…";
    if (els.serialPlcStatus) els.serialPlcStatus.textContent = "";
    if (els.serialAndonStatus) els.serialAndonStatus.textContent = "";
    try {
      const res = await fetch("/api/serial", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          motion: els.selectMotionPort.value,
          plc: els.selectPlcPort.value,
          andon: els.selectAndonPort.value,
          prefeeder: els.selectPrefeederPort ? els.selectPrefeederPort.value : "",
        }),
      });
      const data = await res.json();
      if (!data.ok) {
        const err = data.error || "No se pudo guardar el puerto.";
        if (els.serialMotionStatus) setLinkStatus(els.serialMotionStatus, "No guardado.", false);
        showComErrorInBanner(err);
        await loadSerialPorts();
        return;
      }
      if (els.serialMotionStatus) {
        setLinkStatus(
          els.serialMotionStatus,
          serialLinkText("Motion", data.motion),
          data.motion ? null : false
        );
      }
      if (els.serialPlcStatus) {
        setLinkStatus(
          els.serialPlcStatus,
          serialLinkText("PLC", data.plc),
          data.plc ? null : false
        );
      }
      if (els.serialAndonStatus) {
        const hint = andonConnectionStatusText(data);
        if (hint) {
          setLinkStatus(els.serialAndonStatus, hint, null);
          serialAndonSkipped = !!data.andon_skipped;
        } else {
          serialAndonSkipped = !!data.andon_skipped;
          setLinkStatus(
            els.serialAndonStatus,
            serialLinkText("Andon", data.andon),
            data.andon ? null : false
          );
        }
      }
      if (els.serialPrefeederStatus && els.selectPrefeederPort) {
        const pf = data.prefeeder || "";
        setLinkStatus(
          els.serialPrefeederStatus,
          pf ? serialLinkText("PreAlimentador", pf) : "PreAlimentador: sin puerto (opcional).",
          pf ? null : null
        );
      }
    } catch (err) {
      if (els.serialMotionStatus) els.serialMotionStatus.textContent = "Sin conexión con el servidor.";
    }
  }

  async function loadChecklistBypass() {
    if (!els.toggleChecklistBypass) return;
    try {
      const res = await fetch("/api/settings/checklist-bypass");
      const data = await res.json();
      if (data.ok) els.toggleChecklistBypass.checked = data.enabled === true;
    } catch (err) {
      pushLocalEvent("No se pudo leer el bypass del checklist.", "error");
    }
  }

  async function saveChecklistBypass() {
    if (!els.toggleChecklistBypass) return;
    const enabled = els.toggleChecklistBypass.checked;
    try {
      const res = await fetch("/api/settings/checklist-bypass", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: enabled }),
      });
      const data = await res.json();
      if (!data.ok) throw new Error(data.error || "No se pudo guardar.");
      els.toggleChecklistBypass.checked = data.enabled === true;
      pushLocalEvent(data.enabled ? "Bypass de checklist activo." : "Checklist obligatorio.");
      await fetchState();
    } catch (err) {
      els.toggleChecklistBypass.checked = !enabled;
      pushLocalEvent(err.message || "No se pudo guardar el bypass del checklist.", "error");
    }
  }

  function openSettings() {
    els.parametrosModal.hidden = true;
    els.settingsModal.hidden = false;
    loadChecklistBypass();
    els.settingsModalClose.focus();
  }

  function closeSettings() {
    els.settingsModal.hidden = true;
    els.btnAppSettings.focus();
  }

  if (els.selectMotionPort) els.selectMotionPort.addEventListener("change", saveSerialPorts);
  if (els.selectPlcPort) els.selectPlcPort.addEventListener("change", saveSerialPorts);
  if (els.selectAndonPort) els.selectAndonPort.addEventListener("change", saveSerialPorts);
  if (els.selectPrefeederPort) els.selectPrefeederPort.addEventListener("change", saveSerialPorts);
  async function testModuleCom(moduleName, statusEl) {
    if (moduleName === "andon" && serialAndonSkipped) {
      setLinkStatus(statusEl, "Andon no usado. No hay COM que probar.", null);
      return;
    }
    setLinkStatus(statusEl, "Probando…", null);
    try {
      const res = await fetch("/api/serial/test", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ module: moduleName }),
      });
      const data = await res.json();
      if (data.ok) {
        setLinkStatus(statusEl, data.message, true);
        fetchState();
      } else {
        showComErrorInBanner(data.error || "El COM no respondió.");
        setLinkStatus(statusEl, "Sin respuesta.", false);
      }
    } catch (err) {
      showComErrorInBanner("Sin conexión con el servidor.");
      setLinkStatus(statusEl, "Sin conexión.", false);
    }
  }

  if (els.btnRefreshPorts) els.btnRefreshPorts.addEventListener("click", loadSerialPorts);
  if (els.btnMotionTest) {
    els.btnMotionTest.addEventListener("click", function () {
      testModuleCom("motion", els.serialMotionStatus);
    });
  }
  if (els.btnPlcTest) {
    els.btnPlcTest.addEventListener("click", function () {
      testModuleCom("plc", els.serialPlcStatus);
    });
  }
  if (els.btnAndonTest) {
    els.btnAndonTest.addEventListener("click", function () {
      testModuleCom("andon", els.serialAndonStatus);
    });
  }
  if (els.btnPrefeederTest) {
    els.btnPrefeederTest.addEventListener("click", function () {
      testModuleCom("prefeeder", els.serialPrefeederStatus);
    });
  }
  /* --- fin ANDON_SKIP_OPCIONAL (serial / conexión Andon en JS) — TODO_BORRAR --- */

  if (els.prefeederTuneReload) {
    els.prefeederTuneReload.addEventListener("click", loadPrefeederTune);
  }
  if (els.prefeederStatusRefresh) {
    els.prefeederStatusRefresh.addEventListener("click", pollPrefeeder);
  }
  if (els.prefeederTuneForm) {
    els.prefeederTuneForm.querySelectorAll("[data-pf-save]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        const side = btn.getAttribute("data-pf-save");
        if (side === "L" || side === "R") savePrefeederSide(side);
      });
    });
  }

  els.btnAppSettings.addEventListener("click", openSettings);
  els.settingsModalClose.addEventListener("click", closeSettings);
  els.parametrosModalClose.addEventListener("click", function () {
    closeParametros(false);
  });
  els.parametrosModalBack.addEventListener("click", function () {
    closeParametros(true);
  });
  els.settingsModal.addEventListener("click", function (e) {
    if (e.target === els.settingsModal) closeSettings();
  });
  els.parametrosModal.addEventListener("click", function (e) {
    if (e.target === els.parametrosModal) closeParametros(false);
  });
  document.addEventListener("keydown", function (e) {
    if (e.key !== "Escape") return;
    if (els.empleadoModal && !els.empleadoModal.hidden) {
      closeEmpleado();
      return;
    }
    if (els.checklistModal && !els.checklistModal.hidden) {
      closeChecklist();
      return;
    }
    if (els.mantModal && !els.mantModal.hidden) {
      closeMantenimiento();
      return;
    }
    if (!els.parametrosModal.hidden) {
      closeParametros(true);
      return;
    }
    if (!els.settingsModal.hidden) closeSettings();
  });

  if (els.checklistModalClose) els.checklistModalClose.addEventListener("click", closeChecklist);
  if (els.checklistModal) {
    els.checklistModal.addEventListener("click", function (e) {
      if (e.target === els.checklistModal) closeChecklist();
    });
  }
  if (els.checklistBody) {
    els.checklistBody.addEventListener("click", function (e) {
      const btn = e.target.closest("[data-check]");
      if (!btn || btn.disabled) return;
      const op = btn.getAttribute("data-check");
      const extra = {};
      if (op === "checklist_empezar") {
        const input = document.getElementById("checkEmpleado");
        extra.empleado = input ? input.value.trim() : "";
      }
      if (op === "checklist_medicion") {
        const left = document.getElementById("checkMmL");
        const right = document.getElementById("checkMmR");
        extra.mm_l = left ? left.value : "";
        extra.mm_r = right ? right.value : "";
      }
      sendAction("checklist", op, extra);
    });
  }
  if (els.mantModalClose) els.mantModalClose.addEventListener("click", closeMantenimiento);
  if (els.mantModal) {
    els.mantModal.addEventListener("click", function (e) {
      if (e.target === els.mantModal) closeMantenimiento();
    });
  }
  if (els.mantReset) {
    els.mantReset.addEventListener("click", function () {
      const empleado = els.mantEmpleado ? els.mantEmpleado.value.trim() : "";
      sendAction("mantenimiento", "mantenimiento_reset", { empleado: empleado });
    });
  }
  if (els.empleadoModalClose) els.empleadoModalClose.addEventListener("click", closeEmpleado);
  if (els.empleadoConfirm) els.empleadoConfirm.addEventListener("click", confirmEmpleado);
  if (els.empleadoInput) {
    els.empleadoInput.addEventListener("keydown", function (e) {
      if (e.key === "Enter") {
        e.preventDefault();
        confirmEmpleado();
      }
    });
  }
  if (els.empleadoModal) {
    els.empleadoModal.addEventListener("click", function (e) {
      if (e.target === els.empleadoModal) closeEmpleado();
    });
  }

  if (els.btnClearEvents) els.btnClearEvents.addEventListener("click", clearEventsLog);

  els.toggleEventsPanel.addEventListener("change", function () {
    setEventsPanelVisible(els.toggleEventsPanel.checked);
  });
  if (els.toggleChecklistBypass) {
    els.toggleChecklistBypass.addEventListener("change", saveChecklistBypass);
  }

  els.settingsTabs.forEach(function (btn) {
    btn.addEventListener("click", function () {
      activateSettingsTab(btn.dataset.tab);
    });
  });

  if (els.motionTuneForm) {
    els.motionTuneForm.addEventListener("submit", saveMotionTune);
  }
  if (els.motionTuneReload) {
    els.motionTuneReload.addEventListener("click", loadMotionTune);
  }
  if (els.motionStatusRefresh) {
    els.motionStatusRefresh.addEventListener("click", pollEncoder);
  }
  if (els.cycleParamsForm) {
    els.cycleParamsForm.addEventListener("submit", saveCycleParams);
  }
  if (els.cycleStepStart) {
    els.cycleStepStart.addEventListener("click", function () {
      sendAction("proceso", "step_start");
    });
  }
  if (els.cycleStepNext) {
    els.cycleStepNext.addEventListener("click", function () {
      sendAction("proceso", "step_next");
    });
  }
  if (els.cycleStepStop) {
    els.cycleStepStop.addEventListener("click", function () {
      sendAction("proceso", "stop");
    });
  }

  const lmMoveMm = document.getElementById("lmMoveMm");
  if (lmMoveMm) {
    lmMoveMm.addEventListener("input", motionRefreshPuuReadout);
  }

  if (els.motionTuneForm) {
    els.motionTuneForm.querySelectorAll("[data-motion-op]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        runMotionButton(btn);
      });
    });
  }

  function plcButton(name) {
    if (!els.plcManualForm) return null;
    return els.plcManualForm.querySelector('[data-plc-name="' + name + '"]');
  }

  function setPlcPressed(btn, on) {
    if (!btn) return;
    btn.setAttribute("aria-pressed", on ? "true" : "false");
    btn.classList.toggle("is-active", !!on);
    if (!on) btn.classList.remove("is-vivid");
    setToggleLabel(btn, on);
  }

  function setPlcHint(text) {
    if (!els.plcEsp32Hint) return;
    els.plcEsp32Hint.hidden = !text;
    els.plcEsp32Hint.textContent = text || "";
  }

  function applyPlcStatus(status) {
    if (!status) return;
    PLC_FAULTS.forEach(function (pair) {
      const el = document.getElementById(pair[0]);
      if (el) el.classList.toggle("is-on", status[pair[1]] === true);
    });
    if (els.plcGraceNote) els.plcGraceNote.hidden = status.boot_grace !== true;
    const blowerBtn = els.plcManualForm
      ? els.plcManualForm.querySelector('[data-plc-op="blower"]')
      : null;
    if (typeof status.blower === "boolean") setPlcPressed(blowerBtn, status.blower);
    if (status.pulse_busy) return;
    PLC_OUTPUTS.forEach(function (pair) {
      if (typeof status[pair[0]] === "boolean") setPlcPressed(plcButton(pair[1]), status[pair[0]]);
    });
  }

  function clearPlcOutputs() {
    PLC_OUTPUTS.forEach(function (pair) {
      setPlcPressed(plcButton(pair[1]), false);
    });
    const blowerBtn = els.plcManualForm
      ? els.plcManualForm.querySelector('[data-plc-op="blower"]')
      : null;
    setPlcPressed(blowerBtn, false);
  }

  async function sendPlcOp(op, extra) {
    const body = Object.assign({ op: op }, extra || {});
    try {
      const res = await fetch("/api/plc/cmd", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se armó la orden PLC.", "error");
        return null;
      }
      if (!data.sent) {
        pushLocalEvent(data.reason || "No se envió la orden PLC.", "error");
        return null;
      }
      if (data.command_ok !== true) {
        pushLocalEvent("El MCU PLC no confirmó la orden: " + op, "error");
        return null;
      }
      if (data.sent) {
        pushLocalEvent("PLC: orden enviada.");
        setPlcHint("Listo.");
      } else {
        const reason = data.reason || "No se envió al puerto.";
        pushLocalEvent(reason, "error");
        setPlcHint(reason);
      }
      if (data.status) applyPlcStatus(data.status);
      return data;
    } catch (err) {
      pushLocalEvent("Error al enviar la orden PLC.", "error");
      return null;
    }
  }

  async function runPlcButton(btn) {
    const op = btn.getAttribute("data-plc-op");
    if (!op) return;
    flashButton(btn);
    if (op === "valve") {
      const name = btn.getAttribute("data-plc-name");
      const on = btn.getAttribute("aria-pressed") !== "true";
      await sendPlcOp("valve", { name: name, on: on });
      return;
    }
    if (op === "blower") {
      const on = btn.getAttribute("aria-pressed") !== "true";
      if (!on) {
        await sendPlcOp("blower_off");
        return;
      }
      const raw = els.plcBlowerMs ? String(els.plcBlowerMs.value).trim() : "2000";
      const ms = raw === "" ? 2000 : Number(raw);
      if (!Number.isFinite(ms)) {
        pushLocalEvent("El tiempo del blower tiene que ser un número.", "error");
        return;
      }
      await sendPlcOp("blower_on", { ms: ms });
      return;
    }
    if (op === "all_off" || op === "reset") {
      await sendPlcOp(op);
    }
  }

  async function pollPlc() {
    try {
      const res = await fetch("/api/plc/status");
      const data = await res.json();
      if (data.ok && data.status) applyPlcStatus(data.status);
    } catch (err) {
      /* sin puerto: se conserva el estado de los botones */
    }
  }

  function setPlcPoll(on) {
    if (plcPollTimer) {
      window.clearInterval(plcPollTimer);
      plcPollTimer = null;
    }
    if (!on) return;
    pollPlc();
    plcPollTimer = window.setInterval(pollPlc, 400);
  }

  function setPfHint(text) {
    if (!els.prefeederEsp32Hint) return;
    els.prefeederEsp32Hint.hidden = !text;
    els.prefeederEsp32Hint.textContent = text || "";
  }

  function prefeederFormFromTune(tune) {
    if (!els.prefeederTuneForm || !tune) return;
    els.prefeederTuneForm.querySelectorAll("[data-pf-side][data-pf-key]").forEach(function (input) {
      const side = input.getAttribute("data-pf-side");
      const key = input.getAttribute("data-pf-key");
      const block = tune[side];
      if (!block || block[key] == null) return;
      input.value = String(block[key]);
    });
  }

  function prefeederSideFromForm(side) {
    const out = {};
    if (!els.prefeederTuneForm) return out;
    els.prefeederTuneForm.querySelectorAll('[data-pf-side="' + side + '"][data-pf-key]').forEach(function (input) {
      const key = input.getAttribute("data-pf-key");
      const raw = input.value.trim();
      if (raw === "") return;
      const num = Number(raw);
      if (!Number.isFinite(num)) return;
      out[key] = num;
    });
    return out;
  }

  function pfPaintBanner(el, sideData) {
    if (!el) return;
    if (!sideData || !sideData.online) {
      el.textContent = sideData ? "Sin enlace" : "—";
      return;
    }
    if (sideData.state === "error") {
      el.textContent = "—";
      return;
    }
    if (sideData.state === "busy") {
      el.textContent = "Busy";
      return;
    }
    el.textContent = "Idle";
  }

  function pfPaintSensor(ledEl, on, faultStyle) {
    if (!ledEl) return;
    const state = ledEl.querySelector(".motion-laser__state");
    const label = on ? (faultStyle ? "Activo" : "OK") : "Off";
    ledEl.classList.toggle("is-on", !!on);
    ledEl.classList.toggle("is-fault", !!(on && faultStyle));
    if (state) state.textContent = label;
  }

  function applyPrefeederStatus(status) {
    if (!status) return;
    const tuneFromMaster = {};
    ["L", "R"].forEach(function (side) {
      const block = status[side];
      pfPaintBanner(side === "L" ? els.pfBanL : els.pfBanR, block);
      const sen = (block && block.sensors) || {};
      pfPaintSensor(document.getElementById("pfCyl" + side), !!sen.cylinder, true);
      pfPaintSensor(document.getElementById("pfHose" + side), !!sen.hose, true);
      pfPaintSensor(document.getElementById("pfHolg" + side), !!sen.holgura, false);
      if (block && block.cfg) tuneFromMaster[side] = block.cfg;
    });
    // Leer actuales del maestro sin pisar si el operador está editando.
    if (els.prefeederTuneForm && Object.keys(tuneFromMaster).length) {
      const active = document.activeElement;
      const editing = !!(active && els.prefeederTuneForm.contains(active));
      if (!editing) prefeederFormFromTune(tuneFromMaster);
    }
  }

  async function loadPrefeederTune() {
    try {
      const res = await fetch("/api/prefeeder/tune");
      const data = await res.json();
      if (data.ok && data.tune) prefeederFormFromTune(data.tune);
    } catch (err) {
      pushLocalEvent("No se pudo cargar parámetros del PreAlimentador.", "error");
    }
    await pollPrefeeder();
  }

  async function savePrefeederSide(side) {
    const patch = {};
    patch[side] = prefeederSideFromForm(side);
    try {
      const res = await fetch("/api/prefeeder/tune", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ tune: patch }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se guardó el lado " + side + ".", "error");
        return;
      }
      if (data.tune) prefeederFormFromTune(data.tune);
      const pushed = data.pushed && data.pushed[side];
      const applied = data.applied && data.applied[side];
      if (applied === false) {
        setPfHint("No quedó aplicado en el maestro (revisa COM). Los valores no se guardaron en el ESP32.");
        pushLocalEvent(
          "PreAlimentador " + side + ": el maestro no refleja los parámetros guardados.",
          "error"
        );
      } else if (!pushed) {
        setPfHint("Sin respuesta del maestro por serial; no se guardó en el ESP32.");
        pushLocalEvent("No se confirmó el envío al maestro (" + side + ").", "error");
      } else {
        setPfHint("Guardado en el maestro. Solo cambiará con otro Guardar desde el HMI.");
        pushLocalEvent("PreAlimentador " + side + " guardado en el maestro.");
      }
      await pollPrefeeder();
    } catch (err) {
      pushLocalEvent("Error al guardar PreAlimentador " + side + ".", "error");
    }
  }

  async function pollPrefeeder() {
    try {
      const res = await fetch("/api/prefeeder/status?force=1");
      const data = await res.json();
      if (data.ok && data.status) applyPrefeederStatus(data.status);
    } catch (err) {
      /* sin puerto */
    }
  }

  function setPrefeederPoll(on) {
    if (pfPollTimer) {
      window.clearInterval(pfPollTimer);
      pfPollTimer = null;
    }
    if (!on) return;
    pollPrefeeder();
    pfPollTimer = window.setInterval(pollPrefeeder, 700);
  }

  if (els.plcManualForm) {
    els.plcManualForm.addEventListener("submit", function (ev) {
      ev.preventDefault();
    });
    els.plcManualForm.querySelectorAll("[data-plc-op]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        runPlcButton(btn);
      });
    });
  }

  if (els.selectModelo) {
    els.selectModelo.addEventListener("change", function () {
      if (syncingModelSelect) return;
      selectModelo(els.selectModelo.value, true);
    });
  }

  if (els.metricCantidad) {
    els.metricCantidad.addEventListener("change", function () {
      const n = parseInt(String(els.metricCantidad.value).trim(), 10);
      if (!Number.isFinite(n) || n < 1) {
        pushLocalEvent("La cantidad del lote tiene que ser un entero de 1 o más.", "error");
        return;
      }
      saveCantidad(n);
    });
  }

  if (els.metricCompensacion) {
    els.metricCompensacion.addEventListener("change", function () {
      const mm = Number(String(els.metricCompensacion.value).trim());
      if (!Number.isFinite(mm)) {
        pushLocalEvent("La compensación de medición tiene que ser un número.", "error");
        return;
      }
      saveMeasureComp(mm, false);
    });
  }

  if (els.themeToggle) {
    els.themeToggle.addEventListener("click", toggleTheme);
  }

  function actionButton(action) {
    return document.querySelector('[data-action="' + action + '"]');
  }

  function setButtonOn(btn, on) {
    if (!btn) return;
    btn.classList.toggle("is-active", !!on);
    btn.setAttribute("aria-pressed", on ? "true" : "false");
    /* flashButton puede dejar is-vivid si is-active llega antes del timeout */
    btn.classList.remove("is-vivid");
  }

  function syncHolgura(action, on, lado) {
    const btn = actionButton(action);
    if (!btn) return;
    setButtonOn(btn, on);
    btn.textContent = on ? "Restaurar holgura " + lado : "Omitir holgura " + lado;
  }

  function syncRoutine(state) {
    const rutina = state.rutina || "";
    const fase = state.rutina_fase || "";
    const purga = rutina === "purga";
    const cicloCorriendo = state.estado === "running";
    const purgaBtn = actionButton("purga");
    if (purgaBtn) {
      purgaBtn.disabled = cicloCorriendo || (!!rutina && !purga);
      setButtonOn(purgaBtn, purga);
    }
    const home = actionButton("posicion_inicial");
    if (home) {
      home.disabled = !!state.rutina_manos || cicloCorriendo || (!!rutina && rutina !== "purga");
      setButtonOn(home, rutina === "posicion_inicial");
    }
    syncHolgura("omitir_holgura_izquierda", !!state.omitir_holgura_l, "izquierda");
    syncHolgura("omitir_holgura_derecha", !!state.omitir_holgura_r, "derecha");

    const pfIniciar = document.querySelector('[data-block="prealimentador"] [data-action="iniciar"]');
    if (pfIniciar) {
      const pf = state.prefeeder || {};
      if (pf.com === false) {
        pfIniciar.title = "Sin COM del maestro (estabilización omitida).";
      } else if (pf.ready) {
        pfIniciar.title = "Ya está listo y bloqueado.";
      } else if (pf.com && !pf.stable) {
        pfIniciar.title = "Estabiliza ambos lados y bloquea.";
      } else {
        pfIniciar.title = "Estabiliza y bloquea el PreAlimentador.";
      }
    }

    if (els.statusBannerDetail) {
      els.statusBannerDetail.classList.toggle("is-warn", !!state.rutina_manos);
    }
    if (els.statusBannerActions) {
      const showPurge = purga && fase === "espera";
      const showTray = !!state.charola_llena;
      els.statusBannerActions.hidden = !(showPurge || showTray);
      els.statusBannerActions.querySelectorAll("[data-action]").forEach(function (btn) {
        const trayBtn = btn.dataset.action === "charola_vacia";
        const show = trayBtn ? showTray : showPurge;
        btn.hidden = !show;
        btn.disabled = !show;
      });
    }
    renderChecklist(state);
    if (els.mantCounter) {
      els.mantCounter.textContent =
        "Ciclos desde el último reset: " + (state.ciclos_mant || 0);
    }
  }

  function checkButton(label, op, disabled) {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "btn btn-config";
    btn.textContent = label;
    btn.setAttribute("data-check", op);
    btn.disabled = !!disabled;
    return btn;
  }

  function renderChecklist(state) {
    if (!els.checklistModal || els.checklistModal.hidden || !els.checklistBody) return;
    const ch = state.checklist || {};
    const key = [
      ch.ok,
      state.checklist_bypass,
      ch.paso,
      ch.empleado,
      ch.eje_ok,
      ch.recorrido_ok,
      ch.alimentador_ok,
      ch.purga_ok,
      ch.piezas_cortadas,
      state.ciclos_mant,
      state.rutina,
    ].join("|");
    const live = document.getElementById("checklistLive");
    if (key === checklistViewKey && live) {
      live.textContent = state.mensaje || "";
      return;
    }
    checklistViewKey = key;
    const body = els.checklistBody;
    body.innerHTML = "";
    const hint = document.createElement("p");
    hint.className = "modal__hint";
    const liveLine = document.createElement("p");
    liveLine.id = "checklistLive";
    liveLine.className = "modal__hint";
    liveLine.textContent = state.mensaje || "";
    const actions = document.createElement("div");
    actions.className = "checklist-actions";

    if (ch.ok || state.checklist_bypass) {
      hint.textContent = state.checklist_bypass
        ? "El bypass está activo. El ciclo puede iniciar sin completar esta revisión."
        : ch.empleado
          ? "Revisión completa. Empleado " + ch.empleado + "."
          : "Este arranque no exige revisión. Puedes hacerla ahora.";
      actions.appendChild(checkButton("Nueva revisión", "checklist_nueva", !!state.rutina));
      body.appendChild(hint);
      body.appendChild(liveLine);
      body.appendChild(actions);
      return;
    }

    const paso = ch.paso || 1;
    if (paso === 1) {
      const ciclos = state.ciclos_mant || 0;
      hint.textContent =
        "Paso 1 de 6. Ciclos desde mantenimiento: " +
        ciclos +
        ". El tope es 10 000.";
      const field = document.createElement("label");
      field.className = "modal-field";
      field.textContent = "Empleado (5 dígitos)";
      const input = document.createElement("input");
      input.id = "checkEmpleado";
      input.maxLength = 5;
      input.inputMode = "numeric";
      input.autocomplete = "off";
      field.appendChild(input);
      actions.appendChild(
        checkButton("Continuar", "checklist_empezar", ciclos >= 10000)
      );
      body.appendChild(hint);
      body.appendChild(field);
      body.appendChild(liveLine);
      body.appendChild(actions);
      return;
    }
    if (paso === 2) {
      hint.textContent = "Paso 2 de 6. Prueba de aire: se accionan holder y encoder y se leen las fallas del PLC.";
      actions.appendChild(checkButton("Probar aire", "checklist_aire"));
    } else if (paso === 3) {
      hint.textContent = "Paso 3 de 6. Revisa a ojo que el material no quede ni flojo ni tenso en ambos lados.";
      actions.appendChild(checkButton("Confirmo holgura", "checklist_holgura"));
    } else if (paso === 4) {
      hint.textContent = "Paso 4 de 6. Primero la referencia mecánica del eje. Después el recorrido y la confirmación de que regresó.";
      actions.appendChild(checkButton("Llevar a referencia", "checklist_home", !!state.rutina));
      actions.appendChild(checkButton("Hacer recorrido", "checklist_recorrido", !ch.eje_ok || !!state.rutina));
      actions.appendChild(checkButton("Regresó bien", "checklist_eje_listo", !ch.eje_ok || !ch.recorrido_ok));
    } else if (paso === 5) {
      hint.textContent =
        "Paso 5 de 6. Prepara los alimentadores CAN y, si hay COM del PreAlimentador, estabilízalo. Luego la purga. Alimentar, Limpiar, Continuar y Terminar quedan en el banner superior.";
      actions.appendChild(checkButton("Preparar alimentador", "checklist_preparar", !!state.rutina));
      actions.appendChild(checkButton("Hacer purga", "checklist_purga", !ch.alimentador_ok || !!state.rutina));
      actions.appendChild(checkButton("Purga lista", "checklist_purga_hecha", !ch.purga_ok));
    } else {
      hint.textContent = "Paso 6 de 6. Corta dos piezas del modelo cargado. Cada lado debe medir la longitud del modelo, dentro de la tolerancia del ciclo. La LM Guide recorre esa longitud menos 55 mm.";
      if (!ch.piezas_cortadas) {
        actions.appendChild(checkButton("Cortar 2 piezas", "checklist_piezas", !!state.rutina));
      } else {
        const left = document.createElement("label");
        left.className = "modal-field";
        left.textContent = "Longitud medida izquierda (mm)";
        const inL = document.createElement("input");
        inL.id = "checkMmL";
        inL.inputMode = "decimal";
        left.appendChild(inL);
        const right = document.createElement("label");
        right.className = "modal-field";
        right.textContent = "Longitud medida derecha (mm)";
        const inR = document.createElement("input");
        inR.id = "checkMmR";
        inR.inputMode = "decimal";
        right.appendChild(inR);
        body.appendChild(hint);
        body.appendChild(left);
        body.appendChild(right);
        body.appendChild(liveLine);
        actions.appendChild(checkButton("Guardar medición", "checklist_medicion"));
        body.appendChild(actions);
        return;
      }
    }
    body.appendChild(hint);
    body.appendChild(liveLine);
    body.appendChild(actions);
  }

  function openChecklist() {
    checklistViewKey = "";
    if (els.checklistModal) els.checklistModal.hidden = false;
    fetchState();
  }

  function closeChecklist() {
    if (els.checklistModal) els.checklistModal.hidden = true;
    checklistViewKey = "";
  }

  function openMantenimiento() {
    if (els.mantModal) els.mantModal.hidden = false;
    if (els.mantEmpleado) {
      els.mantEmpleado.value = "";
      els.mantEmpleado.focus();
    }
    fetchState();
  }

  function closeMantenimiento() {
    if (els.mantModal) els.mantModal.hidden = true;
  }

  function askEmpleado(done) {
    empleadoWait = done;
    if (!els.empleadoModal) return;
    if (els.empleadoInput) els.empleadoInput.value = "";
    els.empleadoModal.hidden = false;
    if (els.empleadoInput) els.empleadoInput.focus();
  }

  function closeEmpleado() {
    empleadoWait = null;
    if (els.empleadoModal) els.empleadoModal.hidden = true;
  }

  function confirmEmpleado() {
    const empleado = els.empleadoInput ? els.empleadoInput.value.trim() : "";
    if (!/^\d{5}$/.test(empleado)) {
      pushLocalEvent("El número de empleado son cinco dígitos.", "error");
      return;
    }
    const done = empleadoWait;
    empleadoWait = null;
    if (els.empleadoModal) els.empleadoModal.hidden = true;
    if (done) done(empleado);
  }

  if (els.statusBannerActions) {
    els.statusBannerActions.addEventListener("click", function (e) {
      const btn = e.target.closest("[data-action]");
      if (!btn || btn.disabled) return;
      flashButton(btn);
      sendAction("operacion_manual", btn.dataset.action);
    });
  }

  document.querySelectorAll("[data-pf-hold]").forEach(function (btn) {
    const side = btn.getAttribute("data-pf-hold");
    if (side !== "L" && side !== "R") return;
    let held = false;
    async function setForce(on) {
      if (held === on) return;
      held = on;
      setButtonOn(btn, on);
      try {
        await fetch("/api/prefeeder/cmd", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ op: "set_force_feed", side: side, on: on }),
        });
      } catch (err) {
        /* sin servidor */
      }
    }
    btn.addEventListener("pointerdown", function (ev) {
      ev.preventDefault();
      try {
        btn.setPointerCapture(ev.pointerId);
      } catch (e) {
        /* ignore */
      }
      setForce(true);
    });
    function release() {
      setForce(false);
    }
    btn.addEventListener("pointerup", release);
    btn.addEventListener("pointercancel", release);
    btn.addEventListener("lostpointercapture", release);
  });

  document.querySelectorAll("[data-block]").forEach(function (container) {
    const block = container.dataset.block;
    container.querySelectorAll("[data-action]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        if (btn.hasAttribute("data-pf-hold")) return;
        const action = btn.dataset.action;
        flashButton(btn);

        if (block === "configuracion" && action === "parametros") {
          sendAction(block, action);
          openParametros();
          return;
        }
        if (block === "configuracion" && action === "checklist") {
          openChecklist();
          closeSettings();
          return;
        }
        if (block === "configuracion" && action === "mantenimiento") {
          openMantenimiento();
          closeSettings();
          return;
        }
        if (action === "omitir_holgura_izquierda" || action === "omitir_holgura_derecha") {
          askEmpleado(function (empleado) {
            sendAction(block, action, { empleado: empleado });
          });
          return;
        }

        sendAction(block, action);

        if (block === "configuracion") {
          closeSettings();
        }
      });
    });
  });

  loadThemePreference();
  loadEventsPreference();
  loadModelos()
    .then(fetchState)
    .then(function (state) {
      syncUiFromPersistedState(state);
      return maybeRestoreModelFromLocal();
    })
    .then(loadCycleConfig);
})();

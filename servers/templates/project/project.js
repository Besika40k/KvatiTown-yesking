// ── Helpers ──────────────────────────────────────────────────────────────────

let manualMode = false;

function postJSON(url, data) {
  return fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(data),
  }).then((r) => r.json());
}

function showStatus(id, msg, type) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = msg;
  el.style.color =
    type === "success" ? "var(--accent-green)" : "var(--accent-red)";
  setTimeout(() => {
    el.textContent = "";
  }, 3000);
}

// Set a slider + its paired number input to value v
function setSliderValue(sliderId, v) {
  const slider = document.getElementById(sliderId);
  const input = document.getElementById(sliderId + "-input");
  if (slider) slider.value = v;
  if (input) input.value = v;
}

// Wire up a slider + its number input so they stay in sync, then call onChange
function syncSliderInput(sliderId, onChange) {
  const slider = document.getElementById(sliderId);
  const input = document.getElementById(sliderId + "-input");
  if (!slider) return;
  slider.addEventListener("input", function () {
    if (input) input.value = this.value;
    onChange();
  });
  if (input) {
    input.addEventListener("change", function () {
      if (slider) slider.value = this.value;
      onChange();
    });
  }
}

// ── HSV sliders ───────────────────────────────────────────────────────────────

// Map from slider DOM id → server key name
const HSV_SLIDER_MAP = {
  yLowH: "yellow_lower_h",
  yHighH: "yellow_upper_h",
  yLowS: "yellow_lower_s",
  yHighS: "yellow_upper_s",
  yLowV: "yellow_lower_v",
  yHighV: "yellow_upper_v",
  wLowH: "white_lower_h",
  wHighH: "white_upper_h",
  wLowS: "white_lower_s",
  wHighS: "white_upper_s",
  wLowV: "white_lower_v",
  wHighV: "white_upper_v",
};

// Wire all HSV sliders
Object.entries(HSV_SLIDER_MAP).forEach(([sliderId, serverKey]) => {
  syncSliderInput(sliderId, () => {
    const val = parseInt(document.getElementById(sliderId).value);
    const payload = {};
    payload[serverKey] = val;
    postJSON("/update_hsv", payload)
      .then(() => showStatus("hsv-status", "HSV Updated!", "success"))
      .catch(() => showStatus("hsv-status", "Error", "error"));
  });
});

// Load current HSV values from server once on page load
fetch("/get_hsv")
  .then((r) => r.json())
  .then((d) => {
    Object.entries(HSV_SLIDER_MAP).forEach(([sliderId, serverKey]) => {
      if (d[serverKey] !== undefined) setSliderValue(sliderId, d[serverKey]);
    });
  })
  .catch(() => {});

// ── Mode toggle ───────────────────────────────────────────────────────────────

function toggleMode(isManual) {
  manualMode = isManual;
  document.getElementById("driveCard").style.display = isManual
    ? "block"
    : "none";
  document.getElementById("modeStatus").textContent =
    "Mode: " + (isManual ? "Manual Drive" : "Navigation");
  document.getElementById("toggleKnob").style.left = isManual ? "26px" : "2px";
  document.getElementById("toggleSlider").style.background = isManual
    ? "rgba(63,185,80,0.3)"
    : "var(--bg-sidebar)";
  document.getElementById("toggleSlider").style.borderColor = isManual
    ? "var(--accent-green)"
    : "var(--border-color)";
  document.getElementById("toggleKnob").style.background = isManual
    ? "var(--accent-green)"
    : "var(--text-muted)";

  postJSON("/set_mode", { manual: isManual }).catch(() =>
    showStatus("modeStatus", "Server error", "error"),
  );

  if (!isManual) releaseAll();
}

// ── Keyboard drive ────────────────────────────────────────────────────────────

const keyState = { up: false, down: false, left: false, right: false };
const keyMap = {
  ArrowUp: "up",
  ArrowDown: "down",
  ArrowLeft: "left",
  ArrowRight: "right",
  w: "up",
  s: "down",
  a: "left",
  d: "right",
  W: "up",
  S: "down",
  A: "left",
  D: "right",
};

function updateKeyDisplay() {
  for (const [key, active] of Object.entries(keyState)) {
    const el = document.getElementById("key-" + key);
    if (el) el.classList.toggle("active", active);
  }
}

function sendKeys() {
  if (!manualMode) return;
  fetch("/keys", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(keyState),
  }).catch(() => {});
}

function releaseAll() {
  Object.keys(keyState).forEach((k) => (keyState[k] = false));
  updateKeyDisplay();
  fetch("/keys", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(keyState),
  }).catch(() => {});
}

document.addEventListener("keydown", (e) => {
  if (!manualMode) return;
  const dir = keyMap[e.key];
  if (dir && !keyState[dir]) {
    e.preventDefault();
    keyState[dir] = true;
    updateKeyDisplay();
    sendKeys();
  }
});
document.addEventListener("keyup", (e) => {
  if (!manualMode) return;
  const dir = keyMap[e.key];
  if (dir) {
    e.preventDefault();
    keyState[dir] = false;
    updateKeyDisplay();
    sendKeys();
  }
});
window.addEventListener("blur", releaseAll);
setInterval(() => {
  if (manualMode && Object.values(keyState).some(Boolean)) sendKeys();
}, 150);

// ── Status polling ────────────────────────────────────────────────────────────

// Detector fields are shown in the Object Detection chip, not the status table
const DETECTION_KEYS = [
  "model_loaded",
  "load_error",
  "trt_building",
  "trt_build_elapsed",
  "detection_backend",
];

function updateModelStatus(data) {
  const el = document.getElementById("model-status");
  if (!el) return;
  if (data.trt_building) {
    el.className = "model-status building";
    el.textContent =
      "Building TensorRT engine… (" + (data.trt_build_elapsed || 0) + "s)";
  } else if (data.model_loaded) {
    el.className = "model-status ok";
    el.textContent =
      "Model loaded" +
      (data.detection_backend ? " (" + data.detection_backend + ")" : "");
  } else {
    el.className = "model-status err";
    el.textContent = data.load_error || "Model not loaded";
  }
}

function refreshStatus() {
  fetch("/status")
    .then((r) => r.json())
    .then((data) => {
      updateModelStatus(data);
      const table = document.getElementById("statusTable");
      const keys = Object.keys(data).filter((k) => !DETECTION_KEYS.includes(k));
      if (keys.length === 0) {
        table.innerHTML =
          '<div style="color:var(--text-muted);text-align:center;padding:12px 0;">No data</div>';
        return;
      }
      table.innerHTML = keys
        .map(
          (k) =>
            `<div class="row">
                    <span class="key">${k}</span>
                    <span class="val">${JSON.stringify(data[k])}</span>
                </div>`,
        )
        .join("");
      document.getElementById("statusDot").style.background =
        "var(--accent-green)";
    })
    .catch(() => {
      document.getElementById("statusDot").style.background =
        "var(--accent-red)";
    });
}

refreshStatus();
setInterval(refreshStatus, 500);

// ── Bot Configuration ────────────────────────────────────────────────────────

function refreshBotList() {
  fetch("/config/bots")
    .then((r) => r.json())
    .then((data) => {
      const select = document.getElementById("botSelect");
      select.innerHTML = "";
      data.bots.forEach((bot) => {
        const opt = document.createElement("option");
        opt.value = bot;
        opt.textContent = bot;
        if (bot === data.current) {
          opt.selected = true;
          opt.textContent = bot + " (current)";
        }
        select.appendChild(opt);
      });
    })
    .catch(() => console.error("Failed to load bot list"));
}

function loadBotConfig() {
  const botName = document.getElementById("botSelect").value;
  if (!botName) return;

  postJSON("/config/load", { bot_name: botName })
    .then((r) => {
      showStatus(
        "botConfigStatus",
        r.message || "Loaded!",
        r.status === "ok" ? "success" : "error",
      );
      if (r.status === "ok") {
        location.reload();
      }
    })
    .catch(() => showStatus("botConfigStatus", "Load failed", "error"));
}

function saveBotConfig() {
  const botName = document.getElementById("botNameInput").value.trim();
  if (!botName) {
    showStatus("botConfigStatus", "Enter bot name", "error");
    return;
  }

  postJSON("/config/save", { bot_name: botName })
    .then((r) => {
      showStatus(
        "botConfigStatus",
        r.message || "Saved!",
        r.status === "ok" ? "success" : "error",
      );
      if (r.status === "ok") {
        document.getElementById("botNameInput").value = "";
        refreshBotList();
        location.reload();
      }
    })
    .catch(() => showStatus("botConfigStatus", "Save failed", "error"));
}

function downloadBotConfig() {
  window.location.href = "/config/download";
}

// Initialize bot list on load
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", refreshBotList);
} else {
  refreshBotList();
}

// ── Dance ─────────────────────────────────────────────────────────────────────

function sendDance() {
  postJSON("/maneuver", { type: "dance", value: 3.0 })
    .then((r) =>
      showStatus(
        "danceStatus",
        r.status === "ok" ? "Dance started!" : r.message || "Error",
        r.status === "ok" ? "success" : "error",
      ),
    )
    .catch(() => showStatus("danceStatus", "Error", "error"));
}

// ── Grid click state machine ─────────────────────────────────────────────────
const GS_IDLE = 0,
  GS_DIR = 1,
  GS_GOAL = 2,
  GS_DONE = 3;
let gridState = GS_IDLE;
let pendingIntersection = null;
let gridStartTile = null;
let gridGoalTile = null;

function showPicker(id, show) {
  const el = document.getElementById(id);
  if (el) el.style.display = show ? "block" : "none";
}

function resetGridSelection() {
  pendingIntersection = null;
  gridState = GS_IDLE;
  showPicker("direction-picker", false);
  if (gridStartTile) gridStartTile.classList.remove("start-selected");
  if (gridGoalTile) gridGoalTile.classList.remove("goal-selected");
  gridStartTile = null;
  gridGoalTile = null;
}

function setDirection(dir) {
  showPicker("direction-picker", false);
  const id = pendingIntersection;
  postJSON("/set_start", { node: id, direction: dir })
    .then((r) =>
      showStatus(
        "grid-click-status",
        "Start: intersection " + id + " " + dir,
        "success",
      ),
    )
    .catch(() => showStatus("grid-click-status", "Server error", "error"));
  gridState = GS_GOAL;
}

function injectLaneCard() {
  const e = [...document.querySelectorAll(".card")].find((c) =>
    c.querySelector(".card-header")?.textContent?.includes("Dance"),
  );
  if (!e) return;
  const d = document.createElement("div");
  d.className = "card";
  d.innerHTML = `<div class="card-header">Lane Control</div><div class="lane-slider-group"><div class="lane-slider-row"><label>P Gain</label><input type="range" id="lc-p" min="0" max="2" step="0.05" value="0.6" oninput="document.getElementById('lc-p-v').textContent=parseFloat(this.value).toFixed(2);applyLaneConfig()"><span id="lc-p-v">0.60</span></div></div><div class="lane-slider-group"><div class="lane-slider-row"><label>D Gain</label><input type="range" id="lc-d" min="0" max="3" step="0.05" value="0.8" oninput="document.getElementById('lc-d-v').textContent=parseFloat(this.value).toFixed(2);applyLaneConfig()"><span id="lc-d-v">0.80</span></div></div><div class="lane-slider-group"><div class="lane-slider-row"><label>Base Speed</label><input type="range" id="lc-s" min="0.02" max="0.4" step="0.01" value="0.08" oninput="document.getElementById('lc-s-v').textContent=parseFloat(this.value).toFixed(2);applyLaneConfig()"><span id="lc-s-v">0.08</span></div></div><div id="lc-status" class="status"></div>`;
  e.parentNode.insertBefore(d, e);
  fetch("/get_lane_config")
    .then((r) => r.json())
    .then((d) => {
      document.getElementById("lc-p").value = d.p_gain;
      document.getElementById("lc-p-v").textContent = d.p_gain.toFixed(2);
      document.getElementById("lc-d").value = d.d_gain;
      document.getElementById("lc-d-v").textContent = d.d_gain.toFixed(2);
      document.getElementById("lc-s").value = d.base_speed;
      document.getElementById("lc-s-v").textContent = d.base_speed.toFixed(2);
    })
    .catch(() => {});
}
function applyLaneConfig() {
  const p = parseFloat(document.getElementById("lc-p").value),
    d = parseFloat(document.getElementById("lc-d").value),
    s = parseFloat(document.getElementById("lc-s").value);
  postJSON("/set_lane_config", { p_gain: p, d_gain: d, base_speed: s })
    .then(() => showStatus("lc-status", "Applied!", "success"))
    .catch(() => showStatus("lc-status", "Error", "error"));
}
document.readyState === "loading"
  ? document.addEventListener("DOMContentLoaded", injectLaneCard)
  : injectLaneCard();
function injectTimingCard() {
  const e = [...document.querySelectorAll(".card")].find((c) =>
    c.querySelector(".card-header")?.textContent?.includes("Dance"),
  );
  if (!e) return;
  const d = document.createElement("div");
  d.className = "card";
  d.innerHTML = `<div class="card-header">Intersection Timing</div><div class="lane-slider-group"><div class="lane-slider-row"><label>Creep Time</label><input type="range" id="tc-fct" min="0.1" max="3" step="0.05" value="0.8" oninput="document.getElementById('tc-fct-v').textContent=parseFloat(this.value).toFixed(2);applyTimingConfig()"><span id="tc-fct-v">0.80</span></div></div><div class="lane-slider-group"><div class="lane-slider-row"><label>Exit Timeout</label><input type="range" id="tc-ext" min="0.5" max="8" step="0.5" value="4" oninput="document.getElementById('tc-ext-v').textContent=parseFloat(this.value).toFixed(1);applyTimingConfig()"><span id="tc-ext-v">4.0</span></div></div><div class="lane-slider-group"><div class="lane-slider-row"><label>Fwd Through</label><input type="range" id="tc-tfwd" min="0.1" max="5" step="0.1" value="1" oninput="document.getElementById('tc-tfwd-v').textContent=parseFloat(this.value).toFixed(1);applyTimingConfig()"><span id="tc-tfwd-v">1.0</span></div></div><div class="lane-slider-group"><div class="lane-slider-row"><label>Left Turn</label><input type="range" id="tc-tl" min="0.1" max="3" step="0.05" value="1.1" oninput="document.getElementById('tc-tl-v').textContent=parseFloat(this.value).toFixed(2);applyTimingConfig()"><span id="tc-tl-v">1.10</span></div></div><div class="lane-slider-group"><div class="lane-slider-row"><label>Right Turn</label><input type="range" id="tc-tr" min="0.1" max="3" step="0.05" value="0.8" oninput="document.getElementById('tc-tr-v').textContent=parseFloat(this.value).toFixed(2);applyTimingConfig()"><span id="tc-tr-v">0.80</span></div></div><div class="lane-slider-group"><div class="lane-slider-row"><label>Turnaround</label><input type="range" id="tc-tta" min="0.1" max="6" step="0.05" value="3.2" oninput="document.getElementById('tc-tta-v').textContent=parseFloat(this.value).toFixed(2);applyTimingConfig()"><span id="tc-tta-v">3.20</span></div></div><div id="tc-status" class="status"></div>`;
  e.parentNode.insertBefore(d, e);
  fetch("/get_timing_config")
    .then((r) => r.json())
    .then((cfg) => {
      document.getElementById("tc-fct").value = cfg.forward_clear_time;
      document.getElementById("tc-fct-v").textContent =
        cfg.forward_clear_time.toFixed(2);
      document.getElementById("tc-ext").value = cfg.exit_timeout;
      document.getElementById("tc-ext-v").textContent =
        cfg.exit_timeout.toFixed(1);
      document.getElementById("tc-tfwd").value = cfg.turn_time_forward;
      document.getElementById("tc-tfwd-v").textContent =
        cfg.turn_time_forward.toFixed(1);
      document.getElementById("tc-tl").value = cfg.turn_time_left;
      document.getElementById("tc-tl-v").textContent =
        cfg.turn_time_left.toFixed(2);
      document.getElementById("tc-tr").value = cfg.turn_time_right;
      document.getElementById("tc-tr-v").textContent =
        cfg.turn_time_right.toFixed(2);
      document.getElementById("tc-tta").value = cfg.turn_time_turnaround;
      document.getElementById("tc-tta-v").textContent =
        cfg.turn_time_turnaround.toFixed(2);
    })
    .catch(() => {});
}
function applyTimingConfig() {
  postJSON("/set_timing_config", {
    forward_clear_time: parseFloat(document.getElementById("tc-fct").value),
    exit_timeout: parseFloat(document.getElementById("tc-ext").value),
    turn_time_forward: parseFloat(document.getElementById("tc-tfwd").value),
    turn_time_left: parseFloat(document.getElementById("tc-tl").value),
    turn_time_right: parseFloat(document.getElementById("tc-tr").value),
    turn_time_turnaround: parseFloat(document.getElementById("tc-tta").value),
  })
    .then(() => showStatus("tc-status", "Applied!", "success"))
    .catch(() => showStatus("tc-status", "Error", "error"));
}
document.readyState === "loading"
  ? document.addEventListener("DOMContentLoaded", injectTimingCard)
  : injectTimingCard();

document.addEventListener("DOMContentLoaded", () => {
  const totalCols = 7;
  const totalRows = 9;

  // Mapping from grid tile (c=horizontal, r=vertical) to intersection ID
  const TILE_INTERSECTION_MAP = {
    "1,5": 2,
    "3,5": 3,
    "4,1": 1,
  };

  const gridOverlay = document.getElementById("standaloneGridOverlay");
  if (!gridOverlay) return;

  for (let r = 1; r <= totalRows; r++) {
    for (let c = 1; c <= totalCols; c++) {
      const tile = document.createElement("button");
      tile.className = "standalone-tile";
      tile._c = c;
      tile._r = r;

      const key = c + "," + r;
      if (TILE_INTERSECTION_MAP[key] != null) {
        tile.classList.add("valid-tile");
        tile.setAttribute(
          "title",
          "Intersection " + TILE_INTERSECTION_MAP[key],
        );
      }

      tile.addEventListener("click", () => {
        const intersectionId = TILE_INTERSECTION_MAP[key];
        if (intersectionId == null) {
          showStatus(
            "grid-click-status",
            "No intersection at this tile",
            "error",
          );
          return;
        }

        if (gridState === GS_IDLE || gridState === GS_DONE) {
          resetGridSelection();
          gridStartTile = tile;
          tile.classList.add("start-selected");
          pendingIntersection = intersectionId;
          gridState = GS_DIR;
          showPicker("direction-picker", true);
          showStatus(
            "grid-click-status",
            "Start: intersection " + intersectionId + " — choose direction",
            "success",
          );
        } else if (gridState === GS_DIR) {
          if (gridStartTile) gridStartTile.classList.remove("start-selected");
          gridStartTile = tile;
          tile.classList.add("start-selected");
          pendingIntersection = intersectionId;
          showStatus(
            "grid-click-status",
            "Start: intersection " + intersectionId + " — choose direction",
            "success",
          );
        } else if (gridState === GS_GOAL) {
          if (tile === gridStartTile) return;
          if (gridGoalTile) gridGoalTile.classList.remove("goal-selected");
          gridGoalTile = tile;
          tile.classList.add("goal-selected");
          postJSON("/set_goal", { node: intersectionId })
            .then((r) => {
              let msg = "Goal: intersection " + intersectionId;
              if (r.path) msg += "  Path: " + r.path.join(" \u2192 ");
              showStatus("grid-click-status", msg, "success");
            })
            .catch(() =>
              showStatus("grid-click-status", "Server error", "error"),
            );
          gridState = GS_DONE;
        }
      });

      gridOverlay.appendChild(tile);
    }
  }
});

from .base import render_template

_CONTENT = '''
    <div class="container">
        <div class="video-section">
            <img src="/video" class="stream" id="videoStream">
        </div>

        <div class="controls-section">

            <div class="card">
                <div class="card-header">Track Map Router</div>
                <p style="font-size: 11px; color: var(--text-muted); margin-bottom: 8px;">
                    1st click: start point + direction. 2nd click: end point. 3rd click: reset.
                </p>
                <div class="standalone-map-container">
                    <img src="/config/kiu_map.png" alt="Track Map Grid" class="map-grid-underlay">
                    <div id="standaloneGridOverlay"></div>
                </div>
                <div id="grid-click-status" class="status" style="margin-top: 8px; font-size: 12px;"></div>
                <div id="direction-picker" style="display:none; margin-top: 8px; text-align:center;">
                    <div style="font-size:12px;color:var(--text-muted);margin-bottom:6px;">Choose bot direction:</div>
                    <div style="display:flex;gap:6px;justify-content:center;">
                        <button class="button" style="flex:1" onclick="setDirection('N')">N</button>
                        <button class="button" style="flex:1" onclick="setDirection('E')">E</button>
                        <button class="button" style="flex:1" onclick="setDirection('S')">S</button>
                        <button class="button" style="flex:1" onclick="setDirection('W')">W</button>
                    </div>
                </div>
            </div>

            <div class="card">
                <div class="card-header">
                    Status
                    <span id="statusDot" style="width:8px;height:8px;border-radius:50%;
                        background:var(--accent-green);display:inline-block;"></span>
                </div>
                <div id="statusTable" style="font-size:12px;">
                    <div style="color:var(--text-muted);text-align:center;padding:12px 0;">
                        Waiting for data...
                    </div>
                </div>
            </div>



            <div class="card">
                <div class="card-header">Mode</div>
                <div style="display:flex;align-items:center;gap:12px;padding:4px 0;">
                    <span style="font-size:13px;color:var(--accent-blue);font-weight:600;">Navigation</span>
                    <label class="bb8-toggle">
                        <input class="bb8-toggle__checkbox" type="checkbox" id="driveToggle" onchange="toggleMode(this.checked)">
                        <div class="bb8-toggle__container">
                          <div class="bb8-toggle__scenery">
                            <div class="bb8-toggle__star"></div>
                            <div class="bb8-toggle__star"></div>
                            <div class="bb8-toggle__star"></div>
                            <div class="bb8-toggle__star"></div>
                            <div class="bb8-toggle__star"></div>
                            <div class="bb8-toggle__star"></div>
                            <div class="bb8-toggle__star"></div>
                            <div class="tatto-1"></div>
                            <div class="tatto-2"></div>
                            <div class="gomrassen"></div>
                            <div class="hermes"></div>
                            <div class="chenini"></div>
                            <div class="bb8-toggle__cloud"></div>
                            <div class="bb8-toggle__cloud"></div>
                            <div class="bb8-toggle__cloud"></div>
                          </div>
                          <div class="bb8">
                            <div class="bb8__head-container">
                              <div class="bb8__antenna"></div>
                              <div class="bb8__antenna"></div>
                              <div class="bb8__head"></div>
                            </div>
                            <div class="bb8__body"></div>
                          </div>
                          <div class="artificial__hidden">
                            <div class="bb8__shadow"></div>
                          </div>
                        </div>
                      </label>
                    <span style="font-size:13px;color:var(--accent-blue);font-weight:600;">Manual Drive</span>
                </div>
                <div id="modeStatus" style="font-size:12px;color:var(--text-muted);margin-top:4px;">Mode: <span style="color:var(--accent-blue);">Navigation</span></div>
            </div>

            <div class="card" id="driveCard" style="display:none;">
                <div class="card-header">Drive</div>
                <div class="key-display">
                    <div class="key-box key-up"    id="key-up">&#9650;</div>
                    <div class="key-box key-left"  id="key-left">&#9664;</div>
                    <div class="key-box key-down"  id="key-down">&#9660;</div>
                    <div class="key-box key-right" id="key-right">&#9654;</div>
                </div>
                <p style="text-align:center;font-size:11px;color:var(--text-muted)">use arrow keys</p>
            </div>



            <div class="card">
                <div class="card-header">Dance Maneuver</div>
                <div style="display:flex;flex-direction:column;gap:8px;">
                    <button class="button" onclick="sendDance()">
                      Dance
                    </button>
                    <div id="danceStatus" class="status"></div>
                </div>
            </div>

            <div class="card">
                <div class="card-header">HSV Color Calibration</div>

                <div class="hsv-section-title white">White Line (right / solid)</div>

                <div class="slider-group">
                    <div class="slider-label"><span>Hue Low</span><span style="color:var(--text-muted)">0-179</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wLowH" min="0" max="179" value="0" class="slider">
                        <input type="number" id="wLowH-input" min="0" max="179" value="0" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Hue High</span><span style="color:var(--text-muted)">0-179</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wHighH" min="0" max="179" value="179" class="slider">
                        <input type="number" id="wHighH-input" min="0" max="179" value="179" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Sat Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wLowS" min="0" max="255" value="0" class="slider">
                        <input type="number" id="wLowS-input" min="0" max="255" value="0" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Sat High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wHighS" min="0" max="255" value="255" class="slider">
                        <input type="number" id="wHighS-input" min="0" max="255" value="255" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wLowV" min="0" max="255" value="150" class="slider">
                        <input type="number" id="wLowV-input" min="0" max="255" value="150" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wHighV" min="0" max="255" value="255" class="slider">
                        <input type="number" id="wHighV-input" min="0" max="255" value="255" class="input-box">
                    </div>
                </div>

                <div class="hsv-section-title yellow">Yellow Line (left / dashed)</div>

                <div class="slider-group">
                    <div class="slider-label"><span>Hue Low</span><span style="color:var(--text-muted)">0-179</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yLowH" min="0" max="179" value="20" class="slider">
                        <input type="number" id="yLowH-input" min="0" max="179" value="20" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Hue High</span><span style="color:var(--text-muted)">0-179</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yHighH" min="0" max="179" value="40" class="slider">
                        <input type="number" id="yHighH-input" min="0" max="179" value="40" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Sat Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yLowS" min="0" max="255" value="100" class="slider">
                        <input type="number" id="yLowS-input" min="0" max="255" value="100" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Sat High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yHighS" min="0" max="255" value="255" class="slider">
                        <input type="number" id="yHighS-input" min="0" max="255" value="255" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yLowV" min="0" max="255" value="100" class="slider">
                        <input type="number" id="yLowV-input" min="0" max="255" value="100" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yHighV" min="0" max="255" value="255" class="slider">
                        <input type="number" id="yHighV-input" min="0" max="255" value="255" class="input-box">
                    </div>
                </div>

                <div id="hsv-status" class="status"></div>
            </div>

        </div>
    </div>
'''

_EXTRA_CSS = '''

#statusTable .row {
    display: flex;
    justify-content: space-between;
    gap: 8px;
    padding: 6px 0;
    border-bottom: 1px solid var(--border-color);
    align-items: baseline;
}
#statusTable .row:last-child { border-bottom: none; }
#statusTable .key  { color: var(--text-secondary); font-size: 12px; flex-shrink: 0; }
#statusTable .val  { color: var(--text-primary); font-weight: 500; font-size: 13px; font-family: monospace; text-align: right; word-break: break-word; min-width: 0; flex-shrink: 1; }

.key-display {
    display: grid;
    grid-template-areas: ".    up   ." "left down right";
    grid-template-columns: repeat(3, 48px);
    grid-template-rows: repeat(2, 48px);
    gap: 4px;
    justify-content: center;
    margin: 8px 0;
}
.key-box {
    display: flex;
    align-items: center;
    justify-content: center;
    background: var(--bg-sidebar);
    border: 2px solid var(--border-color);
    border-radius: 8px;
    font-size: 20px;
    font-weight: 600;
    color: var(--text-muted);
    transition: all 0.1s;
    user-select: none;
}
.key-box.active { background: rgba(212,167,44,0.2); border-color: var(--accent-green); color: var(--accent-green); }
.key-up    { grid-area: up; }
.key-down  { grid-area: down; }
.key-left  { grid-area: left; }
.key-right { grid-area: right; }
.hsv-section-title { font-size: 13px; font-weight: 600; color: var(--text-secondary); margin: 12px 0 8px; text-transform: uppercase; letter-spacing: 0.5px; }
.hsv-section-title.yellow { color: #f1c40f; }
.hsv-section-title.white  { color: #ecf0f1; }

.model-status { padding: 6px 10px; border-radius: 4px; font-size: 12px; }
.model-status.ok      { background: rgba(212,167,44,0.1);  border: 1px solid rgba(212,167,44,0.3);  color: var(--accent-green); }
.model-status.err     { background: rgba(248,81,73,0.1);  border: 1px solid rgba(248,81,73,0.3);  color: var(--accent-red); }
.model-status.building{ background: rgba(210,153,34,0.1); border: 1px solid rgba(210,153,34,0.3); color: #d6a63a; }

/* Styles for Standalone Track Grid Panels */
.standalone-map-container {
    position: relative;
    display: inline-block;
    width: 100%;
    background: #0f141c;
    border-radius: 6px;
    overflow: hidden;
    border: 1px solid var(--border-color);
}
.map-grid-underlay {
    display: block;
    width: 100%;
    height: auto;
    opacity: 0.85;
}
#standaloneGridOverlay {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    display: grid;
    grid-template-columns: repeat(7, 1fr);
    grid-template-rows: repeat(9, 1fr);
}
.standalone-tile {
    background: transparent;
    border: 1px solid rgba(255, 255, 255, 0.04);
    cursor: pointer;
    padding: 0;
    margin: 0;
    transition: background 0.1s ease;
}
.standalone-tile:hover {
    background: rgba(255, 159, 67, 0.25);
    outline: 1px solid #ff9f43;
    z-index: 5;
}
.standalone-tile.start-selected {
    background: rgba(212, 167, 44, 0.2);
    outline: 2px solid var(--accent-green);
    z-index: 10;
}
.standalone-tile.goal-selected {
    background: rgba(248, 81, 73, 0.2);
    outline: 2px solid var(--accent-red);
    z-index: 10;
}
.standalone-tile.valid-tile {
    background: rgba(31, 111, 235, 0.08);
    border-color: rgba(31, 111, 235, 0.25);
}
.standalone-tile.valid-tile:hover {
    background: rgba(31, 111, 235, 0.2);
    outline: 1px solid var(--accent-blue);
}

#star-field {
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  pointer-events: none;
  z-index: -1;
  overflow: hidden;
}

.star {
  position: absolute;
  background-color: #fff;
  border-radius: 50%;
  opacity: 0;
  animation: twinkle var(--duration, 3s) infinite ease-in-out var(--delay, 0s);
}

@keyframes twinkle {
  0%, 100% { opacity: 0; transform: scale(0.5); }
  50% { opacity: var(--max-opacity, 0.8); transform: scale(1); }
}

/* Custom scrollbar for controls column */
.controls-section::-webkit-scrollbar {
  width: 8px;
}
.controls-section::-webkit-scrollbar-track {
  background: var(--bg-dark);
  border-radius: 4px;
}
.controls-section::-webkit-scrollbar-thumb {
  background: #3e424e;
  border-radius: 4px;
  border: 1px solid var(--border-color);
  transition: background 0.2s ease;
}
.controls-section::-webkit-scrollbar-thumb:hover {
  background: var(--accent-blue);
  border-color: var(--accent-blue-hover);
}
.controls-section {
  scrollbar-width: thin;
  scrollbar-color: #3e424e var(--bg-dark);
}
'''

_EXTRA_JS = '''
// ── Helpers ──────────────────────────────────────────────────────────────────

let manualMode = false;

function postJSON(url, data) {
    return fetch(url, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(data)
    }).then(r => r.json());
}

function showStatus(id, msg, type) {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = msg;
    el.style.color = type === 'success' ? 'var(--accent-green)' : 'var(--accent-red)';
    setTimeout(() => { el.textContent = ''; }, 3000);
}

// Set a slider + its paired number input to value v
function setSliderValue(sliderId, v) {
    const slider = document.getElementById(sliderId);
    const input  = document.getElementById(sliderId + '-input');
    if (slider) slider.value = v;
    if (input)  input.value  = v;
}

// Wire up a slider + its number input so they stay in sync, then call onChange
function syncSliderInput(sliderId, onChange) {
    const slider = document.getElementById(sliderId);
    const input  = document.getElementById(sliderId + '-input');
    if (!slider) return;
    slider.addEventListener('input', function () {
        if (input) input.value = this.value;
        onChange();
    });
    if (input) {
        input.addEventListener('change', function () {
            if (slider) slider.value = this.value;
            onChange();
        });
    }
}

// ── HSV sliders ───────────────────────────────────────────────────────────────

// Map from slider DOM id → server key name
const HSV_SLIDER_MAP = {
    'yLowH':  'yellow_lower_h', 'yHighH': 'yellow_upper_h',
    'yLowS':  'yellow_lower_s', 'yHighS': 'yellow_upper_s',
    'yLowV':  'yellow_lower_v', 'yHighV': 'yellow_upper_v',
    'wLowH':  'white_lower_h',  'wHighH': 'white_upper_h',
    'wLowS':  'white_lower_s',  'wHighS': 'white_upper_s',
    'wLowV':  'white_lower_v',  'wHighV': 'white_upper_v',
};

// Wire all HSV sliders
Object.entries(HSV_SLIDER_MAP).forEach(([sliderId, serverKey]) => {
    syncSliderInput(sliderId, () => {
        const val = parseInt(document.getElementById(sliderId).value);
        const payload = {};
        payload[serverKey] = val;
        postJSON('/update_hsv', payload)
            .then(() => showStatus('hsv-status', 'HSV Updated!', 'success'))
            .catch(() => showStatus('hsv-status', 'Error', 'error'));
    });
});

// Load current HSV values from server once on page load
fetch('/get_hsv')
    .then(r => r.json())
    .then(d => {
        Object.entries(HSV_SLIDER_MAP).forEach(([sliderId, serverKey]) => {
            if (d[serverKey] !== undefined) setSliderValue(sliderId, d[serverKey]);
        });
    })
    .catch(() => {});

// ── Mode toggle ───────────────────────────────────────────────────────────────

function toggleMode(isManual) {
    manualMode = isManual;
    document.getElementById('driveCard').style.display = isManual ? 'block' : 'none';
    document.getElementById('modeStatus').innerHTML = 'Mode: <span style="color:var(--accent-blue);">' + (isManual ? 'Manual Drive' : 'Navigation') + '</span>';

    postJSON('/set_mode', {manual: isManual})
        .catch(() => showStatus('modeStatus', 'Server error', 'error'));

    if (!isManual) releaseAll();
}

// ── Keyboard drive ────────────────────────────────────────────────────────────

const keyState = {up: false, down: false, left: false, right: false};
const keyMap = {
    'ArrowUp': 'up', 'ArrowDown': 'down', 'ArrowLeft': 'left', 'ArrowRight': 'right',
    'w': 'up', 's': 'down', 'a': 'left', 'd': 'right',
    'W': 'up', 'S': 'down', 'A': 'left', 'D': 'right',
};

function updateKeyDisplay() {
    for (const [key, active] of Object.entries(keyState)) {
        const el = document.getElementById('key-' + key);
        if (el) el.classList.toggle('active', active);
    }
}

function sendKeys() {
    if (!manualMode) return;
    fetch('/keys', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(keyState)
    }).catch(() => {});
}

function releaseAll() {
    Object.keys(keyState).forEach(k => keyState[k] = false);
    updateKeyDisplay();
    fetch('/keys', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(keyState)
    }).catch(() => {});
}

document.addEventListener('keydown', e => {
    if (!manualMode) return;
    const dir = keyMap[e.key];
    if (dir && !keyState[dir]) { e.preventDefault(); keyState[dir] = true; updateKeyDisplay(); sendKeys(); }
});
document.addEventListener('keyup', e => {
    if (!manualMode) return;
    const dir = keyMap[e.key];
    if (dir) { e.preventDefault(); keyState[dir] = false; updateKeyDisplay(); sendKeys(); }
});
window.addEventListener('blur', releaseAll);
setInterval(() => { if (manualMode && Object.values(keyState).some(Boolean)) sendKeys(); }, 150);

// ── Status polling ────────────────────────────────────────────────────────────

// Detector fields are shown in the Object Detection chip, not the status table
const DETECTION_KEYS = ['model_loaded', 'load_error', 'trt_building',
                        'trt_build_elapsed', 'detection_backend'];

function updateModelStatus(data) {
    const el = document.getElementById('model-status');
    if (!el) return;
    if (data.trt_building) {
        el.className = 'model-status building';
        el.textContent = 'Building TensorRT engine… (' + (data.trt_build_elapsed || 0) + 's)';
    } else if (data.model_loaded) {
        el.className = 'model-status ok';
        el.textContent = 'Model loaded' +
            (data.detection_backend ? ' (' + data.detection_backend + ')' : '');
    } else {
        el.className = 'model-status err';
        el.textContent = data.load_error || 'Model not loaded';
    }
}

function refreshStatus() {
    fetch('/status')
        .then(r => r.json())
        .then(data => {
            updateModelStatus(data);
            const table = document.getElementById('statusTable');
            const keys = Object.keys(data).filter(k => !DETECTION_KEYS.includes(k));
            if (keys.length === 0) {
                table.innerHTML = '<div style="color:var(--text-muted);text-align:center;padding:12px 0;">No data</div>';
                return;
            }
            table.innerHTML = keys.map(k =>
                `<div class="row">
                    <span class="key">${k}</span>
                    <span class="val">${JSON.stringify(data[k])}</span>
                </div>`
            ).join('');
            document.getElementById('statusDot').style.background = 'var(--accent-green)';
        })
        .catch(() => {
            document.getElementById('statusDot').style.background = 'var(--accent-red)';
        });
}

refreshStatus();
setInterval(refreshStatus, 500);

// ── Dance ─────────────────────────────────────────────────────────────────────

function sendDance() {
    postJSON('/maneuver', {type: 'dance', value: 3.0})
        .then(r => showStatus('danceStatus', r.status === 'ok' ? 'Dance started!' : (r.message || 'Error'), r.status === 'ok' ? 'success' : 'error'))
        .catch(() => showStatus('danceStatus', 'Error', 'error'));
}

// ── Grid click state machine ─────────────────────────────────────────────────
const GS_IDLE = 0, GS_DIR = 1, GS_GOAL = 2, GS_DONE = 3;
let gridState = GS_IDLE;
let pendingIntersection = null;
let gridStartTile = null;
let gridGoalTile = null;

function showPicker(id, show) {
    const el = document.getElementById(id);
    if (el) el.style.display = show ? 'block' : 'none';
}

function resetGridSelection() {
    pendingIntersection = null;
    gridState = GS_IDLE;
    showPicker('direction-picker', false);
    if (gridStartTile) gridStartTile.classList.remove('start-selected');
    if (gridGoalTile) gridGoalTile.classList.remove('goal-selected');
    gridStartTile = null;
    gridGoalTile = null;
}

function setDirection(dir) {
    showPicker('direction-picker', false);
    const id = pendingIntersection;
    postJSON('/set_start', { node: id, direction: dir })
        .then(r => showStatus('grid-click-status',
            'Start: intersection ' + id + ' ' + dir, 'success'))
        .catch(() => showStatus('grid-click-status', 'Server error', 'error'));
    gridState = GS_GOAL;
}

document.addEventListener("DOMContentLoaded", () => {
    const totalCols = 7;
    const totalRows = 9;

    // Mapping from grid tile (c=horizontal, r=vertical) to intersection ID
    const TILE_INTERSECTION_MAP = {
        '1,5': 2,
        '3,5': 3,
        '4,1': 1,
    };

    const gridOverlay = document.getElementById('standaloneGridOverlay');
    if (!gridOverlay) return;

    for (let r = 1; r <= totalRows; r++) {
        for (let c = 1; c <= totalCols; c++) {
            const tile = document.createElement('button');
            tile.className = 'standalone-tile';
            tile._c = c; tile._r = r;

            const key = c + ',' + r;
            if (TILE_INTERSECTION_MAP[key] != null) {
                tile.classList.add('valid-tile');
                tile.setAttribute('title', 'Intersection ' + TILE_INTERSECTION_MAP[key]);
            }

            tile.addEventListener('click', () => {
                const intersectionId = TILE_INTERSECTION_MAP[key];
                if (intersectionId == null) {
                    showStatus('grid-click-status',
                        'No intersection at this tile', 'error');
                    return;
                }

                if (gridState === GS_IDLE || gridState === GS_DONE) {
                    resetGridSelection();
                    gridStartTile = tile;
                    tile.classList.add('start-selected');
                    pendingIntersection = intersectionId;
                    gridState = GS_DIR;
                    showPicker('direction-picker', true);
                    showStatus('grid-click-status',
                        'Start: intersection ' + intersectionId + ' — choose direction', 'success');
                } else if (gridState === GS_DIR) {
                    if (gridStartTile) gridStartTile.classList.remove('start-selected');
                    gridStartTile = tile;
                    tile.classList.add('start-selected');
                    pendingIntersection = intersectionId;
                    showStatus('grid-click-status',
                        'Start: intersection ' + intersectionId + ' — choose direction', 'success');
                } else if (gridState === GS_GOAL) {
                    if (tile === gridStartTile) return;
                    if (gridGoalTile) gridGoalTile.classList.remove('goal-selected');
                    gridGoalTile = tile;
                    tile.classList.add('goal-selected');
                    postJSON('/set_goal', { node: intersectionId })
                        .then(r => {
                            let msg = 'Goal: intersection ' + intersectionId;
                            if (r.path) msg += '  Path: ' + r.path.join(' \u2192 ');
                            showStatus('grid-click-status', msg, 'success');
                        })
                        .catch(() => showStatus('grid-click-status', 'Server error', 'error'));
                    gridState = GS_DONE;
                }
            });

            gridOverlay.appendChild(tile);
        }
    }

    // Prepend gold-filtered crown logo next to the title
    const h1 = document.querySelector('.header h1');
    if (h1) {
        const logo = document.createElement('img');
        logo.src = 'https://images.vexels.com/media/users/3/246966/isolated/preview/850b9d596ac1e333e477ef721141b9de-fleur-de-lis-king-crown.png';
        logo.alt = 'Crown Logo';
        logo.style.height = '48px'; // Slightly taller as this asset looks better larger
        logo.style.verticalAlign = 'middle';
        
        h1.insertBefore(logo, h1.firstChild);
        h1.style.display = 'flex';
        h1.style.alignItems = 'center';
        h1.style.justifyContent = 'center';
        h1.style.gap = '12px';
    }

    // Twinkling stars generation
    const starField = document.createElement('div');
    starField.id = 'star-field';
    document.body.appendChild(starField);

    const starCount = 200;
    for (let i = 0; i < starCount; i++) {
        const star = document.createElement('div');
        star.className = 'star';
        
        const size = Math.random() * 4 + 2;
        const x = Math.random() * 100;
        const y = Math.random() * 100;
        const duration = Math.random() * 4 + 2;
        const delay = Math.random() * 5;
        const maxOpacity = Math.random() * 0.7 + 0.3;

        star.style.width = size + 'px';
        star.style.height = size + 'px';
        star.style.left = x + 'vw';
        star.style.top = y + 'vh';
        star.style.setProperty('--duration', duration + 's');
        star.style.setProperty('--delay', delay + 's');
        star.style.setProperty('--max-opacity', maxOpacity);

        starField.appendChild(star);
    }
});
'''


def get_template(title='Project', subtitle='Real Duckiebot'):
  return render_template(
      title=title,
      subtitle=subtitle,
      content_html=_CONTENT,
      extra_css=_EXTRA_CSS,
      extra_js=_EXTRA_JS,
  )


PROJECT_TEMPLATE = get_template()
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

            <!-- HSV Calibration card -->
            <div class="card">
                <div class="card-header">HSV Color Calibration</div>

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
                    <div class="slider-label"><span>Saturation Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yLowS" min="0" max="255" value="80" class="slider">
                        <input type="number" id="yLowS-input" min="0" max="255" value="80" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Saturation High</span><span style="color:var(--text-muted)">0-255</span></div>
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

                <div class="hsv-section-title white" style="margin-top:20px">White Line (right / solid)</div>

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
                    <div class="slider-label"><span>Saturation Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wLowS" min="0" max="255" value="0" class="slider">
                        <input type="number" id="wLowS-input" min="0" max="255" value="0" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Saturation High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wHighS" min="0" max="255" value="40" class="slider">
                        <input type="number" id="wHighS-input" min="0" max="255" value="40" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wLowV" min="0" max="255" value="180" class="slider">
                        <input type="number" id="wLowV-input" min="0" max="255" value="180" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wHighV" min="0" max="255" value="255" class="slider">
                        <input type="number" id="wHighV-input" min="0" max="255" value="255" class="input-box">
                    </div>
                </div>

                <div id="hsv-status" class="status"></div>
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
                    <span style="font-size:13px;color:var(--text-secondary);">Navigation</span>
                    <label style="position:relative;display:inline-block;width:48px;height:26px;">
                        <input type="checkbox" id="driveToggle" onchange="toggleMode(this.checked)"
                            style="opacity:0;width:0;height:0;">
                        <span id="toggleSlider" style="position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;
                            background:var(--bg-sidebar);border:2px solid var(--border-color);border-radius:26px;
                            transition:.3s;">
                            <span style="position:absolute;content:\'\';height:18px;width:18px;left:2px;bottom:2px;
                                background:var(--text-muted);border-radius:50%;transition:.3s;display:block;"
                                id="toggleKnob"></span>
                        </span>
                    </label>
                    <span style="font-size:13px;color:var(--text-secondary);">Manual Drive</span>
                </div>
                <div id="modeStatus" style="font-size:12px;color:var(--text-muted);margin-top:4px;">Mode: Navigation</div>
            </div>

            <div class="card" id="driveCard" style="display:none;">
                <div class="card-header">Drive</div>
                <div class="key-display">
                    <div class="key-box key-up"    id="key-up">&#9650;</div>
                    <div class="key-box key-left"  id="key-left">&#9664;</div>
                    <div class="key-box key-down"  id="key-down">&#9660;</div>
                    <div class="key-box key-right" id="key-right">&#9654;</div>
                </div>
                <p style="text-align:center;font-size:11px;color:var(--text-muted)">Arrow keys or WASD</p>
            </div>



            <div class="card">
                <div class="card-header">Dance Maneuver</div>
                <div style="display:flex;flex-direction:column;gap:8px;">
                    <button class="button" onclick="sendDance()">Dance</button>
                    <div id="danceStatus" class="status"></div>
                </div>
            </div>
        </div>
    </div>
'''

_EXTRA_CSS = '''
#statusTable .row {
    display: flex;
    justify-content: space-between;
    padding: 6px 0;
    border-bottom: 1px solid var(--border-color);
    align-items: baseline;
}
#statusTable .row:last-child { border-bottom: none; }
#statusTable .key  { color: var(--text-secondary); font-size: 12px; }
#statusTable .val  { color: var(--text-primary);   font-weight: 500; font-size: 13px; font-family: monospace; }

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
    transition: all 0.15s ease;
    user-select: none;
}
.key-box.active { background: rgba(63,185,80,0.2); border-color: var(--accent-green); color: var(--accent-green); box-shadow: 0 0 8px rgba(63,185,80,0.25); }
.hsv-section-title { font-size: 13px; font-weight: 600; color: var(--text-secondary); margin: 12px 0 8px; text-transform: uppercase; letter-spacing: 0.5px; }
.hsv-section-title.yellow { color: #f1c40f; }
.hsv-section-title.white  { color: #ecf0f1; }
.key-up    { grid-area: up; }
.key-down  { grid-area: down; }
.key-left  { grid-area: left; }
.key-right { grid-area: right; }
/* Styles for Standalone Track Grid Panels */
.standalone-map-container {
    position: relative;
    display: inline-block;
    width: 100%;
    background: #0f141c;
    border-radius: 8px;
    overflow: hidden;
    border: 1px solid var(--border-color);
    box-shadow: inset 0 1px 3px rgba(0,0,0,0.3);
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
    transition: background 0.15s ease, outline 0.15s ease;
}
.standalone-tile:hover {
    background: rgba(255, 159, 67, 0.25);
    outline: 1px solid #ff9f43;
    z-index: 5;
}
.standalone-tile.start-selected {
    background: rgba(63, 185, 80, 0.2);
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

const HSV_SLIDER_MAP = {
    'yLowH':  'yellow_lower_h', 'yHighH': 'yellow_upper_h',
    'yLowS':  'yellow_lower_s', 'yHighS': 'yellow_upper_s',
    'yLowV':  'yellow_lower_v', 'yHighV': 'yellow_upper_v',
    'wLowH':  'white_lower_h',  'wHighH': 'white_upper_h',
    'wLowS':  'white_lower_s',  'wHighS': 'white_upper_s',
    'wLowV':  'white_lower_v',  'wHighV': 'white_upper_v',
};

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
    document.getElementById('modeStatus').textContent = 'Mode: ' + (isManual ? 'Manual Drive' : 'Navigation');
    document.getElementById('toggleKnob').style.left = isManual ? '26px' : '2px';
    document.getElementById('toggleSlider').style.background = isManual ? 'rgba(63,185,80,0.3)' : 'var(--bg-sidebar)';
    document.getElementById('toggleSlider').style.borderColor = isManual ? 'var(--accent-green)' : 'var(--border-color)';
    document.getElementById('toggleKnob').style.background = isManual ? 'var(--accent-green)' : 'var(--text-muted)';

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

function refreshStatus() {
    fetch('/status')
        .then(r => r.json())
        .then(data => {
            const table = document.getElementById('statusTable');
            const keys = Object.keys(data);
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

// Sync mode with server on load — keys are enabled immediately when server is in manual mode
fetch('/status').then(r => r.json()).then(data => {
    const serverManual = data.mode === 'manual';
    document.getElementById('driveToggle').checked = serverManual;
    toggleMode(serverManual);
}).catch(() => { });

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
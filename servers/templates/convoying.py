from .base import render_template, BASE_CSS, BASE_JS

_EXTRA_CSS = '''
.distance-badge {
    display: inline-block;
    padding: 4px 12px;
    border-radius: 12px;
    font-size: 13px;
    font-weight: 700;
    letter-spacing: 0.5px;
    text-transform: uppercase;
}
.distance-badge.LOST      { background: rgba(248,81,73,0.15);  border: 1px solid var(--accent-red);    color: var(--accent-red); }
.distance-badge.TOO_CLOSE { background: rgba(248,81,73,0.15);  border: 1px solid var(--accent-red);    color: var(--accent-red); }
.distance-badge.CLOSE     { background: rgba(210,153,34,0.15); border: 1px solid var(--accent-orange); color: var(--accent-orange); }
.distance-badge.GOOD      { background: rgba(63,185,80,0.15);  border: 1px solid var(--accent-green);  color: var(--accent-green); }
.distance-badge.FAR       { background: rgba(31,111,235,0.15); border: 1px solid var(--accent-blue);   color: var(--accent-blue); }

/* Follower state machine badge -- distinct palette from distance-badge so
   the two concepts (overall state vs. raw distance bucket) read as
   visually separate at a glance. */
.state-badge {
    display: inline-block;
    padding: 4px 12px;
    border-radius: 12px;
    font-size: 13px;
    font-weight: 700;
    letter-spacing: 0.5px;
    text-transform: uppercase;
}
.state-badge.SEARCH          { background: rgba(139,148,158,0.15); border: 1px solid var(--text-muted);    color: var(--text-muted); }
.state-badge.FOLLOW_LEFT     { background: rgba(31,111,235,0.15);  border: 1px solid var(--accent-blue);   color: var(--accent-blue); }
.state-badge.FOLLOW_CENTER   { background: rgba(63,185,80,0.15);   border: 1px solid var(--accent-green);  color: var(--accent-green); }
.state-badge.FOLLOW_RIGHT    { background: rgba(31,111,235,0.15);  border: 1px solid var(--accent-blue);   color: var(--accent-blue); }
.state-badge.STOPPED         { background: rgba(210,153,34,0.15);  border: 1px solid var(--accent-orange); color: var(--accent-orange); }
.state-badge.LOST_TARGET     { background: rgba(210,153,34,0.15);  border: 1px solid #d6a63a;              color: #d6a63a; }
.state-badge.TOO_CLOSE_STATE { background: rgba(248,81,73,0.15);   border: 1px solid var(--accent-red);    color: var(--accent-red); }

.speed-bar-row {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 6px;
    font-size: 12px;
}
.speed-bar-label { width: 38px; color: var(--text-muted); flex-shrink: 0; }
.speed-bar-track {
    flex: 1;
    height: 8px;
    background: var(--bg-sidebar);
    border-radius: 4px;
    overflow: hidden;
    border: 1px solid var(--border-color);
}
.speed-bar-fill {
    height: 100%;
    border-radius: 4px;
    background: var(--accent-blue);
    transition: width 0.15s;
}
.speed-bar-value { width: 38px; text-align: right; color: var(--text-secondary); flex-shrink: 0; font-variant-numeric: tabular-nums; }

.model-status { padding: 6px 10px; border-radius: 4px; font-size: 12px; margin-bottom: 10px; }
.model-status.ok  { background: rgba(63,185,80,0.1);  border: 1px solid rgba(63,185,80,0.3);  color: var(--accent-green); }
.model-status.err { background: rgba(248,81,73,0.1);  border: 1px solid rgba(248,81,73,0.3);  color: var(--accent-red); }
.model-status.loading { background: rgba(210,153,34,0.1); border: 1px solid rgba(210,153,34,0.3); color: #d6a63a; }

.reason-text { font-size: 11px; color: var(--text-muted); margin-top: 4px; font-style: italic; }

/* Manual driving */
.manual-toggle-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 12px;
}
.manual-badge {
    display: inline-block;
    padding: 4px 12px;
    border-radius: 12px;
    font-size: 12px;
    font-weight: 700;
    letter-spacing: 0.5px;
    text-transform: uppercase;
}
.manual-badge.off { background: rgba(139,148,158,0.15); border: 1px solid var(--text-muted); color: var(--text-muted); }
.manual-badge.on  { background: rgba(210,153,34,0.15);  border: 1px solid #d6a63a;            color: #d6a63a; }

.dpad {
    display: grid;
    grid-template-columns: 56px 56px 56px;
    grid-template-rows: 56px 56px 56px;
    gap: 6px;
    justify-content: center;
    margin: 14px 0 6px;
    user-select: none;
}
.dpad-btn {
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 22px;
    background: var(--bg-sidebar);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    color: var(--text-secondary);
    cursor: pointer;
    touch-action: none;
}
.dpad-btn:active, .dpad-btn.active {
    background: var(--accent-blue);
    color: #fff;
    border-color: var(--accent-blue);
}
.dpad-btn.disabled {
    opacity: 0.35;
    cursor: not-allowed;
}
.dpad-up    { grid-column: 2; grid-row: 1; }
.dpad-left  { grid-column: 1; grid-row: 2; }
.dpad-stop  { grid-column: 2; grid-row: 2; font-size: 13px; }
.dpad-right { grid-column: 3; grid-row: 2; }
.dpad-down  { grid-column: 2; grid-row: 3; }

.manual-hint { font-size: 11px; color: var(--text-muted); text-align: center; margin-top: 4px; }
'''

_CONTENT = '''
    <div class="container">
        <div class="video-section">
            <img src="/video" class="stream">
        </div>

        <div class="controls-section">

            <!-- Drive Control -->
            <div class="card">
                <div class="card-header">Drive Control</div>
                <div style="display:flex;align-items:center;gap:12px;margin-bottom:12px">
                    <span id="run-indicator" style="width:14px;height:14px;border-radius:50%;background:#e74c3c;flex-shrink:0;display:inline-block"></span>
                    <span id="run-label" style="font-size:14px;font-weight:600;color:var(--text-secondary)">STOPPED</span>
                </div>
                <button class="button success" onclick="post('/start')">▶ Start</button>
                <button class="button danger"  onclick="post('/stop')">■ Stop</button>
                <button class="button"         onclick="post('/reset')">↺ Reset Tracker</button>
                <div id="ctrl-status" class="status"></div>
            </div>

            <!-- Manual Driving -->
            <div class="card">
                <div class="card-header">Manual Driving</div>
                <div class="manual-toggle-row">
                    <span id="manual-badge" class="manual-badge off">AUTONOMOUS</span>
                    <button class="button" id="manual-toggle-btn" onclick="toggleManual()">Enable Manual Mode</button>
                </div>

                <div class="dpad" id="dpad">
                    <div class="dpad-btn dpad-up disabled"    data-key="ArrowUp">▲</div>
                    <div class="dpad-btn dpad-left disabled"  data-key="ArrowLeft">◀</div>
                    <div class="dpad-btn dpad-stop disabled"  id="dpad-stop">STOP</div>
                    <div class="dpad-btn dpad-right disabled" data-key="ArrowRight">▶</div>
                    <div class="dpad-btn dpad-down disabled"  data-key="ArrowDown">▼</div>
                </div>
                <div class="manual-hint" id="manual-hint">Enable manual mode, then use arrow keys or the buttons above. Keys can be held and combined (e.g. ↑ + ←).</div>
            </div>

            <!-- Follower State -->
            <div class="card">
                <div class="card-header">Follower State</div>
                <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:10px">
                    <span style="font-size:13px;color:var(--text-secondary)">Current state</span>
                    <span id="state-badge" class="state-badge SEARCH">SEARCH</span>
                </div>
                <div class="reason-text" id="state-reason">—</div>
            </div>

            <!-- Model Status -->
            <div class="card">
                <div class="card-header">System</div>
                <div id="model-status" class="model-status loading">Loading detection model...</div>
                <div class="config-item">
                    <span class="config-label">Host</span>
                    <span class="config-value">{{ hostname }}</span>
                </div>
                <div class="config-item">
                    <span class="config-label">Lane frames</span>
                    <span class="config-value" id="lane-frames">0</span>
                </div>
            </div>

            <!-- Target Status -->
            <div class="card">
                <div class="card-header">Target Truck &amp; Marker</div>
                <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:10px">
                    <span style="font-size:13px;color:var(--text-secondary)">Distance state</span>
                    <span id="distance-badge" class="distance-badge LOST">LOST</span>
                </div>
                <div class="stats-grid" style="grid-template-columns:1fr 1fr 1fr;">
                    <div class="stat-box">
                        <div class="stat-value" id="target-score">—</div>
                        <div class="stat-label">Score</div>
                    </div>
                    <div class="stat-box">
                        <div class="stat-value" id="target-area">—</div>
                        <div class="stat-label">Area px²</div>
                    </div>
                    <div class="stat-box">
                        <div class="stat-value" id="target-bottom">—</div>
                        <div class="stat-label">Bottom Y</div>
                    </div>
                </div>
                <div class="stats-grid" style="grid-template-columns:1fr;margin-top:8px">
                    <div class="stat-box">
                        <div class="stat-value" id="target-dots">—</div>
                        <div class="stat-label">Marker dots detected</div>
                    </div>
                </div>
                <div class="reason-text" id="target-reason">waiting for detections...</div>
            </div>

            <!-- Speeds -->
            <div class="card">
                <div class="card-header">
                    Wheel Speeds
                    <span id="multiplier-badge" style="font-size:11px;font-weight:400;color:var(--text-muted)">×1.00</span>
                </div>
                <div class="speed-bar-row">
                    <span class="speed-bar-label">Lane L</span>
                    <div class="speed-bar-track"><div class="speed-bar-fill" id="bar-lane-l" style="width:0%"></div></div>
                    <span class="speed-bar-value" id="val-lane-l">0.000</span>
                </div>
                <div class="speed-bar-row">
                    <span class="speed-bar-label">Lane R</span>
                    <div class="speed-bar-track"><div class="speed-bar-fill" id="bar-lane-r" style="width:0%"></div></div>
                    <span class="speed-bar-value" id="val-lane-r">0.000</span>
                </div>
                <div class="speed-bar-row" style="margin-top:6px">
                    <span class="speed-bar-label">Final L</span>
                    <div class="speed-bar-track"><div class="speed-bar-fill" id="bar-final-l" style="width:0%;background:var(--accent-green)"></div></div>
                    <span class="speed-bar-value" id="val-final-l">0.000</span>
                </div>
                <div class="speed-bar-row">
                    <span class="speed-bar-label">Final R</span>
                    <div class="speed-bar-track"><div class="speed-bar-fill" id="bar-final-r" style="width:0%;background:var(--accent-green)"></div></div>
                    <span class="speed-bar-value" id="val-final-r">0.000</span>
                </div>
                <div class="reason-text" id="command-reason">—</div>
            </div>

            <!-- Tuning -->
            <div class="card">
                <div class="card-header">Speed &amp; Steering Tuning</div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Close (slow down)</span>
                        <span id="close-val">0.05</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="close-slider" min="0" max="1" step="0.05" value="0.05"
                               oninput="document.getElementById('close-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Good (maintain)</span>
                        <span id="good-val">0.75</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="good-slider" min="0" max="1.5" step="0.05" value="0.75"
                               oninput="document.getElementById('good-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Far (catch up)</span>
                        <span id="far-val">1.20</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="far-slider" min="0" max="2" step="0.05" value="1.20"
                               oninput="document.getElementById('far-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Steering gain</span>
                        <span id="steer-gain-val">1.25</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="steer-gain-slider" min="0" max="3" step="0.05" value="1.25"
                               oninput="document.getElementById('steer-gain-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Lane weight</span>
                        <span id="lane-gain-val">0.65</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="lane-gain-slider" min="0" max="2" step="0.05" value="0.65"
                               oninput="document.getElementById('lane-gain-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Leader weight</span>
                        <span id="leader-gain-val">0.85</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="leader-gain-slider" min="0" max="2" step="0.05" value="0.85"
                               oninput="document.getElementById('leader-gain-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Search speed</span>
                        <span id="search-speed-val">0.08</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="search-speed-slider" min="0" max="0.3" step="0.01" value="0.08"
                               oninput="document.getElementById('search-speed-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div id="tune-status" class="status"></div>
            </div>

        </div>
    </div>
'''

_EXTRA_JS = '''
function post(path) {
    fetch(path, {method: 'POST'})
        .then(r => r.json())
        .then(data => {
            const msg = data.status || JSON.stringify(data);
            showStatus('ctrl-status', msg, 'success');
        })
        .catch(() => showStatus('ctrl-status', 'Request failed', 'error'));
}

function postJSONraw(path, data) {
    return fetch(path, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(data),
    }).then(r => r.json());
}

function setBar(barId, valId, value, maxVal) {
    const pct = Math.min(100, Math.max(0, (value / maxVal) * 100));
    document.getElementById(barId).style.width = pct + '%';
    document.getElementById(valId).textContent = value.toFixed(3);
}

// ---------------------------------------------------------------------
// Manual driving
// ---------------------------------------------------------------------

let manualMode = false;

// Currently-held direction keys/buttons. forward/turn are recomputed from
// this set every time it changes, so holding e.g. ArrowUp + ArrowLeft
// combines naturally (forward=1, turn=-1) without extra bookkeeping.
const heldDirections = new Set();

const KEY_TO_DIR = {
    'ArrowUp': 'up',
    'ArrowDown': 'down',
    'ArrowLeft': 'left',
    'ArrowRight': 'right',
};

function toggleManual() {
    const path = manualMode ? '/manual/disable' : '/manual/enable';
    fetch(path, {method: 'POST'})
        .then(r => r.json())
        .then(data => {
            manualMode = !!data.manual_mode;
            heldDirections.clear();
            updateManualUI();
            if (manualMode) {
                sendManualDrive();
            }
        })
        .catch(() => showStatus('ctrl-status', 'Manual mode toggle failed', 'error'));
}

function updateManualUI() {
    const badge = document.getElementById('manual-badge');
    const btn = document.getElementById('manual-toggle-btn');
    const dpadButtons = document.querySelectorAll('#dpad .dpad-btn');

    if (manualMode) {
        badge.textContent = 'MANUAL';
        badge.className = 'manual-badge on';
        btn.textContent = 'Disable Manual Mode';
    } else {
        badge.textContent = 'AUTONOMOUS';
        badge.className = 'manual-badge off';
        btn.textContent = 'Enable Manual Mode';
    }

    dpadButtons.forEach(el => {
        el.classList.toggle('disabled', !manualMode);
    });
}

function directionsToForwardTurn() {
    let forward = 0.0;
    let turn = 0.0;
    if (heldDirections.has('up'))    forward += 1.0;
    if (heldDirections.has('down'))  forward -= 1.0;
    if (heldDirections.has('right')) turn += 1.0;
    if (heldDirections.has('left'))  turn -= 1.0;
    return {forward: Math.max(-1, Math.min(1, forward)), turn: Math.max(-1, Math.min(1, turn))};
}

function sendManualDrive() {
    if (!manualMode) return;
    const {forward, turn} = directionsToForwardTurn();
    postJSONraw('/manual/drive', {forward, turn}).catch(() => {});
}

function pressDirection(dir) {
    if (!manualMode) return;
    if (!heldDirections.has(dir)) {
        heldDirections.add(dir);
        sendManualDrive();
    }
}

function releaseDirection(dir) {
    if (heldDirections.has(dir)) {
        heldDirections.delete(dir);
        sendManualDrive();
    }
}

function releaseAllDirections() {
    if (heldDirections.size > 0) {
        heldDirections.clear();
        sendManualDrive();
    }
}

// Keyboard: arrow keys, held = continuous drive, released = stop that axis.
window.addEventListener('keydown', (e) => {
    const dir = KEY_TO_DIR[e.key];
    if (!dir) return;
    e.preventDefault();
    pressDirection(dir);
});

window.addEventListener('keyup', (e) => {
    const dir = KEY_TO_DIR[e.key];
    if (!dir) return;
    e.preventDefault();
    releaseDirection(dir);
});

// If the window/tab loses focus while a key is held, stop driving --
// otherwise a held arrow key could keep the bot moving with no way to
// release it (e.g. alt-tabbing away).
window.addEventListener('blur', releaseAllDirections);

// On-screen D-pad buttons (mouse + touch), same press/hold/release model.
document.addEventListener('DOMContentLoaded', () => {
    const dpadButtons = document.querySelectorAll('#dpad .dpad-btn[data-key]');
    dpadButtons.forEach(el => {
        const dir = KEY_TO_DIR[el.dataset.key];
        const start = (e) => { e.preventDefault(); pressDirection(dir); el.classList.add('active'); };
        const end = (e) => { e.preventDefault(); releaseDirection(dir); el.classList.remove('active'); };
        el.addEventListener('mousedown', start);
        el.addEventListener('mouseup', end);
        el.addEventListener('mouseleave', end);
        el.addEventListener('touchstart', start, {passive: false});
        el.addEventListener('touchend', end, {passive: false});
        el.addEventListener('touchcancel', end, {passive: false});
    });

    const stopBtn = document.getElementById('dpad-stop');
    stopBtn.addEventListener('click', () => {
        if (!manualMode) return;
        releaseAllDirections();
    });

    updateManualUI();
});

// ---------------------------------------------------------------------

function updateStatus() {
    fetch('/status')
        .then(r => r.json())
        .then(data => {
            // Running indicator
            const running = data.running;
            document.getElementById('run-indicator').style.background = running ? '#3fb950' : '#e74c3c';
            document.getElementById('run-label').textContent = running ? 'RUNNING' : 'STOPPED';

            // Manual mode (in case it was changed from elsewhere, e.g. another tab)
            if (data.manual_mode !== manualMode) {
                manualMode = !!data.manual_mode;
                if (!manualMode) heldDirections.clear();
                updateManualUI();
            }

            // Model
            const modelEl = document.getElementById('model-status');
            if (data.model_loaded) {
                modelEl.className = 'model-status ok';
                modelEl.textContent = '✓ Detection model loaded';
            } else if (data.model_load_error) {
                modelEl.className = 'model-status err';
                modelEl.textContent = '✗ ' + data.model_load_error;
            } else {
                modelEl.className = 'model-status loading';
                modelEl.textContent = 'Loading detection model...';
            }

            document.getElementById('lane-frames').textContent = data.lane_frame_count || 0;

            // Follower state machine (primary state, distinct from raw distance_state)
            const followerState = data.follower_state || 'SEARCH';
            const stateBadge = document.getElementById('state-badge');
            stateBadge.textContent = followerState;
            stateBadge.className = 'state-badge ' + followerState;

            const cmd = data.command;
            document.getElementById('state-reason').textContent = cmd ? cmd.reason : '—';

            // Target / marker
            const t = data.target;
            const distState = t ? t.distance_state : 'LOST';
            const badge = document.getElementById('distance-badge');
            badge.textContent = distState;
            badge.className = 'distance-badge ' + distState;

            document.getElementById('target-score').textContent  = t && t.found ? t.score.toFixed(2) : '—';
            document.getElementById('target-area').textContent   = t && t.found ? t.area : '—';
            document.getElementById('target-bottom').textContent = t && t.found ? t.bottom_y : '—';
            document.getElementById('target-dots').textContent   = t ? (t.dot_count || 0) : '—';
            document.getElementById('target-reason').textContent = t ? t.reason : '—';

            // Speeds
            const laneMax = 1.5;
            setBar('bar-lane-l',  'val-lane-l',  data.lane_left  || 0, laneMax);
            setBar('bar-lane-r',  'val-lane-r',  data.lane_right || 0, laneMax);

            const mult = cmd ? cmd.speed_multiplier : 0;
            document.getElementById('multiplier-badge').textContent = '×' + mult.toFixed(2);

            let reasonText = cmd ? cmd.reason : '—';
            if (manualMode) {
                reasonText = '(manual driving — autonomous command shown for reference only)';
            }
            document.getElementById('command-reason').textContent = reasonText;

            // While in manual mode, show the actual manual wheel speeds
            // being sent to the motors instead of the (ignored) autonomous
            // command, so the bars reflect what the robot is really doing.
            if (manualMode) {
                setBar('bar-final-l', 'val-final-l', data.manual_left  || 0, laneMax);
                setBar('bar-final-r', 'val-final-r', data.manual_right || 0, laneMax);
            } else {
                setBar('bar-final-l', 'val-final-l', cmd ? cmd.left_speed  : 0, laneMax);
                setBar('bar-final-r', 'val-final-r', cmd ? cmd.right_speed : 0, laneMax);
            }
        })
        .catch(() => {});
}

function sendConfig() {
    const data = {
        close_multiplier: parseFloat(document.getElementById('close-slider').value),
        good_multiplier:  parseFloat(document.getElementById('good-slider').value),
        far_multiplier:   parseFloat(document.getElementById('far-slider').value),
        steering_gain:    parseFloat(document.getElementById('steer-gain-slider').value),
        lane_gain:        parseFloat(document.getElementById('lane-gain-slider').value),
        leader_gain:      parseFloat(document.getElementById('leader-gain-slider').value),
        search_speed:     parseFloat(document.getElementById('search-speed-slider').value),
    };
    postJSONraw('/update_config', data)
        .then(() => showStatus('tune-status', 'Config updated', 'success'))
        .catch(() => showStatus('tune-status', 'Update failed', 'error'));
}

setInterval(updateStatus, 300);
updateStatus();
'''


CONVOYING_TEMPLATE = render_template(
    title    = "Convoying — Marker-Based Following",
    subtitle = "State machine (SEARCH / FOLLOW_LEFT / FOLLOW_CENTER / FOLLOW_RIGHT / STOPPED / LOST_TARGET) driven by the leader's marker bracket",
    content_html = _CONTENT,
    extra_css    = _EXTRA_CSS,
    extra_js     = _EXTRA_JS,
)

from .base import render_template

_EXTRA_CSS = '''
.status-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
    margin-top: 8px;
}

.status-box {
    text-align: center;
    padding: 8px;
    background: var(--bg-sidebar);
    border: 1px solid var(--border-color);
    border-radius: 6px;
}

.status-value {
    font-size: 18px;
    font-weight: 700;
    font-family: monospace;
    color: var(--accent-blue);
}

.status-label {
    font-size: 11px;
    color: var(--text-muted);
    text-transform: uppercase;
    margin-top: 3px;
}

.convoy-buttons {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 6px;
    margin-top: 8px;
}

.convoy-btn {
    padding: 8px;
    font-size: 13px;
    border: 1px solid var(--border-color);
    border-radius: 4px;
    cursor: pointer;
    background: var(--bg-sidebar);
    color: var(--text-secondary);
    font-family: 'Inter', sans-serif;
}

.convoy-btn:hover {
    border-color: var(--accent-blue);
    color: var(--text-primary);
}

.instructions {
    font-size: 12px;
    color: var(--text-secondary);
    line-height: 1.6;
}

.instructions code {
    background: var(--bg-sidebar);
    padding: 2px 6px;
    border-radius: 3px;
    font-size: 11px;
    color: var(--accent-orange);
}

.file-path {
    background: var(--bg-sidebar);
    border: 1px solid var(--border-color);
    border-radius: 4px;
    padding: 8px 12px;
    font-family: monospace;
    font-size: 12px;
    color: var(--accent-green);
    margin: 8px 0;
    word-break: break-all;
}

.running-ok {
    color: var(--accent-green);
}

.running-stop {
    color: var(--accent-red);
}
'''

_CONTENT = '''
    <div class="container">
        <div class="video-section">
            <img src="{{ url_for('video') }}" class="stream">
        </div>

        <div class="controls-section">
            <div class="card">
                <div class="card-header">Convoying Control</div>

                <div class="convoy-buttons">
                    <button class="convoy-btn" onclick="post('/start')">Start</button>
                    <button class="convoy-btn" onclick="post('/stop')">Stop</button>
                    <button class="convoy-btn" onclick="post('/reset')">Reset</button>
                </div>

                <div class="status-grid">
                    <div class="status-box">
                        <div class="status-value" id="running-state">false</div>
                        <div class="status-label">Running</div>
                    </div>
                    <div class="status-box">
                        <div class="status-value" id="target-state">LOST</div>
                        <div class="status-label">Target</div>
                    </div>
                    <div class="status-box">
                        <div class="status-value" id="left-speed">0.00</div>
                        <div class="status-label">Left wheel</div>
                    </div>
                    <div class="status-box">
                        <div class="status-value" id="right-speed">0.00</div>
                        <div class="status-label">Right wheel</div>
                    </div>
                </div>
            </div>

            <div class="card">
                <div class="card-header">Assignment</div>
                <div class="instructions">
                    <p>Convoying combines three systems:</p>
                    <p>
                        Object detection finds the truck.<br>
                        Lane servoing keeps the robot in the lane.<br>
                        Convoying adjusts speed based on truck distance.
                    </p>

                    <p>Main student files:</p>
                    <div class="file-path">tasks/convoying/packages/target_tracker_activity.py</div>
                    <div class="file-path">tasks/convoying/packages/convoy_controller_activity.py</div>

                    <p style="margin-top:8px">
                        Truck far = speed up.<br>
                        Truck good distance = normal speed.<br>
                        Truck close = slow down.<br>
                        Truck too close or lost = stop.
                    </p>
                </div>
            </div>

            <div class="card">
                <div class="card-header">Debug</div>
                <div class="instructions">
                    <p id="target-reason">Target reason: none</p>
                    <p id="command-reason">Command reason: none</p>
                    <p id="multiplier">Multiplier: 0.00</p>
                </div>
            </div>
        </div>
    </div>
'''

_JS = '''
    function post(path) {
        fetch(path, {method: 'POST'})
            .then(r => r.json())
            .then(data => console.log(data))
            .catch(() => {});
    }

    function setText(id, value) {
        const el = document.getElementById(id);
        if (el) el.textContent = value;
    }

    function updateStatus() {
        fetch('/status')
            .then(r => r.json())
            .then(data => {
                setText('running-state', data.running ? 'true' : 'false');

                const runningEl = document.getElementById('running-state');
                if (runningEl) {
                    runningEl.classList.toggle('running-ok', data.running);
                    runningEl.classList.toggle('running-stop', !data.running);
                }

                const target = data.target || {};
                const command = data.command || {};

                setText('target-state', target.distance_state || 'LOST');
                setText('left-speed', command.left_speed !== undefined ? command.left_speed.toFixed(2) : '0.00');
                setText('right-speed', command.right_speed !== undefined ? command.right_speed.toFixed(2) : '0.00');

                setText('target-reason', 'Target reason: ' + (target.reason || 'none'));
                setText('command-reason', 'Command reason: ' + (command.reason || 'none'));
                setText('multiplier', 'Multiplier: ' + (
                    command.speed_multiplier !== undefined
                        ? command.speed_multiplier.toFixed(2)
                        : '0.00'
                ));
            })
            .catch(() => {});
    }

    setInterval(updateStatus, 500);
    updateStatus();
'''

CONVOYING_TEMPLATE = render_template(
    'Convoying',
    'Follow the truck while staying in lane',
    _CONTENT,
    extra_css=_EXTRA_CSS,
    extra_js=_JS,
)
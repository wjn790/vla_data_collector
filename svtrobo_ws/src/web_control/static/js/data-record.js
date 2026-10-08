/**
 * SVTROBO Web Control - Data Recording Module
 * Floating button for one-click data collection:
 *   - Camera images (D405 #1, D405 #2, ZED 2i)
 *   - Motor status (/chassis/joint_states, /chassis/diagnostics)
 *   - Chassis commands (/svtrobot_cmd)
 *   - Lift commands (/lift_control_cmd)
 */

const DataRecord = {
    recording: false,
    startTime: null,
    durationTimer: null,
    statusTimer: null,
    serverUrl: '',
    _disabled: true,
    _idlePollTimer: null,
    _lastKnownRunning: false,

    init() {
        // Derive server URL (same as Camera module)
        const loc = window.location;
        this.serverUrl = loc.protocol + '//' + loc.host;

        // Create floating button
        const btn = document.createElement('div');
        btn.id = 'record-btn';
        btn.innerHTML =
            '<div class="record-dot"></div>' +
            '<span class="record-text">录制</span>';
        btn.addEventListener('click', () => this.toggle());
        document.body.appendChild(btn);

        // Timer overlay
        const timer = document.createElement('div');
        timer.id = 'record-timer';
        document.body.appendChild(timer);

        // Initial disabled state
        this.updateUI();

        // Recording status is an HTTP backend state, not a ROS/master-control state.
        // Keep a low-frequency poll always running so recordings started by gamepad
        // or backend are still reflected even when ROS reconnects or this page is read-only.
        this._startIdlePolling();
    },

    enable() {
        this._disabled = false;
        this.updateUI();
        // Check if recording is already in progress (page refresh / reconnect case).
        this.checkStatus();
        this._startIdlePolling();
    },

    disable() {
        // Do not stop backend recording or status polling here. Recording is an HTTP
        // backend state and may have been started externally; disabling controls
        // should only prevent user actions from this page.
        this._disabled = true;
        this.updateUI();
    },

    async toggle() {
        // When a recording is already running, always allow opening the panel so
        // read-only / reconnecting pages can see backend progress. Starting a new
        // recording remains gated by _disabled inside start().
        if (this._disabled && !this.recording) return;
        if (typeof RecPanel !== 'undefined') RecPanel.show();
        return;
    },

    async start() {
        if (this._disabled) return;
        try {
            const resp = await fetch(this.serverUrl + '/recording/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
            });
            if (!resp.ok) {
                this.showToast('服务器返回错误 (' + resp.status + ')，请重启 Web 服务器', 'error');
                return;
            }
            const data = await resp.json();
            if (data.ok) {
                this.recording = true;
                this.startTime = Date.now();
                this.updateUI();
                this.startTimers();
                // Show large status panel
                // Panel already open, just update state
            } else {
                this.showToast('启动录制失败: ' + (data.message || '未知错误'), 'error');
            }
        } catch (e) {
            this.showToast('无法连接服务器，请检查 Web 服务是否运行', 'error');
        }
    },

    async stop() {
        try {
            const resp = await fetch(this.serverUrl + '/recording/stop', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
            });
            if (!resp.ok) {
                this.showToast('服务器返回错误 (' + resp.status + ')', 'error');
                return;
            }
            const data = await resp.json();
            if (data.ok) {
                this.recording = false;
                this.stopTimers();
                this.updateUI();
                if (typeof RecPanel !== 'undefined') RecPanel.onRecordingStopped();
                const dur = data.duration || 0;
                const path = data.path || '';
                // Check for watchdog abort
                const msg = data.message || '';
                if (msg.includes('ABORTED') || msg.includes('abort')) {
                    this.showToast('⚠ 录制被异常中断！请检查相机连接\n时长 ' + dur + 's', 'error');
                } else {
                    this.showToast('录制完成！时长 ' + dur + 's\n保存至: ' + path, 'success');
                }
            } else {
                this.showToast('停止录制失败: ' + (data.message || '未知错误'), 'error');
            }
        } catch (e) {
            this.showToast('无法连接服务器', 'error');
        }
    },

    _startIdlePolling() {
        this._stopIdlePolling();
        // Keep low frequency to avoid adding load on the Jetson/web server.
        this._idlePollTimer = setInterval(() => this.checkStatus(), 5000);
    },

    _stopIdlePolling() {
        if (this._idlePollTimer) {
            clearInterval(this._idlePollTimer);
            this._idlePollTimer = null;
        }
    },

    async checkStatus() {
        try {
            const resp = await fetch(this.serverUrl + '/recording/status');
            if (!resp.ok) return;
            const data = await resp.json();
            const wasRunning = this._lastKnownRunning;
            this._lastKnownRunning = data.running === true;

            if (data.running) {
                const newlyDetected = !this.recording || !wasRunning;
                if (!this.recording) {
                    // Recording started externally (e.g. gamepad/backend) or page reconnected.
                    this.recording = true;
                    this.startTime = Date.now() - data.elapsed * 1000;
                    this.updateUI();
                    this.startTimers();
                } else {
                    // Update elapsed time to stay in sync.
                    this.startTime = Date.now() - data.elapsed * 1000;
                }
                // Sync panel state even if the page is currently disabled/read-only.
                if (typeof RecPanel !== 'undefined') {
                    if (newlyDetected) RecPanel.syncRecordingState(data);
                    if (newlyDetected) RecPanel.show();
                }
            } else if (this.recording) {
                // Recording stopped externally (e.g. gamepad/watchdog)
                this.recording = false;
                this.stopTimers();
                this.updateUI();
                if (typeof RecPanel !== 'undefined') RecPanel.onRecordingStopped();
            }
        } catch (e) {
            // Server unreachable, ignore silently
        }
    },

    startTimers() {
        if (this.durationTimer) clearInterval(this.durationTimer);
        if (this.statusTimer) clearInterval(this.statusTimer);
        this.durationTimer = setInterval(() => this.updateTimer(), 1000);
        // Do not add another /recording/status timer here. The always-on idle
        // poll already checks every 5s; duplicating it would add unnecessary load.
        this.statusTimer = null;
        this.updateTimer();
    },

    stopTimers() {
        if (this.durationTimer) {
            clearInterval(this.durationTimer);
            this.durationTimer = null;
        }
        if (this.statusTimer) {
            clearInterval(this.statusTimer);
            this.statusTimer = null;
        }
        document.getElementById('record-timer').style.display = 'none';
    },

    updateUI() {
        const btn = document.getElementById('record-btn');
        const timer = document.getElementById('record-timer');

        if (!btn) return;

        // Disabled state (not connected)
        if (this._disabled && !this.recording) {
            btn.classList.add('record-disabled');
            btn.classList.remove('recording');
            btn.querySelector('.record-text').textContent = '录制';
            timer.style.display = 'none';
            return;
        }

        btn.classList.remove('record-disabled');

        if (this.recording) {
            btn.classList.add('recording');
            btn.querySelector('.record-text').textContent = '停止';
            timer.style.display = 'block';
        } else {
            btn.classList.remove('recording');
            btn.querySelector('.record-text').textContent = '录制';
            timer.style.display = 'none';
        }
    },

    updateTimer() {
        if (!this.startTime) return;
        const elapsed = Math.floor((Date.now() - this.startTime) / 1000);
        const hh = String(Math.floor(elapsed / 3600)).padStart(2, '0');
        const mm = String(Math.floor((elapsed % 3600) / 60)).padStart(2, '0');
        const ss = String(elapsed % 60).padStart(2, '0');
        const display = elapsed >= 3600 ? hh + ':' + mm + ':' + ss : mm + ':' + ss;
        document.getElementById('record-timer').textContent = display;
    },

    showToast(message, type) {
        // Remove existing toast
        const old = document.getElementById('record-toast');
        if (old) old.remove();

        const toast = document.createElement('div');
        toast.id = 'record-toast';
        toast.className = 'record-toast record-toast-' + (type || 'info');
        toast.textContent = message;

        document.body.appendChild(toast);

        // Auto-remove after 4 seconds
        setTimeout(() => {
            toast.classList.add('record-toast-fade');
            setTimeout(() => toast.remove(), 500);
        }, 4000);
    },
};

document.addEventListener('DOMContentLoaded', () => {
    DataRecord.init();
    if (typeof RecPanel !== 'undefined') RecPanel.init();
});

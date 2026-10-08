/**
 * SVTROBO Recording Status Panel v3
 * 3-row layout, merged ZED, expanded ROS topics with real counts, no auto-close on stop.
 * Panel opens first, recording starts only when user clicks "start" inside panel.
 */

const RecPanel = {
    overlay: null,
    panel: null,
    timerEl: null,
    pollTimer: null,
    serverUrl: '',
    abortDetected: false,
    recordingDone: false,
    isRecording: false,
    _localTimer: null,
    _justStarted: false,

    ROWS: [
        ['zed_color', 'd405_1', 'd405_2'],
        ['zed_depth', 'pointcloud', 'imu'],
        ['ros_cmd', 'ros_lift', 'ros_joy'],
        ['ros_chassis', 'ros_diag', 'distance'],
    ],

    SOURCE_CFG: {
        'zed_color':   { label: 'ZED 彩色图', unit: '帧' },
        'd405_1':      { label: 'D405 #1',    unit: '帧' },
        'd405_2':      { label: 'D405 #2',    unit: '帧' },
        'zed_depth':   { label: 'ZED 深度图', unit: '帧' },
        'pointcloud':  { label: 'ZED 点云',   unit: '帧' },
        'imu':         { label: 'IMU',        unit: '条' },
        'ros_cmd':     { label: '底盘命令',   unit: '条' },
        'ros_joy':     { label: '手柄输入',   unit: '条' },
        'ros_lift':    { label: '升降控制',   unit: '条' },
        'ros_chassis': { label: '底盘状态',   unit: '条' },
        'ros_diag':    { label: '诊断信息',   unit: '条' },
        'distance':    { label: '测距传感器', unit: '条' },
    },

    init() {
        this.serverUrl = location.protocol + '//' + location.host;

        this.overlay = document.createElement('div');
        this.overlay.id = 'rec-panel-overlay';

        this.panel = document.createElement('div');
        this.panel.id = 'rec-panel';

        // Header
        const header = document.createElement('div');
        header.id = 'rec-panel-header';
        header.innerHTML =
            '<div id="rec-panel-title"><span class="rec-dot standby"></span><span>就绪</span></div>' +
            '<button id="rec-panel-close" title="关闭面板">&times;</button>';
        this.panel.appendChild(header);

        // Timer
        this.timerEl = document.createElement('div');
        this.timerEl.id = 'rec-panel-timer';
        this.timerEl.textContent = '00:00';
        this.panel.appendChild(this.timerEl);

        // 3 rows
        for (let i = 0; i < this.ROWS.length; i++) {
            const row = document.createElement('div');
            row.className = 'rec-row';

            const items = document.createElement('div');
            items.className = 'rec-row-items';
            // All rows use default 3-column grid — no special class needed
            for (const key of this.ROWS[i]) {
                const cfg = this.SOURCE_CFG[key];
                const item = document.createElement('div');
                item.className = 'rec-item';
                item.id = 'rec-item-' + key;
                item.innerHTML =
                    '<div class="rec-light idle" id="rec-light-' + key + '"></div>' +
                    '<div class="rec-item-info">' +
                        '<div class="rec-item-label">' + cfg.label + '</div>' +
                        '<div class="rec-item-value" id="rec-val-' + key + '">--</div>' +
                    '</div>';
                items.appendChild(item);
            }
            row.appendChild(items);
            this.panel.appendChild(row);
        }

        // Button area: two buttons, toggle visibility
        const btnWrap = document.createElement('div');
        btnWrap.id = 'rec-panel-stop';

        const startBtn = document.createElement('button');
        startBtn.id = 'rec-panel-start-btn';
        startBtn.className = 'rec-action-btn start';
        startBtn.textContent = '●  开始录制';
        startBtn.addEventListener('click', () => this._startRecording());

        const stopBtn = document.createElement('button');
        stopBtn.id = 'rec-panel-stop-btn';
        stopBtn.className = 'rec-action-btn stop';
        stopBtn.textContent = '■  停止录制';
        stopBtn.addEventListener('click', () => this._stopRecording());

        btnWrap.appendChild(startBtn);
        btnWrap.appendChild(stopBtn);
        this.panel.appendChild(btnWrap);

        this.overlay.appendChild(this.panel);
        document.body.appendChild(this.overlay);

        document.getElementById('rec-panel-close').addEventListener('click', () => this.hide());
        this.overlay.addEventListener('click', (e) => {
            if (e.target === this.overlay) this.hide();
        });
    },

    show() {
        // Check if already recording (e.g. external start)
        this._checkCurrentState();
        this.overlay.classList.add('active');
        // Start polling to sync state; keep interval low to protect Jetson load.
        this._startPolling();
    },

    syncRecordingState(data) {
        // Synchronous state reset used by DataRecord's always-on status poll.
        // This avoids stale "recordingDone" / aborted state blocking a new backend recording.
        if (!data || !data.running) return;
        this._stopLocalTimer();
        this._lastElapsed = undefined;
        this.isRecording = true;
        this.recordingDone = false;
        this.abortDetected = false;
        this.panel.classList.remove('aborted');
        this.timerEl.classList.remove('error-state');
        this._setRecordingUI();
        if (data.elapsed !== undefined) {
            this._lastElapsed = data.elapsed;
            this.updateTimer(data.elapsed);
        }
        if (data.sources) this.updateSources(data.sources);
    },

    hide() {
        this.overlay.classList.remove('active');
        this._stopLocalTimer();
        this._stopPolling();
    },

    async _checkCurrentState() {
        try {
            const resp = await fetch(this.serverUrl + '/recording/status');
            if (!resp.ok) return;
            const data = await resp.json();
            if (data.running) {
                // New recording detected — reset display from previous session
                this._stopLocalTimer();
                this._lastElapsed = undefined;
                this.timerEl.textContent = '00:00';
                for (const key of Object.keys(this.SOURCE_CFG)) {
                    this._setLight(key, 'idle');
                    document.getElementById('rec-val-' + key).textContent = '--';
                }
                this.syncRecordingState(data);
            } else {
                // Not recording, show standby
                this.isRecording = false;
                this._setStandbyUI();
            }
        } catch (e) {}
    },

    _setStandbyUI() {
        this.abortDetected = false;
        this.recordingDone = false;
        this.panel.classList.remove('aborted');
        this.timerEl.classList.remove('error-state');
        this.timerEl.textContent = '00:00';

        const dot = this.panel.querySelector('.rec-dot');
        dot.className = 'rec-dot standby';
        this.panel.querySelector('#rec-panel-title span:last-child').textContent = '就绪';

        document.getElementById('rec-panel-start-btn').style.display = '';
        document.getElementById('rec-panel-stop-btn').style.display = 'none';

        this._stopLocalTimer();
        this._lastElapsed = undefined;
        for (const key of Object.keys(this.SOURCE_CFG)) {
            this._setLight(key, 'idle');
            document.getElementById('rec-val-' + key).textContent = '--';
            document.getElementById('rec-item-' + key).classList.remove('has-error');
        }
    },

    _setRecordingUI() {
        const dot = this.panel.querySelector('.rec-dot');
        dot.className = 'rec-dot recording';
        this.panel.querySelector('#rec-panel-title span:last-child').textContent = '录制中';

        document.getElementById('rec-panel-start-btn').style.display = 'none';
        document.getElementById('rec-panel-stop-btn').style.display = '';
        this._startLocalTimer();
    },

    async _startRecording() {
        this._justStarted = true;
        try {
            const resp = await fetch(this.serverUrl + '/recording/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
            });
            const data = await resp.json();
            if (data.ok) {
                this.isRecording = true;
                this.recordingDone = false;
                this._setRecordingUI();
                // Sync DataRecord state
                if (DataRecord) {
                    DataRecord.recording = true;
                    DataRecord.startTime = Date.now();
                    DataRecord.startTimers();
                    DataRecord.updateUI();
                }
                // Start polling with 10s interval
                this._startPolling();
                // Clear justStarted guard after 3 seconds
                setTimeout(() => { this._justStarted = false; }, 3000);
            } else {
                this._showToast('启动录制失败: ' + (data.message || '未知错误'), 'error');
            }
        } catch (e) {
            this._showToast('无法连接服务器', 'error');
        }
    },

    async _stopRecording() {
        if (!this.isRecording) return;
        try {
            const resp = await fetch(this.serverUrl + '/recording/stop', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
            });
            const data = await resp.json();
            if (data.ok) {
                this.isRecording = false;
                this.recordingDone = true;
                this._stopLocalTimer();
                this._stopPolling();
                // Final poll to show last counts
                this._poll(true);
                // Update title and status light
                this.panel.querySelector('#rec-panel-title span:last-child').textContent = '录制完成';
                this.panel.querySelector('.rec-dot').className = 'rec-dot stopped';
                // Toggle buttons: show start, hide stop
                document.getElementById('rec-panel-start-btn').style.display = '';
                document.getElementById('rec-panel-stop-btn').style.display = 'none';
                // Sync DataRecord state
                if (DataRecord) {
                    DataRecord.recording = false;
                    DataRecord.stopTimers();
                    DataRecord.updateUI();
                    const dur = data.duration || 0;
                    const path = data.path || '';
                    const msg = data.message || '';
                    if (msg.includes('ABORTED') || msg.includes('abort')) {
                        DataRecord.showToast('⚠ 录制被异常中断！请检查相机连接\n时长 ' + dur + 's', 'error');
                    } else {
                        DataRecord.showToast('录制完成！时长 ' + dur + 's\n保存至: ' + path, 'success');
                    }
                }
            }
        } catch (e) {}
    },

    /** Called when recording stops externally (from DataRecord idle polling) */
    onRecordingStopped() {
        this.recordingDone = true;
        this.isRecording = false;
        this._stopLocalTimer();
        this._stopPolling();
        this._poll(true);
        this.panel.querySelector('#rec-panel-title span:last-child').textContent = '录制完成';
        this.panel.querySelector('.rec-dot').className = 'rec-dot stopped';
        document.getElementById('rec-panel-start-btn').style.display = '';
        document.getElementById('rec-panel-stop-btn').style.display = 'none';
    },

    updateTimer(elapsed) {
        const totalSec = Math.floor(elapsed);
        const hh = String(Math.floor(totalSec / 3600)).padStart(2, '0');
        const mm = String(Math.floor((totalSec % 3600) / 60)).padStart(2, '0');
        const ss = String(totalSec % 60).padStart(2, '0');
        this.timerEl.textContent = totalSec >= 3600 ? hh + ':' + mm + ':' + ss : mm + ':' + ss;
    },

    updateSources(sources) {
        if (!sources) return;

        let hasError = false;

        this._updateCameraSource('zed_color', sources['zed_left'], sources['zed_right']);
        this._updateSimpleSource('d405_1', sources['d405_1'], '未接入');
        this._updateSimpleSource('d405_2', sources['d405_2'], '未接入');

        this._updateSimpleSource('zed_depth', sources['zed_depth']);
        this._updateSimpleSource('pointcloud', sources['pointcloud']);
        this._updateSimpleSource('imu', sources['imu']);

        const rosKeys = ['ros_cmd', 'ros_lift', 'ros_joy', 'ros_chassis', 'ros_diag'];
        for (const key of rosKeys) {
            const src = sources[key];
            if (src) {
                const status = src.status || 'idle';
                this._setLight(key, status);
                const valEl = document.getElementById('rec-val-' + key);
                const itemEl = document.getElementById('rec-item-' + key);
                if (status === 'error') {
                    itemEl.classList.add('has-error');
                    hasError = true;
                } else {
                    itemEl.classList.remove('has-error');
                }
                if (status === 'ok' && src.count > 0) {
                    valEl.innerHTML = '<span>' + src.count + '</span> 条';
                } else if (status === 'idle') {
                    valEl.textContent = '无数据';
                } else {
                    valEl.textContent = '已断开';
                }
            }
        }

        // Distance sensor
        this._updateSimpleSource('distance', sources['distance']);

        if (hasError && !this.abortDetected) {
            const dot = this.panel.querySelector('.rec-dot');
            if (this.isRecording) dot.className = 'rec-dot error';
        }
    },

    _updateCameraSource(key, leftSrc, rightSrc) {
        const lightEl = document.getElementById('rec-light-' + key);
        const valEl = document.getElementById('rec-val-' + key);
        const itemEl = document.getElementById('rec-item-' + key);
        if (!lightEl) return;

        if (!leftSrc && !rightSrc) {
            this._setLight(key, 'idle');
            valEl.textContent = '--';
            itemEl.classList.remove('has-error');
            return;
        }

        const leftCount = (leftSrc && leftSrc.count) || 0;
        const rightCount = (rightSrc && rightSrc.count) || 0;
        const total = leftCount + rightCount;
        const status = leftSrc ? (leftSrc.status || 'idle') : 'idle';

        this._setLight(key, status);
        if (status === 'error') {
            itemEl.classList.add('has-error');
        } else {
            itemEl.classList.remove('has-error');
        }

        if (leftSrc && leftSrc.note) {
            valEl.textContent = leftSrc.note;
        } else if (total > 0) {
            valEl.innerHTML = '<span>' + total + '</span> 帧';
        } else {
            valEl.textContent = '等待中';
        }
    },

    _updateSimpleSource(key, src, idleText) {
        const lightEl = document.getElementById('rec-light-' + key);
        const valEl = document.getElementById('rec-val-' + key);
        const itemEl = document.getElementById('rec-item-' + key);
        const cfg = this.SOURCE_CFG[key];
        if (!lightEl) return;

        if (!src) {
            this._setLight(key, 'idle');
            valEl.textContent = idleText || '未接入';
            itemEl.classList.remove('has-error');
            return;
        }

        const status = src.status || 'idle';
        this._setLight(key, status);

        if (status === 'error') {
            itemEl.classList.add('has-error');
        } else {
            itemEl.classList.remove('has-error');
        }

        if (src.note) {
            valEl.textContent = src.note;
        } else if (cfg.unit && src.count !== undefined) {
            const count = src.count || 0;
            valEl.innerHTML = '<span>' + count + '</span> ' + cfg.unit;
        } else {
            valEl.textContent = status === 'ok' ? '正常' : '等待中';
        }
    },

    showAborted() {
        if (this.abortDetected) return;
        this.abortDetected = true;
        this.recordingDone = true;
        this.isRecording = false;
        this._stopLocalTimer();
        this.panel.classList.add('aborted');
        this.timerEl.classList.add('error-state');
        const dot = this.panel.querySelector('.rec-dot');
        dot.className = 'rec-dot error';
        this.panel.querySelector('#rec-panel-title span:last-child').textContent = '⚠ 录制异常中断';
        document.getElementById('rec-panel-start-btn').style.display = '';
        document.getElementById('rec-panel-stop-btn').style.display = 'none';
        for (const key of Object.keys(this.SOURCE_CFG)) {
            const valEl = document.getElementById('rec-val-' + key);
            if (valEl && valEl.textContent !== '未接入' && valEl.textContent !== '--') {
                this._setLight(key, 'error');
            }
        }
    },

    _showToast(msg, type) {
        if (DataRecord && DataRecord.showToast) {
            DataRecord.showToast(msg, type);
        }
    },

    _setLight(key, status) {
        const el = document.getElementById('rec-light-' + key);
        if (el) el.className = 'rec-light ' + (status || 'idle');
    },


    _startLocalTimer() {
        this._stopLocalTimer();
        this._localTimer = setInterval(() => {
            // Fetch elapsed from server more frequently (every 2s) for smooth timer
            // But also do a lightweight local increment
            if (this.isRecording && this._lastElapsed !== undefined) {
                this._lastElapsed += 1;
                this.updateTimer(this._lastElapsed);
            }
        }, 1000);
    },

    _stopLocalTimer() {
        if (this._localTimer) {
            clearInterval(this._localTimer);
            this._localTimer = null;
        }
    },

    _startPolling() {
        this._stopPolling();
        this._poll();
        this.pollTimer = setInterval(() => this._poll(), 10000);
    },

    _stopPolling() {
        if (this.pollTimer) {
            clearInterval(this.pollTimer);
            this.pollTimer = null;
        }
    },

    async _poll(isFinal) {
        try {
            const resp = await fetch(this.serverUrl + '/recording/status');
            if (!resp.ok) return;
            const data = await resp.json();

            if (!data.running) {
                if (this.abortDetected) return;
                const msg = data.message || '';
                if (msg.includes('ABORTED') || msg.includes('abort')) {
                    this.showAborted();
                } else if (this.isRecording && !this._justStarted) {
                    // Stopped externally while we thought we were recording
                    // (skip if we just started to avoid race condition)
                    this.isRecording = false;
                    this.recordingDone = true;
                    if (data.sources) this.updateSources(data.sources);
                    if (data.elapsed !== undefined) this.updateTimer(data.elapsed);
                    this.panel.querySelector('#rec-panel-title span:last-child').textContent = '录制完成';
                    this.panel.querySelector('.rec-dot').className = 'rec-dot stopped';
                    document.getElementById('rec-panel-start-btn').style.display = '';
                    document.getElementById('rec-panel-stop-btn').style.display = 'none';
                    if (DataRecord && DataRecord.recording) {
                        DataRecord.recording = false;
                        DataRecord.stopTimers();
                        DataRecord.updateUI();
                    }
                }
                if (!isFinal) this._stopPolling();
                return;
            }

            // Recording is running — always sync from backend. Do not let stale
            // recordingDone/abort flags from a previous session block the panel.
            this.syncRecordingState(data);
            if (DataRecord && !DataRecord.recording) {
                DataRecord.recording = true;
                DataRecord.startTime = Date.now() - data.elapsed * 1000;
                DataRecord.startTimers();
                DataRecord.updateUI();
            }

            if (data.elapsed !== undefined) {
                this._lastElapsed = data.elapsed;
                this.updateTimer(data.elapsed);
            }
            if (data.sources) this.updateSources(data.sources);
        } catch (e) {
            if (!this.recordingDone) {
                for (const key of Object.keys(this.SOURCE_CFG)) {
                    this._setLight(key, 'error');
                }
            }
        }
    },
};

// Patched imu-status.js - adds IMU dot + offline text
const IMUStatus = {
    topic: null,
    magTopic: null,
    tempTopic: null,
    count: 0,
    lastTime: 0,
    freqEl: null,
    pollTimer: null,
    useHttp: false,
    ws: null,
    wsReconnectTimer: null,
    wsConnected: false,
    _dotSet: false,

    init(ros) {
        this._dotSet = false;
        this.freqEl = document.getElementById('imu-freq');
        this.connectWebSocket();
    },

    setDot(online) {
        const dot = document.getElementById('dot-imu');
        if (dot) dot.className = 'section-dot ' + (online ? 'online' : 'offline-dot');
    },

    connectWebSocket() {
        if (this.ws && (this.ws.readyState === WebSocket.CONNECTING || this.ws.readyState === WebSocket.OPEN)) {
            return;
        }

        const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = protocol + '//' + location.host + '/ws/imu';

        try {
            this.ws = new WebSocket(wsUrl);

            this.ws.onopen = () => {
                this.wsConnected = true;
                this.stopHttpPoll();
                if (this.wsReconnectTimer) {
                    clearTimeout(this.wsReconnectTimer);
                    this.wsReconnectTimer = null;
                }
            };

            this.ws.onmessage = (event) => {
                try {
                    const json = JSON.parse(event.data);
                    if (json.ok && json.data) {
                        this.updateFromHttp(json.data);
                    }
                } catch (e) {}
            };

            this.ws.onclose = () => {
                this.wsConnected = false;
                this.ws = null;
                this.startHttpPoll();
                this.scheduleWsReconnect();
            };

            this.ws.onerror = () => {};
        } catch (e) {
            this.startHttpPoll();
        }
    },

    scheduleWsReconnect() {
        if (this.wsReconnectTimer) return;
        this.wsReconnectTimer = setTimeout(() => {
            this.wsReconnectTimer = null;
            this.connectWebSocket();
        }, 3000);
    },

    startHttpPoll() {
        if (this.pollTimer) return;
        this.useHttp = true;
        this._poll();
        this.pollTimer = setInterval(() => this._poll(), 100);
    },

    stopHttpPoll() {
        this.useHttp = false;
        if (this.pollTimer) {
            clearInterval(this.pollTimer);
            this.pollTimer = null;
        }
    },

    async _poll() {
        try {
            const resp = await fetch('/api/imu');
            const json = await resp.json();
            if (json.ok && json.data) {
                this.updateFromHttp(json.data);
            }
        } catch (e) {}
    },

    updateFromHttp(data) {
        if (!this._dotSet) { this.setDot(true); this._dotSet = true; }

        const a = data.accel;
        this.setVal('imu-acc-x', a[0], 3);
        this.setVal('imu-acc-y', a[1], 3);
        this.setVal('imu-acc-z', a[2], 3);

        const g = data.gyro_dps;
        this.setVal('imu-gyro-x', g[0], 2);
        this.setVal('imu-gyro-y', g[1], 2);
        this.setVal('imu-gyro-z', g[2], 2);

        const m = data.mag;
        this.setVal('imu-mag-x', m[0], 2);
        this.setVal('imu-mag-y', m[1], 2);
        this.setVal('imu-mag-z', m[2], 2);

        const temp = document.getElementById('imu-temp');
        if (temp) temp.textContent = data.imu_temp.toFixed(1) + ' °C';

        this.count++;
        const now = performance.now();
        if (this.lastTime > 0 && now - this.lastTime >= 1000) {
            const hz = (this.count * 1000 / (now - this.lastTime)).toFixed(0);
            if (this.freqEl) this.freqEl.textContent = hz + ' Hz';
            this.count = 0;
            this.lastTime = now;
        } else if (this.lastTime === 0) {
            this.lastTime = now;
        }
    },

    disable() {
        this.setDot(false);
        this._dotSet = false;
        this.stopHttpPoll();
        if (this.ws) {
            this.ws.onclose = null;
            this.ws.close();
            this.ws = null;
        }
        if (this.wsReconnectTimer) {
            clearTimeout(this.wsReconnectTimer);
            this.wsReconnectTimer = null;
        }
        const ids = ['imu-acc-x','imu-acc-y','imu-acc-z',
                      'imu-gyro-x','imu-gyro-y','imu-gyro-z',
                      'imu-mag-x','imu-mag-y','imu-mag-z'];
        for (const id of ids) {
            const el = document.getElementById(id);
            if (el) el.textContent = '--';
        }
        const temp = document.getElementById('imu-temp');
        if (temp) temp.textContent = '-- °C';
        const freq = document.getElementById('imu-freq');
        if (freq) freq.textContent = '-- Hz';
    },

    setVal(id, value, decimals) {
        const el = document.getElementById(id);
        if (el) {
            const v = typeof value === 'number' ? value : 0;
            el.textContent = (v >= 0 ? '+' : '') + v.toFixed(decimals);
        }
    },
};

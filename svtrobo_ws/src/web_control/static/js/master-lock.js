/**
 * SVTROBO Master Lock
 * Only one browser tab can control the robot. Additional tabs are read-only.
 *
 * Flow:
 *   - On connect, request master with steal=true (grabs lock from any existing tab)
 *   - All tabs heartbeat every 5s via POST /api/master/request
 *   - Master: heartbeat returns master=true, keeps control
 *   - If heartbeat returns master=false (someone stole), switch to read-only
 *   - Non-master tabs also heartbeat; can detect when lock is free
 *   - Cross-tab notification via BroadcastChannel when lock changes
 */

const MasterLock = {
    sessionId: null,
    isMaster: false,
    heartbeatTimer: null,
    serverUrl: '',
    _onMasterChange: null,
    _bc: null,  // BroadcastChannel

    init(serverUrl, onChange) {
        this.serverUrl = serverUrl;
        this._onMasterChange = onChange;
        this.isMaster = false;
        this._stopHeartbeat();

        if (!sessionStorage.getItem('master_session_id')) {
            // crypto.randomUUID() only in secure contexts (HTTPS/localhost)
            // Use UUID v4 fallback for plain HTTP
            var _uuid = (typeof crypto.randomUUID === 'function')
                ? crypto.randomUUID()
                : 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
                    var r = Math.random() * 16 | 0;
                    return (c === 'x' ? r : (r & 0x3 | 0x8)).toString(16);
                  });
            sessionStorage.setItem('master_session_id', _uuid);
        }
        this.sessionId = sessionStorage.getItem('master_session_id');

        // BroadcastChannel for instant cross-tab notification
        try {
            this._bc = new BroadcastChannel('svtrobot_master_lock');
            this._bc.onmessage = (event) => {
                const msg = event.data;
                if (msg.type === 'lock_stolen' && msg.session_id !== this.sessionId) {
                    // Another tab just stole the lock
                    if (this.isMaster) {
                        this.isMaster = false;
                        if (this._onMasterChange) {
                            this._onMasterChange(false, msg.holder || '?');
                        }
                    }
                }
            };
        } catch (e) {
            // BroadcastChannel not supported (rare), rely on heartbeat polling
        }
    },

    async request() {
        try {
            const resp = await fetch(this.serverUrl + '/api/master/request', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    session_id: this.sessionId,
                    steal: true,
                }),
            });
            const data = await resp.json();
            if (data.session_id) {
                this.sessionId = data.session_id;
                sessionStorage.setItem('master_session_id', data.session_id);
            }
            const wasMaster = this.isMaster;
            this.isMaster = data.master === true;

            // Notify other tabs that we stole the lock
            if (this.isMaster && this._bc) {
                this._bc.postMessage({ type: 'lock_stolen', session_id: this.sessionId, holder: '新标签页' });
            }

            // Always trigger callback on first request or when status changes
            if (this._onMasterChange) {
                if (!wasMaster || wasMaster !== this.isMaster) {
                    this._onMasterChange(this.isMaster, data.holder);
                }
            }

            // Start heartbeat for all tabs (master and non-master)
            this._startHeartbeat();
            return data;
        } catch (e) {
            console.error('[MasterLock] request failed:', e);
            return { master: false };
        }
    },

    async release() {
        this._stopHeartbeat();
        try {
            await fetch(this.serverUrl + '/api/master/release', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ session_id: this.sessionId }),
            });
        } catch (e) {}
        const wasMaster = this.isMaster;
        this.isMaster = false;
        if (wasMaster && this._onMasterChange) {
            this._onMasterChange(false);
        }
    },

    _startHeartbeat() {
        this._stopHeartbeat();
        this.heartbeatTimer = setInterval(() => this._heartbeat(), 5000);
    },

    async _heartbeat() {
        try {
            const resp = await fetch(this.serverUrl + '/api/master/request', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    session_id: this.sessionId,
                    steal: false,
                }),
            });
            const data = await resp.json();
            const wasMaster = this.isMaster;
            this.isMaster = data.master === true;
            if (wasMaster !== this.isMaster && this._onMasterChange) {
                this._onMasterChange(this.isMaster, data.holder);
            }
        } catch (e) {}
    },

    _stopHeartbeat() {
        if (this.heartbeatTimer) {
            clearInterval(this.heartbeatTimer);
            this.heartbeatTimer = null;
        }
    },
};

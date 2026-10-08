/**
 * SVTROBO Web Control - Camera Feed Module
 */

const Camera = {
    serverUrl: '',
    enabled: false,
    cameras: ['d405_1', 'd405_2', 'zed'],

    init(serverUrl) {
        this.serverUrl = serverUrl;
        this.setupButtons();
        this.setEnabled(true);
        this.refreshStatus();
        // Auto-refresh status every 5s
        setInterval(() => this.refreshStatus(), 5000);

        // Fullscreen buttons
        document.querySelectorAll('.cam-fullscreen-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const camName = btn.dataset.cam;
                const container = document.getElementById('cam-' + camName + '-feed').parentElement;
                if (document.fullscreenElement) {
                    document.exitFullscreen();
                } else {
                    container.requestFullscreen().catch(() => {});
                }
            });
        });

        // Stop all cameras when page is refreshed or closed
        window.addEventListener('beforeunload', () => {
            for (const name of this.cameras) {
                fetch(`${this.serverUrl}/camera/stop`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ camera: name }),
                    keepalive: true,
                }).catch(() => {});
            }
        });
    },

    setEnabled(enabled) {
        this.enabled = enabled;
        if (!enabled) {
            this.stopAll();
        }
        for (const name of this.cameras) {
            const safeId = name.replace('_', '-');
            const startBtn = document.getElementById(`cam-${safeId}-start`);
            const stopBtn = document.getElementById(`cam-${safeId}-stop`);
            if (startBtn) startBtn.disabled = !enabled;
            if (stopBtn) stopBtn.disabled = !enabled;
        }
    },

    async stopAll() {
        for (const name of this.cameras) {
            const safeId = name.replace('_', '-');
            const img = document.getElementById(`cam-${safeId}-feed`);
            const fallback = document.getElementById(`cam-${safeId}-fallback`);
            const fsBtn = document.querySelector(`.cam-fullscreen-btn[data-cam="${safeId}"]`);
            if (img) img.src = '';
            if (fallback) {
                fallback.textContent = '相机未启动';
                fallback.style.display = 'block';
            }
            if (fsBtn) fsBtn.style.display = 'none';
            try {
                await fetch(`${this.serverUrl}/camera/stop`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ camera: name }),
                });
            } catch (e) {}
        }
        this.refreshStatus();
    },

    setupButtons() {
        for (const name of this.cameras) {
            const safeId = name.replace('_', '-');
            const startBtn = document.getElementById(`cam-${safeId}-start`);
            const stopBtn = document.getElementById(`cam-${safeId}-stop`);
            if (startBtn) startBtn.addEventListener('click', () => this.startCamera(name));
            if (stopBtn) stopBtn.addEventListener('click', () => this.stopCamera(name));
        }
    },

    async startCamera(name) {
        if (!this.enabled) return;
        const safeId = name.replace('_', '-');
        const fallback = document.getElementById(`cam-${safeId}-fallback`);
        const displayName = {d405_1: 'D405 (1)', d405_2: 'D405 (2)', zed: 'ZED 2i'}[name] || name;

        if (fallback) {
            fallback.textContent = '正在启动...';
            fallback.style.display = 'block';
        }

        try {
            const resp = await fetch(`${this.serverUrl}/camera/start`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ camera: name }),
            });
            const data = await resp.json();
            const img = document.getElementById(`cam-${safeId}-feed`);
            const fsBtn = document.querySelector(`.cam-fullscreen-btn[data-cam="${safeId}"]`);

            if (data.ok) {
                if (img) img.src = `${this.serverUrl}/camera/${name}`;
                if (fallback) fallback.style.display = 'none';
                if (fsBtn) fsBtn.style.display = 'block';
            } else {
                if (img) img.src = '';
                if (fsBtn) fsBtn.style.display = 'none';
                if (fallback) {
                    const msg = data.message || '';
                    if (msg.includes('未找到') || msg.includes('not found') || msg.includes('No such')) {
                        fallback.textContent = `${displayName} 设备未连接`;
                    } else {
                        fallback.textContent = '启动失败: ' + msg;
                    }
                    fallback.style.display = 'block';
                }
            }
            this.refreshStatus();
        } catch (e) {
            console.error('Start camera error:', e);
            if (fallback) {
                fallback.textContent = `${displayName} 请求失败，请检查服务`;
                fallback.style.display = 'block';
            }
        }
    },

    async stopCamera(name) {
        try {
            const safeId = name.replace('_', '-');
            const img = document.getElementById(`cam-${safeId}-feed`);
            const fallback = document.getElementById(`cam-${safeId}-fallback`);
            const fsBtn = document.querySelector(`.cam-fullscreen-btn[data-cam="${safeId}"]`);
            if (img) img.src = '';
            if (fallback) {
                fallback.textContent = '相机已关闭';
                fallback.style.display = 'block';
            }
            if (fsBtn) fsBtn.style.display = 'none';

            await fetch(`${this.serverUrl}/camera/stop`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ camera: name }),
            });
            this.refreshStatus();
        } catch (e) {
            console.error('Stop camera error:', e);
        }
    },

    async refreshStatus() {
        try {
            const resp = await fetch(`${this.serverUrl}/camera/status`);
            const status = await resp.json();
            for (const name of this.cameras) {
                const safeId = name.replace('_', '-');
                const indicator = document.getElementById(`cam-${safeId}-status`);
                const fallback = document.getElementById(`cam-${safeId}-fallback`);
                const img = document.getElementById(`cam-${safeId}-feed`);
                const fsBtn = document.querySelector(`.cam-fullscreen-btn[data-cam="${safeId}"]`);

                if (indicator) {
                    const running = status[name]?.running;
                    indicator.textContent = running ? '运行中' : '已停止';
                    indicator.className = 'cam-status ' + (running ? 'cam-running' : 'cam-stopped');

                    if (!running && fallback && img) {
                        img.src = '';
                        if (fsBtn) fsBtn.style.display = 'none';
                        // Preserve error messages from startCamera, only reset default text
                        const isDefault = !fallback.textContent ||
                            fallback.textContent === '相机未启动' ||
                            fallback.textContent === '正在启动...';
                        if (isDefault) {
                            fallback.textContent = '相机未启动';
                        }
                        fallback.style.display = 'block';
                    } else if (running && fallback) {
                        fallback.style.display = 'none';
                        if (fsBtn) fsBtn.style.display = 'block';
                    }
                }
            }
        } catch (e) {
            // Server might not be reachable
        }
    },
};

/**
 * SVTROBO Web Control - Chassis Control Module
 * WASD/Q/E keyboard control, direction indicator, speed slider.
 */

const Chassis = {
    cmdTopic: null,
    feedbackTopic: null,
    speed: 0.05,
    rotSpeed: 0.5,
    activeKeys: new Set(),
    publishInterval: null,
    currentVx: 0,
    currentVy: 0,
    currentWz: 0,
    _disabled: false,

    keyMap: {
        'KeyW': { vx:  1, vy:  0, wz:  0 },
        'KeyS': { vx: -1, vy:  0, wz:  0 },
        'KeyA': { vx:  0, vy:  1, wz:  0 },
        'KeyD': { vx:  0, vy: -1, wz:  0 },
        'KeyQ': { vx:  0, vy:  0, wz:  1 },
        'KeyE': { vx:  0, vy:  0, wz: -1 },
    },

    init(ros) {
        this.cmdTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/svtrobot_cmd',
            messageType: 'geometry_msgs/Twist',
        });

        this.feedbackTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/chassis/cmd_feedback',
            messageType: 'geometry_msgs/Twist',
        });
        this.feedbackTopic.subscribe((msg) => {
            this.setDot(true);
            this.currentVx = msg.linear.x;
            this.currentVy = msg.linear.y;
            this.currentWz = msg.angular.z;
            this.updateFeedbackDisplay();
        });

        this.setupKeyboard();
        this.setupSpeedSlider();
        this.updateFeedbackDisplay();
    },

    setDot(online) {
        const dot = document.getElementById('dot-chassis-ctrl');
        if (dot) dot.className = 'section-dot ' + (online ? 'online' : 'offline-dot');
    },

    setupKeyboard() {
        const panel = document.getElementById('chassis-panel');

        panel.addEventListener('click', () => { panel.focus(); });
        panel.addEventListener('focus', () => {
            document.getElementById('keyboard-hint').style.display = 'none';
        });
        panel.addEventListener('blur', () => {
            document.getElementById('keyboard-hint').style.display = 'flex';
            this.stopAll();
        });

        panel.addEventListener('keydown', (e) => {
            if (e.code === 'Space') { e.preventDefault(); this.stopAll(); return; }
            if (this.keyMap[e.code]) {
                e.preventDefault();
                if (!this.activeKeys.has(e.code)) {
                    this.activeKeys.add(e.code);
                    this.updateDirection();
                    this.startPublishing();
                }
            }
        });

        panel.addEventListener('keyup', (e) => {
            if (this.keyMap[e.code]) {
                e.preventDefault();
                this.activeKeys.delete(e.code);
                this.updateDirection();
                if (this.activeKeys.size === 0) this.stopAll();
            }
        });

        panel.setAttribute('tabindex', '0');
    },

    setupSpeedSlider() {
        const slider = document.getElementById('speed-slider');
        const display = document.getElementById('speed-value');
        slider.addEventListener('input', () => {
            this.speed = parseFloat(slider.value);
            display.textContent = this.speed.toFixed(2);
        });
    },

    computeVelocity() {
        let vx = 0, vy = 0, wz = 0;
        for (const key of this.activeKeys) {
            const dir = this.keyMap[key];
            if (dir) { vx += dir.vx; vy += dir.vy; wz += dir.wz; }
        }
        const mag = Math.sqrt(vx * vx + vy * vy);
        if (mag > 1) { vx /= mag; vy /= mag; }
        return { vx: vx * this.speed, vy: vy * this.speed, wz: wz * this.rotSpeed };
    },

    publish() {
        if (this._disabled || !this.cmdTopic || this.activeKeys.size === 0) return;
        const vel = this.computeVelocity();
        const msg = new ROSLIB.Message({
            linear: { x: vel.vx, y: vel.vy, z: 0 },
            angular: { x: 0, y: 0, z: vel.wz },
        });
        this.cmdTopic.publish(msg);
    },

    startPublishing() {
        if (this.publishInterval) return;
        this.publish();
        this.publishInterval = setInterval(() => this.publish(), 50);
    },

    stopAll() {
        if (this.publishInterval) {
            clearInterval(this.publishInterval);
            this.publishInterval = null;
        }
        this.activeKeys.clear();
        this.updateDirection();
        if (this.cmdTopic) {
            this.cmdTopic.publish(new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 },
            }));
        }
    },

    updateDirection() {
        document.querySelectorAll('.dir-segment, .rot-indicator').forEach(el => el.classList.remove('active'));
        if (this.activeKeys.size === 0) return;
        let vx = 0, vy = 0, wz = 0;
        for (const key of this.activeKeys) {
            const dir = this.keyMap[key];
            if (dir) { vx += dir.vx; vy += dir.vy; wz += dir.wz; }
        }
        if (wz > 0) this.activateDir('ccw');
        if (wz < 0) this.activateDir('cw');
        if (Math.abs(vx) > 0.01 || Math.abs(vy) > 0.01) {
            const angle = Math.atan2(vy, vx);
            const idx = Math.round(angle / (Math.PI / 4));
            const dirMap = { 0: 'n', 1: 'nw', 2: 'w', 3: 'sw', 4: 's', '-4': 's', '-3': 'se', '-2': 'e', '-1': 'ne' };
            const dir = dirMap[String(idx)] || 'n';
            this.activateDir(dir);
        }
    },

    activateDir(name) {
        const el = document.getElementById('dir-' + name);
        if (el) el.classList.add('active');
    },

    disable() {
        this.setDot(false);
        this._disabled = true;
        this.stopAll();
    },

    enable() {
        this._disabled = false;
    },

    updateFeedbackDisplay() {
        const el = document.getElementById('feedback-display');
        if (el) {
            el.textContent = `vx=${this.currentVx.toFixed(2)} m/s, vy=${this.currentVy.toFixed(2)} m/s, angularZ=${this.currentWz.toFixed(2)} rad/s`;
        }
    },
};

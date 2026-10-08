// Patched diagnostics.js - adds battery dot + offline text
const Diagnostics = {
    diagTopic: null,

    VBUS_MIN: 19.8,
    VBUS_MAX: 25.2,

    init(ros) {
        this.diagTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/chassis/diagnostics',
            messageType: 'chassis_control/msg/ChassisDiagnostics',
        });

        this.diagTopic.subscribe((msg) => {
            this.setDot(true);
            this.updateBatteryDisplay(msg.vbus);
            let errorCodes = msg.motor_error_codes;
            if (typeof errorCodes === 'string') {
                const raw = atob(errorCodes);
                errorCodes = [];
                for (let i = 0; i < raw.length; i++) {
                    errorCodes.push(raw.charCodeAt(i));
                }
            }
            this.updateMotorDisplay(msg.motor_temperatures, errorCodes);
        });
    },

    setDot(online) {
        const dot = document.getElementById('dot-battery');
        if (dot) dot.className = 'section-dot ' + (online ? 'online' : 'offline-dot');
    },

    updateBatteryDisplay(vbus) {
        const voltageEl = document.getElementById('battery-voltage');
        const percentEl = document.getElementById('battery-percent');

        if (vbus < 1.0) return;

        if (voltageEl) {
            voltageEl.textContent = vbus.toFixed(1) + ' V';
        }

        const percent = Math.max(0, Math.min(100,
            ((vbus - this.VBUS_MIN) / (this.VBUS_MAX - this.VBUS_MIN)) * 100
        ));
        if (percentEl) {
            percentEl.textContent = Math.round(percent) + ' %';
            if (percent > 50) {
                percentEl.style.color = '#10b981';
            } else if (percent > 20) {
                percentEl.style.color = '#f59e0b';
            } else {
                percentEl.style.color = '#ef4444';
            }
        }
    },

    disable() {
        this.setDot(false);
        const voltageEl = document.getElementById('battery-voltage');
        const percentEl = document.getElementById('battery-percent');
        const tempEl = document.getElementById('battery-temp');
        if (voltageEl) voltageEl.textContent = '-- V';
        if (percentEl) { percentEl.textContent = '-- %'; percentEl.style.color = ''; }
        if (tempEl) tempEl.textContent = '-- °C';

        const positions = ['fl', 'fr', 'rl', 'rr'];
        for (const pos of positions) {
            const tEl = document.getElementById('motor-' + pos + '-temp');
            const eEl = document.getElementById('motor-' + pos + '-err');
            if (tEl) { tEl.textContent = '-- °C'; tEl.style.color = ''; }
            if (eEl) { eEl.textContent = '--'; eEl.className = 'motor-diag-error'; }
        }
    },

    updateMotorDisplay(temperatures, errorCodes) {
        const positions = ['fl', 'fr', 'rl', 'rr'];
        let maxTemp = 0;

        for (let i = 0; i < 4; i++) {
            const tempEl = document.getElementById('motor-' + positions[i] + '-temp');
            const errEl = document.getElementById('motor-' + positions[i] + '-err');

            if (tempEl) {
                tempEl.textContent = temperatures[i].toFixed(1) + ' °C';
                if (temperatures[i] > 70) {
                    tempEl.style.color = '#ef4444';
                } else if (temperatures[i] > 50) {
                    tempEl.style.color = '#f59e0b';
                } else {
                    tempEl.style.color = '';
                }
            }

            if (errEl) {
                if (errorCodes[i] === 0) {
                    errEl.textContent = 'OK';
                    errEl.className = 'motor-diag-error ok';
                } else {
                    errEl.textContent = 'ERR:0x' + errorCodes[i].toString(16).toUpperCase();
                    errEl.className = 'motor-diag-error';
                }
            }

            if (temperatures[i] > maxTemp) {
                maxTemp = temperatures[i];
            }
        }

        if (maxTemp > 0) {
            const tempEl = document.getElementById('battery-temp');
            if (tempEl) {
                tempEl.textContent = maxTemp.toFixed(1) + ' °C';
            }
        }
    },
};

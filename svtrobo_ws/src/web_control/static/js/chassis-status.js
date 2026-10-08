/**
 * SVTROBO Web Control - Chassis Status Module
 * Monitors chassis joint_states & diagnostics.
 */

const ChassisStatus = {
    jointTopic: null,
    diagTopic: null,

    init(ros) {
        this.jointTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/chassis/joint_states',
            messageType: 'sensor_msgs/msg/JointState',
        });
        this.jointTopic.subscribe((msg) => {
            this.setDot(true);
            this.updateSteerDisplay(msg);
        });

        this.diagTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/chassis/diagnostics',
            messageType: 'chassis_control/msg/ChassisDiagnostics',
        });
        this.diagTopic.subscribe((msg) => {
            this.updateWheelActual(msg.wheel_speeds_actual);
        });
    },

    setDot(online) {
        const dot = document.getElementById('dot-motors');
        if (dot) dot.className = 'section-dot ' + (online ? 'online' : 'offline-dot');
    },

    disable() {
        this.setDot(false);
        const positions = ['fl', 'fr', 'rl', 'rr'];
        for (const pos of positions) {
            const angleEl = document.getElementById('steer-' + pos + '-angle');
            const velEl = document.getElementById('steer-' + pos + '-vel');
            const torqueEl = document.getElementById('steer-' + pos + '-torque');
            const targetEl = document.getElementById('wheel-' + pos + '-target');
            const actualEl = document.getElementById('wheel-' + pos + '-actual');
            if (angleEl) angleEl.textContent = '--';
            if (velEl) velEl.textContent = '--';
            if (torqueEl) torqueEl.textContent = '--';
            if (targetEl) targetEl.textContent = '--';
            if (actualEl) { actualEl.textContent = '--'; actualEl.style.color = ''; }
        }
    },

    updateSteerDisplay(msg) {
        const positions = ['fl', 'fr', 'rl', 'rr'];
        if (!msg.position || !msg.velocity || !msg.effort) return;

        for (let i = 0; i < 4; i++) {
            const angleDeg = (msg.position[i] * 180.0 / Math.PI).toFixed(1);
            const angleEl = document.getElementById('steer-' + positions[i] + '-angle');
            if (angleEl) angleEl.textContent = angleDeg + '\u00B0';

            const velEl = document.getElementById('steer-' + positions[i] + '-vel');
            if (velEl) velEl.textContent = msg.velocity[i].toFixed(2) + ' rad/s';

            const torqueEl = document.getElementById('steer-' + positions[i] + '-torque');
            if (torqueEl) torqueEl.textContent = msg.effort[i].toFixed(2) + ' Nm';

            const wheelTargetEl = document.getElementById('wheel-' + positions[i] + '-target');
            if (wheelTargetEl) wheelTargetEl.textContent = Math.abs(msg.velocity[i + 4]).toFixed(0) + ' RPM';
        }
    },

    updateWheelActual(speeds) {
        if (!speeds || speeds.length < 4) return;
        const positions = ['fl', 'fr', 'rl', 'rr'];
        for (let i = 0; i < 4; i++) {
            const el = document.getElementById('wheel-' + positions[i] + '-actual');
            if (el) {
                const rpm = Math.abs(speeds[i]).toFixed(1);
                el.textContent = rpm + ' RPM';
                el.style.color = Math.abs(speeds[i]) > 1.0 ? '#3b82f6' : '';
            }
        }
    },
};

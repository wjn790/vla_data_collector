/**
 * SVTROBO Web Control - Lift Control Module
 */

const Lift = {
    liftTopic: null,
    liftStateTopic: null,
    speed: 300,
    _disabled: false,

    init(ros) {
        this.liftTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/lift_control_cmd',
            messageType: 'std_msgs/msg/Int32MultiArray',
        });

        this.setupButtons();
        this.setupSpeedSlider();
    },

    setDot(online) {
        const dot = document.getElementById('dot-lift-ctrl');
        if (dot) dot.className = 'section-dot ' + (online ? 'online' : 'offline-dot');
    },

    setupButtons() {
        document.getElementById('lift-up').addEventListener('click', () => this.publish(1));
        document.getElementById('lift-down').addEventListener('click', () => this.publish(-1));
        document.getElementById('lift-stop').addEventListener('click', () => this.publish(0));
    },

    setupSpeedSlider() {
        const slider = document.getElementById('lift-speed-slider');
        const display = document.getElementById('lift-speed-value');
        slider.addEventListener('input', () => {
            this.speed = parseInt(slider.value);
            display.textContent = this.speed;
        });
    },

    publish(direction) {
        if (this._disabled || !this.liftTopic) return;
        const msg = new ROSLIB.Message({
            data: [direction, this.speed],
        });
        this.liftTopic.publish(msg);
    },

    disable() {
        this.setDot(false);
        this._disabled = true;
        if (this.liftTopic) {
            this.liftTopic.publish(new ROSLIB.Message({ data: [0, 0] }));
        }
    },

    enable() {
        this._disabled = false;
    },
};

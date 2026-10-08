/**
 * SVTROBO Web Control - Hardware Status Bar
 * Monitors hardware via ROS nodes (when connected) and HTTP endpoints (always).
 * Shows all devices even before ROS connection; grays out on disconnect.
 */

const HardwareStatus = {
    ros: null,
    _timers: [],

    devices: {
        chassis:  { label: '\u5E95\u76D8', icon: '\u2699\uFE0F', status: 'offline' },
        lift:     { label: '\u5347\u964D', icon: '\u2B06\uFE0F', status: 'offline' },
        f710:     { label: '\u624B\u67C4', icon: '\uD83C\uDFAE', status: 'offline' },
        d405_1:   { label: 'D405 #1', icon: '\uD83D\uDCF7', status: 'offline' },
        d405_2:   { label: 'D405 #2', icon: '\uD83D\uDCF7', status: 'offline' },
        zed:      { label: 'ZED 2i', icon: '\uD83C\uDF9E', status: 'offline' },
        imu:      { label: 'IMU', icon: '\uD83E\uDDED', status: 'offline' },
        laser:    { label: '\u6D4B\u8DDD', icon: '\uD83D\uDCA0', status: 'offline' },
        arm_left: { label: '\u5DE6\u81C2', icon: '\uD83E\uDDBE', status: 'offline' },
        arm_right:{ label: '\u53F3\u81C2', icon: '\uD83E\uDDBE', status: 'offline' },
        hand_left:{ label: '\u5DE6\u624B', icon: '\uD83E\uDD1A', status: 'offline' },
        hand_right:{label: '\u53F3\u624B', icon: '\uD83E\uDD1A', status: 'offline' },
    },

    init(ros) {
        this.ros = ros;
        this._clearTimers();
        this._startNodeMonitor();
        this._startCameraPoll();
        this._startImuPoll();
        this._startDistancePoll();
        this.render();
    },

    _clearTimers() {
        this._timers.forEach(function(t) { clearInterval(t); });
        this._timers = [];
    },

    _startNodeMonitor() {
        var self = this;
        var check = function() {
            if (!self.ros || !self.ros.isConnected) return;
            try {
                var svc = new ROSLIB.Service({
                    ros: self.ros,
                    name: '/rosapi/nodes',
                    serviceType: 'rosapi/Nodes',
                });
                svc.callService(new ROSLIB.ServiceRequest({}), function(result) {
                    if (!result || !result.nodes) return;
                    var nodes = result.nodes;
                    self._setStatus('chassis', nodes.includes('/chassis_control'));
                    self._setStatus('lift', nodes.includes('/lift_control'));
                    self._checkPublishers('/left_forward_position_controller/commands', 'arm_left');
                    self._checkPublishers('/right_forward_position_controller/commands', 'arm_right');
                    self._checkTopic('/f710/status', 'f710');
                    self._checkPublishers('/cb_left_hand_state', 'hand_left');
                    self._checkPublishers('/cb_right_hand_state', 'hand_right');
                });
            } catch (e) {}
        };
        check();
        this._timers.push(setInterval(check, 3000));
    },

    _checkTopic(topicName, deviceKey) {
        var self = this;
        try {
            var svc = new ROSLIB.Service({
                ros: this.ros,
                name: '/rosapi/topic_type',
                serviceType: 'rosapi/TopicType',
            });
            svc.callService(
                new ROSLIB.ServiceRequest({ topic: topicName }),
                function(result) { self._setStatus(deviceKey, !!(result && result.type)); },
                function() { self._setStatus(deviceKey, false); }
            );
        } catch (e) { this._setStatus(deviceKey, false); }
    },

    _checkPublishers(topicName, deviceKey) {
        var self = this;
        try {
            var svc = new ROSLIB.Service({
                ros: self.ros,
                name: '/rosapi/publishers',
                serviceType: 'rosapi/Publishers',
            });
            svc.callService(
                new ROSLIB.ServiceRequest({ topic: topicName }),
                function(result) { self._setStatus(deviceKey, !!(result && result.publishers && result.publishers.length > 0)); },
                function() { self._setStatus(deviceKey, false); }
            );
        } catch (e) { this._setStatus(deviceKey, false); }
    },

    _startImuPoll() {
        var self = this;
        var poll = async function() {
            try {
                var resp = await fetch('/api/imu');
                var json = await resp.json();
                self._setStatus('imu', json.ok && !!json.data);
            } catch { self._setStatus('imu', false); }
        };
        poll();
        this._timers.push(setInterval(poll, 3000));
    },

    _startDistancePoll() {
        var self = this;
        var poll = async function() {
            try {
                var resp = await fetch('/api/sensors/distance');
                var json = await resp.json();
                self._setStatus('laser', json.ok === true);
            } catch { self._setStatus('laser', false); }
        };
        poll();
        this._timers.push(setInterval(poll, 3000));
    },

    _startCameraPoll() {
        var self = this;
        var poll = async function() {
            try {
                var resp = await fetch('/camera/status');
                var status = await resp.json();
                ['d405_1', 'd405_2', 'zed'].forEach(function(name) {
                    var info = status[name];
                    if (!info) {
                        self._setStatus(name, false);
                    } else if (info.error) {
                        self.devices[name].status = 'error';
                        self.render();
                    } else {
                        self._setStatus(name, info.running || !!info.device);
                    }
                });
            } catch {
                var s = self;
                ['d405_1', 'd405_2', 'zed'].forEach(function(n) { s._setStatus(n, false); });
            }
        };
        poll();
        this._timers.push(setInterval(poll, 5000));
    },

    _setStatus(deviceKey, online) {
        var prev = this.devices[deviceKey].status;
        this.devices[deviceKey].status = online ? 'online' : 'offline';
        if (prev !== this.devices[deviceKey].status) this.render();
    },

    initStandalone() {
        this._startCameraPoll();
        this._startImuPoll();
        this._startDistancePoll();
        this.render();
    },

    disable() {
        this._clearTimers();
        Object.keys(this.devices).forEach(function(key) {
            this.devices[key].status = 'offline';
        }.bind(this));
        this.render();
    },

    render() {
        var bar = document.getElementById('hardware-status-bar');
        if (!bar) return;

        var keys = Object.keys(this.devices);
        var html = '';
        var onlineCount = 0;
        var totalCount = keys.length;
        var self = this;

        keys.forEach(function(key) {
            var dev = this.devices[key];
            var cls = 'hw-' + dev.status;
            var txt = { online: '\u5728\u7EBF', offline: '\u79BB\u7EBF', error: '\u5F02\u5E38', unknown: '\u68C0\u6D4B\u4E2D' }[dev.status] || '\u672A\u77E5';
            if (dev.status === 'online') onlineCount++;

            html += '<div class="hw-item ' + cls + '" title="' + dev.label + ': ' + txt + '">'
                + '<span class="hw-icon">' + dev.icon + '</span>'
                + '<span class="hw-label">' + dev.label + '</span>'
                + '<span class="hw-dot"></span>'
                + '</div>';
        }.bind(this));

        html += '<span class="hw-summary">'
            + '<span class="hw-count good">' + onlineCount + '</span>'
            + '/' + totalCount + ' \u5728\u7EBF'
            + '</span>';

        bar.innerHTML = html;
    },
};

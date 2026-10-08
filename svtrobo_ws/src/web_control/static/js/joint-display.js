/**
 * SVTROBO Web Control - Joint Display Module
 * Subscribes to /joint_states (arms) and /cb_*_hand_state (hands).
 * Displays per-joint position/velocity/effort in 4 sections.
 */

const JointDisplay = {
    ros: null,
    _topics: [],
    _armData: { left: {}, right: {} },
    _handData: { left: {}, right: {} },
    _connected: { left_arm: false, right_arm: false, left_hand: false, right_hand: false },

    init(ros) {
        this.ros = ros;
        this._clearTopics();
        this._subscribeArms();
        this._subscribeHands();
        this._renderAll();
    },

    _clearTopics() {
        this._topics.forEach(function(t) { t.unsubscribe(); });
        this._topics = [];
    },

    _subscribeArms() {
        var self = this;
        var topic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/joint_states',
            messageType: 'sensor_msgs/msg/JointState',
        });
        topic.subscribe(function(msg) {
            if (!msg.name) return;
            var leftJoints = {}, rightJoints = {};
            for (var i = 0; i < msg.name.length; i++) {
                var n = msg.name[i];
                if (n.startsWith('openarm_left_')) {
                    leftJoints[n] = { pos: msg.position[i], vel: msg.velocity[i], eff: msg.effort[i] };
                } else if (n.startsWith('openarm_right_')) {
                    rightJoints[n] = { pos: msg.position[i], vel: msg.velocity[i], eff: msg.effort[i] };
                }
            }
            if (Object.keys(leftJoints).length > 0) {
                self._armData.left = leftJoints;
                if (!self._connected.left_arm) { self._connected.left_arm = true; self._updateDot('left_arm', true); }
                self._renderArm('left');
            }
            if (Object.keys(rightJoints).length > 0) {
                self._armData.right = rightJoints;
                if (!self._connected.right_arm) { self._connected.right_arm = true; self._updateDot('right_arm', true); }
                self._renderArm('right');
            }
        });
        this._topics.push(topic);
    },

    _subscribeHands() {
        var self = this;
        var sides = ['left', 'right'];
        sides.forEach(function(side) {
            var topic = new ROSLIB.Topic({
                ros: self.ros,
                name: '/cb_' + side + '_hand_state',
                messageType: 'sensor_msgs/msg/JointState',
            });
            topic.subscribe(function(msg) {
                if (!msg.name) return;
                self._connected[side + '_hand'] = true;
                self._updateDot(side + '_hand', true);
                var joints = {};
                for (var i = 0; i < msg.name.length; i++) {
                    joints[msg.name[i]] = { pos: msg.position[i], vel: msg.velocity[i] };
                }
                self._handData[side] = joints;
                self._renderHand(side);
            });
            self._topics.push(topic);
        });
    },

    _updateDot(key, online) {
        var dot = document.getElementById('dot-' + key);
        if (dot) {
            dot.className = 'joint-section-dot ' + (online ? 'online' : 'offline-dot');
        }
        var section = document.getElementById('section-' + key);
        if (section) {
            if (online) section.classList.remove('offline');
            else section.classList.add('offline');
        }
    },

    _renderArm(side) {
        var tbody = document.getElementById('arm-' + side + '-body');
        if (!tbody) return;
        var data = this._armData[side];
        if (Object.keys(data).length === 0) return;

        // Sort: joint1..joint7 then finger
        var names = Object.keys(data).sort(function(a, b) {
            var order = { joint1:1, joint2:2, joint3:3, joint4:4, joint5:5, joint6:6, joint7:7 };
            var ka = a.replace('openarm_' + side + '_', '');
            var kb = b.replace('openarm_' + side + '_', '');
            return (order[ka] || 99) - (order[kb] || 99);
        });

        var html = '';
        names = names.filter(function(n) { return n.indexOf('finger_joint') === -1; });
        names.forEach(function(name) {
            var short = name.replace('openarm_' + side + '_', '');
            var d = data[name];
            var posDeg = (d.pos * 180 / Math.PI).toFixed(2);
            var vel = d.vel.toFixed(3);
            var eff = d.eff.toFixed(3);
            html += '<tr><td class="joint-name-cell">' + short + '</td>'
                + '<td>' + posDeg + '\u00B0</td>'
                + '<td>' + vel + '</td>'
                + '<td>' + eff + '</td></tr>';
        });
        tbody.innerHTML = html;
    },

    _renderHand(side) {
        var tbody = document.getElementById('hand-' + side + '-body');
        if (!tbody) return;
        var data = this._handData[side];
        if (Object.keys(data).length === 0) return;

        // Friendly names
        var nameMap = {
            thumb_cmc_pitch: '\u62C7\u6307\u4FEF\u4EF0',
            thumb_cmc_yaw: '\u62C7\u6307\u504F\u8F6C',
            index_mcp_pitch: '\u98DF\u6307',
            middle_mcp_pitch: '\u4E2D\u6307',
            ring_mcp_pitch: '\u65E0\u540D\u6307',
            pinky_mcp_pitch: '\u5C0F\u6307',
        };
        var order = ['thumb_cmc_pitch','thumb_cmc_yaw','index_mcp_pitch','middle_mcp_pitch','ring_mcp_pitch','pinky_mcp_pitch'];

        var html = '';
        order.forEach(function(name) {
            var d = data[name];
            if (!d) return;
            var label = nameMap[name] || name;
            html += '<tr><td class="joint-name-cell">' + label + '</td>'
                + '<td>' + d.pos.toFixed(1) + '</td>'
                + '<td>' + (d.vel !== undefined ? d.vel.toFixed(1) : '--') + '</td></tr>';
        });
        tbody.innerHTML = html;
    },

    _renderAll() {
        ['left', 'right'].forEach(function(side) {
            // Arm offline placeholders
            var armBody = document.getElementById('arm-' + side + '-body');
            if (armBody && Object.keys(this._armData[side]).length === 0) {
                armBody.innerHTML = '<tr><td colspan="4" class="joint-offline-text">\u672A\u8FDE\u63A5</td></tr>';
            }
            // Hand offline placeholders
            var handBody = document.getElementById('hand-' + side + '-body');
            if (handBody && Object.keys(this._handData[side]).length === 0) {
                handBody.innerHTML = '<tr><td colspan="3" class="joint-offline-text">\u672A\u8FDE\u63A5</td></tr>';
            }
        }.bind(this));
    },

    disable() {
        this._clearTopics();
        this._armData = { left: {}, right: {} };
        this._handData = { left: {}, right: {} };
        this._connected = { left_arm: false, right_arm: false, left_hand: false, right_hand: false };
        ['left_arm', 'right_arm', 'left_hand', 'right_hand'].forEach(function(k) {
            this._updateDot(k, false);
        }.bind(this));
        this._renderAll();
    },
};

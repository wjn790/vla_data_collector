/**
 * SVTROBO Web Control - Distance Sensor Panel
 * Polls /api/sensors/distance for SEN0492 laser range finder data.
 * Standalone (no ROS dependency).
 */

const DistanceSensor = {
    _timer: null,
    _lastOk: false,

    initStandalone() {
        this._startPoll();
    },

    disable() {
        if (this._timer) {
            clearInterval(this._timer);
            this._timer = null;
        }
        // Reset display
        this._updateUI(null);
    },

    _startPoll() {
        var self = this;
        var poll = function() {
            fetch('/api/sensors/distance')
                .then(function(r) { return r.json(); })
                .then(function(data) {
                    if (data && data.ok) {
                        self._lastOk = true;
                        self._updateUI(data);
                    } else {
                        self._lastOk = false;
                        self._updateUI(null);
                    }
                })
                .catch(function() {
                    self._lastOk = false;
                    self._updateUI(null);
                });
        };
        poll();
        this._timer = setInterval(poll, 500);
    },

    _updateUI(data) {
        var dot = document.getElementById('dot-distance');
        var sensors = ['front', 'right', 'rear', 'left'];
        var readings = (data && data.data) ? data.data : null;
        var hasData = data && data.ok && readings;

        if (dot) {
            if (hasData) {
                dot.classList.remove('offline-dot');
                dot.classList.add('online');
            } else {
                dot.classList.remove('online');
                dot.classList.add('offline-dot');
            }
        }

        for (var i = 0; i < sensors.length; i++) {
            var name = sensors[i];
            var valEl = document.getElementById('dist-' + name);
            if (valEl) {
                if (hasData && readings[name] !== undefined && readings[name] !== null) {
                    valEl.textContent = readings[name];
                    valEl.classList.add('dist-active');
                } else {
                    valEl.textContent = '--';
                    valEl.classList.remove('dist-active');
                }
            }
        }
    },
};

// Patched status.js - adds topics dot
const StatusMonitor = {
    ros: null,

    init(ros) {
        this.ros = ros;
        this.refreshTopics();
        this._timer = setInterval(() => this.refreshTopics(), 5000);
    },

    setDot(online) {
        const dot = document.getElementById('dot-topics');
        if (dot) dot.className = 'section-dot ' + (online ? 'online' : 'offline-dot');
    },

    disable() {
        this.setDot(false);
        if (this._timer) { clearInterval(this._timer); this._timer = null; }
        const container = document.getElementById('topic-list');
        if (container) container.innerHTML = '<div class="topic-empty">未连接</div>';
        const countEl = document.getElementById('topic-count');
        if (countEl) countEl.textContent = '0';
    },

    async refreshTopics() {
        if (!this.ros || !this.ros.isConnected) return;

        try {
            const topicsClient = new ROSLIB.Service({
                ros: this.ros,
                name: '/rosapi/topics',
                serviceType: 'rosapi/Topics',
            });

            const request = new ROSLIB.ServiceRequest({});
            topicsClient.callService(request, (result) => {
                this.setDot(true);
                this.updateTopicList(result.topics);
            });
        } catch (e) {
            console.error('Failed to fetch topics:', e);
        }
    },

    updateTopicList(topics) {
        const container = document.getElementById('topic-list');
        if (!container) return;

        if (!topics || topics.length === 0) {
            container.innerHTML = '<div class="topic-empty">暂无订阅话题</div>';
            return;
        }

        const sorted = topics.sort();
        container.innerHTML = sorted.map(t => `<div class="topic-item">${t}</div>`).join('');
        document.getElementById('topic-count').textContent = sorted.length;
    },
};

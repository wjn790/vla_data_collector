#!/bin/bash
# PCAN err-71 Runtime Monitor and Auto-Recovery
# Monitors can0-can5 interfaces, detects DOWN state or new pcan err-71,
# and automatically recovers CAN interfaces.
# If recovery fails, triggers system reboot.

# ── Environment ──────────────────────────────────────────────
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/svt/svtrobo_ws/devel/setup.bash 2>/dev/null || true

# ── Configuration ────────────────────────────────────────────
CAN_INTERFACES="can0 can1 can2 can3 can4 can5"
CHECK_INTERVAL=10
RECOVERY_WAIT=5
LOG_FILE="/tmp/pcan_monitor.log"
DMESG_ERR_MARKER="/tmp/pcan_monitor_last_err_count"
MAX_RECOVERY_RETRIES=2

# ── Logging ──────────────────────────────────────────────────
log() {
    local msg="[$(date '+%Y-%m-%d %H:%M:%S')] $*"
    echo "$msg" >> "$LOG_FILE" 2>/dev/null
    logger -t pcan-monitor "$*"
}

# ── Get CAN config for an interface ──────────────────────────
get_can_config() {
    local iface="$1"
    case "$iface" in
        can0|can1)
            echo "bitrate 1000000 dbitrate 5000000 fd on"
            ;;
        can2|can3)
            echo "bitrate 1000000"
            ;;
        can4|can5)
            echo "bitrate 500000"
            ;;
        *)
            echo ""
            ;;
    esac
}

# ── Check if a CAN interface is UP ───────────────────────────
is_can_up() {
    local iface="$1"
    local output
    output=$(ip -details link show "$iface" 2>/dev/null) || return 1
    echo "$output" | grep -q "state UP"
}

# ── Recover a single CAN interface ──────────────────────────
recover_can() {
    local iface="$1"
    local config
    config=$(get_can_config "$iface")

    if [[ -z "$config" ]]; then
        log "ERROR: Unknown interface $iface, skipping"
        return 1
    fi

    log "RECOVERY: Bringing $iface down..."
    echo '<sudo密码，在真机上填写>' | sudo -S ip link set "$iface" down 2>/dev/null || true
    sleep 1

    log "RECOVERY: Configuring $iface with: $config"
    echo '<sudo密码，在真机上填写>' | sudo -S ip link set "$iface" type can $config 2>&1 || true

    log "RECOVERY: Bringing $iface up..."
    echo '<sudo密码，在真机上填写>' | sudo -S ip link set "$iface" up 2>&1 || true

    sleep "$RECOVERY_WAIT"

    if is_can_up "$iface"; then
        log "RECOVERY SUCCESS: $iface is UP"
        return 0
    else
        log "RECOVERY FAILED: $iface is still DOWN"
        return 1
    fi
}

# ── Get current pcan error count from dmesg ─────────────────
get_pcan_err_count() {
    local count
    count=$(echo '<sudo密码，在真机上填写>' | sudo -S dmesg 2>/dev/null | grep -ci "pcan.*err" 2>/dev/null) || count=0
    echo "${count:-0}"
}

# ── Check svtrobo-can service is active ─────────────────────
is_svtrobo_can_active() {
    systemctl is-active --quiet svtrobo-can.service 2>/dev/null
}

# ── Initialize error marker ─────────────────────────────────
init_err_marker() {
    local count
    count=$(get_pcan_err_count)
    echo "$count" > "$DMESG_ERR_MARKER"
    log "INIT: pcan error count baseline = $count"
}

# ── Main monitoring loop ────────────────────────────────────
main() {
    log "STARTED: PCAN err-71 monitor (interval=${CHECK_INTERVAL}s, interfaces=$CAN_INTERFACES)"
    init_err_marker

    local recovery_failed_count=0

    while $running; do
        # Only monitor when svtrobo-can is active
        if ! is_svtrobo_can_active; then
            sleep "$CHECK_INTERVAL" &
            wait $!
            continue
        fi

        local need_recovery=false
        local down_interfaces=""

        # 1. Check each CAN interface state
        for iface in $CAN_INTERFACES; do
            if ! is_can_up "$iface"; then
                log "DETECTED: $iface is DOWN"
                need_recovery=true
                down_interfaces="$down_interfaces $iface"
            fi
        done

        # 2. Check for new pcan err-71 in dmesg
        local current_err_count
        current_err_count=$(get_pcan_err_count)
        local last_err_count=0
        if [[ -f "$DMESG_ERR_MARKER" ]]; then
            last_err_count=$(cat "$DMESG_ERR_MARKER")
        fi

        if [[ "$current_err_count" -gt "$last_err_count" ]]; then
            log "DETECTED: New pcan errors (was $last_err_count, now $current_err_count)"
            echo '<sudo密码，在真机上填写>' | sudo -S dmesg 2>/dev/null | grep -i "pcan.*err" | tail -5 | while read -r line; do
                log "DMESG: $line"
            done

            for iface in $CAN_INTERFACES; do
                if ! is_can_up "$iface"; then
                    if [[ "$down_interfaces" != *" $iface"* ]]; then
                        down_interfaces="$down_interfaces $iface"
                    fi
                fi
            done

            if [[ -z "$down_interfaces" ]]; then
                log "INFO: pcan error detected but all interfaces still UP, monitoring closely"
            fi
            need_recovery=true
        fi

        # Update error marker
        echo "$current_err_count" > "$DMESG_ERR_MARKER"

        # 3. Recovery if needed
        if $need_recovery && [[ -n "$down_interfaces" ]]; then
            log "RECOVERY START: Affected interfaces:$down_interfaces"
            local all_recovered=true

            for iface in $down_interfaces; do
                local retry=0
                local recovered=false
                while [[ $retry -lt "$MAX_RECOVERY_RETRIES" ]]; do
                    if recover_can "$iface"; then
                        recovered=true
                        break
                    fi
                    retry=$((retry + 1))
                    log "RECOVERY RETRY: $iface attempt $retry/$MAX_RECOVERY_RETRIES"
                done

                if ! $recovered; then
                    all_recovered=false
                    log "CRITICAL: Failed to recover $iface after $MAX_RECOVERY_RETRIES attempts"
                fi
            done

            if ! $all_recovered; then
                recovery_failed_count=$((recovery_failed_count + 1))
                log "CRITICAL: Recovery failed ($recovery_failed_count consecutive failures)"

                if [[ $recovery_failed_count -ge 2 ]]; then
                    log "FATAL: Multiple recovery failures, giving up (auto-reboot disabled)"
                    exit 1
                fi
            else
                recovery_failed_count=0
            fi
        fi

        sleep "$CHECK_INTERVAL" &
        wait $!
    done
}

# ── Signal handling ─────────────────────────────────────────
running=true
trap 'running=false; log "SIGTERM received, exiting"' SIGTERM SIGINT

main "$@"
log "STOPPED: PCAN monitor shut down gracefully"

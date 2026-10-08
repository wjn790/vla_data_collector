#!/bin/bash
set -e
log() { logger -t svtrobo-can "$@"; }

wait_for_can0() {
    local waited=0
    while [ $waited -lt 60 ] && ! ip link show can0 >/dev/null 2>&1; do
        sleep 1; waited=$((waited+1))
    done
    log "can0 appeared after ${waited}s"
    [ $waited -ge 60 ] && { log "FATAL can0 not found after 60s"; return 1; }
    return 0
}

configure_can() {
    for if in can0 can1; do
        ip link set $if down 2>/dev/null || true
        ip link set $if type can bitrate 1000000 dbitrate 5000000 fd on 2>/dev/null || true
        ip link set $if up 2>/dev/null || true
    done
    for if in can2 can3; do
        ip link set $if down 2>/dev/null || true
        ip link set $if type can bitrate 1000000 2>/dev/null || true
        ip link set $if up 2>/dev/null || true
    done
    for if in can4 can5; do
        ip link set $if down 2>/dev/null || true
        ip link set $if type can bitrate 500000 2>/dev/null || true
        ip link set $if up 2>/dev/null || true
    done
}

wait_pcan_stable() {
    local err_before=$(dmesg | grep -c "pcan.*err" 2>/dev/null || echo 0)
    local waited=0
    while [ $waited -lt 10 ]; do
        sleep 1
        local err_now=$(dmesg | grep -c "pcan.*err" 2>/dev/null || echo 0)
        [ "$err_now" -eq "$err_before" ] && break
        err_before=$err_now; waited=$((waited+1))
    done
    log "PCAN stabilized after ${waited}s (errors: $err_before)"
    echo "$err_before"
}

check_interfaces_up() {
    local up=0
    for if in can0 can1 can2 can3 can4 can5; do
        ip -details link show $if 2>/dev/null | grep -q "state UP" && up=$((up+1))
    done
    echo "$up"
}

deep_reset() {
    log "Deep resetting PCAN USB driver..."
    for if in can0 can1 can2 can3 can4 can5; do
        ip link set $if down 2>/dev/null || true
    done
    sleep 1
    timeout 15 modprobe -r pcan 2>/dev/null || log "WARN: modprobe -r timed out"
    sleep 1
    modprobe pcan 2>/dev/null || true
    sleep 2
    local waited=0
    while [ $waited -lt 20 ] && ! ip link show can0 >/dev/null 2>&1; do
        sleep 1; waited=$((waited+1))
    done
    sleep 1
    configure_can
    sleep 2
    local up=$(check_interfaces_up)
    log "Deep reset done ($up/6 UP)"
    return 0
}

log "Starting CAN setup..."
wait_for_can0 || exit 1
sleep 3
err_count=$(wait_pcan_stable)
log "Configuring CAN interfaces..."
configure_can
sleep 2
up_count=$(check_interfaces_up)
new_errors=$(( $(dmesg | grep -c "pcan.*err" 2>/dev/null || echo 0) - err_count ))
log "First config: $up_count/6 UP, $new_errors new errors"
if [ "$new_errors" -gt 0 ] || [ "$up_count" -lt 6 ]; then
    log "Issues detected, attempting deep reset..."
    deep_reset
fi
chmod 777 /dev/ttyACM0 2>/dev/null || true
rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
log "CAN setup complete"

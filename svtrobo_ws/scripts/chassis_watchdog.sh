#!/bin/bash
# chassis_watchdog.sh - Monitor chassis_control_node and restart service on crash or all-zero data
# P0-1: Auto-detect and restart when chassis_control_node dies
# P0-2: Detect all-zero joint_states (CAN disconnected) and restart, max 3 retries

# Graceful shutdown on SIGTERM
running=true
trap 'running=false; echo SIGTERM received, exiting >> /tmp/chassis_watchdog.log' SIGTERM SIGINT

# Source ROS environment
source /opt/ros/humble/setup.bash
source /home/svt/svtrobo_ws/install/setup.bash

LOG_FILE="/tmp/chassis_watchdog.log"
NODE_NAME="chassis_control"
SERVICE_NAME="svtrobo-chassis"
CHECK_INTERVAL=5
FAIL_THRESHOLD=2

# All-zero detection
ZERO_CHECK_INTERVAL=30
ZERO_FAIL_THRESHOLD=3
ZERO_MAX_RETRIES=3

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$LOG_FILE"
}

fail_count=0
zero_fail_count=0
zero_retry_count=0

log "=== chassis_watchdog started ==="

while $running; do
    # Only monitor when svtrobo-chassis service is active
    if ! systemctl is-active --quiet "$SERVICE_NAME"; then
        if [ $fail_count -ne 0 ] || [ $zero_fail_count -ne 0 ]; then
            log "Service $SERVICE_NAME is not active, resetting fail counters"
            fail_count=0
            zero_fail_count=0
        fi
        sleep $CHECK_INTERVAL &
        wait $!
        continue
    fi

    # Check if chassis_control node is alive via ROS2 CLI
    if ! ros2 node list 2>/dev/null | grep -q "/${NODE_NAME}"; then
        fail_count=$((fail_count + 1))
        log "WARNING: /${NODE_NAME} not detected (fail ${fail_count}/${FAIL_THRESHOLD})"

        if [ $fail_count -ge $FAIL_THRESHOLD ]; then
            log "ACTION: Restarting $SERVICE_NAME (node /${NODE_NAME} missing for ${fail_count} consecutive checks)"
            SUDO_ASKPASS=/tmp/askpass.sh sudo -A systemctl restart "$SERVICE_NAME"
            log "ACTION: $SERVICE_NAME restart triggered"
            fail_count=0
            zero_fail_count=0
            sleep 10 &
            wait $!
            continue
        fi
    else
        if [ $fail_count -ne 0 ]; then
            log "OK: /${NODE_NAME} recovered, resetting fail counter"
        fi
        fail_count=0
    fi

    # All-zero data detection (CAN disconnected)
    if [ $zero_retry_count -lt $ZERO_MAX_RETRIES ]; then
        diag=$(source /opt/ros/humble/setup.bash && source /home/svt/svtrobo_ws/install/setup.bash && \
               export ROS_DOMAIN_ID=56 && export ROS_LOCALHOST_ONLY=1 && \
               timeout 3 ros2 topic echo /chassis/diagnostics --once 2>/dev/null)

        if [ -n "$diag" ]; then
            vbus=$(echo "$diag" | grep "vbus:" | awk '{print $2}')
            if [ -n "$vbus" ] && [ "$(echo "$vbus < 1.0" | bc -l 2>/dev/null)" = "1" ]; then
                zero_fail_count=$((zero_fail_count + 1))
                log "WARNING: vbus=${vbus} (all-zero data), fail ${zero_fail_count}/${ZERO_FAIL_THRESHOLD}, retry ${zero_retry_count}/${ZERO_MAX_RETRIES}"

                if [ $zero_fail_count -ge $ZERO_FAIL_THRESHOLD ]; then
                    zero_retry_count=$((zero_retry_count + 1))
                    log "ACTION: Restarting $SERVICE_NAME (all-zero data, attempt ${zero_retry_count}/${ZERO_MAX_RETRIES})"
                    SUDO_ASKPASS=/tmp/askpass.sh sudo -A systemctl restart "$SERVICE_NAME"
                    fail_count=0
                    zero_fail_count=0
                    sleep 10 &
                    wait $!
                    continue
                fi
            else
                if [ $zero_fail_count -ne 0 ]; then
                    log "OK: vbus=${vbus}, data recovered, resetting zero fail counter"
                fi
                zero_fail_count=0
            fi
        else
            zero_fail_count=$((zero_fail_count + 1))
            log "WARNING: no diagnostics data, fail ${zero_fail_count}/${ZERO_FAIL_THRESHOLD}"
        fi
    fi

    sleep $CHECK_INTERVAL &
    wait $!
done

log "=== chassis_watchdog stopped gracefully ==="

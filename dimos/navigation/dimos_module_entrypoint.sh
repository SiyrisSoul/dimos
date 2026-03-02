#!/bin/bash

MODE="${MODE:-unity_sim}"
USE_ROUTE_PLANNER="${USE_ROUTE_PLANNER:-true}"
USE_RVIZ="${USE_RVIZ:-false}"
ENABLE_FOXGLOVE="${ENABLE_FOXGLOVE:-false}"
LOCALIZATION_METHOD="${LOCALIZATION_METHOD:-arise_slam}"
BAGFILE_PATH="${BAGFILE_PATH:-}"

UNITY_BRIDGE_CONNECT_TIMEOUT_SEC="${UNITY_BRIDGE_CONNECT_TIMEOUT_SEC:-25}"
UNITY_BRIDGE_RETRY_INTERVAL_SEC="${UNITY_BRIDGE_RETRY_INTERVAL_SEC:-2}"

STACK_ROOT="/ros2_ws/src/ros-navigation-autonomy-stack"
UNITY_EXECUTABLE="${STACK_ROOT}/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
UNITY_MESH_DIR="${STACK_ROOT}/src/base_autonomy/vehicle_simulator/mesh/unity"

# 
# Source 
# 
echo "[entrypoint] Sourcing ROS env..."
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ros2_ws/install/setup.bash
source /opt/dimos-venv/bin/activate

# 
# cli helpers (when connecting to docker)
# 

# rosspy
cat > /usr/bin/rosspy <<'EOS'
#!/bin/bash
    source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
    source /ros2_ws/install/setup.bash
    source /opt/dimos-venv/bin/activate
    exec python3 -m dimos.utils.cli.rosspy.run_rosspy "$@"
EOS
chmod +x /usr/bin/rosspy

# x11_doctor
cat > /usr/bin/x11_doctor <<'EOS'
#!/usr/bin/env bash
    ok=true
    echo "=== X11 Doctor ==="

    # 1. DISPLAY
    echo ""
    echo "--- DISPLAY ---"
    if [ -z "${DISPLAY:-}" ]; then
        echo "  FAIL  DISPLAY is not set"
        ok=false
    else
        echo "  OK    DISPLAY=${DISPLAY}"
    fi

    # 2. X11 unix socket directory
    echo ""
    echo "--- /tmp/.X11-unix socket directory ---"
    if [ ! -d /tmp/.X11-unix ]; then
        echo "  FAIL  /tmp/.X11-unix does not exist (volume not mounted?)"
        ok=false
    else
        sockets
        sockets=$(ls /tmp/.X11-unix 2>/dev/null)
        if [ -z "$sockets" ]; then
            echo "  WARN  /tmp/.X11-unix exists but is empty (no display sockets)"
            ok=false
        else
            echo "  OK    /tmp/.X11-unix contents: $sockets"
        fi
    fi

    # 3. Socket for the specific DISPLAY
    echo ""
    echo "--- Display socket for DISPLAY=${DISPLAY:-<unset>} ---"
    if [ -n "${DISPLAY:-}" ]; then
        display_num=$(echo "${DISPLAY}" | sed 's/.*:\([0-9]*\).*/\1/')
        sock="/tmp/.X11-unix/X${display_num}"
        if [ -S "$sock" ]; then
            echo "  OK    $sock exists and is a socket"
            ls -la "$sock"
        else
            echo "  FAIL  $sock not found or not a socket"
            ok=false
        fi
    else
        echo "  SKIP  (DISPLAY not set)"
    fi

    # 4. XAUTHORITY file
    echo ""
    echo "--- XAUTHORITY ---"
    xauth_file="${XAUTHORITY:-$HOME/.Xauthority}"
    if [ -z "${XAUTHORITY:-}" ]; then
        echo "  WARN  XAUTHORITY env var not set; defaulting to $xauth_file"
    else
        echo "  OK    XAUTHORITY=${XAUTHORITY}"
    fi
    if [ -f "$xauth_file" ]; then
        echo "  OK    $xauth_file exists ($(wc -c < "$xauth_file") bytes)"
        ls -la "$xauth_file"
    else
        echo "  FAIL  $xauth_file not found (Xauthority not mounted?)"
        ok=false
    fi

    # 5. xauth cookie list
    echo ""
    echo "--- xauth cookie entries ---"
    if command -v xauth >/dev/null 2>&1; then
        cookie_out=$(XAUTHORITY="$xauth_file" xauth list 2>&1)
        if [ -z "$cookie_out" ]; then
            echo "  WARN  xauth list returned no entries (cookie file empty or wrong display)"
            ok=false
        else
            echo "  OK    cookies found:"
            echo "$cookie_out" | sed 's/^/        /'
        fi
    else
        echo "  WARN  xauth not installed; cannot check cookies"
    fi

    # 6. Live connection test
    echo ""
    echo "--- Live connection test ---"
    if command -v xdpyinfo >/dev/null 2>&1; then
        if DISPLAY="${DISPLAY:-:0}" XAUTHORITY="$xauth_file" xdpyinfo >/dev/null 2>&1; then
            echo "  OK    xdpyinfo connected to ${DISPLAY:-:0} successfully"
        else
            echo "  FAIL  xdpyinfo could not connect to ${DISPLAY:-:0}"
            DISPLAY="${DISPLAY:-:0}" XAUTHORITY="$xauth_file" xdpyinfo 2>&1 | head -5 | sed 's/^/        /'
            ok=false
        fi
    elif command -v xclock >/dev/null 2>&1; then
        if DISPLAY="${DISPLAY:-:0}" XAUTHORITY="$xauth_file" xclock -display "${DISPLAY:-:0}" &
            sleep 1 && kill %1 2>/dev/null; then
            echo "  OK    xclock launched on ${DISPLAY:-:0}"
        else
            echo "  FAIL  xclock could not connect"
            ok=false
        fi
    else
        echo "  SKIP  neither xdpyinfo nor xclock installed; skipping live test"
        echo "        Install with: apt-get install -y x11-utils"
    fi

    # 7. Summary
    echo ""
    echo "=== Summary ==="
    if $ok; then
        echo "  All checks passed — X11 should work."
    else
        echo "  One or more checks failed."
        echo ""
        echo "  Common fixes:"
        echo "    • Mount the socket:   -v /tmp/.X11-unix:/tmp/.X11-unix"
        echo "    • Mount the cookie:   -v \${XAUTHORITY:-\$HOME/.Xauthority}:/tmp/.Xauthority:ro"
        echo "    • Set env vars:       -e DISPLAY -e XAUTHORITY=/tmp/.Xauthority"
        echo "    • Allow local X:      xhost +local:  (run on host, less safe)"
    fi
    echo ""
EOS
chmod +x /usr/bin/x11_doctor

# 
# 
# 
# sanity checks and setup
# 
#
# 

# 
# dimos
# 
if ! [ -d "/workspace/dimos" ]; then
    echo "the dimos codebase must be mounted to /workspace/dimos for the codebase to work"
    exit 1
fi
export PYTHONPATH="/workspace/dimos:${PYTHONPATH:-}"
if ! pip install -e /workspace/dimos >/tmp/dimos_pip_install.log 2>&1; then
    cat /tmp/dimos_pip_install.log
    echo "[entrypoint] WARNING: pip install -e failed; see /tmp/dimos_pip_install.log"
    exit 2
fi

# 
# dds config
# 
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [ -f "/ros2_ws/config/custom_fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/custom_fastdds.xml
elif [ -f "/ros2_ws/config/fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds.xml
fi

# 
# launch setup
# 
if [ ! -d "$STACK_ROOT" ]; then
    echo "[entrypoint] ERROR: stack root not found: $STACK_ROOT"
    exit 3
fi
cd "$STACK_ROOT"

LAUNCH_ARGS="enable_bridge:=false"
if [ "$LOCALIZATION_METHOD" = "fastlio" ]; then
    LAUNCH_ARGS="use_fastlio2:=true ${LAUNCH_ARGS}"
fi

# 
# unity sim helpers
# 
# complicated because of retry system (needed as an alternative to "sleep 5" and praying its enough)
start_unity() {
    if [ ! -f "$UNITY_EXECUTABLE" ]; then
        echo "[entrypoint] ERROR: Unity executable not found: $UNITY_EXECUTABLE"
        exit 1
    fi

    # These files are expected by CMU/TARE sim assets. Missing files usually
    # indicate a bad mount and can break downstream map-dependent behavior.
    for required in map.ply traversable_area.ply; do
        if [ ! -f "$UNITY_MESH_DIR/$required" ]; then
            echo "[entrypoint] WARNING: missing $UNITY_MESH_DIR/$required"
        fi
    done

    echo "[entrypoint] Starting Unity: $UNITY_EXECUTABLE"
    "$UNITY_EXECUTABLE" &
    UNITY_PID=$!
    echo "[entrypoint] Unity PID: $UNITY_PID"
}

start_ros_nav_stack() {
    setsid bash -c "
        source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
        source /ros2_ws/install/setup.bash
        cd ${STACK_ROOT}
        ros2 launch vehicle_simulator ${LAUNCH_FILE} ${LAUNCH_ARGS}
    " &
    ROS_NAV_PID=$!
    echo "[entrypoint] ROS nav stack PID: $ROS_NAV_PID"
}

stop_ros_nav_stack() {
    if [ -n "$ROS_NAV_PID" ] && kill -0 "$ROS_NAV_PID" 2>/dev/null; then
        kill -TERM "-$ROS_NAV_PID" 2>/dev/null || kill -TERM "$ROS_NAV_PID" 2>/dev/null || true
        for _ in 1 2 3 4 5; do
            kill -0 "$ROS_NAV_PID" 2>/dev/null || break
            sleep 1
        done
        kill -KILL "-$ROS_NAV_PID" 2>/dev/null || kill -KILL "$ROS_NAV_PID" 2>/dev/null || true
    fi
}

has_established_bridge_tcp() {
    if ! command -v ss >/dev/null 2>&1; then
        return 0
    fi
    ss -Htn state established '( sport = :10000 or dport = :10000 )' 2>/dev/null | grep -q .
}

unity_topics_ready() {
    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"

    echo "$topics" | grep -Eq '^/registered_scan$' || return 1
    echo "$topics" | grep -Eq '^/camera/image/compressed$' || return 1
    return 0
}

bridge_ready() {
    # Check only that Unity has established the TCP connection to the bridge.
    # unity_topics_ready (ros2 topic list) is intentionally skipped: DDS
    # discovery is too slow/unreliable to use as a readiness gate inside the
    # container — ros2 topic list consistently fails to see Unity bridge topics
    # within any reasonable window even though the publishers ARE registered.
    has_established_bridge_tcp || return 1
    return 0
}

launch_with_retry() {
    local attempt=1

    while true; do
        echo "[entrypoint] Launch attempt ${attempt}: ros2 launch vehicle_simulator ${LAUNCH_FILE} ${LAUNCH_ARGS}"
        start_ros_nav_stack

        local deadline=$((SECONDS + UNITY_BRIDGE_CONNECT_TIMEOUT_SEC))
        while [ "$SECONDS" -lt "$deadline" ]; do
            if bridge_ready; then
                echo "[entrypoint] Unity bridge ready: /registered_scan and /camera/image/compressed present."
                return 0
            fi

            if ! kill -0 "$ROS_NAV_PID" 2>/dev/null; then
                echo "[entrypoint] ROS nav stack exited during bridge startup."
                break
            fi
            sleep 1
        done

        cat <<EOM
================================================================================
================================================================================
==================== UNITY BRIDGE STARTUP TIMEOUT ERROR ========================
================================================================================
[entrypoint] Attempt ${attempt} exceeded UNITY_BRIDGE_CONNECT_TIMEOUT_SEC=${UNITY_BRIDGE_CONNECT_TIMEOUT_SEC}
[entrypoint] Bridge did not become ready in time.
[entrypoint] Required topics missing: /registered_scan and /camera/image/compressed
[entrypoint] Stopping ROS nav stack and retrying in ${UNITY_BRIDGE_RETRY_INTERVAL_SEC}s.
================================================================================
================================================================================
EOM

        stop_ros_nav_stack
        attempt=$((attempt + 1))
        sleep "$UNITY_BRIDGE_RETRY_INTERVAL_SEC"
    done
}

# 
# 
# arg simplification
# 
# 
if [ "$MODE" = "unity_sim" ] || [ -z "$MODE" ]; then
    MODE="simulation"
fi

# 
# 
# Main: Mode selection / behavior
# 
# 
if [ "$MODE" = "simulation" ]; then
    if [ "$USE_ROUTE_PLANNER" = "true" ]; then
        LAUNCH_FILE="system_simulation_with_route_planner.launch.py"
    else
        LAUNCH_FILE="system_simulation.launch.py"
    fi
    start_unity
    launch_with_retry
elif [ "$MODE" = "hardware" ]; then 
    echo "[entrypoint] MODE=$MODE is non-simulation; launching ROS nav stack once."
    start_ros_nav_stack
else
    echo "MODE must be one of: 'simulation' or 'hardware' but got '$MODE'"
    exit 19
fi

# 
# 
# optional services
# 
# 
if [ "$USE_RVIZ" = "true" ]; then
    if [ "$USE_ROUTE_PLANNER" = "true" ]; then
        RVIZ_CFG="/ros2_ws/src/ros-navigation-autonomy-stack/src/route_planner/far_planner/rviz/default.rviz"
    else
        RVIZ_CFG="/ros2_ws/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz" 
    fi
    # check if file exists
    if ! [ -f "$RVIZ_CFG" ]; then
        echo "RVIZ_CFG was set to '$RVIZ_CFG' but that file doesn't exist"
        exit 19
    fi
    ros2 run rviz2 rviz2 -d "$RVIZ_CFG" &
elif ! [ "$USE_RVIZ" = "false" ]; then
    echo "USE_RVIZ must be true or false but got: $USE_RVIZ"
    exit 20
fi


# Convert /foxglove_teleop Twist → /cmd_vel TwistStamped
if [ "$ENABLE_FOXGLOVE" = "true" ]; then
    if [ -f "/usr/local/bin/twist_relay.py" ]; then
        python3 /usr/local/bin/twist_relay.py &
    else
        echo "unable to start foxglove relay!"
        exit 21
    fi
elif ! [ "$ENABLE_FOXGLOVE" = "false" ]; then
    echo "ENABLE_FOXGLOVE must be true or false but got: $ENABLE_FOXGLOVE"
    exit 22
fi 

# start module (when being run from )
if [ "$#" -gt 0 ]; then
    exec python -m dimos.core.docker_runner run "$@"
fi

# Otherwise keep container alive with the nav stack process.
wait "$ROS_NAV_PID"

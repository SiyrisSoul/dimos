#!/bin/bash
# Entrypoint for running ROSNav as a DimOS DockerModule.
#
# Differences from run_both.sh:
#   - Does not start DimOS Python directly; the docker_runner does that via --payload
#   - Passes "$@" to `python -m dimos.core.docker_runner run` so the DockerModule
#     framework can inject the --payload JSON at launch time
#
# Environment variables (all optional):
#   START_ROS_NAV   - Start the ROS navigation stack in the background (default: true)
#                     Set to "false" for hardware mode where ROS is already running
#   START_UNITY_SIM - Launch the Unity-based vehicle_simulator within the nav stack
#                     (default: false).  Requires START_ROS_NAV=true.
#                     Set to "true" to run system_simulation_with_route_planner.launch.py
#                     in the background; leave "false" to skip the Unity simulation
#                     (e.g. when connecting to a real robot or an external sim).
#   ROS_DISTRO      - ROS distribution (default: humble)
#   DISPLAY         - X11 display; RViz is skipped if display socket is absent

set -e

# Mark git directories as safe (avoids dubious ownership warnings)
git config --global --add safe.directory /workspace/dimos 2>/dev/null || true
git config --global --add safe.directory /ros2_ws/src/ros-navigation-autonomy-stack 2>/dev/null || true

# Source ROS environment
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ros2_ws/install/setup.bash

# Activate the dimos Python venv
source /opt/dimos-venv/bin/activate

# Ensure we use the live dimos source from the host
# This keeps the container in sync with local code (no stale wheel issues).
if [ -d "/workspace/dimos" ]; then
    echo "[dimos_module_entrypoint] Installing dimos from /workspace/dimos (pip -e)"
    pip install -e /workspace/dimos >/tmp/dimos_pip_install.log 2>&1 || {
        echo "[dimos_module_entrypoint] WARNING: pip -e /workspace/dimos failed; see /tmp/dimos_pip_install.log"
    }
fi

# DDS configuration
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds.xml
if [ -f "/ros2_ws/config/custom_fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/custom_fastdds.xml
fi

# Optionally start the ROS navigation stack in the background.
# Skip when START_ROS_NAV=false (e.g. hardware mode where the nav stack runs elsewhere).
if [ "${START_ROS_NAV:-true}" = "true" ]; then
    # Only launch RViz if a display is available
    DISPLAY_NUM=$(echo "${DISPLAY:-:0}" | sed 's/://')
    if [ ! -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ]; then
        echo "[dimos_module_entrypoint] No display available, RViz will not launch"
    fi

    # Optionally launch the Unity-based vehicle_simulator simulation stack.
    # Controlled by START_UNITY_SIM (default: false).
    if [ "${START_UNITY_SIM:-false}" = "true" ]; then
        echo "[dimos_module_entrypoint] START_UNITY_SIM=true: launching Unity simulation stack in background..."
        cd /ros2_ws/src/ros-navigation-autonomy-stack

        # Start Unity environment inside the container (same as run_both.sh)
        UNITY_EXECUTABLE="./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
        if [ -f "$UNITY_EXECUTABLE" ]; then
            echo "[dimos_module_entrypoint] Starting Unity environment: $UNITY_EXECUTABLE"
            "$UNITY_EXECUTABLE" &
            UNITY_PID=$!
        else
            echo "[dimos_module_entrypoint] WARNING: Unity executable not found at $UNITY_EXECUTABLE"
            exit 1
        fi

        # Launch Unity-based vehicle_simulator + planners
        setsid bash -c "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            ros2 launch vehicle_simulator system_simulation_with_route_planner.launch.py \
            enable_bridge:=false" &
        SIM_PID=$!
        echo "[dimos_module_entrypoint] Unity simulation + nav stack started (SIM_PID: $SIM_PID, UNITY_PID: ${UNITY_PID:-none}), waiting for init..."
        sleep 5
    else
        echo "[dimos_module_entrypoint] START_UNITY_SIM=false: skipping Unity simulation launch"
    fi
fi

# Hand off to the DimOS module runner.
# The DockerModule framework passes: --payload '{"module_path":..., "args":..., "kwargs":...}'
exec python -m dimos.core.docker_runner run "$@"

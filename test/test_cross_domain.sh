#!/bin/bash
# =============================================================================
# Integration test: cross-domain communication via ros2_websocket_proxy
#
# Prerequisites:
#   - ROS 2 is sourced:       source /opt/ros/${ROS_DISTRO}/setup.bash
#   - Workspace is sourced:   source install/setup.bash
#   - Run from the colcon workspace root.
#
# What this test does:
#   1. Starts the WebSocket server in ROS_DOMAIN_ID=1
#   2. Starts the WebSocket client in ROS_DOMAIN_ID=2
#   3. Publishes a String on /domain1_to_domain2 in Domain 1 and verifies
#      it arrives in Domain 2  (forward path).
#   4. Publishes a String on /domain2_to_domain1 in Domain 2 and verifies
#      it arrives in Domain 1  (reverse path).
# =============================================================================
set -uo pipefail

TIMEOUT_SEC=30
RESULT=1

cleanup() {
    echo ""
    echo "--- Cleaning up background processes ---"
    jobs -p | xargs -r kill 2>/dev/null || true
    wait 2>/dev/null || true
    if [ "$RESULT" -eq 0 ]; then
        echo ""
        echo "========================================="
        echo " ALL INTEGRATION TESTS PASSED"
        echo "========================================="
    else
        echo ""
        echo "========================================="
        echo " INTEGRATION TESTS FAILED"
        echo "========================================="
    fi
    exit "$RESULT"
}
trap cleanup EXIT INT TERM

echo "============================================="
echo " Cross-Domain Communication Integration Test"
echo "============================================="
echo ""

# ------------------------------------------------------------------
# Step 1 — Start WebSocket server in Domain 1
# ------------------------------------------------------------------
echo "[1/6] Starting WebSocket server (ROS_DOMAIN_ID=1, port 9090)..."
ROS_DOMAIN_ID=1 ros2 run ros2_websocket_proxy generic_server \
    --ros-args -p yaml_file:=test_server_topics.yaml -p port:=9090 &
sleep 3

# ------------------------------------------------------------------
# Step 2 — Start WebSocket client in Domain 2
# ------------------------------------------------------------------
echo "[2/6] Starting WebSocket client (ROS_DOMAIN_ID=2, ws://localhost:9090)..."
ROS_DOMAIN_ID=2 ros2 run ros2_websocket_proxy generic_client \
    --ros-args -p yaml_file:=test_client_topics.yaml \
               -p ws_url:=ws://localhost:9090 &
sleep 5

# ------------------------------------------------------------------
# Step 3 — Test: Domain 1 → Domain 2
# ------------------------------------------------------------------
echo "[3/6] Subscribing on Domain 2 for /domain1_to_domain2 ..."

ROS_DOMAIN_ID=2 timeout "$TIMEOUT_SEC" \
    ros2 topic echo --once /domain1_to_domain2 std_msgs/msg/String \
    > /tmp/ci_test1.txt 2>&1 &
ECHO1_PID=$!
sleep 2

echo "      Publishing on Domain 1 to /domain1_to_domain2 ..."
ROS_DOMAIN_ID=1 ros2 topic pub -r 4 /domain1_to_domain2 \
    std_msgs/msg/String "{data: 'hello_from_domain1'}" > /dev/null 2>&1 &
PUB1_PID=$!

set +e
wait "$ECHO1_PID"
ECHO1_RC=$?
set -e
kill "$PUB1_PID" 2>/dev/null || true

if [ "$ECHO1_RC" -eq 0 ] && grep -q "hello_from_domain1" /tmp/ci_test1.txt; then
    echo "[4/6] PASSED: Domain 1 → Domain 2"
else
    echo "[4/6] FAILED: Domain 1 → Domain 2  (exit code=$ECHO1_RC)"
    echo "--- captured output ---"
    cat /tmp/ci_test1.txt 2>/dev/null || true
    exit 1
fi

echo ""

# ------------------------------------------------------------------
# Step 4 — Test: Domain 2 → Domain 1
# ------------------------------------------------------------------
echo "[5/6] Subscribing on Domain 1 for /domain2_to_domain1 ..."

ROS_DOMAIN_ID=1 timeout "$TIMEOUT_SEC" \
    ros2 topic echo --once /domain2_to_domain1 std_msgs/msg/String \
    > /tmp/ci_test2.txt 2>&1 &
ECHO2_PID=$!
sleep 2

echo "      Publishing on Domain 2 to /domain2_to_domain1 ..."
ROS_DOMAIN_ID=2 ros2 topic pub -r 4 /domain2_to_domain1 \
    std_msgs/msg/String "{data: 'hello_from_domain2'}" > /dev/null 2>&1 &
PUB2_PID=$!

set +e
wait "$ECHO2_PID"
ECHO2_RC=$?
set -e
kill "$PUB2_PID" 2>/dev/null || true

if [ "$ECHO2_RC" -eq 0 ] && grep -q "hello_from_domain2" /tmp/ci_test2.txt; then
    echo "[6/6] PASSED: Domain 2 → Domain 1"
else
    echo "[6/6] FAILED: Domain 2 → Domain 1  (exit code=$ECHO2_RC)"
    echo "--- captured output ---"
    cat /tmp/ci_test2.txt 2>/dev/null || true
    exit 1
fi

RESULT=0

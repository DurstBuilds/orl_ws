#!/bin/bash
# Bring up can0 with 1 Mbit/s and auto-restart after bus-off (recommended for MCP251x / Pi).
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-1000000}"
RESTART_MS="${3:-100}"

echo "Configuring ${IFACE}: bitrate=${BITRATE} restart-ms=${RESTART_MS}"
sudo ip link set "${IFACE}" down 2>/dev/null || true
sudo ip link set "${IFACE}" type can bitrate "${BITRATE}" restart-ms "${RESTART_MS}"
sudo ip link set "${IFACE}" up
ip -details link show "${IFACE}" | grep -E 'state|can state|restart-ms|bitrate'

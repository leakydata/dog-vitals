#!/usr/bin/env bash
# Start the UI.
#
# The BLE link lives in the server, not the browser -- see tools/server.py for
# why. Any browser works; no Web Bluetooth, no permission prompt, no device
# picker, and the stream survives a page reload because the radio connection
# belongs to this process rather than to the tab.
set -euo pipefail
cd "$(dirname "$0")/.."
PORT="${1:-8420}"
exec python3 tools/server.py --port "$PORT"

#!/usr/bin/env bash
# Bootstrap a local venv (pyserial) and launch the ZhiWatch USB viewer.
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=.venv-viewer
if [ ! -x "$VENV/bin/python" ]; then
    python3 -m venv "$VENV"
fi
if ! "$VENV/bin/python" -c "import serial" >/dev/null 2>&1; then
    "$VENV/bin/pip" install --quiet pyserial
fi
exec "$VENV/bin/python" tools/zhiwatch_viewer.py "$@"

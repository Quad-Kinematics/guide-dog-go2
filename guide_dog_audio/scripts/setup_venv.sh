#!/bin/bash
# Creates <workspace>/venv with the pinned packages from requirements.txt
# (gTTS for tts_node, go2_webrtc_driver for go2_rtc_keepalive). Needs internet
# (PyPI) and a Python >= 3.10: PYTHON=..., default pyenv's 3.11.9.
# Usage: src/guide_dog_audio/scripts/setup_venv.sh
set -e
PKG_DIR="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
WS="$(cd "$PKG_DIR/../.." && pwd)"
VENV="$WS/venv"
PY="${PYTHON:-$HOME/.pyenv/versions/3.11.9/bin/python3.11}"
if [[ ! -x "$PY" ]]; then
    echo "Python not found: $PY (set PYTHON to a Python >= 3.10)" >&2
    exit 1
fi
"$PY" -m venv "$VENV"
touch "$VENV/COLCON_IGNORE"   # keep colcon from crawling site-packages
"$VENV/bin/python" -m pip install --upgrade pip
"$VENV/bin/python" -m pip install -r "$PKG_DIR/requirements.txt"
"$VENV/bin/python" -c "import gtts, go2_webrtc_driver.webrtc_driver; print('venv ok:', '$VENV')"

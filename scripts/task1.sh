#!/bin/bash
# task1.sh —— 启动 scripts/task/task1.py

SCRIPT_DIR="$(dirname "$0")"
python3 "$SCRIPT_DIR/task/task1.py" "$@"

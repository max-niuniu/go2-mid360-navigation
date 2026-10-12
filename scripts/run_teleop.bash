#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
source "${SCRIPT_DIR}/setup_env.bash"

exec python3 -m go2_mid360_panel.keyboard_input "$@"

#!/usr/bin/env bash

# Regenerate the production web monitor bundled by xbot2_gui_server.
#
# This script can be launched from any working directory. Vite writes the
# generated HTML, JavaScript, CSS, and protobuf schemas to:
#   server/src/xbot2_gui_server/monitor/
#
# Usage:
#   ./regenerate_web.bash
#   ./regenerate_web.bash --clean-install  # reinstall exactly from package-lock.json

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="${SCRIPT_DIR}/web"

if ! command -v npm >/dev/null 2>&1; then
    echo "error: npm was not found; install a current Node.js release first" >&2
    exit 1
fi

cd "${WEB_DIR}"

# npm ci is deterministic: it installs the exact versions recorded in
# package-lock.json. Avoid doing that work on every rebuild unless explicitly
# requested or node_modules has not been created yet.
if [[ "${1:-}" == "--clean-install" ]] || [[ ! -d node_modules ]]; then
    echo "Installing locked web dependencies..."
    npm ci
elif [[ -n "${1:-}" ]]; then
    echo "error: unknown option '${1}'" >&2
    echo "usage: $0 [--clean-install]" >&2
    exit 2
fi

echo "Building the XBot2 web monitor..."
npm run build:server

echo
echo "Web monitor regenerated in:"
echo "  ${SCRIPT_DIR}/server/src/xbot2_gui_server/monitor"

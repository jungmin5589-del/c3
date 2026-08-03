#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[안내] v1.9에서는 통합 멀티 AMR 실행기로 연결됩니다."
exec "$SCRIPT_DIR/run_complete_system.sh"

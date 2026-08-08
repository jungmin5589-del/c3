#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$ROOT/04_run_ocr_mission_robot.sh" amr1 "${1:-}"

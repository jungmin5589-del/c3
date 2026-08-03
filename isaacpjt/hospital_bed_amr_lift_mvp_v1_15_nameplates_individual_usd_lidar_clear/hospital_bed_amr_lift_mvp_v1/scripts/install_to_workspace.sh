#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${1:-$HOME/cobot3_ws/src/hospital_bed_amr_lift_mvp_v1}"

mkdir -p "$(dirname "$TARGET")"
if [[ -e "$TARGET" ]]; then
  BACKUP="${TARGET}_backup_$(date +%Y%m%d_%H%M%S)"
  mv "$TARGET" "$BACKUP"
  echo "[백업] 기존 폴더: $BACKUP"
fi
cp -a "$PROJECT_DIR" "$TARGET"
echo "[설치 완료] $TARGET"

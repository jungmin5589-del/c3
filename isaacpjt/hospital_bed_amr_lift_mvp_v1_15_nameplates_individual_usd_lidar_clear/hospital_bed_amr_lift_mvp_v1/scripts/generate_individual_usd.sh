#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_DIR="$PROJECT_DIR/individual_usd_assets"
OUT="$PROJECT_DIR/output/individual_usd"

if [[ ! -d "$SOURCE_DIR" ]]; then
  echo "[오류] 개별 USD 자산 폴더가 없습니다: $SOURCE_DIR" >&2
  exit 1
fi

rm -rf "$OUT"
mkdir -p "$OUT"
cp -a "$SOURCE_DIR/." "$OUT/"

echo "[완료] 개별 USD 복사: $OUT"
echo "  - AMR1.usd / AMR2.usd"
echo "  - HospitalBed_SeoSuwon.usd"
echo "  - HospitalBed_KimSeoul.usd"
echo "  - HospitalBed_ParkIncheon.usd"

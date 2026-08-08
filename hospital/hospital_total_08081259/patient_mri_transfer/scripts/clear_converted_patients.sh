#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rm -f "$ROOT/assets/patient_breathless/converted/"*.usd "$ROOT/assets/patient_breathless/converted/"*.usda "$ROOT/assets/patient_breathless/converted/"*.usdc 2>/dev/null || true
rm -f "$ROOT/assets/patient_sleeping/converted/"*.usd "$ROOT/assets/patient_sleeping/converted/"*.usda "$ROOT/assets/patient_sleeping/converted/"*.usdc 2>/dev/null || true
echo "[PASS] 기존 변환 USD 삭제 완료. 다음 실행 때 FBX를 다시 변환합니다."

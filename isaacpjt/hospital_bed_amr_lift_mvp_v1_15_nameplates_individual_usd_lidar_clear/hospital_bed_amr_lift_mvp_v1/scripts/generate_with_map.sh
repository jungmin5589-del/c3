#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "사용법: $0 /absolute/path/to/hospital_map.usd_or_glb" >&2
  exit 2
fi
MAP_PATH="$(realpath "$1")"
if [[ ! -f "$MAP_PATH" ]]; then
  echo "[오류] 맵 파일이 없습니다: $MAP_PATH" >&2
  exit 1
fi
export HOSPITAL_MAP_USD="$MAP_PATH"
"$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/generate_all.sh"

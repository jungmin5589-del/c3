#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-}"
if [[ "$MODE" != "rviz" && "$MODE" != "slam" ]]; then
  echo "사용법: $0 rviz|slam" >&2
  echo "rviz: world -> amrX/odom 임시 정적 TF 사용" >&2
  echo "slam: world -> odom 비활성화, SLAM/AMCL의 map -> odom 허용" >&2
  exit 2
fi
python3 - "$PROJECT_DIR/config/amr_config.json" "$MODE" <<'PY2'
import json, sys
from pathlib import Path
path = Path(sys.argv[1])
mode = sys.argv[2]
data = json.loads(path.read_text(encoding='utf-8'))
data.setdefault('navigation', {})['publish_world_to_odom_for_rviz'] = mode == 'rviz'
path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(f"[완료] TF mode={mode}, publish_world_to_odom_for_rviz={mode == 'rviz'}")
PY2

#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export ROS_DOMAIN_ID=117
exec python3 "$ROOT/patient_transport_manager.py" "$@"

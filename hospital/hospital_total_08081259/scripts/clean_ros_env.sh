#!/usr/bin/env bash
# Remove Isaac Sim libraries before using the system ROS 2 Python 3.10 environment.
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-/home/peter-msi/isaacsim-5.1.0}"
filter_path() {
  local input="${1:-}" item out=""
  IFS=':' read -r -a items <<< "$input"
  for item in "${items[@]}"; do
    [[ -z "$item" ]] && continue
    case "$item" in
      "$ISAAC_SIM_DIR"|"$ISAAC_SIM_DIR"/*) continue ;;
    esac
    out="${out:+$out:}$item"
  done
  printf '%s' "$out"
}
export PYTHONPATH="$(filter_path "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(filter_path "${LD_LIBRARY_PATH:-}")"
export PATH="$(filter_path "${PATH:-}")"
unset PYTHONHOME || true

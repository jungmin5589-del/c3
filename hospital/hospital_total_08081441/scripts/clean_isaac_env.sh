#!/usr/bin/env bash
# Remove system ROS Python 3.10 paths before starting Isaac Sim Python 3.11.
filter_path() {
  local input="${1:-}" item out=""
  IFS=':' read -r -a items <<< "$input"
  for item in "${items[@]}"; do
    [[ -z "$item" ]] && continue
    case "$item" in
      /opt/ros/*|*/site-packages/paddle*|*/hospital_ocr_ros310/*) continue ;;
    esac
    out="${out:+$out:}$item"
  done
  printf '%s' "$out"
}
export PYTHONPATH="$(filter_path "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(filter_path "${LD_LIBRARY_PATH:-}")"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONHOME || true
unset ROS_VERSION ROS_PYTHON_VERSION || true

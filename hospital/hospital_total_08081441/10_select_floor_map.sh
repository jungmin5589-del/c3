#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="$ROOT/ros2_ws/src/hospital_nav2/maps"
INSTALL_MAP_DIR="$ROOT/ros2_ws/install/hospital_nav2/share/hospital_nav2/maps"

FLOOR="${1:-}"

case "$FLOOR" in
    1|1f|1F)
        FLOOR_NAME="1f"
        ;;
    2|2f|2F)
        FLOOR_NAME="2f"
        ;;
    *)
        echo "Usage: $0 1f"
        echo "       $0 2f"
        exit 1
        ;;
esac

SOURCE_PNG="$MAP_DIR/hospital_map_${FLOOR_NAME}.png"
SOURCE_YAML="$MAP_DIR/hospital_map_${FLOOR_NAME}.yaml"

if [[ ! -f "$SOURCE_PNG" ]]; then
    echo "[ERROR] Missing: $SOURCE_PNG"
    exit 1
fi

if [[ ! -f "$SOURCE_YAML" ]]; then
    echo "[ERROR] Missing: $SOURCE_YAML"
    exit 1
fi

cp "$SOURCE_PNG" "$MAP_DIR/hospital_map.png"
cp "$SOURCE_YAML" "$MAP_DIR/hospital_map.yaml"

sed -i 's|^image:.*|image: hospital_map.png|' "$MAP_DIR/hospital_map.yaml"

if [[ -d "$INSTALL_MAP_DIR" && ! -L "$INSTALL_MAP_DIR" ]]; then
    cp -f "$MAP_DIR/hospital_map.png" "$INSTALL_MAP_DIR/hospital_map.png" 2>/dev/null || true
    cp -f "$MAP_DIR/hospital_map.yaml" "$INSTALL_MAP_DIR/hospital_map.yaml" 2>/dev/null || true
fi

echo "========================================"
echo "SELECTED FLOOR: $FLOOR_NAME"
echo "MAP: $MAP_DIR/hospital_map.png"
echo "YAML: $MAP_DIR/hospital_map.yaml"
echo "Restart Nav2 after switching floors."
echo "========================================"

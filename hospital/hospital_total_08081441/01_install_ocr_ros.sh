#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${OCR_ROS_VENV:-$HOME/.venvs/hospital_ocr_ros310}"

sudo apt update
sudo apt install -y python3-venv python3-pip python3-colcon-common-extensions ros-humble-launch-ros

mkdir -p "$(dirname "$VENV")"
if [[ ! -x "$VENV/bin/python" ]]; then
  python3 -m venv --system-site-packages "$VENV"
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set +u
# shellcheck disable=SC1090
source "$VENV/bin/activate"
python -m pip install --upgrade pip setuptools wheel
python -m pip install --force-reinstall -r "$ROOT/requirements_ocr_ros.txt"

export PADDLE_PDX_MODEL_SOURCE=BOS
python - <<'PY'
import sys
import numpy, cv2, paddle, paddlex, paddleocr, rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from paddleocr import TextRecognition
print("Python:", sys.version)
print("NumPy:", numpy.__version__)
print("OpenCV:", cv2.__version__)
print("Paddle:", paddle.__version__)
print("PaddleX:", paddlex.__version__)
print("PaddleOCR:", paddleocr.__version__)
print("ROS messages: OK", Image, String)
print("TextRecognition import: OK", TextRecognition)
print("Downloading/loading Korean model once...")
_ = TextRecognition(model_name="korean_PP-OCRv5_mobile_rec", device="cpu", cpu_threads=4)
print("Korean OCR model cache: READY")
PY

echo "[DONE] OCR ROS venv: $VENV"
echo "Next: cd $ROOT && ./02_build_ros_ws.sh"

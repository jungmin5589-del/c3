# v1.14 — AMR당 360도 LiDAR 1대 + 전·후방 RGB-D 카메라 + 침대 양쪽 진입구

## 센서 구성

각 AMR에 다음 센서를 배치한다.

- 중앙 상단 360도 RTX LiDAR 1대
- 전방 RGB-D 카메라 1대
- 후방 RGB-D 카메라 1대

LiDAR는 `Example_Rotary_2D`를 사용하며 AMR 중앙 `x=0, y=0, z=0.245 m`에 배치한다. Viewport에서 쉽게 찾을 수 있도록 청록색 받침대와 하우징을 추가했다.

카메라는 AMR 전·후단에 배치하며 두 AMR 모두 기본 활성화된다. GPU 부하를 줄이기 위해 기본값은 640×360, 15 Hz이다.

## ROS 2 토픽

AMR1:

```text
/amr1/scan
/amr1/point_cloud
/amr1/camera/front/color/image_raw
/amr1/camera/front/depth/image_raw
/amr1/camera/front/camera_info
/amr1/camera/rear/color/image_raw
/amr1/camera/rear/depth/image_raw
/amr1/camera/rear/camera_info
```

AMR2는 `/amr2/...`로 동일하다.

## TF

```text
amr1/base_link
├─ amr1/lidar_link
├─ amr1/front_camera_link
└─ amr1/rear_camera_link
```

AMR2도 같은 구조를 사용한다.

## SLAM 사용 원칙

SLAM Toolbox에는 기존과 같이 AMR당 하나의 `/scan`만 입력한다. 전·후방 카메라는 도킹, 사람 감지, 후진 안전 확인용으로 사용하며 LiDAR SLAM 구조를 변경하지 않는다.

## 침대 양쪽 가운데 개방

기존 GLB는 후방 하단 가운데 횡봉만 제거되어 있었다. v1.14는 동일한 영역을 전방에도 대칭으로 제거했다.

- 기본 에셋: `assets/bed_amr_clear.glb`
- 명시적 파일: `assets/bed_amr_clear_front_rear_open.glb`
- 이전 후방 전용 파일은 비교용으로 유지한다.

전방 숨김 Collider도 하나의 막힌 박스에서 좌·우 기둥과 상부 빔으로 분할해 하단 중앙 진입 공간을 비웠다. 따라서 AMR가 침대의 어느 쪽 끝에서도 리프트 어댑터 아래로 접근할 수 있다.

## 생성과 실행

```bash
./scripts/generate_all.sh
./scripts/run_complete_system.sh
```

기존 USD에는 변경 사항이 자동 반영되지 않으므로 v1.14 적용 후 반드시 Stage를 다시 생성해야 한다.

## 카메라 캡처

AMR1 전·후방 카메라를 모두 저장:

```bash
./scripts/capture_camera.sh
```

직접 선택:

```bash
python.sh scripts/capture_camera.py \
  --config config/amr_config.json \
  --stage output/hospital_bed_amr_multi_map_ready.usd \
  --output-dir output/sensor_capture \
  --amr AMR1 \
  --camera both
```

## 검증 범위

Python 문법, JSON, Shell 문법, GLB 메시 구조 및 전·후방 개방 면 삭제는 로컬 검증한다. RTX LiDAR 렌더링, 카메라 ROS 발행, 실제 360도 스캔 사각지대와 침대 진입 물리는 Isaac Sim 5.1에서 최종 확인해야 한다.

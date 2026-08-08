# 프로젝트 4 최종 테스트본

## 동작 범위

이번 버전은 다음 시나리오만 수행합니다.

```text
텔레옵으로 김서울 이름표 앞까지 이동
→ O 키 1회
→ AMR1 전면 이미지 10프레임을 OCR launch 노드가 수신
→ 이름 + 생년월일을 후보 3명과 비교
→ 김서울 / 2000-11-02 검증
→ 이름표 사각형 bbox 중심 X를 계속 발행
→ Isaac Sim이 Q/E와 같은 횡이동으로 화면 중심 X와 일치시킴
→ X 정렬이 안정되면 1.0m 전진
→ 정지
```

사용하지 않는 기능:

- 가장 가까운 침대 계산
- 화면 Y 중심 검사
- Isaac Sim 내부 PaddleOCR
- `third_party`
- 2초 간격 AMR 속도·상태 반복 출력

## 중앙 정렬 기준

카메라 해상도는 640×360입니다.

```text
화면 중심 X = 640 / 2 = 320
이름표 중심 X = bbox_x + bbox_width / 2
오차 = 이름표 중심 X - 화면 중심 X
```

- 오차가 허용 범위 밖이면 Q/E와 같은 횡이동을 수행합니다.
- 허용 범위는 `±12 px`입니다.
- 동일한 프레임을 3회 세는 것이 아니라, OCR 노드가 새로 발행한 `TRACKING` 메시지 3개가 연속으로 중앙 범위에 들어와야 합니다.
- Y 좌표는 결과 메시지에 포함될 수 있지만 Isaac Sim 제어에는 사용하지 않습니다.
- 미션 Pose에서 카메라 높이와 로봇 방향이 고정된다는 시나리오를 전제로 합니다.

## 사용 토픽 — AMR1

| 방향 | 토픽 | 타입 | 용도 |
|---|---|---|---|
| Isaac → OCR | `/amr1/camera/front/color/image_raw` | `sensor_msgs/msg/Image` | 전면 카메라 영상 |
| Isaac → OCR | `/amr1/ocr/request` | `std_msgs/msg/String` | O 키 검증 요청 |
| OCR → Isaac | `/amr1/ocr/result` | `std_msgs/msg/String` | OCR 결과와 bbox 중심 X |
| Isaac → OCR | `/amr1/ocr/control` | `std_msgs/msg/String` | 추적 종료 |
| Isaac → 외부 | `/amr1/align/status` | `std_msgs/msg/String` | 자동 접근 상태 |

## 최초 준비

```bash
cd /home/rokey/hospital_bed_amr_project
chmod +x *.sh scripts/*.sh
./check_project.sh
```

현재 가상환경을 확인합니다.

```bash
/home/rokey/.venvs/hospital_ocr_ros310/bin/python -c \
"import sys, paddleocr; print(sys.executable); print(paddleocr.__version__)"
```

`3.1.1`이 나오면 설치를 다시 할 필요가 없습니다. import가 실패할 때만 실행합니다.

```bash
cd /home/rokey/hospital_bed_amr_project
./01_install_ocr_ros.sh
```

이번 ZIP의 소스가 바뀌었으므로 ROS2 패키지는 반드시 다시 빌드합니다.

```bash
cd /home/rokey/hospital_bed_amr_project
./02_build_ros_ws.sh
```

빌드 마지막에 다음이 보여야 합니다.

```text
[FIXED] shebang: #!/home/rokey/.venvs/hospital_ocr_ros310/bin/python
[PASS] OCR package import
[DONE] ROS 2 OCR package built
```

## 직접 테스트 — 터미널 2개

### 터미널 1: Isaac Sim

이 터미널에서는 `/opt/ros/humble/setup.bash`를 직접 source하지 않습니다.

```bash
cd /home/rokey/hospital_bed_amr_project
./03_run_isaac.sh
```

Isaac Sim이 열리면 Viewport를 한 번 클릭합니다.

### 터미널 2: AMR1 OCR launch

```bash
cd /home/rokey/hospital_bed_amr_project
./04_run_ocr_amr1.sh
```

아래 로그가 나온 뒤에만 O를 누릅니다.

```text
[amr1] OCR model ready
[amr1] image SUB   : /amr1/camera/front/color/image_raw
[amr1] request SUB : /amr1/ocr/request
[amr1] result PUB  : /amr1/ocr/result
```

## 실제 시나리오 시험 순서

1. 터미널 1의 Isaac Viewport를 클릭합니다.
2. `W/S/A/D/Q/E`로 AMR1을 김서울 이름표 앞까지 이동합니다.
3. 이름표가 전면 카메라에 완전히 보이도록 합니다.
4. X 자동 정렬을 눈으로 확인하려면 이름표를 일부러 화면 중앙보다 조금 왼쪽이나 오른쪽에 둡니다.
5. 모든 이동 키에서 손을 뗍니다.
6. `O`를 **한 번만** 누릅니다.
7. 터미널 2에서 10프레임 수집과 김서울 검증을 확인합니다.
8. 검증에 성공하면 터미널 1에서 `ALIGN_X Q-equivalent` 또는 `ALIGN_X E-equivalent` 로그가 한 번 나옵니다.
9. AMR1이 옆으로 움직이며 이름표 사각형 중심 X를 화면 중심 X에 맞춥니다.
10. `±12px` 범위에 새 tracking 결과가 연속 3회 들어오면 1.0m 전진합니다.
11. 1.0m 이동 후 정지합니다.

## 정상 로그

터미널 1:

```text
[AMR1] AUTO=WAITING_OCR — 10-frame OCR request published
[AMR1] OCR request: expected=김서울 2000-11-02 request_id=...
[AMR1] AUTO=ALIGNING_X — patient verified; rectangular nameplate X tracking started
[AMR1] VERIFIED: 김서울 2000-11-02 score=...
[AMR1] ALIGN_X Q-equivalent: plate_x=..., screen_x=320.0, error=...px
```

또는 반대 방향이면:

```text
[AMR1] ALIGN_X E-equivalent: ...
```

중앙에 들어오면:

```text
[AMR1] ALIGN_X inside tolerance: plate_x=..., screen_x=320.0, error=...px
[AMR1] AUTO=FORWARD_1M — nameplate X centred for 3 tracking messages
[AMR1] AUTO=COMPLETE — moved 1.000 m
```

터미널 2:

```text
[amr1] request=... expected=김서울 2000-11-02; collecting 10 frames
[amr1] 10 frames captured; OCR worker started
[amr1] VERIFIED 김서울 2000-11-02 score=...; tracking started
[amr1] tracking stopped by Isaac command
```

## 불필요한 반복 로그 제거

이전의 다음 출력은 완전히 제거했습니다.

```text
AMR1: v=(...) AUTO=... | AMR2: v=(...) AUTO=...
```

정상 실행에서는 상태가 바뀔 때와 정렬 방향이 바뀔 때만 핵심 로그가 출력됩니다.

## 횡이동 방향이 반대일 때

실제 카메라/AMR 축 방향에 따라 최초 한 번 부호 확인이 필요할 수 있습니다. 이름표가 화면 오른쪽에 있는데 AMR이 더 잘못된 방향으로 움직이면 `SPACE`로 정지하고 다음 파일을 엽니다.

```bash
nano /home/rokey/hospital_bed_amr_project/config/isaac_config.json
```

AMR1의 값을:

```json
"image_error_to_lateral_sign": -1.0
```

다음처럼 바꿉니다.

```json
"image_error_to_lateral_sign": 1.0
```

저장 후 터미널 1의 Isaac Sim만 다시 실행하면 됩니다. ROS2 재빌드는 필요 없습니다.

## OCR 실패

OCR 증거가 없고 후보 점수가 모두 0일 때 첫 후보인 서수원을 선택하지 않습니다.

```text
REJECTED selected=NONE NONE score=0.0
```

실패하면 AMR은 정렬하거나 전진하지 않습니다.

---

## Nav2 추가판 안내

이 폴더에는 AMR1 Nav2 + RViz2 기능이 추가되어 있습니다. 처음 실행하기 전에 `README_NAV2_KO.md`와 `FINAL_RUN_COMMANDS_NAV2_KO.txt`를 먼저 확인하세요.

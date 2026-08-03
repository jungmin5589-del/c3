# v1.15 업데이트

# v1.15 이름표 + 개별 USD + LiDAR 하우징 수정

- 침대 3대 각각 중앙 이름표 Plane/OmniPBR 적용
- Bed1 서수원, Bed2 김서울, Bed3 박인천
- 이름표 Y=0.0으로 양쪽 바퀴 정중앙 정렬
- LiDAR는 Example_Rotary_2D 유지
- lidar_link를 바닥 Z=0.285 m로 배치
- 하우징 전체를 스캔 평면 아래로 이동해 자가 가림 제거
- AMR1/AMR2, 침대 3대 개별 USD 출력 스크립트 추가

> v1.14.1: front/rear ROS 2 camera runtime publishing fix (double initialization + SDG warm-up).

# 병원 침대 이송 AMR v1.14 — 360도 LiDAR + 전·후방 RGB-D 카메라 + 양쪽 침대 진입구

## 구성

- AMR 2대
  - `/World/AMR1`, ROS 2 namespace `/amr1`
  - `/World/AMR2`, ROS 2 namespace `/amr2`
- 병원 침대 3대
  - `/World/HospitalBed1`
  - `/World/HospitalBed2`
  - `/World/HospitalBed3`
- 선택적 병원 맵 Reference
  - `/World/HospitalMap`
- AMR별 리프트, 옴니 주행, 차량식 곡선 회전, 자기식 근접 정렬·FixedJoint 결합
- AMR별 `/cmd_vel`, 리프트, 자석, E-stop ROS 2 명령
- AMR당 중앙 360도 RTX LiDAR 1대, Odometry, TF 및 공유 /clock
- AMR당 전방 RGB-D 카메라 1대와 후방 RGB-D 카메라 1대
- AMR1·AMR2 모두 카메라 2대 기본 활성화
- 침대 전·후방 하단 가운데 개방 및 전방 Collider 분할

## 기본 배치

```text
AMR1: (-2.0, -1.25)
AMR2: (-2.0,  1.25)

Bed1: (1.5, -2.0)
Bed2: (1.5,  0.0)
Bed3: (1.5,  2.0)
```

모든 위치는 `config/amr_config.json`의 `fleet`, `beds`에서 변경할 수 있다.

## 생성

맵 없이 AMR 2대와 침대 3대만 생성:

```bash
cd "$HOME/Downloads/hospital_bed_amr_lift_mvp_v1"
./scripts/generate_all.sh
```

결과:

```text
output/hospital_bed_amr_multi_map_ready.usd
```

## 병원 맵과 결합

병원 맵이 USD 또는 GLB 형식일 때:

```bash
./scripts/generate_with_map.sh "/절대경로/hospital_map.usd"
# 또는 hospital_map.glb
```

또는:

```bash
export HOSPITAL_MAP_USD="/절대경로/hospital_map.usd"
./scripts/generate_all.sh
```

GLB 맵은 먼저 USD로 자동 변환되고, 맵은 `/World/HospitalMap` 아래에 Reference된다. 맵 자체에 바닥 Collider가 있다면 v1.14 기본 설정은 별도의 GroundPlane을 생성하지 않는다.

## 실행

```bash
./scripts/run_complete_system.sh
```

## 키보드 조작

Viewport를 한 번 클릭한 뒤 조작한다.

두 AMR는 선택 전환 없이 동시에 조작할 수 있다.

```text
[AMR1]
W / S              전진 / 후진
A / D              좌회전 / 우회전
Q / E              왼쪽 / 오른쪽 옴니 횡이동
R / V              리프트 상승 / 하강
Left Shift         자기식 결합 ON/OFF 토글
Space              비상 정지

[AMR2]
↑ / ↓              전진 / 후진
← / →              좌회전 / 우회전
/ + ,              왼쪽 옴니 횡이동
/ + .              오른쪽 옴니 횡이동
[ / ]              리프트 상승 / 하강
Right Shift        자기식 결합 ON/OFF 토글
Enter              비상 정지
```

동시 조작 예시:

```text
W + ↑              AMR1과 AMR2가 동시에 전진
W + D + ↑ + ←      AMR1은 전진 우회전, AMR2는 전진 좌회전
Q + / + .         AMR1은 왼쪽 횡이동, AMR2는 오른쪽 횡이동
```

## 속도

```text
미적재 전후       1.70 m/s
미적재 횡이동     1.70 m/s
미적재 회전       2.80 rad/s

적재 전후         0.90 m/s
적재 횡이동       0.76 m/s
적재 회전         1.30 rad/s
```

v1.14는 v1.13의 완전 고정 결합과 고속 프로파일을 유지하면서 중앙 360도 LiDAR, 전·후방 카메라, 침대 양쪽 진입구를 추가했다.

> 주의: 이 속도는 시뮬레이션 기능 검증용이다. 침대를 적재한 상태에서는 좁은 병실, 도킹, 사람 주변에서 별도의 저속·충돌 감속 계층을 사용해야 한다.

## 침대 리프트 어댑터 시각 연결 보강

기존의 리프트 레일·행거 위에 다음 가시 구조물을 추가했다.

- 전·후방 횡방향 Connection Beam 2개
- 좌·우 Mount Plate 4개
- Bed rigid body 아래에 포함되므로 침대와 함께 움직임
- 시각용 비충돌 구조이므로 AMR 진입 공간과 기존 도킹 물리는 유지

생성된 Stage에서 `HospitalBed*/LiftAdapter/ConnectionBeam*`, `MountPlate*` Prim으로 확인할 수 있다.

## ROS 2 제어

AMR1 전진:

```bash
ros2 topic pub -r 10 /amr1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

AMR2 오른쪽 횡이동:

```bash
ros2 topic pub -r 10 /amr2/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: -0.6, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

AMR2 전진 우회전:

```bash
ros2 topic pub -r 10 /amr2/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.6}}"
```

리프트 25 mm:

```bash
ros2 topic pub --once /amr1/lift_cmd std_msgs/msg/Float64 "{data: 0.025}"
```

자석 ON/OFF:

```bash
ros2 topic pub --once /amr1/magnet_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /amr1/magnet_cmd std_msgs/msg/Bool "{data: false}"
```

비상 정지:

```bash
ros2 topic pub --once /amr1/estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /amr1/estop std_msgs/msg/Bool "{data: false}"
```

## AMR별 주요 토픽

```text
/amr1/cmd_vel                 /amr2/cmd_vel
/amr1/lift_cmd                /amr2/lift_cmd
/amr1/magnet_cmd              /amr2/magnet_cmd
/amr1/magnet_strength         /amr2/magnet_strength
/amr1/estop                   /amr2/estop
/amr1/odom                    /amr2/odom
/amr1/bed_attached            /amr2/bed_attached
/amr1/scan                    /amr2/scan
/amr1/point_cloud             /amr2/point_cloud
/amr1/camera/front/color/image_raw
/amr1/camera/front/depth/image_raw
/amr1/camera/rear/color/image_raw
/amr1/camera/rear/depth/image_raw
/amr2/camera/front/color/image_raw
/amr2/camera/front/depth/image_raw
/amr2/camera/rear/color/image_raw
/amr2/camera/rear/depth/image_raw
```

확인:

```bash
./scripts/show_fleet_topics.sh
```

키보드 teleop:

```bash
./scripts/teleop_amr1.sh
./scripts/teleop_amr2.sh
```

두 개의 별도 터미널에서 각각 실행하면 두 AMR에 독립적으로 명령할 수 있다.

## TF 구조

```text
world
├─ amr1/odom
│  └─ amr1/base_link
│     ├─ amr1/lidar_link
│     ├─ amr1/front_camera_link
│     └─ amr1/rear_camera_link
└─ amr2/odom
   └─ amr2/base_link
      ├─ amr2/lidar_link
      ├─ amr2/front_camera_link
      └─ amr2/rear_camera_link
```

RViz Fixed Frame을 `world`로 설정하면 두 로봇을 같은 화면에서 볼 수 있다.

## 자기식 결합 — wheel_tow 기본 모드

각 AMR은 침대 3대 중 가장 가까우며 다른 AMR이 사용하지 않는 침대를 선택한다.

```text
38 cm 이내       근접 정렬 시작
5.5 cm 이내      결합 거리 조건
방향 오차 6도    결합 방향 조건
리프트 약 2.5 mm 어댑터 틈만 닫음
침대 바퀴        바닥에 계속 접지
```

기본 `wheel_tow` 모드에서는 침대를 들어 올리지 않는다. 리프트와 침대 사이에 파단 가능한 FixedJoint를 만들고, 침대 캐스터에는 저마찰 재질을 적용해 AMR가 전체 침대 무게가 아니라 주로 구름 저항을 이기도록 근사한다.

자기력은 10~100% 범위로 조절할 수 있다. 세기가 커지면 정렬 보조와 Joint의 파단 힘·토크가 증가한다.

```bash
ros2 topic pub --once /amr1/magnet_strength std_msgs/msg/Float64 "{data: 70.0}"
ros2 topic pub --once /amr2/magnet_strength std_msgs/msg/Float64 "{data: 50.0}"
```

기존처럼 침대를 들어 운반하려면 `config/amr_config.json`에서 다음처럼 바꾼다.

```json
"coupling_mode": "lift_carry"
```

`/amr1/bed_attached`, `/amr2/bed_attached`에서 결합된 침대 Prim 경로를 확인할 수 있다. 자세한 내용은 `docs/V1_12_MAGNETIC_WHEEL_TOW.md`를 참고한다.

## 카메라 단독 캡처

```bash
./scripts/capture_camera.sh
```

AMR1의 전방·후방 카메라 RGB와 Depth를 `output/sensor_capture`에 저장한다. `--amr AMR2`와 `--camera front|rear|both` 옵션도 사용할 수 있다.

## 중요한 한계

- 옴니휠 외형과 바퀴 회전은 구현되어 있지만, 안정적인 MVP를 위해 차체 속도를 직접 제어한다.
- 실제 메카넘 롤러 접촉력만으로 차체가 움직이는 완전한 접촉 물리 모델은 아니다.
- wheel_tow는 실제 전자기장 해석이 아니라 정렬 보조와 파단 가능한 Joint를 이용한 근사 모델이다.
- 침대 캐스터는 실제 회전·조향 Joint가 아니라 저마찰 WheelProxy이다.
- Odometry는 Isaac Sim의 Ground Truth pose 기반이다.
- 맵 Reference가 충돌체를 자동 생성하지는 않는다. 맵 USD 안에 Collider가 있어야 한다.
- LiDAR 2대와 RGB-D 카메라 4대는 GPU 사용량을 높인다. 기본 카메라는 640×360, 15 Hz이며 성능이 부족하면 해상도·주기를 낮추거나 특정 AMR 카메라를 비활성화한다.
- 이 패키지는 Python·JSON·Shell 문법과 파일 구성을 검증했으며, Isaac Sim 5.1 실제 런타임 검증은 사용자 환경에서 필요하다.

## 자율주행 준비 1~6단계

이 패키지는 다음 인터페이스를 포함한다.

```text
/clock
/tf
/tf_static
/amr1/odom        /amr2/odom
/amr1/scan        /amr2/scan
/amr1/cmd_vel     /amr2/cmd_vel
```

검사:

```bash
./scripts/check_nav_stage_1_to_6.sh
```

ROS 속도 명령 시험:

```bash
./scripts/test_cmd_vel.sh amr1 forward 2
./scripts/test_cmd_vel.sh amr2 right 2
```

SLAM을 시작하기 전에는 TF 부모 충돌을 막기 위해:

```bash
./scripts/set_tf_mode.sh slam
```

을 실행한 뒤 통합 시스템을 재시작한다. 자세한 내용은 `docs/NAVIGATION_STAGE_1_TO_6.md`를 참고한다.


## v1.13 철컥 완전 고정 결합

- Player 1: `Left Shift` 준비, `C` 결합, `X` 해제
- Player 2: `Right Shift` 준비, `\` 결합, `Backspace` 해제
- 결합 시 침대가 AMR 중심으로 스냅되고 파단되지 않는 FixedJoint가 생성된다.
- 자세한 내용: `docs/V1_13_HARD_SNAP_MAGNETIC_LOCK.md`


## v1.14 센서 및 침대 양쪽 개방

- AMR당 `Example_Rotary_2D` 360도 LiDAR 1대를 중앙 `z=0.245 m`에 배치했다.
- Viewport에서 잘 보이도록 청록색 LiDAR 받침대와 하우징을 추가했다.
- AMR당 전방·후방 RGB-D 카메라를 추가했으며 두 AMR 모두 기본 활성화된다.
- 카메라 기본값은 640×360, 15 Hz이다.
- 침대 GLB의 후방 하단 가운데 개방을 전방에도 대칭 적용했다.
- 전방 숨김 Collider를 좌·우 기둥과 상부 빔으로 분할해 하단 중앙 통로를 실제로 비웠다.
- 자세한 내용: `docs/V1_14_360_LIDAR_DUAL_CAMERA_DUAL_END_OPENING.md`

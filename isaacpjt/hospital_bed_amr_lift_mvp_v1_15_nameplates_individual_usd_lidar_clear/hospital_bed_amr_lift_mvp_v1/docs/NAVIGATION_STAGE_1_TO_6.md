# 자율주행 준비 1~6단계 — v1.14

이 버전은 SLAM/Nav2를 실행하기 전에 Isaac Sim이 제공해야 할 기본 인터페이스를 완성한다.

## 구현 단계

1. `/clock`: Isaac Sim Timeline의 시뮬레이션 시간을 60 Hz로 발행
2. `/amr1/odom`, `/amr2/odom`: 시작 위치를 0으로 삼는 Ground Truth Odometry를 30 Hz로 발행
3. `/tf`: `amrX/odom -> amrX/base_link` 동적 TF 발행
4. `/tf_static`: `base_link -> lidar_link`, `base_link -> front_camera_link`, `base_link -> rear_camera_link` 고정 TF 발행
5. `/amr1/cmd_vel`, `/amr2/cmd_vel`: `geometry_msgs/Twist` 구독
6. `check_nav_stage_1_to_6.sh`: 토픽, 속도, Subscriber, TF 연결 통합 검사

## Player 2 새 리프트 키

- `[` : 리프트 상승
- `]` : 리프트 하강

키보드 이벤트 이름은 각각 `LEFT_BRACKET`, `RIGHT_BRACKET`이다.

## 실행

```bash
./scripts/run_complete_system.sh
```

다른 터미널에서:

```bash
./scripts/check_nav_stage_1_to_6.sh
```

## cmd_vel 시험

```bash
./scripts/test_cmd_vel.sh amr1 forward 2
./scripts/test_cmd_vel.sh amr1 right 2
./scripts/test_cmd_vel.sh amr2 arc_right 2
```

`linear.x`는 전후, `linear.y`는 옴니 횡이동, `angular.z`는 회전이다. 명령이 끊기면 설정된 timeout 이후 자동 정지한다.

## TF 모드

현재 RViz에서 두 AMR를 한 화면에 보기 위해 기본적으로 다음 임시 루트를 발행한다.

```text
world
├─ amr1/odom -> amr1/base_link -> sensors
└─ amr2/odom -> amr2/base_link -> sensors
```

SLAM Toolbox 또는 AMCL을 시작하기 전에는 반드시:

```bash
./scripts/set_tf_mode.sh slam
```

을 실행하고 통합 시스템을 다시 켠다. 그러면 `world -> odom`이 사라져 SLAM/AMCL이 `map -> odom`을 발행할 수 있다. 테스트 화면으로 되돌릴 때는:

```bash
./scripts/set_tf_mode.sh rviz
```

## 예상 토픽

```text
/clock
/tf
/tf_static
/amr1/odom
/amr2/odom
/amr1/scan
/amr2/scan
/amr1/cmd_vel
/amr2/cmd_vel
```

`/map`, `/amcl_pose`, `/plan`, Costmap 토픽은 이 버전에서 만들지 않는다. 이들은 다음 단계의 SLAM Toolbox, Map Server, AMCL, Nav2가 생성한다.


## v1.14 센서 입력

SLAM Toolbox에는 AMR당 중앙 360도 LiDAR의 `/amr1/scan` 또는 `/amr2/scan` 하나만 입력한다. 전·후방 카메라는 SLAM 입력에 합치지 않으며 도킹과 안전 인식에 사용한다.

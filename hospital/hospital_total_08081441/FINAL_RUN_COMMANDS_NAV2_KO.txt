# AMR1 초기 위치 잠금 + 복도/문 중앙 주행 버전

## 이번 버전의 동작

### 1. 초기 위치는 한 번만 지정

- AMCL을 실행하지 않습니다.
- RViz의 `2D Pose Estimate`를 처음 한 번만 받습니다.
- 그 위치를 실제 AMR 위치로 확정하고 `map -> odom`을 고정합니다.
- 두 번째 `2D Pose Estimate`부터는 무시합니다.
- 위치를 다시 지정하려면 Nav2/RViz를 종료하고 다시 실행합니다.
- AMCL ParticleCloud가 없으므로 작은 초록색 입자 구름도 없습니다.
- LiDAR는 장애물 감지와 Local/Global Costmap에 계속 사용합니다.

### 2. 복도와 문 중앙 경로

RViz의 `2D Goal Pose`는 `/center_goal`로 전달됩니다.

중앙 경로 노드는 다음 순서로 처리합니다.

1. 클릭한 목표 주변에서 안전한 중앙점을 찾습니다.
2. 지도 벽까지의 여유 거리를 계산합니다.
3. 벽에서 멀수록 우선하도록 4방향 A* 경로를 계산합니다.
4. 방향 전환 횟수를 줄입니다.
5. 직선 구간별로 `FollowPath`에 전달합니다.
6. 코너마다 완전히 정지한 뒤 다음 방향으로 이동합니다.

주황색 선이 `/centerline_path`입니다.

### 3. 목적지 도착 후 정지

- 마지막 직선 구간이 완료되면 `/cmd_vel`에 0을 반복 발행합니다.
- 이후 새 목표가 들어오기 전까지 추가 이동 명령을 보내지 않습니다.
- Isaac 쪽 명령 타임아웃 후에도 AMR 명령은 0으로 유지됩니다.

## 처음 실행

### 터미널 1: 기존 프로세스 종료

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./00_stop_all.sh
```

### 터미널 1: 빌드

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
chmod +x *.sh scripts/*.sh
./02_build_ros_ws.sh
```

### 터미널 1: Isaac Sim 실행

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./03_run_isaac.sh
```

이 스크립트는 Timeline을 자동으로 Play 상태로 만듭니다. 실행 중에는 Isaac Sim에서 Stop, Reset, Stage 재열기를 하지 마세요.

### 터미널 2: 토픽 확인

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml

ros2 topic list | grep -E '^/clock$|^/scan$|^/odom$|^/tf$|^/cmd_vel$'
```

### 터미널 3: 층 선택

1층:

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./10_select_floor_map.sh 1f
```

2층:

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./10_select_floor_map.sh 2f
```

### 터미널 3: Nav2와 RViz 실행

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./09_run_nav2_amr1.sh
```

## RViz 조작 순서

1. `2D Pose Estimate`를 누릅니다.
2. 지도에서 AMR의 실제 위치를 클릭하고, AMR이 바라보는 방향으로 드래그합니다.
3. 이 위치는 한 번만 잠깁니다.
4. `2D Goal Pose`로 목적지를 지정합니다.
5. 목표는 문이나 복도의 중앙 안전점으로 자동 보정됩니다.

## 정상 확인

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./12_check_center_navigation.sh
```

정상 상태:

```text
/pose_lock_localizer
/centerline_navigator
/controller_server
/map_server
/rviz2

data: true
/map
/scan
/odom
/center_goal
/centerline_path
```

## 위치를 다시 찍어야 할 때

초기 위치는 한 번만 받으므로 Nav2를 재시작합니다.

```bash
pkill -f rviz2
pkill -f pose_lock_localizer
pkill -f centerline_navigator
pkill -f "ros2 launch hospital_nav2"

cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./09_run_nav2_amr1.sh
```

그 후 `2D Pose Estimate`를 다시 한 번 찍습니다.

## 화면 표시

RViz에는 다음이 표시됩니다.

- Navigation Map
- Global Costmap
- Local Costmap
- RobotModel
- LiDAR: 청록색 점
- Centerline Path: 주황색 선
- Local Plan: 청록색 선

AMCL ParticleCloud는 실행하거나 표시하지 않습니다.

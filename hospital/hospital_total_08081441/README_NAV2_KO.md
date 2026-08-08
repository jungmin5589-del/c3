# Hospital AMR1 Nav2 + RViz2 추가판

이 버전은 기존 OCR, 카메라, 키보드 이동 기능을 유지하면서 AMR1에 Nav2 인터페이스를 추가합니다.

## 추가된 통신

- `/cmd_vel` 구독: Nav2 속도 명령을 AMR1 `/World/AMR1/base_link`에 적용
- `/odom` 발행
- `/tf`: `odom -> base_link`
- `/tf_static`: `base_link -> base_scan`은 robot_state_publisher가 발행
- `/scan`: 실행할 때 AMR1 아래에 생성되는 PhysX 2D LiDAR
- `/clock`: Nav2의 `use_sim_time:=true`용 시간

기존 키보드 입력이나 OCR 자동 접근이 실행되는 동안에는 해당 명령이 Nav2보다 우선합니다.

## 1. Nav2 설치

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./07_install_nav2.sh
```

## 2. Occupancy Map 만들기

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
export ISAAC_SIM_DIR=/mnt/isaac45/isaacsim_5.1
./08_open_occupancy_map.sh
```

Isaac Sim에서 다음 순서로 설정합니다.

1. Stage에서 `/World/AMR1`, `/World/AMR2` 눈을 꺼서 비활성화합니다.
2. Stage에서 `/World/HospitalMap`을 선택합니다.
3. `Tools -> Robotics -> Occupancy Map`을 엽니다.
4. Cell Size는 `0.05`, Lower Z는 `0.10`, Upper Z는 `0.62`로 입력합니다.
5. `BOUND SELECTION` 후 `CALCULATE`를 누릅니다.
6. `VISUALIZE IMAGE`에서 Rotate Image를 `180`, Coordinate Type을 ROS YAML 방식으로 선택합니다.
7. 아래 폴더에 정확히 저장합니다.

```text
ros2_ws/src/hospital_nav2/maps/hospital_map.png
ros2_ws/src/hospital_nav2/maps/hospital_map.yaml
```

YAML 첫 줄은 반드시 다음이어야 합니다.

```yaml
image: hospital_map.png
```

맵을 저장한 후 Isaac Sim 편집기 창은 닫습니다.

## 3. ROS 워크스페이스 빌드

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
./02_build_ros_ws.sh
```

## 4. Isaac Sim 실행 - 터미널 1

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
export ISAAC_SIM_DIR=/mnt/isaac45/isaacsim_5.1
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./03_run_isaac.sh
```

정상 로그 예시:

```text
[NAV2 SUB] /cmd_vel
[NAV2 PUB] /odom
[NAV2 PUB] /scan
[NAV2 PUB] /clock
```

## 5. Nav2 + RViz2 실행 - 터미널 2

```bash
cd ~/Downloads/hospital_bed_amr_project_nav2_v1
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./09_run_nav2_amr1.sh
```

## 6. RViz에서 목적지 보내기

1. RViz에서 `2D Pose Estimate`를 누릅니다.
2. 맵에서 현재 AMR1 위치를 클릭하고 AMR1이 보는 방향으로 드래그합니다.
3. LaserScan과 로봇 위치가 맵에 맞는지 확인합니다.
4. `Nav2 Goal`을 누릅니다.
5. 목적지를 클릭하고 도착 시 바라볼 방향으로 드래그합니다.

## 7. 통신만 먼저 검사

```bash
./10_check_nav2_topics.sh
```

`/cmd_vel` 연결부터 확인하려면 다음을 실행합니다.

```bash
./11_test_cmd_vel_amr1.sh
```

## 중요한 점

- AMR1을 제어하는 실제 Prim은 `/World/AMR1/base_link`입니다.
- `/World/AMR1` 부모 Prim 좌표는 실제 주행 Pose로 사용하지 않습니다.
- 지도 생성 시 AMR들을 비활성화하지 않으면 로봇 자체가 지도 장애물로 들어갑니다.
- OCR `O` 키를 누르면 OCR 자동 접근이 Nav2보다 우선합니다. OCR 테스트 전에는 RViz의 현재 Goal을 취소하는 편이 안전합니다.
- 이 패키지는 소스 정적 검사를 완료했지만 실제 Isaac Sim 5.1 런타임은 이 제작 환경에서 실행할 수 없어 첫 실행 시 LiDAR 높이와 Occupancy Map 원점은 사용자 맵에 맞게 확인해야 합니다.

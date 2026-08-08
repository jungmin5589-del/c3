# Domain 115 / AMR2 동일 Nav2 / 자동 엘리베이터

## 변경 범위

- `ROS_DOMAIN_ID=115` 고정
- AMR1의 `nav2_params.yaml`, `centerline_navigator.py`, `pose_lock_localizer.py`, AMR1 launch는 원본 그대로 유지
- AMR2는 AMR1 로봇을 런타임 복제하고 `/amr2/*` 토픽과 `amr2/*` TF로 동일 Nav2 스택 실행
- 엘리베이터 문·리프트 구현은 사용자가 제공한 `엘리베이터.zip`의 `elevator_map_only.py` 사용
- O 키는 엘리베이터 트리거로 사용하지 않음

## 자동 시퀀스

1. Nav2가 엘리베이터 지정 X/Y 좌표에 도착하며 최종 yaw는 강제하지 않음
2. 매니저가 `/elevator/amr_arrived` 발행
3. 1층 문 열림
4. AMR1이 3.0 m 전진
5. 문 닫힘 및 FixedJoint 결합
6. 엘리베이터 2층 상승
7. 2층 문 열림
8. `/map_server/load_map`으로 `hospital_map_2f.yaml` 로드
9. AMR1이 3.0 m 후진하여 하차
10. 제자리 360도 회전
11. `/amr1/elevator/arrived=True` 및 `/elevator/status` COMPLETE 발행

## 처음 실행

```bash
cd /home/rokey/hospital_bed_amr_project_nav2_v1_kimseoul_dual_elevator_domain115
chmod +x *.sh scripts/*.sh
./00_stop_all.sh
./02_build_ros_ws.sh
./10_select_floor_map.sh 1f
```

### 터미널 1 — Isaac Sim Python 3.11

```bash
cd /home/rokey/hospital_bed_amr_project_nav2_v1_kimseoul_dual_elevator_domain115
./03_run_isaac.sh
```

이 터미널에서는 `/opt/ros/humble/setup.bash`를 직접 source하지 않는다.

### 터미널 2 — AMR1 + AMR2 Nav2

```bash
cd /home/rokey/hospital_bed_amr_project_nav2_v1_kimseoul_dual_elevator_domain115
./09_run_nav2_dual.sh
```

### 터미널 3 — AMR1 OCR

```bash
cd /home/rokey/hospital_bed_amr_project_nav2_v1_kimseoul_dual_elevator_domain115
./04_run_ocr_amr1.sh
```

### 터미널 4 — 김서울 전체 시나리오

```bash
cd /home/rokey/hospital_bed_amr_project_nav2_v1_kimseoul_dual_elevator_domain115
./13_run_patient_transport.sh
```

메뉴에서 `1` 입력.

### AMR2 RViz 별도 확인

```bash
cd /home/rokey/hospital_bed_amr_project_nav2_v1_kimseoul_dual_elevator_domain115
./09_open_rviz_amr2.sh
```

AMR2는 Isaac의 `/amr2/world_pose`를 이용해 초기 Pose를 자동 잠근다.

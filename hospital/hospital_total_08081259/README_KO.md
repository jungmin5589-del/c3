# hospital_total_08081259 실행 안내

이 폴더는 Isaac Sim 병원 Stage, AMR1·AMR2, ROS 2 Nav2, OCR 환자 확인, ArUco 침대 중심정렬, 침대 결합, 엘리베이터, MRI 환자 이송을 한 프로젝트에 모은 통합본입니다.

## 현재 노트북 설정

- 프로젝트: `/home/peter-msi/hospital/hospital_total_08081259`
- Isaac Sim: `/home/peter-msi/isaacsim-5.1.0`
- ROS 2: Humble
- ROS Domain ID: `117`
- OCR 가상환경: `~/.venvs/hospital_ocr_ros310`
- 기본 지도: 1층 `hospital_map_1f`

모든 실행 스크립트는 자신의 위치에서 프로젝트 루트를 계산하므로 폴더를 다시 이동하지 않는 한 별도 경로 입력은 필요하지 않습니다.

## 침대 접근 방식

침대 앞에서는 다음 순서로 동작합니다.

1. OCR은 환자 이름·생년월일만 확인합니다.
2. OCR 명찰 bbox 좌표는 이동 제어에 사용하지 않습니다.
3. 해당 환자 침대의 좌우 ArUco pair 중점을 카메라 중심에 맞춥니다.
4. 중심 오차가 `±12px` 안에서 새 메시지 2회 연속 안정되면 환자별 고정 거리를 전진합니다.
5. 전진이 끝난 뒤 마그넷으로 침대를 결합합니다.

ArUco ID는 `김서울=10/11`, `박인천=20/21`, `서수원=30/31`입니다. 두 마커가 모두 보이지 않으면 AMR은 전진하거나 마커를 찾아 돌아다니지 않고 정지 상태로 기다리다가 제한시간 후 실패합니다. 다른 환자의 완전한 pair가 연속 검출되면 잘못된 침대 접근을 막기 위해 실패합니다.

현재 진입 거리는 다음과 같습니다.

- 김서울: `2.87m` — 참고 ArUco 프로젝트에서 초과 진입을 보정한 값
- 박인천: `3.5221m`
- 서수원: `3.1554m`

ArUco는 좌우 중심선만 맞추며 침대까지의 깊이는 측정하지 않습니다. 따라서 마지막 전진 거리는 여전히 환경에 따라 보정이 필요할 수 있습니다.

## 최초 한 번만 준비

```bash
cd /home/peter-msi/hospital/hospital_total_08081259
./02_build_ros_ws.sh
```

현재 노트북에는 OCR 환경과 Nav2가 설치되어 있습니다. 다음 스크립트는 의존성이 없어졌거나 import가 실패할 때만 사용합니다.

```bash
./01_install_ocr_ros.sh
./07_install_nav2.sh
```

ROS 소스를 수정했거나 프로젝트 폴더를 옮겼다면 `./02_build_ros_ws.sh`를 다시 실행합니다.

## 권장: AMR1 단독 시험

듀얼 운행보다 먼저 이 순서로 AMR1 전체 시나리오를 확인합니다.

터미널 1 — Isaac Sim:

```bash
cd /home/peter-msi/hospital/hospital_total_08081259
./03_run_isaac.sh
```

Isaac 터미널에서는 `/opt/ros/humble/setup.bash`를 source하지 마십시오. Stage 로딩 뒤 timeline은 코드에서 자동 시작합니다.

터미널 2 — AMR1 Nav2와 RViz:

```bash
cd /home/peter-msi/hospital/hospital_total_08081259
./09_run_nav2_amr1.sh
```

터미널 3 — AMR1 OCR·ArUco와 전체 미션:

```bash
cd /home/peter-msi/hospital/hospital_total_08081259
./04_run_ocr_mission_amr1.sh 1
```

환자 번호는 `1=김서울`, `2=박인천`, `3=서수원`입니다. 번호를 생략하면 터미널에서 선택합니다.

정상 기동 시 터미널 1에는 아래 로그가 보여야 합니다.

```text
[ARUCO READY] separate_cards=6 expected=6 original_nameplates=UNCHANGED
```

터미널 3에는 `OCR model ready`와 `paired ArUco ready`가 모두 보여야 합니다.

MRI 검사 대기 위치에서 미션 터미널에 안내가 나오면 같은 터미널에서 `K`를 눌러 복귀를 계속합니다.

## AMR1·AMR2 듀얼 실행

AMR1 단독 시험이 성공한 뒤 실행합니다.

현재 듀얼 Nav2 launch는 AMR1과 AMR2가 하나의 `/map_server`를 공유하지만, AMR2 전체 미션은 층 전환 때 `/amr2/map_server/load_map`을 호출합니다. 따라서 아래 구성은 같은 층의 듀얼 주행·충돌회피 시험용으로 먼저 사용하고, AMR2까지 포함한 동시 전체 이송은 map server를 로봇별로 분리한 뒤 진행하십시오.

터미널 1:

```bash
./03_run_isaac.sh
```

터미널 2:

```bash
./09_run_nav2_dual.sh
```

터미널 3 — 동시 경로 충돌 회피:

```bash
./09_run_collision_avoidance.sh
```

터미널 4 — AMR1 미션 예시:

```bash
./04_run_ocr_mission_amr1.sh 1
```

터미널 5 — AMR2 미션 예시:

```bash
./04_run_ocr_mission_amr2.sh 2
```

같은 PC에서 AMR1과 AMR2가 같은 환자를 동시에 선택하면 잠금 파일이 중복 선택을 차단합니다. 각 로봇에는 서로 다른 환자를 선택하십시오.

## 점검과 종료

통합 토픽 점검:

```bash
./10_check_dual_nav2.sh
```

AMR1 ArUco 토픽과 1회 검출 결과 점검:

```bash
./15_check_aruco_runtime.sh amr1
```

카메라 화면에서 마커 ID, pair 중심, 카메라 중심선을 실시간으로 확인:

```bash
./17_view_aruco_debug.sh amr1
```

AMR2를 볼 때는 마지막 인자를 `amr2`로 바꿉니다.

프로젝트 정적 점검:

```bash
./check_project.sh
```

모든 관련 프로세스 종료:

```bash
./00_stop_all.sh
```

## 주의사항

- `04_run_ocr_mission_amr1.sh`와 `04_run_ocr_mission_amr2.sh`는 각각 OCR 노드, ArUco pair 노드, 미션 매니저를 함께 실행합니다.
- 미션 스크립트를 사용할 때 `04_run_ocr_dual.sh` 또는 `13_run_patient_transport.sh`를 중복 실행하지 마십시오.
- 기본 지도는 1층이며, 전체 미션은 엘리베이터 상태에 맞춰 층 지도를 전환합니다.
- 한 노트북에서 Isaac Sim, 듀얼 Nav2, OCR 두 개를 동시에 실행하면 메모리 사용량이 큽니다. 처음에는 Chrome 등 불필요한 프로그램을 종료하고 AMR1 단독으로 확인하십시오.
- `09_run_collision_avoidance.sh`는 듀얼 미션에서만 별도 실행합니다.
- 현재 듀얼 launch의 공유 map server 구조에서는 AMR2 전체 층간 미션의 지도 전환이 실패할 수 있습니다. AMR2 단독 시험은 `09_run_nav2_amr2.sh`와 `04_run_ocr_mission_amr2.sh` 조합을 사용하십시오.
- 실행에 문제가 생기면 먼저 모든 터미널의 `ROS_DOMAIN_ID=117` 출력과 `./02_build_ros_ws.sh` 완료 여부를 확인하십시오.
- ArUco 정렬 중에는 `/cmd_vel`의 `linear.y`를 사용하므로 수정된 `scripts/nav2_bridge.py`가 필요합니다. 폴더를 옮기거나 소스를 덮어쓴 뒤에는 반드시 다시 빌드하십시오.

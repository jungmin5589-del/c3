# 프로젝트 상태

마지막 업데이트: 2026-08-08 15:30 KST

## 현재 목표

- `/home/peter-msi/hospital/hospital_total_08081259`를 현재 통합 시험 대상으로 사용한다. 기존 기준본과 별도 ArUco 참고본은 비교·복구용으로 보존한다.
- ROS 2 통신 도메인은 `ROS_DOMAIN_ID=117`로 통일한다.
- 통합본에서 OCR은 환자 신원만 확인하고, 환자별 좌우 ArUco pair로 침대 중심선을 맞춘 뒤 고정거리 진입하도록 한다.
- 먼저 AMR1 김서울 단독으로 Isaac Sim → Nav2 → OCR 신원확인 → ArUco 중심정렬 → 침대 결합 → 엘리베이터 → MRI → 병실 복귀 전체 시나리오를 재현하고 안정화한다.
- AMR1 전체 시나리오가 안정화된 뒤 듀얼 AMR와 안전 기능을 개선한다.

## 완료된 작업

- 새 프로젝트의 폴더 구조, 실행 진입점, ROS 토픽·서비스 및 전체 미션 흐름을 분석했다.
- Ubuntu 22.04.5, Isaac Sim 5.1.0, ROS 2 Humble, Nav2/RViz 및 OCR 가상환경 설치 상태를 확인했다.
- RTX 5070 Ti Laptop GPU 12GB와 NVIDIA 드라이버 `580.173.02`의 정상 동작을 확인했다.
- Python, JSON, 셸 스크립트 문법 검사를 통과했다.
- Isaac Sim 5.1.0 헤드리스 Stage 검증을 통과했다.
- 모든 실행·점검 `.sh`의 `ROS_DOMAIN_ID`를 `117`로 변경했다.
- `patient_transport_manager.py`의 ROS 재시작 환경과 `config/isaac_config.json`의 기본 도메인도 `117`로 통일했다.
- `03_run_isaac.sh`와 `scripts/clean_ros_env.sh`의 기본 Isaac 경로를 `/home/peter-msi/isaacsim-5.1.0`으로 변경했다.
- 동료 PC에서 복사된 `ros2_ws/build`, `install`, `log`를 제거했다.
- VS Code 인덱스, Python 캐시 및 과거 OCR 출력 등 재생성 가능한 파일을 제거했다.
- 프로젝트 크기를 약 1.3GB에서 159MB로 줄였다.
- 구버전 실행법·좌표·변경 이력이 담긴 루트 `.md`/`.txt` 문서를 제거했다.
- 실행에 필요한 `requirements_ocr_ros.txt`, `PROJECT_STATE.md`, 확장 패키지 README만 보존했다.
- 환자 MRI 확장 README를 현재 3환자 구조와 실제 단축키 기준으로 갱신했다.
- 프로젝트 이동 과정에서 `/home/peter-msi/hospital/hospital_bed_final_0807_0803`에 남은 Git 이력을 홈 프로젝트의 현재 파일과 대조했다.
- 남아 있던 `.git` 이력을 기준 프로젝트로 복원했으며, Git 작업 트리가 변경 사항 없이 정상임을 확인했다.
- 중첩 프로젝트의 재생성 가능한 `.vscode`, `log` 잔여물과 빈 `/home/peter-msi/hospital` 폴더를 제거했다.
- 홈 아래에 `/home/peter-msi/hospital_bed_final_0807_0803` 하나만 남아 있음을 확인했다.
- ROS 2 워크스페이스가 현재 노트북 경로로 빌드되었고 `hospital_nav2`, `hospital_ocr_bridge` 패키지가 검색되는 것을 확인했다.
- 참고 프로젝트 `hospital_bed_amr_cube_transit_fixed_FINAL_ROUNDTRIP_JITTERFIX_EXIT5M_BOTH_OCR_ARUCO_DEBUG_MRI_SAFE_2F_domain120`의 ArUco 침대 도킹 구현을 분석했다.
- ArUco 카드 생성, RGB 검출, 환자별 좌우 pair 중점 계산, OCR 확인 후 중심 정렬과 고정거리 접근으로 이어지는 코드 흐름을 확인했다.
- 참고 프로젝트의 ArUco/MRI 정적 검증 스크립트가 통과하는 것을 확인했다.
- 참고 프로젝트에서 마커가 보이지 않아도 AMR이 진입할 수 있었던 원인을 추가 분석했다.
- 참고 프로젝트에 포함된 `ros2_ws/build/install/log`가 실제 ArUco 소스가 아니라 동료 PC의 이전 Domain 115 프로젝트 빌드 결과임을 확인했다.
- 포함된 install에는 `aruco_pair_node` 실행 파일과 유효한 launch 소스가 없어, 로컬 재빌드 전 실행은 ArUco 기능 검증으로 인정할 수 없음을 확인했다.
- 참고 ArUco 프로젝트의 활성 `.sh` 17개, 설정 JSON과 미션 매니저를 `ROS_DOMAIN_ID=117`로 통일했다.
- 참고 ArUco 프로젝트의 Isaac Sim 기본 경로를 `/home/peter-msi/isaacsim-5.1.0`으로 변경했다.
- 참고 ArUco 프로젝트의 통합 mission launch에 `aruco_pair_node`를 추가하고, ROS bridge가 `Twist.linear.y`를 AMR에 전달하도록 수정했다.
- 빠져 있던 AMR2/dual OCR launch를 패키지 설치 목록에 추가했다.
- 동료 PC의 기존 build/install/log를 제거하고 현재 노트북 경로로 ROS 2 패키지 2개를 재빌드했다.
- 현재 OCR 가상환경 shebang, 로컬 package prefix, ArUco 실행 파일 및 모든 OCR launch 설치를 확인했다.
- Isaac Sim 5.1.0에서 참고 프로젝트 Stage의 헤드리스 검증을 통과했다.
- `아루코마커테스트_침대안쪽으로깊게 들어감.webm`의 도킹 구간을 프레임 단위로 분석했다.
- 영상에서 ArUco 10/11의 수평 오차가 약 96초 `-63.5px`에서 약 100초 `-0.5px CENTERED`로 정상 보정되는 것을 확인했다.
- 마커가 사라진 뒤에도 약 10초간 전진하는 모습이 `3.328m / 0.32m/s = 10.4초` 고정거리 접근 코드와 일치함을 확인했다.
- 2026-08-08 12:22 재시험의 `NO_PLATE_DETECTED` 종료를 분석했다. 저장된 OCR 프레임 2장 모두 명찰 사각형 검출에 실패하여 `plate_frames=0`이었고, OCR 미검출 직후 미션 매니저가 정상적으로 실패 종료한 경우였다.
- 같은 시점의 ArUco 디버그 영상에서는 김서울 pair 10/11과 중심 오차 `+16.0px`가 정상 검출되어, 이번 종료 원인은 ArUco 중앙점 미검출이 아님을 확인했다.
- 현재 ArUco 제어는 예상 pair가 보이지 않으면 정지한 채 새 검출 결과를 기다리고 제한시간 후 실패한다. 마커를 찾기 위한 회전·좌우 스캔 동작은 구현되어 있지 않으며, pair가 검출된 뒤에만 오차 크기에 따라 회전 또는 측면 보정을 수행한다.
- 재시험에서도 침대 중심을 지나 결합하는 현상이 반복되어, 중심정렬 뒤 사용하는 고정 전진거리 `3.328m`가 현재 노트북의 실제 정지 위치와 맞지 않는 것으로 판단했다.
- 접근 코드는 odom 이동량이 3.328m에 도달한 뒤에야 0 속도를 명령한다. 현재 `0.32m/s`, 선형 감속도 `1.0m/s²` 설정에서는 이론상 약 5cm의 추가 정지거리도 발생할 수 있음을 확인했다.
- 결합 직전 Isaac 터미널의 `magnet nearest=... distance=...m` 로그가 AMR base와 가장 가까운 침대의 월드 bounding-box 중심 사이 XY 거리를 출력한다. ArUco 좌우 정렬이 완료되고 AMR이 침대 중심을 지난 것이 눈으로 확인된 경우, 이 값을 첫 고정거리 보정에 사용할 대략적인 초과 거리로 정했다.
- 실제 결합 로그에서 `/World/HospitalBed_KimSeoul distance=0.432m`를 확인했다. 중앙을 지난 방향이 육안으로 확인되었으므로 `3.328 - 0.432 = 2.896m`를 계산상 보정 거리로 산출했고, 최초 재시험값은 2~3cm 짧은 약 `2.87m`로 정했다.
- 동료의 통합본 `/home/peter-msi/hospital/hospital_total_08081259` 구조와 실행 경로를 분석했다. AMR1·AMR2 독립 Nav2/OCR/미션, 복도 우선권·경로 충돌 관리, 엘리베이터 서비스와 3환자 MRI 이송을 합친 버전이며 ArUco 검출 코드는 포함하지 않는다.
- 통합본의 활성 실행 스크립트 18개, `patient_transport_manager.py`, `config/isaac_config.json`을 Domain 117로 통일하고 Isaac 기본 경로를 `/home/peter-msi/isaacsim-5.1.0`으로 변경했다.
- 통합본의 Domain 115·동료 절대경로·중간 변경 이력을 담은 루트 구문서 19개와 중복 MRI README 1개를 제거하고, 현재 노트북용 `README_KO.md` 하나로 실행법을 정리했다. 제거 문서는 `/tmp/hospital_total_08081259_old_docs_20260808.tar.gz`에 임시 백업했다.
- 통합본의 ROS 2 패키지 2개를 현재 폴더 경로로 빌드했고 Nav2 실행 파일 6개, OCR 실행 파일/shebang, AMR1·AMR2 mission launch 설치를 확인했다.
- AMR1/듀얼 Nav2와 AMR1/AMR2 OCR mission launch 파싱을 통과했으며 Isaac 5.1.0 Stage 헤드리스 검증도 종료 코드 0으로 통과했다.
- 통합본에서 서수원 AMR1 미션 실패 로그를 분석했다. Nav2는 OCR 위치까지 정상 도착했지만 OCR 3회 재시도 중 `NO_PLATE_DETECTED`와 잘못된 작은 명찰 셀 검출로 미션 매니저가 종료 코드 1로 먼저 끝났다.
- 같은 실행의 저장 영상에서 서수원 명찰은 선명하게 보였지만 초기 프레임은 plate 검출 0건이었고, 이후에는 전체 명찰 대신 오른쪽 위 이름 셀만 bbox로 선택된 것을 확인했다. 마지막 반복에서는 전체 명찰을 검출해 `VERIFIED 서수원 1990-02-10 score=0.998`에 성공했다.
- OCR 노드가 실패 취소 시 `_frames`만 비우고 `_request`를 남겨 같은 request ID를 계속 재처리하는 결함을 확인했다. 미션 매니저도 align status의 request ID를 검사하지 않아 이전 요청의 `FAILED`를 새 재시도의 실패로 소비할 수 있다. 그 결과 미션 종료 뒤 OCR만 뒤늦게 성공해도 AMR은 움직이지 않는다.
- 통합본 수정 전 핵심 소스를 `/tmp/hospital_total_08081259_pre_aruco_20260808.tar.gz`에 복구용으로 백업했다.
- 참고 ArUco 프로젝트의 `scripts/aruco_nameplate_markers.py`, `aruco_pair_node.py`, ID 10/11·20/21·30/31 PNG 6개를 통합본에 이식했다.
- 통합본 Isaac 런타임이 세 침대의 원래 명찰을 변경하지 않고 session layer에 ArUco 카드 6개를 생성하도록 연결했다.
- AMR1·AMR2의 단독 OCR launch, mission launch 및 dual OCR launch가 각 카메라에 대응하는 ArUco pair 노드를 함께 실행하도록 연결했다.
- 통합 미션 매니저의 침대 접근을 `OCR 신원확인 → 해당 환자 ArUco pair 중심정렬 → 환자별 고정거리 전진 → 결합` 순서로 변경했다. 미션 경로에서는 OCR bbox 좌표를 이동 제어에 사용하지 않는다.
- ArUco pair가 없거나 결과가 0.75초 이상 오래되면 정지하고, 다른 환자 pair가 반복 검출되면 실패하도록 참고본의 안전 동작을 유지했다.
- 참고본의 정렬 설정 `±12px`, 새 메시지 2회 안정, 큰 오차 회전, 작은 오차 `linear.y` 측면보정, 전진속도 `0.32m/s`를 유지했다.
- `scripts/nav2_bridge.py`가 `Twist.linear.y`를 버리지 않고 AMR 컨트롤러로 전달하도록 수정했다.
- 김서울 진입거리는 참고본에서 초과거리 0.432m를 반영해 보정한 `2.87m`를 통합본에도 적용했다. 박인천 `3.5221m`, 서수원 `3.1554m`는 통합본 값을 유지했다.
- OCR 거절·취소 시 `_request`, 프레임, tracking 상태를 완전히 비우고 취소 뒤 늦게 끝난 OCR 결과를 폐기하도록 수정했다.
- `02_build_ros_ws.sh`가 OCR 노드와 ArUco 노드 모두 OCR 가상환경 Python shebang을 사용하도록 갱신하고 ROS 패키지 2개를 재빌드했다.
- 마커 PNG 6개를 OpenCV ArUco `DICT_4X4_50`로 오프라인 검출해 모든 ID가 정확함을 확인했다.
- 실제 Isaac Sim 5.1.0 headless 실행에서 `/World/HospitalBed_KimSeoul`, `ParkIncheon`, `SeoSuwon`을 찾고 `[ARUCO READY] separate_cards=6` 로그를 확인했다.
- `aruco_pair_node` 실제 ROS 기동, AMR1·AMR2·dual launch 해석, Python/JSON/bash 문법, 최종 `check_project.sh` 검사를 통과했다.
- `15_check_aruco_runtime.sh [amr1|amr2]`, `17_view_aruco_debug.sh [amr1|amr2]`를 추가하고 `README_KO.md`를 새 동작과 점검법에 맞게 갱신했다.
- 김서울 전체 시나리오의 MRI 복귀 후 2층 엘리베이터 접근 실패 로그와 11분 30초 이후 영상을 대조했다. AMR은 닫힌 엘리베이터 문 앞까지 접근했지만 목표까지 약 0.31m를 남기고 정지했으며, 오른쪽 90도 회전·문 열림·하강 SERVICE 단계는 시작되지 않았다.
- Nav2 ProgressChecker가 `FollowPath status=6`으로 중단한 뒤 centerline navigator가 두 번 재계획했고, 2층에서는 0.30m 근접 성공 예외가 의도적으로 비활성화되어 `FAILED:RETRY_LIMIT`가 미션 매니저에 전달된 것을 확인했다. 미션 매니저는 이 실패를 받고 종료 코드 1로 끝났다.
- 2층 엘리베이터 목표점 자체는 정적 2층 맵에서 자유 공간이지만, 영상의 전방 카메라에는 닫힌 문이 가까이 보였다. 실제 문 collider/LiDAR local costmap, 침대 결합 상태의 실제 크기와 AMR-only footprint 차이, 고정 목표 좌표의 여유 부족이 결합된 도달 불가 상태일 가능성이 높다.
- Isaac Sim 창에서 실수로 누른 K는 검사완료 키가 아니라 선택한 Prim을 공용 MRI 테이블로 지정하고 설정 JSON에 영구 저장하는 키임을 확인했다. 현재 `patient_transfer.json`의 공용/환자별 `mri_bed_prim` 4곳이 정상 `/World/HospitalRuntimeTargets/MRIPatientTarget`에서 병원 바닥 Prim으로 변경되어 있으며, 환자 소실 현상과 직접 관련된 설정 오염으로 판단했다.

## 다음 작업

1. 다음 재시험 전에 `patient_mri_transfer/config/patient_transfer.json`의 공용/환자별 `mri_bed_prim` 4곳을 `/World/HospitalRuntimeTargets/MRIPatientTarget`으로 복구하고 Isaac Sim을 완전히 재시작한다.
2. 2층 엘리베이터 Nav2 실패 시 최종 AMR pose와 목표점의 X/Y 오차를 각각 기록한다. 닫힌 문 앞의 실제 안전 정지 좌표를 확인한 뒤 2층 엘리베이터 전용 목표 좌표 또는 전용 도착 판정만 조정하고, 전역 Nav2 goal tolerance는 우선 변경하지 않는다.
3. 통합본에서 `./03_run_isaac.sh`, `./09_run_nav2_amr1.sh`, `./04_run_ocr_mission_amr1.sh 1` 순서로 김서울 단독 전체 미션을 실제 GUI 환경에서 다시 시험한다.
4. 정렬 중 `./17_view_aruco_debug.sh amr1`로 ID 10/11, pair 중심 오차, 카메라 중심선을 확인하고 터미널의 `ArUco CENTER stable=2/2` 로그를 기록한다.
5. 마커 하나 또는 둘을 가린 대조 시험에서 AMR이 정지 대기하고 고정거리 전진·결합으로 넘어가지 않는지 확인한다.
6. 김서울 `2.87m` 진입 뒤 결합 시 magnet distance와 실제 정지 위치를 기록한다. 중심 전에 멈추면 2~3cm 단위로 늘리고, 지나치면 줄여 최종값을 확정한다.
7. 김서울 전체 시나리오 성공 후 박인천·서수원을 AMR1에서 각각 시험해 `3.5221m`, `3.1554m`를 보정한다.
8. 서수원 OCR이 여전히 작은 내부 셀을 선택하거나 `NO_PLATE_DETECTED`가 반복되면 명찰 외곽 우선 조건과 검사 프레임 수를 보강한다.
9. AMR1 안정화 뒤 AMR2 단독 미션을 시험한다. 듀얼 전체 층간 운행 전 공유 map server와 `/amr2/map_server/load_map` 불일치를 해결한다.
10. 시간이 허용되면 마지막 약 0.3m 저속 주행 또는 ArUco marker pose 기반 깊이 폐루프를 추가해 고정거리 의존성을 줄인다.
11. 이후 동적 침대 Footprint, 강제 주행 장애물/정체/timeout, 비상정지 우선순위 및 `cmd_vel` mux를 순서대로 개선한다.

## 중요한 설계 결정

- `/home/peter-msi/hospital_bed_final_0807_0803`는 기존 기준본으로 보존하고 현재 통합 개발은 `/home/peter-msi/hospital/hospital_total_08081259`에서 진행한다.
- `/home/peter-msi/hospital`은 통합 프로젝트와 `PROJECT_STATE.md`의 상위 폴더로 사용한다. 동일 프로젝트의 중복 복사본은 만들지 않는다.
- 이름에 `domain120`이 남은 새 프로젝트는 별도 ArUco 시험본으로 유지한다. 실행 설정은 이 노트북 기준 Domain 117로 변경했지만 기준 프로젝트와 파일을 섞지는 않는다.
- `/home/peter-msi/hospital/hospital_total_08081259`는 현재 통합 시험본이며 자체 `README_KO.md`를 실행 기준으로 사용한다.
- 통합본에는 사용자가 요청한 범위에 한해 참고본의 ArUco 카드 생성·pair 검출·중심정렬 알고리즘만 이식했다. 엘리베이터·MRI·듀얼 미션 구조는 통합본을 유지한다.
- 통합 미션에서 OCR은 신원확인만 담당하고 도킹 좌우 중심은 ArUco pair만 사용한다. ArUco는 깊이를 측정하지 않으므로 마지막 진입은 환자별 odom 고정거리로 유지한다.
- ROS 2 통신은 모든 활성 실행 경로에서 Domain ID `117`, RMW `rmw_fastrtps_cpp`를 사용한다.
- 폴더명에 의존하지 않도록 실행 스크립트는 자신의 위치에서 프로젝트 루트를 계산하는 구조를 유지한다.
- Isaac Sim은 내장 Python 3.11과 내장 ROS 2 bridge를 사용하고, Nav2/OCR는 시스템 ROS 2 Humble Python 3.10 환경에서 실행한다.
- Isaac 실행 터미널과 시스템 ROS 터미널의 Python/라이브러리 환경을 섞지 않는다.
- 현재 검증 기준은 AMR1 단독 전체 시나리오이며, 듀얼 운행은 그다음 단계로 둔다.
- 루트의 과거 README와 변경 기록보다 현재 Python 코드와 JSON 설정을 실행 기준으로 사용한다.
- 프로젝트 진행 문서는 `PROJECT_STATE.md`를 단일 기준 문서로 계속 갱신한다.
- 패키지 메타데이터가 참조하는 문서와 설치 파일은 확실한 의존성이므로 정리 대상에서 제외한다.
- 소스, USD, 환자·직원 자산, 작업 백업은 보존한다. 재생성 가능한 빌드 결과와 캐시만 정리한다.

## 주의사항

- 현재 `ros2_ws/build`, `install`, `log`는 이 노트북의 프로젝트 경로로 빌드되어 있다. 프로젝트 경로를 다시 바꾸거나 ROS 소스를 수정하면 `./02_build_ros_ws.sh`로 재빌드한다.
- Isaac 터미널에서는 `/opt/ros/humble/setup.bash`를 직접 source하지 않는다.
- `04_run_ocr_mission.sh`가 OCR 노드와 미션 매니저를 함께 실행하므로 `13_run_patient_transport.sh`를 동시에 실행하지 않는다.
- 침대 결합 후에도 Nav2 Footprint는 약 0.74m 정사각형 AMR 크기로 고정되어 있다.
- 강제 전진·후진에는 LiDAR 장애물 정지, 진행 정체 감지 및 벽시계 timeout이 없다.
- 엘리베이터 자동 시퀀스 중 키보드 비상정지가 우회될 수 있다.
- Nav2, centerline navigator, mission manager 사이에 명시적인 `cmd_vel` mux가 없다.
- 듀얼 Nav2는 하나의 map server를 공유하므로 두 AMR이 서로 다른 층에 있을 수 없다.
- 통합본의 OCR/ArUco launch는 AMR1·AMR2·dual 모두 설치되지만, 전체 듀얼 층간 미션은 map server 구조 때문에 아직 검증되지 않았다.
- 엘리베이터 문 텍스처 하나가 동료 PC의 임시 `/tmp/...` 경로를 참조하여 외관이 깨질 수 있다.
- 삭제된 과거 문서의 실행법이나 좌표를 외부 사본에서 다시 가져오지 말고 현재 코드·설정과 `PROJECT_STATE.md`를 기준으로 한다.
- RAM이 16GB이므로 통합 실행 전 Chrome, VS Code 등 메모리를 많이 사용하는 프로그램을 가능한 한 종료한다.
- 참고 프로젝트의 폴더명에는 `domain120`이 남아 있지만 활성 실행값은 Domain 117이다. 폴더명만 보고 통신 도메인을 판단하지 않는다.
- 참고 프로젝트의 ArUco 측면 속도 전달과 통합 mission launch 누락은 수정되었으며, 소스를 다시 바꾸면 반드시 재빌드한다.
- ArUco는 깊이·3차원 pose를 추정하지 않는다. 영상의 좌우 마커 중점으로 방향만 맞춘 후 환자별 고정거리를 odom 기준으로 전진한다.
- 참고 프로젝트의 ArUco 카드는 원본 USD에 저장된 모델이 아니라 Isaac 시작 시 session layer에 동적으로 생성되므로, 원본 Stage만 열어 보면 마커가 없는 것이 정상이다.
- 제대로 재빌드한 ArUco 미션에서는 예상 pair가 없으면 정렬 대기 또는 실패해야 한다. 마커 없이 진입한다면 이전 OCR 자동접근 프로세스나 다른 `/cmd_vel` publisher가 움직인 것으로 판단한다.
- ArUco 노드는 OCR 노드·미션 매니저와 동시에 독립 실행되므로, 디버그 영상에 pair와 중심 오차가 보여도 OCR 검증이 먼저 성공하기 전에는 미션 매니저가 ArUco 보정 속도를 명령하지 않는다.
- `NO_PLATE_DETECTED`는 ArUco 미검출이 아니라 OCR 명찰 검출기가 검사 프레임에서 사용할 수 있는 명찰 사각형을 하나도 찾지 못했다는 뜻이다.
- 현재 ArUco pair는 좌우 영상 중심만 맞추며 침대까지의 전후 거리나 침대 기하학적 중심을 계산하지 않는다. 정렬 후에는 마커 검출을 중단하고 고정거리를 open-loop로 전진하므로 거리 보정이 틀리면 침대 중앙을 지나칠 수 있다.
- 다른 컴퓨터에서 맞았던 3.328m는 그 환경의 Nav2 정지점·Stage 배치·카메라/AMR 기하와 감속 조건에 종속된 보정값이다. 동일한 pair 중앙정렬만으로 시작점에서 침대까지의 전후 거리가 같아지는 것은 아니다.
- 통합본은 OCR bbox 접근을 사용하지 않고 ArUco pair 중심정렬 후 환자별 고정거리(김서울 2.87m, 박인천 3.5221m, 서수원 3.1554m)를 전진한다. ArUco 정렬·진입 구간만 참고본의 보수적 속도를 사용하고 나머지 통합 미션 속도는 그대로다.
- 통합본의 듀얼 launch는 map server 하나를 공유하지만 AMR2 미션은 `/amr2/map_server/load_map`을 호출한다. 현재 상태에서 AMR2 동시 전체 층간 이송은 지도 전환에서 실패할 수 있다.
- 통합본 Stage도 `/tmp/a777185045a56a4e/textures/elevator-door_texture0.jpg` 누락 경로를 포함한다. Stage/필수 Prim 검증은 통과했지만 엘리베이터 문 재질 외관은 깨질 수 있다.
- 통합본 OCR 취소·거절 반복 실행 결함은 수정했지만, 실제 카메라 입력에서 재시도와 취소가 기대대로 동작하는지는 다음 전체 미션 시험에서 확인해야 한다.
- Isaac Sim 창의 K는 선택한 Prim을 MRI 테이블로 지정하고 `patient_transfer.json`에 저장한다. 검사 완료 K는 반드시 OCR/미션 터미널에 포커스를 둔 뒤 누르며, Isaac Sim에서는 의도적으로 올바른 MRI Prim을 선택한 경우 외에는 K를 누르지 않는다.
- 현재 `patient_transfer.json`의 MRI Prim 설정은 실수로 바닥 Prim에 덮어써진 상태다. 정상 런타임 타깃으로 복구하고 Isaac Sim을 재시작하기 전에는 환자 MRI 이동 시험 결과를 유효한 결과로 간주하지 않는다.
- 2층 엘리베이터 접근 실패는 목적지까지 약 0.31m 남은 상태에서 발생했다. 이 값을 그대로 전역 `xy_goal_tolerance`로 확대하면 다른 코너와 목적지도 일찍 성공 처리될 수 있으므로, 2층 엘리베이터 전용 좌표/판정으로 제한한다.

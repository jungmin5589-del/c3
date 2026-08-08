# Three Patient MRI Transfer Runtime

김서울·박인천·서수원 환자 모델을 각 침대에 배치하고, 실제 운반 침대와 공용 MRI 환자 타깃 사이를 이동시키는 Isaac Sim 확장입니다.

- 환자 3명 개별 상태와 ROS command/status topic 제공
- 공용 MRI 1인 점유 보호
- MRI 진입·이탈 반경 기반 자동 이송
- `K`: Isaac viewport에서 선택한 MRI 테이블 저장
- `L`: 환자 이송 상태 출력

전체 프로젝트 실행법은 프로젝트 루트의 `README_KO.md`를 참조하십시오.

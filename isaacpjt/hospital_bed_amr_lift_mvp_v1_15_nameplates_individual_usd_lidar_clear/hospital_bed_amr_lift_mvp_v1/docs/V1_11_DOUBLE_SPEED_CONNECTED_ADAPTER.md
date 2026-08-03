# v1.11 변경 사항

## 1. AMR 주행 속도 2배

| 상태 | 전후 | 횡이동 | 회전 |
|---|---:|---:|---:|
| 미적재 | 1.70 m/s | 1.70 m/s | 2.80 rad/s |
| 침대 적재 | 0.90 m/s | 0.76 m/s | 1.30 rad/s |

가속도와 감속 반응값, 바퀴 시각 회전 제한도 v1.10 대비 2배로 조정했다. 리프트 상승·하강 속도는 변경하지 않았다.

## 2. 침대와 리프트 지지대의 시각 연결

기존 LiftRail, Hanger, UpperBridge 구조에 다음을 추가했다.

- ConnectionBeamFront / ConnectionBeamRear
- MountPlateFrontLeft / FrontRight / RearLeft / RearRight

Connection Beam은 침대 폭 방향으로 뻗어 Side Frame 영역과 겹치고, Mount Plate는 끝단을 침대 프레임에 고정한 것처럼 보이게 한다. 모두 침대 Root의 자식이므로 침대와 함께 이동한다. 충돌은 비활성화하여 AMR 진입과 도킹에 영향을 주지 않는다.

## 3. 적용 후 필수 작업

기존 USD에는 새 구조물이 자동으로 생기지 않으므로 반드시 Stage를 다시 생성한다.

```bash
./scripts/generate_all.sh
./scripts/run_complete_system.sh
```

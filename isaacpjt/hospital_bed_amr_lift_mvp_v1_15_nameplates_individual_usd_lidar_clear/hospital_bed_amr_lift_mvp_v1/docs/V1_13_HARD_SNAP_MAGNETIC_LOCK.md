# v1.13 — 철컥 스냅 결합 + 완전 고정 마그네틱 잠금

## 핵심 변경

- 마그네틱 준비(ARM)와 실제 결합(LOCK)을 분리했다.
- 침대 가까이에서 전용 결합 키를 누르면 침대의 XY·방향이 AMR 중심으로 즉시 스냅된다.
- 스냅 직후 `FixedJoint`를 생성한다.
- `breakForce`와 `breakTorque`는 무한대로 설정되어 충격이나 고속 이동으로 자동 파단되지 않는다.
- 결합 해제는 전용 해제 키, 마그네틱 OFF, 프로그램 종료로만 수행한다.
- 리프트 높이는 결합 조건에서 제외했다. 침대 바퀴는 바닥에 계속 접지하는 `wheel_tow` 방식이다.

## 조작

### Player 1

```text
Left Shift   마그네틱 준비 ON/OFF
C            철컥 완전 결합
X            결합 해제
```

### Player 2

```text
Right Shift  마그네틱 준비 ON/OFF
\            철컥 완전 결합
Backspace    결합 해제
```

Player 2 횡이동은 이전과 동일하다.

```text
/ + ,        왼쪽 횡이동
/ + .        오른쪽 횡이동
```

## 사용 순서

1. Shift 키로 마그네틱이 `ARMED`인지 확인한다. 기본값은 실행 시 ON이다.
2. 침대 중앙 아래로 AMR를 넣는다.
3. 침대 중심에서 약 30 cm 이내, 방향 오차 20도 이내로 접근한다.
4. Player 1은 `C`, Player 2는 `\`를 한 번 누른다.
5. 터미널의 `CLACK! MAGNETIC HARD LOCK` 및 `HARD_LOCKED` 상태를 확인한다.
6. 이동한다.
7. Player 1은 `X`, Player 2는 `Backspace`로 해제한다.

## ROS 2 명령

AMR1 결합:

```bash
ros2 topic pub --once /amr1/magnet_lock std_msgs/msg/Bool "{data: true}"
```

AMR1 해제:

```bash
ros2 topic pub --once /amr1/magnet_release std_msgs/msg/Bool "{data: true}"
```

AMR2는 `/amr2/...`로 바꾼다.

## 결합 실패 메시지

- `magnet is OFF`: Shift 또는 `/magnet_cmd`로 준비 기능을 켠다.
- `nearest bed ... must be <= 0.300m`: 침대 중심에 더 가까이 접근한다.
- `yaw error`: 침대와 AMR 방향을 더 평행하게 맞춘다.
- `no available bed`: 침대 Prim이 없거나 다른 AMR가 이미 점유 중이다.

## 주의

완전 고정은 시뮬레이션 데모용이다. 바퀴 접지 상태에서 침대와 AMR가 강제로 한 몸처럼 움직이므로, 실제 캐스터 조향 모델이 없으면 급회전 때 바퀴가 미끄러질 수 있다. 적재 상태의 속도 제한을 유지하고 좁은 공간에서는 저속으로 시험한다.

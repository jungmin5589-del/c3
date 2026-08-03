# v1.12 — 자기력 조절형 바퀴 접지 견인 모드

## 목적

기존 `lift_carry` 방식은 리프트를 올려 병원 침대 바퀴의 하중을 줄이거나 바닥에서 띄운 뒤 AMR가 침대 무게를 지지한다.

v1.12 기본값인 `wheel_tow` 방식은 다음과 같이 동작한다.

1. 침대 바퀴는 바닥에 계속 닿아 있다.
2. 리프트는 침대 어댑터와의 작은 틈만 약 2.5 mm 닫는다.
3. AMR 리프트와 침대 사이에 breakable FixedJoint를 만든다.
4. AMR는 침대 전체 무게를 들어 올리는 대신 바퀴의 구름 저항에 가까운 힘을 부담한다.
5. 침대 바퀴 Proxy에는 저마찰 물리 재질을 적용해 캐스터 구름 저항을 근사한다.

이 기능은 실제 전자기장 해석이 아니라, 정렬 보조 속도와 파단 가능한 Joint로 구현한 자기 결합 근사 모델이다.

## 자기력 조절

설정 파일:

```text
config/amr_config.json
```

주요 항목:

```json
"magnetic_dock": {
  "coupling_mode": "wheel_tow",
  "strength_percent": 70.0,
  "break_force_min_n": 600.0,
  "break_force_max_n": 6000.0,
  "break_torque_min_nm": 120.0,
  "break_torque_max_nm": 1800.0
}
```

`strength_percent`가 커지면 다음 두 값이 함께 증가한다.

- 침대 중심으로 정렬하는 보조 속도
- FixedJoint의 파단 힘과 파단 토크

ROS 2에서 조절:

```bash
ros2 topic pub --once /amr1/magnet_strength std_msgs/msg/Float64 "{data: 70.0}"
ros2 topic pub --once /amr2/magnet_strength std_msgs/msg/Float64 "{data: 50.0}"
```

허용 범위는 기본 10~100%이다.

## 결합 모드 전환

바퀴 접지 견인:

```json
"coupling_mode": "wheel_tow"
```

기존 리프트 운반:

```json
"coupling_mode": "lift_carry"
```

설정 변경 후 통합 시스템을 완전히 종료하고 다시 실행한다.

## wheel_tow 리프트 제한

```json
"wheel_tow_contact_lift_m": 0.0025,
"wheel_tow_minimum_lift_for_lock_m": 0.0024,
"wheel_tow_max_lift_m": 0.0035
```

자석이 켜져 있는 동안 리프트 상승량을 이 범위로 제한하므로 침대 바퀴가 바닥에서 뜨지 않도록 한다.

## Player 2 옴니 횡이동

```text
/ + ,    왼쪽 횡이동
/ + .    오른쪽 횡이동
```

`/`를 누른 상태에서 쉼표 또는 마침표를 누른다. 방향키 좌우는 계속 회전 조작으로 사용한다.

## 한계

- 침대 바퀴는 실제 자유회전 캐스터 Joint가 아니라 저마찰 원통형 Proxy이다.
- 따라서 정밀한 캐스터 조향과 회전 저항까지 필요하면 바퀴별 Revolute/Steering Joint 모델이 추가로 필요하다.
- FixedJoint는 결합 중 상대 위치와 자세를 강하게 고정한다. 실제 자석의 미세한 탄성 거동이 필요하면 D6 Joint와 Spring/Damping 모델로 확장한다.
- 고속 회전 시 침대 바퀴 접지와 Joint가 충돌해 흔들릴 수 있으므로 적재 속도 제한과 Collision Monitor를 함께 사용해야 한다.

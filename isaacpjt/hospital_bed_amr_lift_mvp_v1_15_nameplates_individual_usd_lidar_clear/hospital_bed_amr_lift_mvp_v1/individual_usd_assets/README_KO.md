# 병원 침대 이송 AMR 개별 USD 자산 v1.15

## 포함 파일
- `AMR1.usd`, `AMR2.usd`
- `HospitalBed_SeoSuwon.usd`
- `HospitalBed_KimSeoul.usd`
- `HospitalBed_ParkIncheon.usd`
- `All_Assets_Preview.usd`
- `shared/bed_amr_clear_front_rear_open.glb`
- `textures/*.png`

## 침대 이름표
각 이름표는 실제 `Mesh` Plane이며, `OmniPBR.mdl`의 `diffuse_texture`에 PNG가 연결되어 있습니다.

- Plane 중심: `X=+1.015 m, Y=0.000 m, Z=0.670 m`
- 크기: `0.620 m × 0.430 m`
- 같은 끝의 바퀴 중심: `Y=+0.400 m`, `Y=-0.400 m`
- 따라서 이름표 중심 `Y=0.000 m`는 두 바퀴 사이의 정확한 정중앙입니다.

환자 배정:
- Bed1: 서수원 / 1990-02-10
- Bed2: 김서울 / 2000-11-02
- Bed3: 박인천 / 1960-07-23

## LiDAR
- AMR당 2D 360도 RTX LiDAR 1대
- 설정: `Example_Rotary_2D`
- 센서 원점: 바닥 기준 `Z=0.285 m`
- `base_link -> lidar_link`: `Z=0.200 m`
- 하우징 중심: lidar_link 기준 `Z=-0.0425 m`
- 하우징 높이: `0.025 m`
- 하우징 상단은 스캔 평면보다 약 `0.030 m` 아래여서 360도 스캔 평면을 감싸지 않습니다.

## 카메라
AMR당 앞/뒤 카메라 2대가 포함됩니다.
- front camera: X=+0.39 m, Z=0.16 m
- rear camera: X=-0.39 m, Z=0.16 m, yaw=180°

## 가져오기
폴더 구조를 유지한 상태로 USD를 Isaac Sim Content Browser에서 드래그하거나 Reference로 추가하세요.
텍스처와 GLB가 상대 경로로 연결되어 있으므로 USD만 단독으로 다른 폴더에 옮기면 안 됩니다.

RTX LiDAR가 단순 Camera placeholder로만 보일 경우, Isaac Sim Script Editor에서
`scripts/repair_rtx_lidar_after_import.py`를 실행하면 공식 `IsaacSensorCreateRtxLidar` 명령으로
`Example_Rotary_2D` 센서를 다시 만듭니다.

## 검증 범위
- USDA 구조/상대 경로/텍스처 존재 여부 정적 검사 완료
- Python/JSON/Shell 문법 검사 완료
- Isaac Sim 5.1 실제 렌더링·RTX 센서·물리 런타임은 사용자 PC에서 최종 확인 필요

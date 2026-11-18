# 1113ML 신호등 정지 접근 수정

기준: `f1tenth_project_repo-1113ML` 원본. `auto_control`의 두 파일만 변경했습니다.

## 바뀐 동작

1. RED와 정지선이 확인되면 진입 속도와 관계없이 접근 목표 속도를 `traffic_approach_speed_mps`(기본 0.30 m/s)로 설정합니다.
2. 접근하는 동안 정지선 관측 및 차량 이동량을 이용한 남은 거리 갱신은 기존대로 계속합니다.
3. `traffic_approach_electrical_brake_enabled: false`로 접근 중 전류 제동을 끄고 구동 duty를 줄여 감속합니다. 필요하면 Params에서 이 값을 켤 수 있습니다.
4. 정지선 관측이나 제어 입력이 끊기는 보호 정지에서는 전류 제동을 유지합니다. 따라서 이 경우 멀리서 정지한 뒤 재접근하는 동작은 별도 원인 분석이 필요합니다.
5. 앞범퍼와 정지선의 추정 거리가 `traffic_stop_range_max_m`(기본 0.25 m) 이하이면 최종 정지 명령을 유지합니다. 이 값은 **제동 시작 범위**이며 정확히 25 cm에서 차가 멈춘다는 뜻은 아닙니다.

## 젯슨 적용

이 브랜치의 `1113ML/` 폴더에는 수정이 이미 반영된 독립 실행 작업공간이 있습니다. 이 패치는 별도의 기존 1113ML 복사본에 같은 수정만 적용할 때 사용하세요. `1113ML/` 폴더에는 다시 적용하지 마세요.

```bash
mkdir -p ~/Desktop/hsj
git clone --branch 1118 --single-branch https://github.com/colinha02/f1tenth_front_lane.git ~/Desktop/hsj/traffic_stop_1118
cd ~/Desktop/1113ML/f1tenth_project_repo
git apply --check ~/Desktop/hsj/traffic_stop_1118/patches/1113ML_traffic_stop/traffic_stop_1113ML.patch
git apply ~/Desktop/hsj/traffic_stop_1118/patches/1113ML_traffic_stop/traffic_stop_1113ML.patch
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --symlink-install --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

빌드 후 실제 주행 전에 `monitor_only`에서 `/auto_control`의 목표 속도, 정지선 남은 거리, `brake_current_a`, `traffic_phase`를 기록하고 비교하세요. 현재 PC에는 ROS 2와 젯슨 구동 환경이 없어 실차 동작은 검증하지 못했습니다.

원본 1113ML YAML은 앞차축에서 앞범퍼까지 5 cm로 되어 있습니다. 앞서 알려준 실측 7 cm와 2 cm 차이가 있으므로 25 cm 정지 정확도 실험 전에 따로 보정해야 합니다.

# 1113ML 신호등 정지 접근 1차 수정

기준: `f1tenth_project_repo-1113ML` 원본. `auto_control`의 두 파일만 변경했습니다.

## 바뀐 동작

1. RED와 정지선이 확인되면 진입 속도와 관계없이 접근 목표 속도를 `traffic_approach_speed_mps`(기본 0.30 m/s)로 설정합니다.
2. 접근하는 동안 정지선 관측 및 차량 이동량을 이용한 남은 거리 갱신은 기존대로 계속합니다.
3. 먼 거리에서 속도만으로 `urgent`가 되어 전류 제동을 즉시 시작하던 조건을 제거했습니다. 먼저 `traffic_coast_probe_sec`(기본 0.15초) 동안 구동력 감속을 관찰합니다. 관찰 후 감속이 부족하면 전류 제동을 사용할 수 있습니다.
4. 관찰 시간과 제동 응답 시간 동안 차량이 최종 제동 범위에 들어갈 정도로 가까우면 즉시 전류 제동합니다. 접근 제동 중에는 0.30 m/s 근처에서 미리 전류를 내리고 다시 구동으로 전환합니다.
5. 앞범퍼와 정지선의 추정 거리가 `traffic_stop_range_max_m`(기본 0.25 m) 이하이면 최종 정지 명령을 유지합니다. 이 값은 **제동 시작 범위**이며 정확히 25 cm에서 차가 멈춘다는 뜻은 아닙니다.

## 젯슨 적용

이 브랜치는 1113ML 전체 프로젝트가 아니라 수정 파일 두 개와 패치만 담고 있습니다. 기존 1113ML 프로젝트에 적용하세요.

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

# 1113ML 신호등 정지 제어 수정

기준은 `f1tenth_project_repo-1113ML` 원본입니다. 패치는 `auto_control`의 C++와 YAML 두 파일에 적용됩니다. `1113ML/`에는 이미 반영되어 있으므로 그 폴더에 패치를 다시 적용하지 마세요.

## 바뀐 동작

1. 유효 RED를 보면 정지선 확인 전부터 `traffic_red_preapproach_speed_mps`(1.0 m/s)를 상한으로 구동 duty를 줄입니다.
2. 정지선을 확인하면 앞범퍼 간격, 현재 속도와 관측한 duty 감속도를 사용해 목표속도를 매 제어 주기 갱신합니다.
3. 정지선이 잠깐 가려지면 최신 ERPM 속도로 남은 거리를 갱신합니다. 마지막 관측 후 0.9초 또는 이동량 0.5m를 넘으면 보호 정지합니다.
4. `traffic_stop_range_max_m`(0.25m)는 앞범퍼의 **목표 정지 간격**입니다. 전류 제동은 속도, 응답 지연, 추정 제동 감속도로 계산한 더 먼 위치에서 시작합니다. 구동 duty만 줄이는 접근 중에는 전류 제동을 요청하지 않습니다.
5. 전류 제동 감속도의 초기 추정은 0.50m/s²이고, 앞차축부터 앞범퍼까지는 실측 0.07m를 사용합니다. 실제 정지 간격은 아직 실차에서 확인되지 않았습니다.

속도가 1.6m/s이고 제동 감속도가 0.50m/s²라면, 응답 지연을 제외한 제동거리만 약 2.56m입니다. 정지선이 그보다 가까운 시점에 처음 확인되면 25cm 정지는 물리적으로 어려울 수 있습니다. 낮은 속도부터 시험하고 CSV의 `traffic_phase`, `traffic_bumper_gap_m`, `traffic_brake_start_gap_m`, `target_speed_mps`, `speed_mps`, `brake_current_a`를 비교하세요.

## 젯슨 적용

기존에 작동 중인 프로젝트를 보존하려면 별도 폴더에 이 브랜치를 복제하세요.

```bash
mkdir -p ~/Desktop/hsj
git clone --branch 1118 --single-branch https://github.com/colinha02/f1tenth_front_lane.git ~/Desktop/hsj/f1tenth_traffic_stop_dynamic
cd ~/Desktop/hsj/f1tenth_traffic_stop_dynamic/1113ML
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --symlink-install --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

PC에는 ROS 2와 젯슨 구동 환경이 없어 빌드와 실차 정지 동작은 검증하지 못했습니다.

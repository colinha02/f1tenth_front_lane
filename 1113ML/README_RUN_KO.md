# 1113ML 독립 실행 작업공간

이 폴더는 기존 1113ML ROS 2 소스, 설정, ONNX 모델에 신호등 정지 접근 수정 두 파일을 적용한 작업공간입니다. GitHub `1118` 브랜치의 기존 앞 차선 주행 패키지는 상위 `src/`에 그대로 있습니다. 아래 명령은 이 `1113ML/` 폴더에서만 빌드하므로 그 패키지와 젯슨의 기존 프로젝트를 건드리지 않습니다.

학습용 `datasets/`, 과거 주행 `performance_logs/`, 캐시와 빌드 산출물은 포함하지 않았습니다. 처음 실행할 때 TensorRT 엔진을 새로 만드는 시간은 걸릴 수 있습니다. 차량에 이미 설치된 ROS 2 Humble, DepthAI C++ SDK, TensorRT 등의 시스템 의존성은 GitHub 소스에 포함되지 않습니다. SDK 선택은 `DEPTHAI_SETUP.md`를 참고하세요.

## 젯슨의 개인 폴더로 받기

```bash
mkdir -p ~/Desktop/hsj
git clone --branch 1118 --single-branch \
  https://github.com/colinha02/f1tenth_front_lane.git \
  ~/Desktop/hsj/f1tenth_1113ML_traffic_stop
cd ~/Desktop/hsj/f1tenth_1113ML_traffic_stop/1113ML
```

## 빌드

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --symlink-install \
  --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`model_startup_check` 등의 Python 실행 스크립트는 `--symlink-install`에서 원본 파일의 실행 권한이 필요합니다. 이미 받은 작업공간에서 `executable 'model_startup_check' not found`가 나오면 저장소를 업데이트하고 `auto_control`을 다시 설치하세요.

```bash
git -C .. pull --ff-only
colcon build --packages-select auto_control --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ls -l install/auto_control/lib/auto_control/model_startup_check
```

## 신호등 정지 주행 확인

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  manual_test:=false auto_control_mode:=drive \
  traffic_light_enabled:=true traffic_stop_enabled:=true \
  obstacles_enabled:=false avoidance_control_enabled:=false \
  preview_enabled:=true
```

`manual_test:=true`는 신호등 정지 기능을 끄므로 이 실험에는 사용하지 않습니다. 이 명령은 실제 조향·구동 명령을 출력합니다. 정지선 접근 목표 속도는 `src/auto_control/config/auto_control.yaml`의 `traffic_approach_speed_mps: 0.30`입니다.

신호등 접근 중에는 `traffic_approach_electrical_brake_enabled: false`로 전류 제동을 요청하지 않고 구동 duty를 줄여 감속합니다. 정지 범위(앞범퍼 기준 25 cm 이내)에서 목표속도가 0이 되면 최종 전류 제동을 사용합니다. 입력 유실·운전자 STOP의 보호 제동도 유지됩니다. 따라서 멀리서 다시 멈춘다면 CSV의 `traffic_phase`, `control_state`, `brake_current_a`로 입력 유실 여부를 구분하세요. 25 cm는 제동 개시 범위이며 정확한 정지 위치는 실차 측정이 필요합니다.

GitHub의 `patches/1113ML_traffic_stop/`에는 같은 수정의 개별 파일과 패치도 보관되어 있습니다. **이 폴더에는 이미 적용되어 있으므로 패치를 다시 적용하지 마세요.**

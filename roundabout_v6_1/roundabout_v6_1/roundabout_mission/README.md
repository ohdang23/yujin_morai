# v5 ID-independent entry decision

현재 설치/실행/변경/검증 결과는 [docs/V5_ID_INDEPENDENT_ENTRY_DECISION.md](docs/V5_ID_INDEPENDENT_ENTRY_DECISION.md)를 먼저 읽으세요. 아래 내용과 기존 install_v42는 과거 참고 자료입니다.

# 회전교차로 미션 v4.2 (2026-10-04)

현재 설치·실행·동작 설명은 [docs/RUN_V42.md](docs/RUN_V42.md)를 따른다.
v4.2는 접근 중 빈틈 판단으로 인한 반복 정지, 소실 ID의 전역 급정지,
복구 확인이 빈틈을 기다리느라 지연되는 문제를 수정한다. 접근은 2 m/s,
진입은 3~4 m/s 후보를 검증한다. 패키지와 진단 버전은 `4.2.0`이다.
아래 v4/v4.1 설치 명령과 검증은 이전 버전의 기록이다.

## v4에서 업데이트

진입 전 `relevant_track_lost_restart_or_reacquire`로 멈추고 사라진 ID가 계속 누적되는
문제를 수정했다. 기존 브랜치의 클러스터링 → map 변환 → Kalman/Hungarian 추적 →
confirmed 객체 토픽을 그대로 사용한다. 차종 분류 결과는 아니며 카메라 융합도 아니다.

- 재검출 ID가 달라져도 1.5초 이내 예측 위치/속도/크기가 맞고 후보가 양방향으로 하나일 때
  이전 ID 기록을 새 ID로 넘긴다. 혼잡해서 대응이 모호하면 임의로 합치지 않는다.
- 객체 소실 후에는 일단 정지한다. 차량 앞부분이 conflict 이전이고 속도 0.15m/s 미만일 때,
  현재 모든 객체의 충돌/진입 간격/앞차 간격 검사를 통과한 새로운 메시지가 최소 10개,
  최소 2초에 걸쳐 들어와야 과거 소실 기록을 해제한다. 그 뒤 기존 0.8초 진입 확인을 다시 한다.
- 현재 접근 차량이 있거나 센서가 오래됐거나 입력이 반복되면 복구 확인이 진행되지 않는다.
- 현재 경로 앞에 있던 정지 객체가 사라졌거나, 이미 conflict 안에 들어간 상태에서 객체를
  놓친 경우에는 자동 해제하지 않는다. 재검출/진단이 필요하다.
- 이 복구는 현재 검출 결과에 기반한 시뮬레이터 시험 정책이다. 객체 목록에는 가림/자유공간
  정보가 없으므로 모든 차량을 못 보는 상황에서 안전함을 증명하는 기능은 아니다.

이미 v4를 설치했다면 기존 launch를 Ctrl+C로 끈 후 새 ZIP을 Linux 호스트 Downloads에 저장한다.
호스트에서:

```bash
sudo docker cp "$HOME/Downloads/roundabout_mission_v41.zip" morai_noetic_gui:/tmp/roundabout_mission_v41.zip
```

기존 Docker 터미널에서:

```bash
mkdir -p /root/roundabout_backups
cp -a /root/morai_ws/src/ROI/src/control/roundabout_mission /root/roundabout_backups/roundabout_mission_before_v41_$(date +%Y%m%d_%H%M%S)
python3 -m zipfile -e /tmp/roundabout_mission_v41.zip /root/morai_ws/src/ROI/src/control
bash /root/morai_ws/src/ROI/src/control/roundabout_mission/scripts/install_v41.sh
source /opt/ros/noetic/setup.bash
source /root/morai_ws/devel/setup.bash
roslaunch roundabout_mission roundabout_standalone.launch \
  mission_config:=/root/roundabout_mission_v41.yaml \
  morai_host_ip:=192.168.0.147 enable_control:=true rviz:=false
```

`VERIFIED 4.1.0`을 확인한다. 기존 v4 YAML과 CSV는 유지한다. 사용자가 보고한
(-99.17,315.91)은 기존 entry 기준 이전이므로 그 위치에서 재실행할 수 있다.
순간 이동한 경우 EKF가 안정되고 진단상의 좌표가 MORAI 위치와 맞는지 확인한다.

v4.1 설정은 기존 v4 YAML의 낮은 `merge_speed/test_speed_limit`도 그대로 사용하지
않는다. 기본 `minimum_merge_speed=3.0 m/s`를 적용해 이동 차량이 있는 빈틈을
실제로 통과할 수 있게 한다. 접근 구간의 일반 CSV 주행은 `cruise_speed=2.0 m/s`이고,
게이트가 허가한 뒤에만 이 병합 속도를 요청한다.

## 목적과 실행 구성

시작점부터 기존 `data/routes/2026_molit_comp_global_path.txt`를 추종하고,
회전교차로에서 진입 시점 및 속도를 결정하고, 동일 경로를 따라 우측 출구로 나간다.
이 패키지가 CSV를 새로 설계하거나 RRT*로 경로를 바꾸지는 않는다.

- 센서: 기존 morai_udp_bridge, gps_mgeo_converter, ekf_local_enu, lidar_perception 노드 재사용.
- 조향: 기존 `purepursuit_mgeo.path.MgeoPurePursuit` 재사용.
- 페달: 기존 `path_planning.longitudinal_controller.PedalSpeedController` 재사용.
- 시험 제어기: roundabout_test_driver 하나가 `/ctrl_cmd`를 발행한다.
- 판단: roundabout_merge_gate가 atomic JSON으로 정지·목표 속도·상태를 제공한다.
- 기존 통합 주행 launch / RRT* / highway 전략 / old merge gate는 동시에 실행하지 않는다.
- 이 시험에는 카메라/YOLO를 포함하지 않는다. 카메라 융합은 구현되지 않았다.

기존 팀 제어기 전체 파일을 대체한 것은 아니다. 그 제어기가 사용하는 조향/PID 모듈을
재사용한 시험 어댑터다. 최종 제출의 핵심은 core.py와 roundabout_merge_gate.py이며
팀 제어기에 통합할 때 시험 드라이버를 끈다.

`test/rrt`의 `1583f15`와 의존 코드를 비교했고, 해당 브랜치의 조향 모듈로
단위 테스트와 원본 경로 운동학 시험을 통과했다. 브랜치를 바꿀 필요는 없다.

## v1~v3에서 바뀐 사항

1. 중복/역순 센서 패킷은 마지막 정상 샘플을 지우거나 유효시간을 갱신하지 않는다.
2. approach 이전과 exit 이후에는 회전교차로의 LiDAR 정지 판단을 적용하지 않는다.
   GPS/EKF odometry와 판단 노드 heartbeat는 정상이어야 한다.
3. 큰 외접원 대신 방향이 있는 차체/객체 사각형의 연속 SAT로 충돌시각을 계산한다.
   옆의 작은 물체를 일괄 삭제하지 않고 실제 차체 겹침을 검사한다. 정지차도 유지한다.
   상수 속도/방향의 단기 예측이므로 회전 중 정확한 물리 예측은 아니다.
4. 순환차와 자차 경로의 conflict 구간을 같은 위치의 차체 겹침으로 산출한다.
   기존 [1802,1808]와 [55,65]는 지도상 서로 맞지 않았다.
5. 설치 시 로딩되는 소스의 실제 경로/버전을 검증한다. host Downloads와 Docker root
   Downloads는 같은 위치라고 가정하지 않는다.
6. /ctrl_cmd를 브리지와 명시적으로 연결한다. v3의 토픽 분리만으로 원인이 해결됐다고
   볼 수 없으며, v4를 설치하면 v3는 별도 적용할 필요가 없다.

## 설치 (호스트와 Docker 구분)

1. 기존 주행/센서 roslaunch를 Ctrl+C로 종료한다. 컨테이너는 켜 둔다.
2. ZIP를 Linux 호스트의 Downloads에 저장한다.
3. **Linux 호스트**:

```bash
sudo docker cp "$HOME/Downloads/roundabout_mission_v4.zip" morai_noetic_gui:/tmp/roundabout_mission_v4.zip
sudo docker exec -it morai_noetic_gui bash
```

4. **Docker 안**:

```bash
mkdir -p /root/roundabout_backups
cp -a /root/morai_ws/src/ROI/src/control/roundabout_mission /root/roundabout_backups/roundabout_mission_before_v4_$(date +%Y%m%d_%H%M%S)
unzip -o /tmp/roundabout_mission_v4.zip -d /root/morai_ws/src/ROI/src/control
bash /root/morai_ws/src/ROI/src/control/roundabout_mission/scripts/install_v4.sh
```

설치 중 `VERIFIED 4.1.0`이 표시되어야 한다. 기존 `/root/roundabout_mission.yaml`과
자차 CSV, `/root/roundabout_circulating.csv`는 수정하지 않는다.
새 설정은 `/root/roundabout_mission_v4.yaml`에 저장한다. 기존 YAML이 다른 경로라면
install_v4.sh 뒤에 그 경로를 전달한다.

## 실행

MORAI에서 차량을 기존 경로의 시작점에 놓고 외부 제어 모드로 전환한다.
시작 좌표는 대략 (-131.69, -428.33)이다. 앞서 수동으로 배치했던 회전교차로 앞
(-99.36,307.78)에서도 시작할 수 있으나, 첫 검증은 경로 시작점에서 한다.
Teleport/시나리오 리셋 후에는 launch도 재시작하여 EKF/진입 상태를 초기화한다.

```bash
source /opt/ros/noetic/setup.bash
source /root/morai_ws/devel/setup.bash
roslaunch roundabout_mission roundabout_standalone.launch \
  mission_config:=/root/roundabout_mission_v4.yaml \
  morai_host_ip:=192.168.0.147 enable_control:=true rviz:=false
```

다른 launch를 추가 실행할 필요가 없다. 기본 시험 속도는 2 m/s(7.2 km/h)이며
급한 곡선에서는 조향 곡률에 따라 속도 상한을 낮춘다. 첫 시험은 다른 차량 없이
CSV 경로 추종/미션 통과를 확인하고, 다음에는 순환 차량을 추가해 WAIT→진입을 확인한다.
신호등/보행자/일반 장애물 회피 미션까지 시험하는 통합 주행 구성은 아니다.

정상 상태: CRUISE → APPROACH/READY/WAIT → COMMIT → FOLLOW → EXIT.
`WAIT`에서 `READY/COMMIT`으로 바뀐 뒤 상태의 `target_speed`가 0보다 커지는지
확인한다. `predicted_blocking_ids`가 비어 있고 `interval_blocking_ids`만 남는 경우는
현재 위치·속도 기준의 단순 구간검사에는 겹치지만, 경로 기반 3×3 속도 예측에서는
충돌하지 않는 빈틈으로 판정된 경우다.
회전교차로 밖에서는 `allowed=false`여도 정상이다. 이 필드를 전역 정지 조건으로 쓰지 않는다.

## 한 번에 진단

별도 호스트 터미널:

```bash
sudo docker exec -it morai_noetic_gui bash -c 'source /opt/ros/noetic/setup.bash; source /root/morai_ws/devel/setup.bash; rosrun roundabout_mission roundabout_diagnose.py'
```

이 명령은 읽기만 하며 제어 명령을 보내지 않는다. 전체 출력을 전달하면
실행 버전, 파일 경로, publisher/subscriber, IP/포트, 센서 유효성, 미션/드라이버
정지 사유와 실제 명령을 함께 볼 수 있다.

- `emergency_collision_risk`: emergency_track_ids와 위치/차량 자세를 확인한다.
- `obstacles_stale_or_invalid`: 미션 구간의 LiDAR 입력이 끊겼다.
- `stale_or_invalid_input`/`driver_input_stale`: odometry 또는 판단 메시지 유효시간 문제.
- `track_loss_stopping`: 소실 후 정지 중.
- `track_loss_waiting_current_gap`: 현재 검출 차량 때문에 간격 대기 중.
- `track_loss_rechecking_gap`: 새 메시지와 안전 간격을 연속 확인 중.
- `track_loss_recovered_recheck_gap`: 과거 소실 기록 해제 완료; 진입 간격 재확인 후 출발.
- `lost_stationary_obstacle_on_path`: 경로 앞 정지 객체가 사라져 자동 해제 불가.
- `track_loss_inside_conflict_reacquire`: conflict 내부에서 소실되어 재검출 필요.
- `id_handoffs`, `lost_track_ids`, `recovered_track_ids`, `recovery_frames`,
  `recovery_span_s`가 상태 토픽과 CSV 로그에 추가됐다.
- `uncommitted_entry_restart_before_stop_line`: 진입 후 위치에서 새로 실행했다.
- `off_path`: CSV로부터 설정된 허용거리(기본 3m)를 벗어났다.

## 팀 제어기에 최종 통합

기존 센서/브리지가 이미 켜져 있으면 standalone을 실행하지 않는다.
기존 old merge gate를 끄고 아래 판단 노드만 실행한다.

```bash
roslaunch roundabout_mission roundabout_test.launch \
  mission_config:=/root/roundabout_mission_v4.yaml run_driver:=false
```

팀 제어기는 `/roundabout_merge_gate/status`를 받아, PID 전에 다음 계약을 적용한다.

```python
from roundabout_mission.integration import apply_mission
# status는 수신한 JSON; freshness는 header timestamp와 수신 후 monotonic age 모두 검사.
effective_speed, mission_stop, reason = apply_mission(planned_speed, status, freshness)
stop = existing_stop or mission_stop
# stop이면 accel=0/brake, 아니면 기존 PID.compute(effective_speed, actual_speed, now)
# 조향은 기존 controller가 담당. CtrlCmd publisher도 기존 controller 하나만 유지.
```

다른 미션의 제한속도를 올리지 않고 min으로 합성한다. source 파일에는 import 가능한
순수 함수와 테스트가 포함되어 있다. 특정 팀 controller 파일에 직접 적용한 패치는 아니다.

## 대회 네트워크 제한

사용자 제공 2-8/2-4 규정을 기준으로 한다.
- CtrlCmd: `longlCmdType=1`; 기존 UDP 브리지 `cmd_type=1, ctrl_mode=2, gear=4`.
- Windows MORAI Publisher 항목은 반드시 **Competition Vehicle Status**를 선택한다.
  포트 번호만으로 일반 Ego Vehicle Status와 구분할 수 없으므로 항목 이름도 확인한다.
- 속도형(cmd_type=2), 가속도형(cmd_type=3), Ghost/teleport 명령 사용 없음.
- GPS/IMU/LiDAR 센서 입력 활용. 위치는 GPS/IMU EKF의 `/localization/odometry`에서만 사용.
- Competition Vehicle Status 수신은 기존 브리지 그대로. 제공되지 않는 pos/vel_y/vel_z/
  accel_xyz 등을 미션/제어의 입력으로 사용하지 않는다. /Ego_topic은 내부 ROS 호환 토픽명이며
  여기서는 Competition Status UDP를 변환한 출력이다; 금지된 Ego Vehicle Status 수신을 뜻하지 않는다.
- NPC ObjectInfo, Ego Detail Status, 일반 Ego Vehicle Status, Get TrafficLight Status 수신/요청 없음.
- CollisionData는 허용되지만 이 시험에 별도 수신기를 추가하지 않았다.
- CSV/MGeo는 경로 기준 자료이고 다른 차량 위치는 LiDAR에서만 얻는다.
- 내부 ROS 토픽/파라미터 조회와 MORAI 외부 UDP 네트워크를 구분한다.

MORAI 목적지/수신 포트: GPS 3001, IMU 4001, LiDAR 2001, Competition Status 9081
(송신 9080), 제어 Windows 192.168.0.147:9093 (Linux source 9094).
센서 목적지 IP는 기존에 정상 동작한 Linux IP 설정을 사용한다.

## 검증 범위

단위 테스트, 업로드된 도로변 객체 수치 재현, 원본 CSV 기반 운동학 폐루프 시험을 수행한다.
실제 사용자 Windows MORAI/Docker에는 직접 접속하지 않았으므로 실제 통과 성공은 아직 미확인이다.
기하 파라미터와 conflict 구간은 지도 기반 초깃값이며 실시간 LiDAR와 주행 기록으로 검증해야 한다.

```bash
cd /root/morai_ws/src/ROI
export PYTHONPATH="$PWD/src/control/roundabout_mission/src:$PWD/src/control/purepursuit_mgeo/src:$PWD/src/control/path_planning/src:$PYTHONPATH"
python3 -m unittest discover -s src/control/roundabout_mission/test -q
python3 src/control/roundabout_mission/test/replay_empty_route.py /root/roundabout_mission_v4.yaml
```

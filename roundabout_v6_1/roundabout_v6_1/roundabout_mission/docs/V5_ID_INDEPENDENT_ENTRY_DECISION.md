# v5.0.0 — ID-independent entry decision

기준: 사용자가 첨부한 `roundabout_mission_v42_final_modified(2).zip`.
원본 ZIP은 수정하지 않았으며 SHA-256은 validation_v5/static_checks.json에 기록했다.
이 결과는 MORAI 실차량 동작을 검증한 릴리스가 아니라, 로컬 단위 검증을 마친 MORAI 시험용 패키지다.

## 문제와 변경 범위
가림막 뒤에서 bbox/centroid/velocity/ID가 흔들리는데 v4.2는 lost/re-ID 복구와 gap 판단이 결합되어 있었다.
v5는 raw ID를 제어 조건으로 사용하지 않는다. ID는 상태 출력에만 남긴다.
core.py의 복구·handoff·lost-ID 대기를 제거하고 entry_decision.py로 입력 이력과 conflict 평가를 분리했다.
이전 YAML의 reid_*, lost_recheck_* 값은 호환을 위해 남아 있지만 판단에는 사용하지 않는다.
외부 tracker, Pure Pursuit, PID, accel/brake 생성기, 경로 파일은 교체하지 않는다.

## 새 흐름
1. 새로운 sensor timestamp의 관측만 수용한다.
2. 최근 0.8초의 위치/속도 관측을 순환 경로에 따라 전파한다. ID를 key로 저장하지 않는다.
3. 거의 동일한 공간·속도·크기 가설만 중복 제거한다. 서로 다른 split 조각은 보수적으로 남긴다.
4. ROI, 순환 경로와의 횡거리, 속도 >= min_vehicle_speed, 진행방향 cosine 조건으로 후보를 고른다.
5. 차량의 앞/뒤 footprint가 conflict에 들어오고 나가는 시간 구간을 계산한다.
6. ego의 앞 진입부터 뒤 이탈까지의 시간과 비교한다. 앞 buffer 0.8초, 뒤 buffer 1.2초를 적용한다.
7. 기존 sampled footprint collision predictor도 독립적으로 적용한다. 진입 전은 conflict clearance까지,
   COMMIT 후에는 기존처럼 exit까지 예측한다. 이 검사는 순환 후보가 아닌 이동 물체도 포함한다.
8. 위험이 없고 서로 다른 센서 frame에서 0.3초가 확인되면 COMMIT한다. timer tick만으로 확정하지 않는다.

차량이 먼저 통과: object_out + front_buffer < ego_in.
ego가 먼저 통과: ego_out + rear_buffer < object_in.
위 둘 중 하나가 성립하지 않거나 물리 궤적 충돌이 예측되면 위험이다.
단순 ID 변화, ID 개수 증가, ID 소실 자체는 SAFE timer reset 조건이 아니다.
짧은 미검출 중에는 이전 관측에서 전파된 위험이 여전히 timer를 막는다.

## COMMIT과 안전
COMMIT은 mission exit까지 latch된다. 일반 gap blocker가 나타나도 WAIT로 돌아가지 않는다.
TTC, 선행차 최소거리, 실제 footprint collision, odometry/LiDAR watchdog, off-path 검사는 계속 작동한다.
비상 시 state=COMMIT을 유지하면서 stop_required=true, emergency=true, target_speed=0을 출력한다.
기존 driver가 emergency를 받으면 accel=0, brake=1로 처리한다.
정지 구조물은 circulating vehicle 후보가 아니다. 다만 실제 ego 경로를 막는 정지 footprint는 별도 위험이다.
이 정지 장애물은 빈 detection 메시지로 제거하지 않는다. 앞으로의 정지거리 cap도 적용했다.
이는 원본의 정지 선행물체 누락을 보완한 유일한 추가 제동 동작이다. PID/페달 계산식은 변경하지 않았다.
정지 장애물 오검출이 경로에 남으면 정지할 수 있으므로 현장 확인 후 mission을 재시작해야 한다.

## 유지된 설정과 새 설정
원본 config의 모든 기존 key/value를 비교하여 동일함을 확인했다.
- merge_speed / roundabout_speed / maximum_merge_speed / test_speed_limit: 5.0 m/s
- max_accel: 2.5, speed_kp: 0.30, max_accel_pedal: 1.0, max_accel_rate: 3.0
- front_time_buffer_s: 0.8, rear_time_buffer_s: 1.2, safe_confirm_s: 0.3

추가된 값:
- entry_occlusion_hold_s: 0.8 (초). 기존 track_hold_s=0.4는 보존하지만 v5 관측 이력은 새 값을 사용한다.
- entry_vehicle_length_m: 4.5, entry_vehicle_width_m: 1.8 (m).
  순환 차량 후보의 작은 bbox를 이 크기 이상으로 판단한다. 큰 bbox는 줄이지 않는다.
  정지 가림막이나 순환 조건을 만족하지 않는 잡음에는 이 확대를 적용하지 않는다.

주의: 0.8초는 현장 가림 시간을 계측해 확정한 값이 아니다. 그보다 긴 개별 차량 미검출은 보장하지 못한다.
이력을 늘리면 보수적 대기와 연산량도 증가한다. safe_confirm_s를 줄여 이를 해결하지 말 것.

## 수정 파일
- src/roundabout_mission/core.py: 진입 판단 연결, ID recovery 제거, latch/status, 정지 장애물 제동
- src/roundabout_mission/entry_decision.py: 새 관측 이력/후보/ETA/충돌 평가
- src/roundabout_mission/visualization.py: RViz gap MarkerArray
- src/roundabout_mission/ros_io.py: CSV 진단 필드
- src/roundabout_mission/__init__.py, package.xml: 5.0.0 및 marker 의존성
- scripts/roundabout_merge_gate.py: marker publisher
- scripts/prepare_roundabout_v5.py, scripts/install_v5.sh: 기존 설정을 보존하는 설치 경로
- config/roundabout.yaml: 새 파라미터 3개만 추가
- CMakeLists.txt: 새 스크립트/의존성/문서 설치
- test/test_v5_entry.py, test/test_v5_visualization.py: 새 테스트
- README.md 및 docs: 실행/검증 기록

scripts/roundabout_test_driver.py, prediction.py, integration.py, 기존 두 launch는 byte-identical이다.
전체 diff는 validation_v5/source.diff에 있다. 과거 docs의 benchmark 수치는 v5 결과가 아니다.

## 설치 및 실행 — 기존 ROS Noetic 컨테이너
실행 중인 주행 launch를 종료한 뒤, 기존 패키지를 workspace 밖으로 백업한다.
중복 패키지를 src 내부에 backup 이름으로 남기면 catkin이 중복 package를 찾을 수 있다.

```bash
cd /root/morai_ws/src/ROI/src/control
mv roundabout_mission /root/roundabout_mission_v42_backup_$(date +%Y%m%d_%H%M%S)
# 받은 ZIP의 실제 경로를 지정한다.
unzip /root/roundabout_mission_v5_id_independent.zip -d /root/morai_ws/src/ROI/src/control
bash roundabout_mission/scripts/install_v5.sh /root/roundabout_mission_v42.yaml /root/roundabout_mission_v5.yaml
source /root/morai_ws/devel/setup.bash
```

기존 YAML 이름이 다르면 첫 인자를 실제 파일로 바꾼다. 새 출력 YAML이 이미 있으면 다른 이름을 지정한다.
install_v5는 경로나 인덱스/속도를 재계산하지 않는다. install_v42.sh / prepare_roundabout_v4.py는 과거용이므로 사용하지 않는다.
첨부 ZIP에는 실제 CSV와 runtime mission YAML이 없다. 기본 config/roundabout.yaml의 -1 인덱스로 실행하면
의도적으로 오류가 난다. 기존에 잘 주행했던 실제 YAML과 경로가 반드시 필요하다.

```bash
roslaunch roundabout_mission roundabout_standalone.launch \
  mission_config:=/root/roundabout_mission_v5.yaml \
  morai_host_ip:=192.168.0.10 enable_control:=false rviz:=true
```

위 IP는 예시이므로 실제 MORAI Windows IP로 바꾼다. 기존 port/extrinsic 인자가 있었다면 동일하게 붙인다.
입력과 marker를 확인한 다음 같은 명령의 enable_control:=true로 주행한다.
기존 다른 CtrlCmd publisher와 동시에 실행하지 않는다.

## RViz
Fixed Frame = 기존 map_frame (기본 map).
Add → By topic → /roundabout_merge_gate/gap_markers → MarkerArray.
기존 LiDAR RViz 설정은 수정하지 않았으므로 처음 한 번 직접 추가 후 저장한다.
- 초록: COMMIT/FOLLOW가 허가되고 stop/emergency가 아닌 ego 진입 통로
- 노랑: 위험은 없지만 SAFE 확인 중
- 빨강: 위험이 검출되었거나 비상제동 중
- 텍스트: state, reason, 위험 가설 수, SAFE 경과시간
- inactive이면 삭제하고 publisher가 멈추면 marker는 0.5초 뒤 사라진다.

표시 선은 정지선~conflict_end의 ego CSV 경로다. NPC 사이의 실제 측정된 빈 공간이나
가림막 뒤 free-space 지도를 뜻하지 않는다. COMMIT latch 중 장기 예측 위험이 생겨도
즉시 제동 조건이 아니면 허가 통로는 초록으로 유지될 수 있다. 숫자/텍스트와 함께 본다.

## 2026-10-06 MORAI 확인
```bash
rostopic echo /roundabout_merge_gate/status
rostopic echo /roundabout_merge_gate/stop_required
rostopic echo /roundabout_merge_gate/target_speed
rostopic hz /perception/lidar/tracked_obstacles_map
rosbag record -O /root/v5_entry_test.bag \
  /perception/lidar/tracked_obstacles_map /localization/odometry \
  /roundabout_merge_gate/status /roundabout_merge_gate/gap_markers \
  /roundabout_merge_gate/stop_required /roundabout_merge_gate/target_speed /ctrl_cmd
```

중점 status: dangerous_vehicle_count, conflict_candidate_count, minimum_conflict_eta,
ego_conflict_eta, minimum_time_gap, safe_duration, commit_latched, emergency_active,
decision_reason, decision_compute_ms. null ETA는 해당 후보 없음/해당 구간 평가 안 함을 뜻한다.
count는 보수적 관측/충돌 가설 수이며 실제 차량 대수와 같지 않을 수 있다. ID별 차량 수가 아니다.
minimum_time_gap은 시간구간 사이의 signed gap이며 음수면 겹침이다. footprint predictor가
추가로 위험을 검출할 수 있으므로 시간 gap 하나만 보고 permission을 해석하면 안 된다.

성공: 위험차 접근 시 WAIT; ID 54→71/split에도 위험 유지; 통과 후 SAFE 0.3초 확인;
COMMIT 후 일반 ID 변동으로 WAIT 복귀 없음; 실제 근접 충돌은 emergency 제동;
CSV 추종/내부 속도/탈출은 기존과 동일.
실패: 위험 접근 중 녹색 허가, 진입 충돌, 상시 false blocker, ID 변화만으로 대기,
센서 stale 또는 계산지연으로 반복 제동, marker와 status의 불일치.

## 검증과 한계
새 16개 테스트는 요청 CASE 1~8 및 ID 순열 불변성, harmless ID 교체,
신규 frame 확인, watchdog/latch, 정지 장애물 유지, RViz 색/삭제를 포함하고 통과했다.
ROS transport는 실제 통신이 아닌 double을 사용했다. ROS Noetic catkin 빌드/MORAI/RViz GUI는
이 환경에 없어서 실행 검증하지 못했다. Python 3.8 문법, Python 실행 테스트, XML/YAML 파싱은 통과했다.

원본 테스트는 수정하지 않고 두 버전에서 실행했다. v4.2도 99개 중 14개가 실패한다.
v5에는 이미 폐기한 lost/cache/handoff 내부를 직접 조작하는 테스트 및 COMMIT 취소를 기대하는
테스트가 있어 전체 과거 suite는 통과하지 않는다. 모든 실패를 validation_v5에 원문과 분류표로 남겼다.
새 테스트 통과를 전체 과거 suite 통과라고 해석하면 안 된다.
테스트 의존 코드와 맵은 로컬의 별도 ROI checkout을 읽기만 했고 이 ZIP에 복사/수정하지 않았다.
기존 traffic_benchmark의 ROOT가 설치 위치에 의존하므로 비교 실행에서는 ROOT만 해당 checkout으로 지정했다.

성능: 합성 32-object ID churn에서는 median 약 39ms, max 약 45ms.
0.8초 동안 위치/속도 잡음을 추가한 32개 객체에서는 454개의 가설로 약 359ms가 걸렸다.
이를 이유로 안전 가설을 임의 삭제하거나 watchdog 시간을 완화하지 않았다. 실제 PC의 decision_compute_ms를
반드시 확인한다. 20Hz 목표는 50ms 이하이며 300ms 이상은 기존 decision watchdog과 충돌한다.
이런 부하가 실제 발생하면 history 축약/예측 벡터화의 추가 개발이 필요하다.

완전히 가려져 관측된 적 없는 차량, 0.8초를 초과하는 가림, 큰 속도 오차, 잘못된 좌표계/경로,
차량 크기/odom 기준점 오류는 이 변경만으로 해결되지 않는다. 이력 크기 확대는 오대기 가능성이 있다.
새 로직은 추적 ID의 정확한 복원을 보장하는 tracker가 아니다.

로컬 기본 회귀 실행:
```bash
cd /root/morai_ws/src/ROI/src/control/roundabout_mission
PYTHONPATH=src:$PYTHONPATH python3 -m unittest discover -s test -p 'test_v5_*.py' -v
```

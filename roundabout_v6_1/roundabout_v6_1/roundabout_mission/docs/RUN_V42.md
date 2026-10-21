# v4.2 설치와 실행

v4.1 로그의 `track_loss_waiting_current_gap / emergency=true / brake=1`은
즉각적인 충돌을 검출해서 나온 명령이 아니라, 과거 객체 ID 소실 분기에서 나온
명령이었다. 센서 유효성과 제어 토픽 연결은 정상이다.

v4.2는 접근과 진입 허가를 분리한다.

1. `CRUISE`: 원래 CSV를 따라 2 m/s로 주행한다.
2. `APPROACH`: 같은 CSV로 고정 대기 위치까지 접근한다. 먼 순환 차량의 빈틈이
   열리고 닫혀도 APPROACH와 COMMIT 사이를 반복하지 않는다. 앞 장애물과
   즉각적인 충돌 위험은 계속 검사한다.
3. `WAIT`: 정지선 근처에서 현재 모든 추적 객체의 크기·방향·위치·속도를 사용해
   4 / 3.5 / 3 m/s 순서로 후보의 차체 겹침을 검사한다. 안전 여유와 느린 출발
   가정도 동일하게 적용한다. 빠른 후보가 안전하면 그 속도를 선택하고 앞차와
   겹칠 때만 낮은 후보를 확인한다. 가능한 후보를 서로 다른 센서 메시지로
   0.8초 확인한다. 확인 중인 후보는 유지해 속도 선택이 반복해서 바뀌지 않게 한다.
4. `COMMIT/FOLLOW`: 선택한 목표 속도를 PID에 전달하고 기존 Pure Pursuit로
   동일 경로를 추종한다. 앞차 간격과 곡률 제한을 적용하며 우측 출구로 나간다.

움직이는 객체의 소실 기록을 복구하는 2초 / 10프레임 확인은 현재 순환 차량이
진입을 막는 동안에도 진행된다. 기록 해제만으로 진입 허가가 생기지는 않는다.
현재 객체의 충돌 검사와 0.8초 확인이 별도로 필요하다. 정지 장애물이 경로에서
사라지거나 이미 conflict에 들어간 상태에서 중요 객체를 놓치면 재검출이 필요하다.
객체 ID 소실과 일반 대기는 `emergency=false`로 처리한다. 센서 단절·입력 오류와
1초 이내의 충돌 위험에는 즉시 정지한다.
정지 판정 속도(0.25 m/s) 미만의 칼만 속도 잔차는 장기 예측에서 객체를 계속
이동시키지 않는다. 현재 바운딩 박스는 유지해 경로 위의 정지차도 검사한다.

기본 상한은 접근 2 m/s(7.2 km/h), 병합 최대 4 m/s(14.4 km/h)이다.
기존 v4.1 설정을 새 설정 파일로 변환할 때 시험 상한을 4 m/s까지 올린다.
`maximum_merge_speed`와 `test_speed_limit`을 낮추면 그 제한을 넘지 않는다.
빠른 순환 차량이나 조밀한 흐름에서는 모든 후보가 차단되어 계속 대기할 수 있다.
현재 경로 주변의 LiDAR 목록으로 가림 상태를 증명할 수는 없으며 MORAI 검증이 필요하다.

## Linux 호스트에서 파일 복사

새 ZIP을 호스트 Downloads에 저장한 뒤 실행한다. Docker는 이미 켜져 있어도 된다.

```bash
sudo docker cp "$HOME/Downloads/roundabout_mission_v42.zip" morai_noetic_gui:/tmp/roundabout_mission_v42.zip
```

## 현재 Docker 터미널에서 설치

현재 주행 launch를 Ctrl+C로 종료한 뒤 실행한다.

```bash
mkdir -p /root/roundabout_backups
cp -a /root/morai_ws/src/ROI/src/control/roundabout_mission /root/roundabout_backups/roundabout_mission_before_v42_$(date +%Y%m%d_%H%M%S)
python3 -m zipfile -e /tmp/roundabout_mission_v42.zip /root/morai_ws/src/ROI/src/control
bash /root/morai_ws/src/ROI/src/control/roundabout_mission/scripts/install_v42.sh
source /root/morai_ws/devel/setup.bash
```

`VERIFIED 4.2.0`과 `/root/roundabout_mission_v42.yaml` 생성을 확인한다.
설치 스크립트는 v4.1 YAML을 입력으로 사용한다. 없으면 v4 YAML을 사용한다.
기존 CSV와 YAML은 보존하고 새 YAML을 만든다.

```bash
roslaunch roundabout_mission roundabout_standalone.launch \
  mission_config:=/root/roundabout_mission_v42.yaml \
  morai_host_ip:=192.168.0.147 enable_control:=true rviz:=false
```

센서와 제어기는 이 launch 하나에 포함된다. 기존 주행 launch를 함께 켜지 않는다.
현재 CSV의 가까운 점에서 시작할 수 있다. Teleport/시나리오 리셋 시 launch를 재시작하고
GPS/IMU EKF 좌표가 안정된 후 사용한다.

## 진단

같은 Docker의 다른 터미널에서 실행한다.

```bash
source /opt/ros/noetic/setup.bash
source /root/morai_ws/devel/setup.bash
rosrun roundabout_mission roundabout_diagnose.py
```

새 필드 `decision_phase`, `stop_line_distance_m`, `safe_frames`,
`selected_merge_speed_mps`, `merge_speed_candidates`, `blocking_track_details`를 확인한다.
사용자가 보고한 s=875.0755는 기존 대기 위치보다 약 0.327m 앞이다. 현재 설정에서
후차축 대기 위치는 약 (-99.514, 308.104)이며 사용자가 말한 진입 근방 좌표와 다르다.
stop_line 좌표에서 `ego_front_m + stop_margin_m`을 뺀 기준이다.
`ego_front_m`은 기존 뒤차축 기준 3.6m 초깃값이므로 차량 모델에 맞게 검증해야 한다.

반복 정지의 시간 순서를 기록하려면 아래 명령을 사용한다. 파일은 새로 생성하므로
같은 파일이 이미 있으면 `--output`에 다른 이름을 지정한다. 제어 명령을 보내지 않는다.

```bash
rosrun roundabout_mission roundabout_diagnose.py \
  --record-seconds 15 --output /tmp/roundabout_diag_v42.jsonl
```

호스트에서 이 기록을 꺼낼 수 있다.

```bash
sudo docker cp morai_noetic_gui:/tmp/roundabout_diag_v42.jsonl "$HOME/Downloads/roundabout_diag_v42.jsonl"
```

## 최종 팀 제어기 연동 및 대회 네트워크

센서·브리지가 기존 launch에 있으면 `roundabout_test.launch run_driver:=false`로
판단 노드만 사용한다. 팀 제어기는 `integration.apply_mission`으로 PID 전에
목표 속도와 정지 요구를 적용하며 CtrlCmd publisher는 하나만 둔다.

기존 clustering → map 변환 → Kalman/Hungarian confirmed 추적 토픽을 사용한다.
차종 분류나 카메라 융합은 구현되어 있지 않다. 정지 객체도 형상으로 충돌 검사한다.
GPS/IMU EKF로 자차 위치를 얻고 다른 차량은 LiDAR에서 얻는다.
UDP 제어는 cmd_type=1, ctrl_mode=2이며 Competition Vehicle Status를 사용한다.
Ego Detail Status, 일반 Ego Vehicle Status, Get TrafficLight Status, NPC ObjectInfo는 사용하지 않는다.
기존 포트는 GPS 3001, IMU 4001, LiDAR 2001, Competition Status 9081,
Windows 192.168.0.147 제어 9093, Linux 제어 송신 9094이다.

# 회전교차로 V6.1 — 앞차 예측 정지를 상대속도 제어로 변경

사용자가 보낸 원본 V5.0.0을 기준으로 만든 V6의 추가 비교본이다. 패키지와 상태 로그의 버전은 `6.1.0`이다. 앞차 기본 간격은 1 m이고 기존 1792번 정지 위치를 사용한다.

## 앞차를 따라가는 경우

진입 허가 이후 `COMMIT`/`FOLLOW`에서 현재 가장 가까운 **같은 경로의 움직이는 앞차**를 추종할 때 적용한다. raw ID를 고정해서 예외 처리하지 않으므로 ID가 바뀌어도 현재 위치·속도로 다시 앞차를 선택한다.

- 선정한 앞차의 일정 속도 충돌예측 TTC만으로 `Emergency/Stop`을 요청하지 않는다.
- 선정한 움직이는 앞차와 현재 간격이 기존 긴급 간격 1.5 m보다 작다는 이유만으로 정지를 요청하지 않는다.
- 내 차가 더 빠르게 접근하면 앞차 속도와 남은 간격으로 목표 속도를 낮춘다. 실제 PID 드라이버는 속도를 줄이기 위해 브레이크를 사용할 수 있다. **정지 요청을 없애는 것과 브레이크를 없애는 것은 다르다.**
- 앞차가 같은 속도로 가거나 멀어지면, 목표 간격이 부족하다는 이유만으로 목표 속도를 앞차보다 낮춰 정차시키지 않는다.

`desired_gap = 1 + 1.5 × 내 속도`는 기록하고 가속 여유를 판단하는 데 사용한다. V6.1의 이 추종 모드에서는 목표 간격이 부족할 때 앞차 속도에 맞추며, 목표 간격을 회복하려고 앞차보다 느리게 가는 V6의 음수 간격 보정은 사용하지 않는다.

상대속도 제한은 다음 조건을 이용한다.

```text
사용 가능한 현재 간격 = max(0, lead_gap - 1 m)
닫히는 속도 차이 × 반응시간 + 속도 차이² / (2 × 감속도)
    <= 사용 가능한 현재 간격
```

반응시간 기본값은 0.3초, 감속도는 기존 `comfortable_decel: 1.5`를 사용한다. 이 모델에서 허용되는 속도 차이에 앞차의 경로 방향 속도를 더해 속도 상한을 계산한다. 기본 목표 속도와 간격 보정 속도도 함께 비교한다. 두 숫자는 실제 MORAI 차량 응답을 새로 측정한 보장값이 아니다.

예: 내 속도 3.4 m/s, 앞차 2 m/s, 현재 간격 1.3 m인 합성 시험에서 V6은 TTC 약 0.79초로 긴급 정지를 요청했다. V6.1은 선정한 앞차의 예측 정지를 억제하고 **목표 속도 2 m/s**를 요청했으며, 실제 PID 코드가 감속 브레이크를 출력했다.

## 기존 처리를 사용하는 경우

앞차 추종 예외를 적용하려면 ego와 앞차가 전역 경로 중심에서 각각 0.9 m 이내이고, 각자의 경로 진행 방향과 자차 방향/앞차 속도 방향의 코사인이 0.9 이상이어야 한다. 현재 차체 안전 경계가 분리되어 있어야 한다. 옆에서 교차하는 차량을 추종 앞차로 잘못 제외하지 않기 위한 조건이다.

다음 경우는 기존 보호를 사용한다.

- 진입 허가 전의 앞차·뒤차 갭과 충돌예측 판단.
- 뒤차에 관한 TTC 처리, 현재 충돌 판단, 기억·관측 처리. 원본 V5가 COMMIT 이후 명확히 뒤에 있는 차량을 TTC 급제동 원인에서 제외하던 처리도 그대로다. 뒤차 처리를 새로 강화하거나 삭제하지 않았다.
- 정지했거나 매우 느려서 추종 차량으로 선정되지 않는 전방 장애물. 정적 장애물의 제동 여유는 별도 `static_minimum_gap_m: 5.0`으로 보존했다.
- 현재 차체 안전 경계의 겹침, 옆·교차 차량 위험, 인지/위치 입력 단절과 경로 이탈.

앞차가 실제로 멈췄거나 현재 겹침이 발생한 상황까지 계속 주행시키는 시험은 아니다.

## 버전 비교

| 버전 | 판단을 위한 정지 위치 | 앞차 기본 간격 | 주요 차이 |
|---|---|---:|---|
| V5.3 | 1792번 정지 후 차량 있는 scheduled에서 관측 전진, 기본 목표 약 +2.3 m | 5 m | 지나간 앞차 기억·뒤차 관측·예약 판단 보완 |
| V5.4 | scheduled에서 기존 1792 기준 +4.6 m로 저속 이동 후 정지 | 5 m | 새 위치에서 정지 유지 후 갭 판단 |
| V6 | 원본 V5의 1792번 정지 | 1 m | 기존 추종 식의 기본 간격 변경 |
| **V6.1** | 원본 V5의 1792번 정지 | **1 m** | **허가 후 추종 앞차의 예측 정지 대신 상대속도 제어** |

V6/V6.1은 원본 V5에서 분기한 비교본이다. V5.2의 scheduled 예약과 V5.3/V5.4의 관측 전진을 포함하지 않는다. 목표 속도, 기존 PID·페달 변화율과 진입 후 감속 위치는 V5의 설정을 사용한다.

## 설치와 실행

기존 주행을 종료한 뒤 ZIP을 도커의 `/root/roundabout_v6_1.zip`으로 옮기고 도커 터미널에서:

```bash
python3 -m zipfile -e /root/roundabout_v6_1.zip /root/
bash /root/roundabout_v6_1/install.sh
bash /root/roundabout_v6_1/run_trial.sh
```

실행 첫 부분에 `CODE_VERSION=6.1.0 minimum_gap=1.0 time_headway=1.5`가 나오는지 확인한다. 실행 YAML은 `v61_speedtest.yaml`이다. 기본 MORAI IP는 `192.168.0.147`이며 `ROUNDABOUT_MORAI_IP` 환경 변수로 바꿀 수 있다.

설치기는 기존 패키지를 `/root/roundabout_backups/before_v61_.../`에 백업하고 완전한 새 패키지로 바꾼다. V6을 다시 비교하려면 V6의 `install.sh`를 실행한다. 기존 외부 경로와 의존 패키지를 사용한다. catkin 갱신이 필요할 때:

```bash
cd /root/morai_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg roundabout_mission
source devel/setup.bash
```

앞차 제어만 V6 방식으로 비교하려면 `v61_speedtest.yaml`의 `lead_relative_following_enabled`를 `false`로 바꾼다. 사용자 보정 YAML은 `prepare_roundabout_v6.py --existing EXISTING.yaml --output OUTPUT.yaml`로 사본을 만들 수 있다. 원본 5 m YAML도 별도 보존했다.

## 로그 확인

이 보완본의 `run_trial.sh`는 V5.1/V5.2와 같은 방식으로 **rosbag + 판단·제어 CSV/JSONL + 실행 설정·소스·경로 사본 + 요약 + 자동 압축**을 저장한다. 주행 전에 기록 구독을 확인하고, 종료 시 제어 노드를 먼저 종료해 마지막 제동 명령까지 기록한다. 기록 기능을 추가하면서 주행 판단 코드·드라이버·실행 YAML은 바꾸지 않았다.

| 파일 | 저장 내용 |
|---|---|
| `traffic*.bag` | 인지 객체, 자차 odometry, 판단·제어 상태, 실제 `/ctrl_cmd`, ROS 시간·진단 |
| `decision_*.csv`, `decision_*.jsonl` | 프레임별 상태·갭·허가·정지·긴급 여부, V6.1 추종 진단 |
| `control_*.csv`, `control_*.jsonl` | 실제 목표 속도·페달·조향 출력과 V6.1 판단 상태 |
| 설정 YAML, `metadata.json`, `summary.json` | 제공/실행 설정·ROS 파라미터·버전·입력 해시·기록 메시지 수·상태 전환 |
| `sources/`, 경로 파일 사본, 콘솔 로그 | 실제 실행한 소스·제어 의존 코드·전역/회전교차로 경로 |

기본 저장 위치는 `/root/roundabout_trials/v6_1_0_날짜_시간_baseline_PID/`다. 종료할 때 **실행 터미널에서 Ctrl+C를 한 번** 누르고 `SEND_THIS_FILE=...tar.gz`가 나올 때까지 기다린다. 표시된 압축 파일 하나를 보내면 된다. ROS 프로세스 종료와 기록 정리가 끝날 때까지 MORAI를 켜 둔다.

```bash
bash /root/roundabout_v6_1/run_trial.sh --note "multiple_cars" --launch-arg rviz:=false
```

`ROUNDABOUT_TRIAL_ROOT`로 저장 위치를 바꿀 수 있다. `INCOMPLETE`가 출력되면 필수 토픽이나 제어 기록이 누락된 것이므로 압축 파일의 `metadata.json`과 콘솔 로그를 확인한다. 강제 종료로 자동 압축이 완료되지 않았다면 아래 명령으로 기존 기록 폴더를 다시 묶는다.

```bash
rosrun roundabout_mission roundabout_trial_summary.py /root/roundabout_trials/실제_시험폴더 --pack
```

기록기 모의 검증 결과는 `validation/logging_checks.json`에 있다. 이 기록은 프로그램에 들어온 인지 정보와 제어 출력이며, 카메라 영상이나 인지되지 않은 차량의 실제 위치까지 복원하는 기록은 아니다.

V6.1은 CSV와 JSONL에 다음 필드도 저장한다.

| 추가 필드 | 의미 |
|---|---|
| lead_relative_following_active | 이번 프레임에 앞차 상대속도 제어를 적용할 조건이 성립함 |
| lead_gap_control_mode | `relative_speed`이면 새 추종 제어 |
| lead_prediction_stop_suppressed | 선정한 앞차의 TTC 정지 요구를 억제함 |
| lead_close_gap_stop_suppressed | 움직이는 앞차의 간격만으로 정지하던 조건을 억제함 |
| lead_closing_speed_mps | 앞차보다 빠르게 접근하는 양의 속도 차이 |
| lead_relative_speed_cap_mps | 반응시간·현재 여유·감속도를 반영한 속도 상한 |
| decision_reason_detail | 기존 `commit_latched` 표시 전에 실제로 사용한 분기 이유 |

`minimum_ttc`에는 억제된 앞차 TTC도 진단용으로 남긴다. 작은 TTC가 기록되어 있다는 이유만으로 긴급 정지를 요청했다고 해석하지 않고 `Emergency/Stop`, 억제 필드와 실제 브레이크 명령을 함께 본다.

## 검증

V6.1 검사 24개를 통과했다. 뒤차 단독 연속 80프레임의 결과는 V6 기준 코드와 같았고, 실제 PID 드라이버에 판단을 연결해 감속 브레이크와 뒤차 정지 처리도 확인했다. 가감속하는 앞차·ID 변경·0.1초 명령 지연을 포함한 280프레임의 합성 폐루프 모형에서 예측 정지 없이 추종했으며, 모형의 최소 차체 간격은 약 3.81 m였다.

이 모형은 MORAI 실주행 결과가 아니다. ROS Noetic/MORAI 차량 응답, GPS 기준점과 실제 차체 거리, 새 입력에서의 충돌 여부는 아직 실측하지 않았다. 결과와 실행 가능한 검증 스크립트는 `validation/`에 있다. 원본 `docs/`와 `test/`는 과거 이력이며 이번 버전의 전체 테스트 통과를 뜻하지 않는다.

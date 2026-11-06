# 식당 시연 리허설: 진행 상황 (11/6, 두 번째 중단)

노트북을 꺼야 해서 after 배치(손님 비키기 수정 뒤 재측정) 도중에 멈췄다. 결과 원본은 `raw/`에 있다(git 무시, 약 1 GB).

## 브랜치

| 브랜치 | 내용 | 상태 |
| --- | --- | --- |
| `rehearsal/restaurant-demo` | 리허설용 합본: upstream/dev(4b05188, #305·#308 머지 뒤) + origin/feat/fleet-task-aging(#302·#303) + `feat/bringup-congestion-filter` + `fix/nav-restaurant-guest-yield`. 이 폴더의 스크립트·STATUS.md 커밋 | origin push |
| `feat/bringup-congestion-filter` | system.launch `congestion_filter`(none/keepout/speed)를 로봇마다 Nav2로 넘긴다. dev 기준, 기본값은 물류센터 그대로. amr_bringup 테스트 69개 통과, 빌드 경고 0 | origin push(bf9a475), PR 없음 |
| `fix/nav-restaurant-guest-yield` | 식당 손님(customer)도 멈춘 로봇에 비킨다: `obstacle_motion` BODY_RADIUS에 customer 0.25, `yield_robot_size`(선택, 기본은 AMR 0.6 x 0.4라 물류센터 작업자는 그대로). 평상시·피크 손님 yaml의 straight·random 손님에 `yield_to_robots: [amr_01..05]`, `yield_robot_size: [0.565, 0.537]`. curve(customer_3)는 규칙상 못 써서 그대로. 손님 경로는 안 바꿨다. test_obstacle_motion 31개 통과 | origin push(8c4df1b), PR 없음, rehearsal에 merge |
| `feat/bringup-restaurant-world` (11/5) | world·congestion_filter·robots_restaurant.yaml | **쓰지 않는다.** #308이 world·robots_restaurant.yaml을 같은 값으로 넣어 겹친다. congestion_filter만 위 브랜치로 옮겼다 |

dev 변화 (11/5 → 11/6):
- #305(혼잡도 site:=restaurant)와 #308(system.launch world·dynamic_obstacles_file·cost_fn, robots_restaurant.yaml)이 머지됐다.
- 리허설 조건(식당 월드, body:=serving, 손님·구역 파일, keepout 필터)은 바뀌지 않았다. congestion_filter는 여전히 dev에 없다.
- #292·#302·#303은 아직 open이다. dev BT는 그대로 1초 재계획(`RateController hz="1.0"`)이다.

## 시나리오 1-2 (붐비는 통로 돌아가기)

조건:
- 출발 t11 (-8.8, -3.27, 남쪽) → 목표 buffet_1 (-9.9, 4.15, 북쪽). west_aisle로 7.9 m(구역 안 5.1 m), x -7.275 통로로 돌아가면 9.1 m다.
- 로봇 1대(서빙 몸체), 카메라 끔, safety·tracking 끔(stack 기본).
- 피크 손님 + 혼잡도 노드(site:=restaurant guests:=peak)를 켠다.
- 10 s 뒤 목표를 보내고 시뮬레이션 180 s에서 끊는다.
- 끔 = 필터 none(손님은 같다), 켬 = keepout, speed는 1회만 돌린다.

### before (손님 비키기 없음)

| 실행 | 결과 | 서쪽 통로 지난 계획 | 끝 (TF 추정 / GT) | RTF |
| --- | --- | --- | --- | --- |
| none_s1 | ABORTED | 375/375 | (-10.33, 2.40) / GT 없음 | 0.37 |
| none_s2 | ABORTED | 287/287 | (-9.81, 1.65) / GT 없음 | 0.32 |
| none_s3 | CANCELED (180 s) | 대부분 | (-10.44, 0.38) / (-10.18, 2.14) | 0.43 |
| keepout_s1 | ABORTED | 0/338 | (-7.81, -0.61) / GT 없음 | 0.40 |
| keepout_s2 | ABORTED | 0/555 | (-7.72, 1.21) / GT 없음 | 0.26 |
| keepout_s3 | CANCELED (180 s) | 0 | (-8.71, 3.41) / (-8.03, 2.87) | 0.45 |
| speed_s1 | SUCCEEDED **(가짜 성공: GT는 목표에서 2.3 m 남쪽)** | 서쪽 통로 | (-9.84, 4.10) / (-10.00, 1.85) | 0.43 |

### before 원인 분류

| 원인 | 해당 실행 | 근거 |
| --- | --- | --- |
| 안전 노드 정지(WARNING/CRITICAL, 근접 저속) | 없음 | safety:=false라 노드가 안 떠 있다. 로그 0건 |
| 손님이 안 비키고 로봇에 붙어 막음 | none_s3, keepout_s3 (GT 있음) | 정지 구간 동안 가장 가까운 손님이 0.22~0.39 m(중심 거리, 닿는 거리 0.52 m)에 멈춰 있다. 로봇은 회전 명령(w 0.3)을 받는데 GT 위치·방향이 그대로다 |
| 제자리 회전 중 모서리가 가구에 걸림 | speed_s1 (-10.0, 1.85) | 1.6 m 안에 손님이 없다. w 최대 1.0 명령 60 s 동안 GT가 그대로다. 정적 여유 0.40 m ≈ 외접 반경 0.39 m |
| 바퀴 헛돎 → 위치추정 어긋남 → 추정 자세가 테이블 칸 안 → DWA 궤적 없음·복구 "Collision Ahead" | none_s1·s2, keepout_s1·s2, smoke (GT 없어 추정). 잰 것: none_s3 1.8 m, keepout_s3 0.9 m, speed_s1 2.3 m 어긋남 | 끝난 추정 자세의 정적 여유 0.21~0.25 m < 내접 0.27 m. 그 실행들에서 "DWA 유효한 궤적을 못 찾아" 97~129건, 복구 실패 10건 |
| costmap에서 통로가 막혀 보임 | 없음 | 통로 1.46~2.5 m, 가운데 비용 0. 출발 t11에서 footprint 밑 120칸 중 253(내접) 20칸, 254(치명) 0칸 → 정차 자세·inflation은 출발을 막지 않는다. 식당 프로필 inflation은 고치지 않았다 |
| 로봇끼리 막힘 | 시나리오 2 (아래) | |

### after (fix/nav-restaurant-guest-yield 적용) - 중단된 배치

| 실행 | 결과 | 서쪽 통로 지난 계획 | 끝 (TF 추정) | 목표까지 | sim / wall (s) | RTF |
| --- | --- | --- | --- | --- | --- | --- |
| y_none_s1 | SUCCEEDED | 127/150 (구역 안 주행 6.8 m) | (-9.93, 3.95) | 0.20 m | 80 / 175 | 0.46 |
| y_keepout_s1 | SUCCEEDED | 0/183 (구역 안 0 m) | (-9.83, 3.94) | 0.22 m | 95 / 208 | 0.46 |
| y_none_s2 | SUCCEEDED | 46/81 (구역 안 6.4 m) | (-9.94, 3.96) | 0.20 m | 42 / 92 | 0.46 |
| y_keepout_s2 | ABORTED | 7/298 | (-7.72, -1.05) | 5.6 m | 177 / 371 | 0.48 |
| y_none_s3 | CANCELED (180 s) | 90/283 | (-10.20, 2.98) | 1.2 m | 181 / 388 | 0.47 |
| y_keepout_s3 | 중단됨 (부분 bag·로그만, json 없음) | | | | | |

- 주의: after의 SUCCEEDED는 아직 GT로 확인하지 않았다. before speed_s1처럼 가짜 성공일 수 있다.
  확인하려면 `abort_causes.py`로 `end_gt`를 본다.
- 남은 실행: y_keepout_s3(다시), y_speed_s1, y_collect_s1(시나리오 2, 로봇 2대), after 원인 분류, 전후 비교 표.

## 시나리오 2 (회수 호출)

before collect_s1 (로봇 2대 wait_1·2, 평상시 손님, fleet_manager + `fleet_manager_restaurant.yaml`):
- `aging_rate`는 1.667로 확인했다.
- 할당 1 `n1_stall1_t03 → amr_02`, 할당 2 `n2_stall2_t08 → amr_01`(t 8.9 s).
- 두 로봇이 홀 동쪽 (1.48, -1.23)·(2.01, -1.69)에서 0.7 m 사이로 마주 서서 서로 막았다(로봇끼리 교착). "Failed to make progress" 23~25건, backup·spin "Collision Ahead"가 남았다.
- sim 307 s에도 n1·n2가 끝나지 않아 멈췄다(scene2 json 없음, bag·assignments.csv·로그만 있다). 회수 호출 c1·c2는 할당되지 않았다.
- zone_client(교통 제어)가 꺼진 스택이라 누가 비킬지 정하는 것이 없다.

스크립트 수정 (11/6):
- `scene2.py`: 중단(Ctrl-C)이나 시간 초과에도 json을 남긴다(`stopped`: done/timeout/interrupted). `--robots`, `--timeout`(시뮬레이션 s)을 받는다. `rclpy.try_shutdown`.
- `run2.sh`: 세 번째 인자로 로봇 수(기본 2, wait_1부터). aging_rate 확인은 60 s 뒤에 한다(전에는 서버가 뜨기 전에 물어 "Node not found").
- `run12.sh`·`run2.sh`: 실행마다 rosbag을 남긴다(GT odom 로봇·손님, cmd_vel, plan, local_costmap, /fleet/congestion·task_*·robot_states).
- `abort_causes.py`(새로 만듦): 로그 키워드 수, 정지 구간마다 가장 가까운 손님·정적 여유, 밀림(명령 0인데 GT 이동), 추정 대 GT 끝 자세.

## 녹화 전에 할 수 있는 것 (제안)

1. **손님 비키기 켜기.** `fix/nav-restaurant-guest-yield`를 적용하고 after 1-2를 돌렸다. 5회 중 3회는 목표에 닿았다고 나왔고(GT 확인 전), before는 7회 중 0회다(가짜 성공 1회 제외).
2. **녹화는 로봇 1대로 하거나 zone_client를 켠다.** 로봇 2대면 홀 가로 통로에서 서로 막힌다(시나리오 2).
   시나리오 2 순서(회수 호출이 먼저 할당되는지)를 보이는 데는 1대로 충분하다. 1대가 n1을 맡고, 끝나면 큐 n2~n4·c1·c2 중 c1이 먼저 나가야 한다.
3. **위치추정 어긋남 감시.** 몸이 걸린 채 바퀴가 돌면 추정 자세가 m 단위로 밀려 가짜 성공이 나온다. 녹화 때 RViz에 GT와 추정을 같이 띄워 확인한다.
4. **가구 모서리 걸림(speed_s1)이 after에도 남으면** 식당 프로필(nav2_profile:=restaurant)에서만 inflation_radius를 늘리거나(0.55 → 약 0.8) cost_scaling을 낮춰 경로를 의자 등받이에서 떼어 본다. 물류센터 값은 그대로 둔다. 아직 시도하지 않았다.

## 다시 시작하기

```bash
cd ~/dev/amr-fleet-pp
git switch rehearsal/restaurant-demo
docker ps                                   # 다른 Gazebo가 없을 때만
ROS_DOMAIN_ID=77 MSYS_NO_PATHCONV=1 docker compose run -d --rm \
  -e GZ_PARTITION=pp_rehearsal -e IGN_PARTITION=pp_rehearsal dev sleep infinity
C=<위에서 뜬 컨테이너 이름>
export MSYS_NO_PATHCONV=1
docker exec $C bash -c 'cd /ros2_ws && source /opt/ros/humble/setup.bash && colcon build'
docker exec $C mkdir -p /tmp/r12 /tmp/r2
S=experiments/restaurant_rehearsal/scripts
docker cp $S/run12.sh $C:/tmp/r12/; docker cp $S/measure12.py $C:/tmp/r12/
docker cp $S/run2.sh $C:/tmp/r2/;   docker cp $S/scene2.py $C:/tmp/r2/
docker cp $S/abort_causes.py $C:/tmp/

# after 남은 실행 (1-2: 실행마다 약 4~8분, 시나리오 2: 최대 약 40분)
docker exec $C bash -c 'bash /tmp/r12/run12.sh y_keepout_s3 keepout peak 3; bash /tmp/r12/run12.sh y_speed_s1 speed peak 1'
docker exec $C bash /tmp/r2/run2.sh y_collect_s1 1 2      # 로봇 2대 (before와 같은 조건)
docker exec $C bash /tmp/r2/run2.sh y_collect1_s1 1 1     # 로봇 1대 (순서 보이기용)

# 결과 꺼내기 (컨테이너는 --rm이라 멈추면 /tmp가 사라진다)
docker cp $C:/tmp/r12/. experiments/restaurant_rehearsal/raw/scene1_2/
docker cp $C:/tmp/r2/.  experiments/restaurant_rehearsal/raw/scene2/

# 원인 분류 (before·after 모두 raw/scene1_2에 있다. y_ 접두어가 after)
docker cp experiments/restaurant_rehearsal/raw/scene1_2 $C:/tmp/all12
docker exec $C bash -c 'source /ros2_ws/install/setup.bash; python3 /tmp/abort_causes.py /tmp/all12 \
  --map $(ros2 pkg prefix amr_mapping)/share/amr_mapping/maps/restaurant_gt.yaml --out /tmp/all12/abort_causes_all.json'
```

## 파일

| 경로 | 내용 |
| --- | --- |
| `raw/scene1_2/*.json` | 1-2 실행마다 지표·궤적·첫 계획 (y_ = after) |
| `raw/scene1_2/bags/` | none_s3·keepout_s3·speed_s1(before), y_*(after) rosbag |
| `raw/scene1_2/logs/` | system·obst·cong·measure·bag 로그 |
| `raw/scene1_2/abort_causes.json` | before s3 3회 분석 결과 |
| `raw/scene2/` | before collect_s1 bag·assignments.csv·로그 |
| `scripts/` | pairs.py, run12.sh, measure12.py, run2.sh, scene2.py, abort_causes.py |

남은 실행 (11/6 배터리로 중단): y_speed_s1, y_collect_s1(로봇 2대), y_collect1_s1(로봇 1대) — y_keepout_s3는 끝남(json·`bags/y_keepout_s3_done`·`logs_1106/`, 예전 부분 bag `bags/y_keepout_s3`는 지워야 함).

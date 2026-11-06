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

## 11/6 오후 (외출 중 자율 진행) - 배터리 6 %로 중단

배터리가 10 % 아래(6 %)로 떨어져 새 실행을 시작하지 않고 멈췄다. 결과는 `raw/run_1106b/`(scene1_2·scene2: json·logs·bags)에 있다.

### 판단한 것
- 새 dev(#285·#287·#309)는 fleet_metrics·task_generator launch·테스트만 바꿔 리허설 조건과 무관하다. after 배치는 같은 빌드로 마쳤다. 단계 B부터는 수정 브랜치가 새 dev 기준이라 rehearsal 브랜치에 그 dev가 같이 들어왔다.
- 시나리오 2 시간 제한: 로봇 2대 600 s, 1대 900 s(시뮬레이션).
- 4단계 rotate_in_place_clearance는 늘리지 않고 0.0으로 낮췄다. 테이블 정차 자세에서 로봇 중심~테이블이 0.43 m인데, 0.05 여유면 회전에 0.46 m가 필요하다. y_speed_s1이 t11 바로 옆에서 DWA 궤적 없음 146회로 164 s 멈췄다(위치 오차 0.12 m, 손님 0.92 m 밖).
- 6단계: 교통 구역 파일은 물류센터 zones.yaml뿐이고 식당 구역 설정이 없다. zone_client를 켜면 물류센터 좌표 구역이 식당에 걸리므로 켜지 않았다. 식당 구역(6인석 1.0 m 십자 통로, 부스 앞 1.09 m, 홀 가로 통로)을 정하는 것은 팀 결정이라 제안만 한다.
- 3단계: 안전 노드는 system.launch 기본값(1단계)이라 꺼져 있다. 켜려면 tracking도 켜야 한다. 또 safety.yaml 로봇 크기가 AMR 0.6 x 0.4 고정이라 서빙 로봇(0.565 x 0.537)에 맞는 크기 인자가 먼저 필요하다. side_margin도 DWA rotate_in_place_clearance와 같아야 한다(식당 0.0). 아직 못 했다.

### after 배치 마무리 (yield 수정)
| 실행 | 보고 | GT 도착 오차 | 위치 추정 오차 | 주 정체 원인 |
| --- | --- | --- | --- | --- |
| y_none_s1 | SUCCEEDED | 0.15 m | 0.14 m | 서쪽 통로 의자 옆(여유 0.32) 36 s |
| y_keepout_s1 | SUCCEEDED (가짜) | 1.9 m | 1.77 m | 정체 없음, 위치 추정 어긋남 |
| y_none_s2 | SUCCEEDED (허용 0.25 밖) | 0.64 m | 0.46 m | - |
| y_keepout_s2 | ABORTED | 5.9 m | 0.27 m | x -7.67 의자 옆 여유 0.32, DWA 궤적 없음 141회 |
| y_none_s3 | CANCELED | 1.56 m | 0.38 m | 배식대 앞 손님(peak_6)과 서로 기다림 130 s |
| y_keepout_s3 | CANCELED | (bag `bags/y_keepout_s3_done`, 분석 안 함) | | |
| y_speed_s1 | ABORTED (0.36 m 주행) | 7.4 m (출발점 옆) | 0.12 m | t11 옆 제자리 회전 불가, DWA 궤적 없음 146회 |

시나리오 2 (aging 1.667 확인):
- y_collect_s1 (로봇 2대, 600 s):
  - 할당 순서는 n1, n2, **c1, c2**, n3, n4다. 회수 호출이 먼저 할당됐고 테이블 → return_1/2로 완료했다(amr_02).
  - amr_01은 n2를 끝내 못 하고 531 s에 FAILED(201)다. 로봇끼리 막힘이 남았다.
- y_collect1_s1 (로봇 1대, 900 s):
  - n1 완료(75 s) → **c1 할당·완료(139 s)** → c2 할당이다. 회수 호출 두 건이 일반 작업 n2~n4보다 먼저 나갔다.
  - c2는 672 s에 FAILED(201)다(분석 안 함).

### 2단계 위치 추정 (가장 중요)
- 가설: restaurant_gt 상판 상자 때문에 AMCL이 테이블 둘레에서 위치를 못 잡는다.
- 위치추정용 지도를 나눴다: `fix/nav-restaurant-scan-map`(e2cc88f). restaurant_map.py `--scan-height`로 `restaurant_scan` 지도를 만들고, system.launch `planning_map`을 주면 costmap static_layer만 `/map_planning`을 읽는다.
  - 경로 계획 쪽은 keepout 마스크가 아니라 별도 지도 토픽(static_layer map_topic)으로 했다.
  - 이유: keepout 필터 자리는 혼잡도 마스크가 이미 쓴다. 상판은 늘 막혀 있는 정적 물체라 static layer가 맞다. 마스크 값으로 칠하면 치명(254)이 아니라 비용으로 바뀔 수 있다.
- 데이터 확인은 아직 1회뿐이다. y_speed_s1은 t11 근처만 있었고, 두 지도 일치도가 비슷했다(점수 0.691 vs 0.649, 평탄도 0.006 vs 0.006, AMCL 오차 0.12 m). **가설은 아직 확인되지 않았다.**
  - 다음: 주행이 긴 bag(`raw/run_1106b/scene2/bags/*`, amcl_pose·scan 있음)에 `scripts/scan_match.py`를 돌린다.
  - 그다음 단계 B(MAPS=split, 아래 명령)로 전후를 비교한다.

### 만든 브랜치 (push함, PR 없음)
- `fix/nav-restaurant-scan-map` e2cc88f: 위치추정·경로 계획 지도 나누기 (2단계). rehearsal에 merge함
- `fix/nav-restaurant-inflation` 3a2aee3: 식당 프로필 inflation 0.75/0.7, cost_scaling 2.0, rotate_in_place_clearance 0.0 (4단계). 아직 merge·측정 안 함
- `fix/sim-restaurant-guest-patience` 4e2e178: 식당 손님이 로봇이 3 s 동안 0.1 m도 못 움직이면 다시 걷는다 (5단계, yield 브랜치 위). 아직 merge·측정 안 함
- 주의: `feat/bringup-congestion-filter`와 `fix/nav-restaurant-scan-map`은 stack.py·system.launch.py 같은 줄을 고쳐, 둘째로 dev에 들어가는 쪽이 충돌한다.

### 남은 것
- 단계 B (지도 나누기) 6회: 큐 파일 `qB.txt` 내용 = `MAPS=split bash /tmp/r12/run12.sh B_<none|keepout>_s<1..3> <none|keepout> peak <1..3>`
- 단계 C: `fix/nav-restaurant-inflation` merge → 6회. 단계 D: `fix/sim-restaurant-guest-patience` merge → 6회. 그 뒤 시나리오 2 다시.
- 3단계(안전 노드 서빙 크기), 6단계(식당 교통 구역), 7단계(GT 위치 옵션, 2단계가 안 풀리면)는 하지 못했다.

```bash
# 다시 시작: 컨테이너 띄우고 빌드·스크립트 복사는 위 "다시 시작하기"와 같다
bash experiments/restaurant_rehearsal/scripts/runq.sh <컨테이너> <큐 파일>   # 실행마다 배터리 확인, 10 % 아래면 멈춤
```

## 11/6 저녁: 지도 분리 + inflation 수정 측정 (1-2 피크, none·keepout 각 3회)

브랜치 `rehearsal/restaurant-split-inflation` = `rehearsal/restaurant-demo`(지도 분리 merge) + `fix/nav-restaurant-inflation`. 빌드 경고 0, 관련 테스트 277개 통과(13 skip). `MAPS=split`, Gazebo headless, 카메라·RViz 끔. 결과 원본 `raw/run_1106c/scene1_2`.
`feat/bringup-congestion-filter`(bf9a475): #304가 nav2.launch에 넣은 `congestion_filter`를 system.launch에서 로봇마다 넘겨 주는 인자만 더한 브랜치(필터 자체는 #304).

| 실행 | before (지도 1장, inflation 0.55) 보고 / GT 도착 오차 / 위치 오차 | after (지도 분리 + inflation 0.75·2.0) 보고 / GT 도착 오차 / 위치 오차 (AMCL 평균·최대) | after 주 정체 원인 |
| --- | --- | --- | --- |
| none_s1 | SUCCEEDED / 0.15 / 0.14 | SUCCEEDED / 0.07 / 0.11 (0.14·0.23) | 없음 (40 s) |
| none_s2 | SUCCEEDED(허용 밖) / 0.64 / 0.46 | SUCCEEDED / 0.13 / 0.16 (0.15·0.27) | 없음 (52 s) |
| none_s3 | CANCELED / 1.56 / 0.38 | CANCELED / 2.39 / 0.17 (0.21·0.34) | 서쪽 통로 손님 사이 no_progress 12회 |
| keepout_s1 | SUCCEEDED(가짜) / 1.9 / 1.77 | CANCELED / 4.82 / 0.16 (0.15·0.24) | 서쪽 통로 의자 옆(여유 0.42) 156 s, DWA 궤적 없음 139회 |
| keepout_s2 | ABORTED / 5.9 / 0.27 | CANCELED / 3.78 / 0.13 (0.13·0.24) | 서쪽 통로 의자 옆(여유 0.30) 30 s, no_progress 10회 |
| keepout_s3 | CANCELED / - / - | CANCELED / 2.56 / **2.27** (1.28·2.36) | 서 있는 손님 peak_1(0.36 m)에 끼어 바퀴 헛돎 → 위치 추정 어긋남 |

(GT 도착 오차·위치 오차는 m, 위치 오차 = 끝에서 TF 추정과 GT 차이. before는 amcl_pose를 녹화하지 않아 끝값만 있다.)

### 판단
- **지도 분리 가설은 틀렸다.** 6회 모두 GT 자세에서 스캔을 두 지도에 맞춰 보면 restaurant_gt가 오히려 약간 높고(점수 0.56~0.69 vs 0.54~0.66), 평탄도(0.006~0.015)와 최고점 위치(0.1 m)가 같다. 상판 상자가 AMCL을 흐리게 하지 않는다. `fix/nav-restaurant-scan-map`은 PR 대상에서 뺀다.
- **위치 오차의 실제 원인은 바퀴 헛돎이다.** keepout_s3 t=192~237 s: GT는 (-10.11, 1.43)에서 0.02 m도 안 움직이는데 바퀴 오도메트리는 0.18~0.39 m/s로 돈다. 그동안 GT 자세 스캔 점수는 그대로(0.55~0.57, 최고점 0.14 m)인데 AMCL 오차는 0.18 → 2.07 m로 커지고 공분산은 0.12~0.17 m로 작은 채다. 파티클이 오도메트리를 따라 통째로 밀려 정답 근처 파티클이 없어지고, `recovery_alpha_slow/fast: 0.0`이라 다시 퍼지지도 않는다. y_keepout_s1(1.77 m)도 같은 꼴(몸 걸린 채 바퀴 돎)이었다.
  - 후보(안 해 봄): 식당만 recovery_alpha 켜기(예 0.001/0.1), 손님에 끼었을 때 바퀴를 멈추게(안전 노드·DWA 근접 정지), EKF가 IMU 각속도·가속도로 헛돎을 거르게.
- **inflation 수정은 PR 대상에서 뺀다.** none은 3회 중 2회 허용 안 도착(before 1회)으로 나아졌지만, keepout 3회가 모두 돌아가는 길(x -7.275 통로)을 버리고 서쪽 통로로 갔다(계획 302/302, 443/443, 163/356이 서쪽 통로 통과, before는 keepout이면 늘 돌아갔다). 폭 1.46 m 돌아가는 통로 가운데(벽에서 0.73 m)의 비용이 inflation 0.55·3.0에서는 0인데 0.75·2.0에서는 약 100이 되어 keepout 마스크(최대 90)보다 비싸졌다. 시연 1-2의 핵심(붐비는 통로 돌아가기)을 깨므로 넣지 않는다. 의자 옆 정체(여유 0.30~0.42 m)도 그대로 남았다.
- 이 조건으로 1-2를 녹화하면 none은 되지만 keepout(돌아가기)은 안 된다.

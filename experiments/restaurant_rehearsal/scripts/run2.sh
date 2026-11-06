#!/bin/bash
# 시연 2 한 번: run2.sh <label> <seed> [로봇 수, 기본 2] [시간 제한 시뮬레이션 s, 기본 900]. 로봇은 wait_1부터, 평상시 손님, Fleet 서버 + 식당 설정(aging 1.667)
label=$1; seed=$2; n=${3:-2}; robots=$(seq -f 'amr_%02g' 1 $n | paste -sd' ')
D=/tmp/r2; mkdir -p $D/logs
source /ros2_ws/install/setup.bash
fastdds shm clean >/dev/null 2>&1
B=$(ros2 pkg prefix amr_bringup)/share/amr_bringup/config
G=$(ros2 pkg prefix amr_gazebo)/share/amr_gazebo/config
F=$(ros2 pkg prefix amr_fleet_manager)/share/amr_fleet_manager/config
M=$(ros2 pkg prefix amr_mapping)/share/amr_mapping/maps/restaurant_gt.yaml
# MAPS=split이면 위치추정은 restaurant_scan(스캔 높이), 경로 계획은 restaurant_gt (fix/nav-restaurant-scan-map)
MAPARGS="map:=$M"
if [ "${MAPS:-one}" = split ]; then
  MAPARGS="map:=${M%restaurant_gt.yaml}restaurant_scan.yaml planning_map:=$M"
fi
setsid ros2 launch amr_bringup system.launch.py world:=restaurant.sdf body:=serving num_robots:=$n \
  robots_file:=$B/robots_restaurant.yaml $MAPARGS map_origin_in_world:='[0, 0, 0]' \
  nav2:=true fleet:=false seed:=$seed > $D/logs/$label.system.log 2>&1 &
P1=$!
setsid ros2 launch amr_fleet_manager fleet_manager.launch.py use_sim_time:=true \
  extra_params_file:=$F/fleet_manager_restaurant.yaml assignments_csv:=$D/$label.assignments.csv \
  > $D/logs/$label.fleet.log 2>&1 &
P2=$!
sleep 15
setsid ros2 launch amr_gazebo dynamic_obstacles.launch.py config:=$G/dynamic_obstacles_restaurant.yaml seed:=$seed \
  use_sim_time:=true > $D/logs/$label.obst.log 2>&1 &
P3=$!
mkdir -p $D/bags; rm -rf $D/bags/$label
setsid ros2 bag record -o $D/bags/$label -e   '/amr_0[12]/(cmd_vel|plan|ground_truth/odom|local_costmap/costmap|amcl_pose|scan)|/dynamic_obstacles/.*/ground_truth/odom|/fleet/(congestion|task_assignments|task_status|robot_states)'   > $D/logs/$label.bag.log 2>&1 &
PB=$!
(sleep 60; ros2 param get /fleet_manager assignment.aging_rate > $D/logs/$label.aging_rate.txt 2>&1) &
timeout 3600 python3 $D/scene2.py --stations $F/stations_restaurant.yaml --out $D/$label.json --robots $robots --timeout ${4:-900} \
  > $D/logs/$label.scene.log 2>&1
echo "$label exit=$? $(head -1 $D/logs/$label.scene.log | cut -c1-300)"
kill -INT -- -$PB 2>/dev/null; sleep 3
for p in $P3 $P2 $P1; do kill -INT -- -$p 2>/dev/null; done
sleep 8
for p in $P3 $P2 $P1; do kill -KILL -- -$p 2>/dev/null; done
pkill -9 -f "ign gazebo|gz sim|ruby" 2>/dev/null
sleep 3

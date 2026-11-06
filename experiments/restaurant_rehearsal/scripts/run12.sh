#!/bin/bash
# 시연 1-2 한 번: run12.sh <label> <congestion_filter none|keepout|speed> <guests peak|normal> <seed>
# 출발 t11 (-8.8, -3.27, 남쪽), 목표 buffet_1 (-9.9, 4.15, 북쪽)

label=$1; filter=$2; guests=$3; seed=$4
D=/tmp/r12; mkdir -p $D/logs
source /ros2_ws/install/setup.bash
fastdds shm clean >/dev/null 2>&1
G=$(ros2 pkg prefix amr_gazebo)/share/amr_gazebo/config
M=$(ros2 pkg prefix amr_mapping)/share/amr_mapping/maps/restaurant_gt.yaml
# MAPS=split이면 위치추정은 restaurant_scan(스캔 높이), 경로 계획은 restaurant_gt (fix/nav-restaurant-scan-map)
MAPARGS="map:=$M"
if [ "${MAPS:-one}" = split ]; then
  MAPARGS="map:=${M%restaurant_gt.yaml}restaurant_scan.yaml planning_map:=$M"
fi
cat > $D/robots_start.yaml <<EOF
robots:
  amr_01: [-8.8, -3.27, -1.57079633]
EOF
obst=$G/dynamic_obstacles_restaurant.yaml; [ "$guests" = peak ] && obst=$G/dynamic_obstacles_restaurant_peak.yaml
setsid ros2 launch amr_bringup system.launch.py world:=restaurant.sdf body:=serving num_robots:=1 \
  robots_file:=$D/robots_start.yaml $MAPARGS map_origin_in_world:='[0, 0, 0]' \
  nav2:=true fleet:=false bt:=false congestion_filter:=$filter seed:=$seed > $D/logs/$label.system.log 2>&1 &
P1=$!
sleep 15
setsid ros2 launch amr_gazebo dynamic_obstacles.launch.py config:=$obst seed:=$seed use_sim_time:=true \
  > $D/logs/$label.obst.log 2>&1 &
P2=$!
setsid ros2 launch amr_fleet_manager congestion.launch.py site:=restaurant guests:=$guests \
  > $D/logs/$label.cong.log 2>&1 &
P3=$!
mkdir -p $D/bags; rm -rf $D/bags/$label
setsid ros2 bag record -o $D/bags/$label -e   '/amr_0[12]/(cmd_vel|plan|ground_truth/odom|local_costmap/costmap|amcl_pose|scan)|/dynamic_obstacles/.*/ground_truth/odom|/fleet/(congestion|task_assignments|task_status|robot_states)'   > $D/logs/$label.bag.log 2>&1 &
PB=$!
need=""; [ "$filter" != none ] && need=--need-mask
timeout 1500 python3 $D/measure12.py --goal -9.9 4.15 1.5707963 --out $D/$label.json $need --timeout 180 \
  > $D/logs/$label.measure.log 2>&1
echo "$label exit=$? $(tail -1 $D/logs/$label.measure.log | cut -c1-400)"
kill -INT -- -$PB 2>/dev/null; sleep 3
for p in $P3 $P2 $P1; do kill -INT -- -$p 2>/dev/null; done
sleep 8
for p in $P3 $P2 $P1; do kill -KILL -- -$p 2>/dev/null; done
pkill -9 -f "ign gazebo|gz sim|ruby" 2>/dev/null
sleep 3

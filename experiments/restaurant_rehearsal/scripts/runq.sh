#!/bin/bash
# 호스트(Git Bash)에서 실행 목록을 하나씩 돌린다. 실행이 끝날 때마다 배터리를 보고 10 % 아래면 멈춘다.
#   runq.sh <컨테이너> <목록 파일>    목록 한 줄 = 컨테이너 안에서 돌릴 명령 (예: bash /tmp/r12/run12.sh y_speed_s1 speed peak 1)
# 끝난 실행마다 "DONE <명령> | <마지막 출력>", 배터리 때문에 멈추면 "BATTERY <값>"을 찍는다.
C=$1; list=$2
export MSYS_NO_PATHCONV=1
while IFS= read -r cmd; do
  [ -z "$cmd" ] && continue
  out=$(docker exec "$C" bash -c "$cmd" 2>&1 | grep -v Killed | tail -1 | cut -c1-300)
  echo "DONE $cmd | $out"
  b=$(powershell.exe -NoProfile -Command "(Get-CimInstance Win32_Battery).EstimatedChargeRemaining" | tr -d '\r ')
  echo "BATTERY_NOW $b"
  if [ -n "$b" ] && [ "$b" -lt 10 ]; then
    echo "BATTERY $b - 새 실행을 시작하지 않는다"
    exit 3
  fi
done < "$list"
echo "QUEUE_END"

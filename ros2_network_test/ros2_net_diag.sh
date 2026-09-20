#!/bin/bash
# ros2_net_diag.sh
# 관제PC에서 실행: 가벼운 부하를 걸면서 네트워크 지표를 동시에 로깅합니다.
#
# 사용법:
#   ./ros2_net_diag.sh <핑키IP> [부하_Hz] [지속시간_분] [무선인터페이스명]
#
# 예시:
#   ./ros2_net_diag.sh 192.168.0.101 0.1 30 wlan0
#   (핑키1 IP로 30분간, 0.1Hz(10초에 한 번) 부하를 걸면서 진단)
#
# 종료: Ctrl+C 또는 지정한 시간이 지나면 자동 종료됩니다.

set -u

TARGET_IP=${1:?"핑키 IP를 입력하세요. 예: ./ros2_net_diag.sh 192.168.0.101"}
RATE=${2:-0.1}
DURATION_MIN=${3:-30}
IFACE=${4:-wlan0}

LOGDIR="./ros2_net_diag_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"

echo "===================================================="
echo " ROS2 네트워크 진단 시작"
echo " 대상 IP     : $TARGET_IP"
echo " 부하 발행율 : ${RATE} Hz"
echo " 지속 시간   : ${DURATION_MIN} 분"
echo " 인터페이스  : $IFACE"
echo " 로그 폴더   : $LOGDIR"
echo "===================================================="

DURATION_SEC=$((DURATION_MIN * 60))
END_TIME=$((SECONDS + DURATION_SEC))

PIDS=()

cleanup() {
  echo ""
  echo "[종료 중] 백그라운드 프로세스 정리..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null
  done
  wait 2>/dev/null
  echo "[완료] 로그는 $LOGDIR 에 저장되었습니다."
  echo "  - pub.log      : 발행한 부하 토픽 로그"
  echo "  - topic_hz.log : 실제 수신 빈도 (끊기면 여기서 바로 보임)"
  echo "  - ping.log     : 초당 핑 성공/실패, 지연시간"
  echo "  - iface.log    : 5초마다 인터페이스 RX/TX 에러·드랍 카운트"
  exit 0
}
trap cleanup INT TERM

# 1) 가벼운 부하 발행 (백그라운드)
ros2 topic pub /diag_test std_msgs/msg/String "data: hello" -r "$RATE" \
  > "$LOGDIR/pub.log" 2>&1 &
PIDS+=($!)

# 2) 실제 수신 빈도 모니터링 (끊기는 순간이 여기 바로 보임)
ros2 topic hz /diag_test > "$LOGDIR/topic_hz.log" 2>&1 &
PIDS+=($!)

# 3) 초당 핑 로그 (타임스탬프 포함, 손실도 기록)
(
  while [ $SECONDS -lt $END_TIME ]; do
    ts=$(date '+%Y-%m-%d %H:%M:%S')
    result=$(ping -c 1 -W 1 "$TARGET_IP" 2>/dev/null | grep -oP 'time=\K[0-9.]+')
    if [ -z "$result" ]; then
      echo "$ts LOST" >> "$LOGDIR/ping.log"
    else
      echo "$ts OK ${result}ms" >> "$LOGDIR/ping.log"
    fi
    sleep 1
  done
) &
PIDS+=($!)

# 4) 인터페이스 레벨 드랍/에러 카운트 (5초 간격)
(
  while [ $SECONDS -lt $END_TIME ]; do
    ts=$(date '+%Y-%m-%d %H:%M:%S')
    stats=$(ip -s link show "$IFACE" 2>/dev/null | tr '\n' ' ')
    echo "$ts | $stats" >> "$LOGDIR/iface.log"
    sleep 5
  done
) &
PIDS+=($!)

echo "실행 중... (백그라운드 PID: ${PIDS[*]})"
echo "실시간으로 확인하려면 다른 터미널에서:"
echo "  tail -f $LOGDIR/ping.log"
echo "  tail -f $LOGDIR/topic_hz.log"
echo ""
echo "Ctrl+C 로 언제든 중단 가능합니다."

# 지정 시간만큼 대기
while [ $SECONDS -lt $END_TIME ]; do
  sleep 1
done

cleanup

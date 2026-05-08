#!/bin/bash
# 손 마우스 전체 실행 스크립트 (SSH에서 한 번에 실행)

export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
echo "XAUTHORITY=$XAUTHORITY"
source ~/ros2_ws/install/setup.bash

cleanup() {
    echo "종료 중..."
    kill 0
    exit 0
}
trap cleanup SIGINT SIGTERM

# 손 감지 노드
ros2 run jiho_media hand_detection_node.py &

sleep 2

# 손 마우스 노드
python3 ~/ros2_ws/src/jiho_media/scripts/hand_mouse_node.py &

# 시각화
python3 ~/ros2_ws/src/jiho_media/scripts/image_viewer.py &

echo "실행 완료 — Ctrl+C로 전체 종료"
wait

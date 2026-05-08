#!/bin/bash
# jiho_media 손 마우스 실행 스크립트
# 사용법: ./run_hand_mouse.sh [터미널 번호: 1|2|3]
#   1 — 손 감지 노드
#   2 — 손 마우스 노드
#   3 — 시각화

case "$1" in
  1)
    source ~/ros2_ws/install/setup.bash
    ros2 run jiho_media hand_detection_node.py
    ;;
  2)
    export DISPLAY=:0
    export XAUTHORITY=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/src/jiho_media/scripts/hand_mouse_node.py
    ;;
  3)
    export DISPLAY=:0
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/src/jiho_media/scripts/image_viewer.py
    ;;
  *)
    echo "사용법: $0 [1|2|3]"
    echo "  1 — 손 감지 노드 (항상 먼저 실행)"
    echo "  2 — 손 마우스 노드"
    echo "  3 — 시각화"
    ;;
esac

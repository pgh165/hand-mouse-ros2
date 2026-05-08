# Hand Mouse — ROS 2 + MediaPipe

손을 카메라에 보여주면 마우스처럼 제어할 수 있는 ROS 2 패키지입니다.  
MediaPipe HandLandmarker로 손 관절 21개를 인식하고, evdev uinput으로 실제 마우스 이벤트를 생성합니다.

## 시스템 구성

```
카메라
  │
  ▼
hand_detection_node  (MediaPipe)
  ├─► /hand_landmarks           (HandLandmarks 커스텀 메시지)
  └─► /hand_image/compressed    (JPEG 카메라 피드)
          │
          ▼
    hand_mouse_node  (evdev uinput)
      ├─► 마우스 이동 / 클릭 / 드래그
      └─► /hand_mouse/image/compressed  (상태 오버레이)
                  │
                  ▼
          image_viewer  (OpenCV 창)
```

## 패키지 구조

```
jiho_media/
├── msg/
│   ├── Landmark.msg         # x, y, z (정규화 좌표)
│   └── HandLandmarks.msg    # 헤더 + 손 개수 + 랜드마크 배열
└── scripts/
    ├── hand_detection_node.py   # MediaPipe → ROS 2 퍼블리셔
    ├── hand_mouse_node.py       # ROS 2 → 마우스 제어
    ├── image_viewer.py          # 시각화 뷰어
    ├── run_hand_mouse.sh        # 노드별 개별 실행
    └── start_hand_mouse.sh     # 전체 한 번에 실행
```

## 제스처

| 제스처 | 동작 |
|--------|------|
| 손등을 카메라로 | 추적 활성화 (손바닥 방향은 무시) |
| 손 이동 | 마우스 커서 이동 (상대 모드) |
| 엄지 + 검지 핀치 | 좌클릭 |
| 엄지 + 검지 핀치 후 이동 | 드래그 |
| 엄지 + 중지 핀치 | 우클릭 |

> 커서는 손 중앙(중지 MCP, 랜드마크 9번)의 **상대 이동**을 추적합니다.

## 의존성

### ROS 2
- ROS 2 Humble 이상
- `rclpy`, `std_msgs`, `sensor_msgs`

### Python
```bash
pip install mediapipe opencv-python evdev numpy
```

> `evdev` 사용을 위해 사용자가 `input` 그룹에 속해야 합니다:
> ```bash
> sudo usermod -aG input $USER
> ```

## 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select jiho_media
source install/setup.bash
```

## 실행

### 방법 1: 한 번에 전체 실행

```bash
bash ~/ros2_ws/src/jiho_media/scripts/start_hand_mouse.sh
```

### 방법 2: 터미널 3개로 개별 실행

```bash
# 터미널 1 — 손 감지 (항상 먼저)
bash ~/ros2_ws/src/jiho_media/scripts/run_hand_mouse.sh 1

# 터미널 2 — 마우스 제어
bash ~/ros2_ws/src/jiho_media/scripts/run_hand_mouse.sh 2

# 터미널 3 — 시각화
bash ~/ros2_ws/src/jiho_media/scripts/run_hand_mouse.sh 3
```

## 파라미터 (hand_detection_node)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `camera_index` | `0` | 카메라 디바이스 번호 |
| `max_num_hands` | `2` | 최대 인식 손 개수 |
| `min_detection_confidence` | `0.7` | 감지 신뢰도 임계값 |
| `min_tracking_confidence` | `0.5` | 추적 신뢰도 임계값 |
| `publish_rate` | `30.0` | 퍼블리시 주기 (Hz) |
| `model_path` | `""` | MediaPipe 모델 경로 (비워두면 자동 다운로드) |

## 커스텀 메시지

### `jiho_media/Landmark`
```
float32 x  # 정규화 x 좌표 [0.0, 1.0]
float32 y  # 정규화 y 좌표 [0.0, 1.0]
float32 z  # 손목 기준 깊이 (음수 = 카메라에 가까움)
```

### `jiho_media/HandLandmarks`
```
std_msgs/Header header
uint8 num_hands          # 감지된 손 개수 (0~2)
string[] handedness      # "Left" 또는 "Right"
Landmark[] landmarks     # 손당 21개, row-major 순서
```

## 동작 원리

1. **손 감지**: MediaPipe HandLandmarker (VIDEO 모드, float16)로 21개 관절 좌표 추출
2. **손등 판별**: 손목·검지 MCP·소지 MCP의 외적 z값으로 손바닥/손등 구분
3. **커서 이동**: 중지 MCP(9번) 프레임 간 이동량을 지수이동평균으로 스무딩 후 uinput REL_X/Y 이벤트 전송
4. **핀치 감지**: 엄지↔검지 정규화 거리 < 0.06 → 좌클릭/드래그 / 엄지↔중지 < 0.06 → 우클릭
5. **드래그 판단**: 핀치 상태에서 커서 30px 이상 이동 시 BTN_LEFT press, 릴리즈 시 release

## 라이선스

Apache-2.0

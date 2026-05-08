#!/usr/bin/env python3
"""Hand mouse node — left click (pinch), drag (pinch+move), back-of-hand only."""

import os
import math

from evdev import UInput, ecodes as ec


class _Button:
    left  = "left"
    right = "right"

Button = _Button


class MouseController:
    """uinput 기반 마우스 — Wayland/X11 무관하게 작동."""

    def __init__(self):
        cap = {
            ec.EV_KEY: [ec.BTN_LEFT, ec.BTN_RIGHT],
            ec.EV_REL: [ec.REL_X, ec.REL_Y],
        }
        self._ui  = UInput(cap, name="hand-mouse-virtual")
        self._pos = [0, 0]

    @property
    def position(self):
        return tuple(self._pos)

    @position.setter
    def position(self, pos):
        dx = int(pos[0]) - self._pos[0]
        dy = int(pos[1]) - self._pos[1]
        if dx or dy:
            self._ui.write(ec.EV_REL, ec.REL_X, dx)
            self._ui.write(ec.EV_REL, ec.REL_Y, dy)
            self._ui.syn()
        self._pos = [int(pos[0]), int(pos[1])]

    def _btn(self, button):
        return ec.BTN_LEFT if button == "left" else ec.BTN_RIGHT

    def click(self, button):
        b = self._btn(button)
        self._ui.write(ec.EV_KEY, b, 1); self._ui.syn()
        self._ui.write(ec.EV_KEY, b, 0); self._ui.syn()

    def press(self, button):
        b = self._btn(button)
        self._ui.write(ec.EV_KEY, b, 1); self._ui.syn()

    def release(self, button):
        b = self._btn(button)
        self._ui.write(ec.EV_KEY, b, 0); self._ui.syn()

    def close(self):
        self._ui.close()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np

from jiho_media.msg import HandLandmarks

# ── tuning ───────────────────────────────────────────────────────────────────
_SENSITIVITY      = 3.5   # 상대 이동 감도
_SMOOTH_ALPHA     = 0.3   # 속도 스무딩
_PINCH_THRESH     = 0.06
_CLICK_COOLDOWN   = 15
_RELEASE_CONFIRM  = 6
_DRAG_MOVE_THRESH = 30
_DRAG_GRACE       = 10

_CANVAS_H, _CANVAS_W = 480, 640


def _dist(a, b) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _is_back_of_hand(lms, handedness: str) -> bool:
    """손등이 카메라를 향하는지 외적으로 판별."""
    w = lms[0]
    i = lms[5]   # 검지 MCP
    p = lms[17]  # 소지 MCP
    cross_z = (i.x - w.x) * (p.y - w.y) - (i.y - w.y) * (p.x - w.x)
    if handedness == "Right":
        return cross_z > 0
    else:
        return cross_z < 0


def _get_screen_size():
    try:
        import subprocess
        out = subprocess.check_output(
            ["xdpyinfo", "-display", ":0"], text=True, stderr=subprocess.DEVNULL
        )
        for line in out.splitlines():
            if "dimensions:" in line:
                w, h = line.split()[1].split("x")
                return int(w), int(h)
    except Exception:
        pass
    return 1920, 1080


class HandMouseNode(Node):

    def __init__(self):
        super().__init__("hand_mouse_node")

        self._mouse = MouseController()
        self._screen_w, self._screen_h = _get_screen_size()
        self._cursor_x = self._screen_w / 2.0
        self._cursor_y = self._screen_h / 2.0

        self._cooldown      = 0
        self._pinching      = False
        self._pinch_frames  = 0
        self._drag_active   = False
        self._pinch_start_x = 0.0
        self._pinch_start_y = 0.0
        self._release_count = 0
        self._r_pinching    = False
        self._r_cooldown    = 0
        self._status_timer  = 0
        self._prev_tip_x: float | None = None
        self._prev_tip_y: float | None = None
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._cam_frame: np.ndarray | None = None
        self._status        = "no hand"

        self.create_subscription(HandLandmarks, "hand_landmarks", self._cb, 10)
        self.create_subscription(
            CompressedImage, "hand_image/compressed", self._on_cam, 10
        )
        self._pub_img = self.create_publisher(
            CompressedImage, "hand_mouse/image/compressed", 10
        )
        self.get_logger().info(
            f"HandMouseNode started — screen={self._screen_w}x{self._screen_h}"
        )

    def _cb(self, msg: HandLandmarks):
        if self._cooldown > 0:
            self._cooldown -= 1

        if msg.num_hands == 0 or len(msg.landmarks) < 21:
            if self._drag_active:
                self._mouse.release(Button.left)
                self._drag_active = False
            self._pinching      = False
            self._prev_tip_x    = None
            self._prev_tip_y    = None
            self._vel_x         = 0.0
            self._vel_y         = 0.0
            self._status        = "no hand"
            self._publish_canvas(None)
            return

        lms = msg.landmarks[:21]
        handedness = msg.handedness[0] if msg.handedness else "Right"

        # 손등이 아니면 무시
        if not _is_back_of_hand(lms, handedness):
            self._status = "palm (ignored)"
            self._publish_canvas(lms)
            return

        # 커서 이동 (상대 모드) — 손 중앙(중지 MCP, lm[9]) 추적
        tip_x = lms[9].x
        tip_y = lms[9].y
        if self._prev_tip_x is not None:
            dx = -(tip_x - self._prev_tip_x) * self._screen_w * _SENSITIVITY
            dy = -(tip_y - self._prev_tip_y) * self._screen_h * _SENSITIVITY
            self._vel_x = _SMOOTH_ALPHA * dx + (1 - _SMOOTH_ALPHA) * self._vel_x
            self._vel_y = _SMOOTH_ALPHA * dy + (1 - _SMOOTH_ALPHA) * self._vel_y
            self._cursor_x = max(0.0, min(float(self._screen_w), self._cursor_x + self._vel_x))
            self._cursor_y = max(0.0, min(float(self._screen_h), self._cursor_y + self._vel_y))
            self._mouse.position = (int(self._cursor_x), int(self._cursor_y))

        self._prev_tip_x = tip_x
        self._prev_tip_y = tip_y

        pinch   = _dist(lms[4], lms[8]) < _PINCH_THRESH
        r_pinch = _dist(lms[4], lms[12]) < _PINCH_THRESH and not pinch

        if self._r_cooldown > 0:
            self._r_cooldown -= 1

        # ── 우클릭: 엄지+중지 핀치 ───────────────────────────────────────────
        if r_pinch:
            if not self._r_pinching and self._r_cooldown == 0:
                self._mouse.click(Button.right)
                self._r_pinching   = True
                self._r_cooldown   = _CLICK_COOLDOWN
                self._status       = "right click!"
                self._status_timer = 30
                self.get_logger().info("Right click!")
        else:
            self._r_pinching = False

        # ── 드래그 / 좌클릭: 핀치 ────────────────────────────────────────────
        if pinch and not r_pinch:
            self._release_count = 0
            if not self._pinching:
                self._pinching      = True
                self._pinch_frames  = 0
                self._pinch_start_x = self._cursor_x
                self._pinch_start_y = self._cursor_y
            else:
                self._pinch_frames += 1
                move_dist = math.hypot(
                    self._cursor_x - self._pinch_start_x,
                    self._cursor_y - self._pinch_start_y,
                )
                if (not self._drag_active
                        and self._pinch_frames > _DRAG_GRACE
                        and move_dist > _DRAG_MOVE_THRESH):
                    self._mouse.press(Button.left)
                    self._drag_active = True
                    self._status      = "drag"
                    self.get_logger().info("Drag start")
                elif self._drag_active:
                    self._status = "drag"
        else:
            self._release_count += 1
            if self._pinching and self._release_count >= _RELEASE_CONFIRM:
                if self._drag_active:
                    self._mouse.release(Button.left)
                    self._drag_active = False
                    self.get_logger().info("Drag end")
                elif self._cooldown == 0:
                    self._mouse.click(Button.left)
                    self._status       = "left click!"
                    self._status_timer = 30
                    self._cooldown     = _CLICK_COOLDOWN
                    self.get_logger().info("Left click!")
                self._pinching      = False
                self._pinch_frames  = 0
                self._release_count = 0

        if self._status_timer > 0:
            self._status_timer -= 1
        elif self._status not in ("drag", "no hand", "palm (ignored)"):
            self._status = "tracking"

        self._publish_canvas(lms)

    def _on_cam(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            self._cam_frame = cv2.resize(frame, (_CANVAS_W, _CANVAS_H))

    def _publish_canvas(self, lms):
        canvas = np.zeros((_CANVAS_H, _CANVAS_W, 3), dtype=np.uint8)

        status_colors = {
            "no hand":        (100, 100, 100),
            "palm (ignored)": (80, 80, 80),
            "tracking":       (0, 255, 100),
            "left click!":    (0, 180, 255),
            "right click!":   (255, 100, 0),
            "drag":           (255, 200, 0),
        }
        color = status_colors.get(self._status, (200, 200, 200))
        cv2.putText(canvas, self._status, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        legend = [
            "index closed: pause (lift)",
            "pinch + move: drag",
            "thumb+middle: right click",
            "thumb+index: left click",
        ]
        for i, text in enumerate(legend):
            cv2.putText(canvas, text, (10, _CANVAS_H - 20 - i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)

        if lms:
            ix = int(lms[8].x * _CANVAS_W)
            iy = int(lms[8].y * _CANVAS_H)
            tx = int(lms[4].x * _CANVAS_W)
            ty = int(lms[4].y * _CANVAS_H)
            mx = int(lms[9].x * _CANVAS_W)
            my = int(lms[9].y * _CANVAS_H)
            cv2.circle(canvas, (ix, iy), 12, color, -1)
            cv2.circle(canvas, (tx, ty), 8, (200, 200, 200), -1)
            cv2.line(canvas, (tx, ty), (ix, iy), (150, 150, 150), 2)
            cv2.circle(canvas, (mx, my), 10, (0, 0, 255), -1)

        cx = int(self._cursor_x / self._screen_w * _CANVAS_W)
        cy = int(self._cursor_y / self._screen_h * _CANVAS_H)
        cv2.drawMarker(canvas, (cx, cy), (255, 255, 0), cv2.MARKER_CROSS, 20, 2)

        cam = self._cam_frame if self._cam_frame is not None else \
              np.full((_CANVAS_H, _CANVAS_W, 3), 30, dtype=np.uint8)

        ok, buf = cv2.imencode(".jpg", np.hstack([cam, canvas]),
                               [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            img_msg = CompressedImage(format="bgr8; jpeg compressed")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "hand_mouse"
            img_msg.data = buf.tobytes()
            self._pub_img.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandMouseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._drag_active:
            node._mouse.release(Button.left)
        node._mouse.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
gesture_utils.py
================
Hand gesture recognition utilities using MediaPipe Hands.

Landmark index map (21 points):
    0  : WRIST
    1-4 : THUMB  (CMC → MCP → IP → TIP)
    5-8 : INDEX  (MCP → PIP → DIP → TIP)
    9-12: MIDDLE (MCP → PIP → DIP → TIP)
    13-16: RING  (MCP → PIP → DIP → TIP)
    17-20: PINKY (MCP → PIP → DIP → TIP)

Gesture → Command map:
    ✊  FIST          → STOP
    🖐️  OPEN PALM     → FORWARD
    ☝️  INDEX UP      → FORWARD
    👇  INDEX DOWN    → BACKWARD
    👈  INDEX LEFT    → TURN_LEFT
    👉  INDEX RIGHT   → TURN_RIGHT
    ✌️  PEACE (2 fin) → BACKWARD_SLOW
    🤙  3 FINGERS     → FORWARD_FAST
"""

import math
from dataclasses import dataclass
from typing import List, Tuple


# Let's define the available commands for the robot
class GestureCommand:
    STOP          = "STOP"
    FORWARD       = "FORWARD"
    FORWARD_FAST  = "FORWARD_FAST"
    BACKWARD      = "BACKWARD"
    BACKWARD_SLOW = "BACKWARD_SLOW"
    TURN_LEFT     = "TURN_LEFT"
    TURN_RIGHT    = "TURN_RIGHT"


# These are the actual speeds (linear and angular) that correspond to each command
COMMAND_VELOCITIES = {
    GestureCommand.STOP:          (0.0,   0.0),
    GestureCommand.FORWARD:       (0.3,   0.0),
    GestureCommand.FORWARD_FAST:  (0.6,   0.0),
    GestureCommand.BACKWARD:      (-0.3,  0.0),
    GestureCommand.BACKWARD_SLOW: (-0.15, 0.0),
    GestureCommand.TURN_LEFT:     (0.0,   0.8),
    GestureCommand.TURN_RIGHT:    (0.0,  -0.8),
}

# Colors to use when we draw the status overlay on the video feed
COMMAND_COLORS = {
    GestureCommand.STOP:          (0,   0,   255),   # Red
    GestureCommand.FORWARD:       (0,   200, 0),     # Green
    GestureCommand.FORWARD_FAST:  (0,   255, 100),   # Bright green
    GestureCommand.BACKWARD:      (255, 100, 0),     # Blue-orange
    GestureCommand.BACKWARD_SLOW: (255, 200, 0),     # Light blue
    GestureCommand.TURN_LEFT:     (255, 255, 0),     # Cyan
    GestureCommand.TURN_RIGHT:    (0,   255, 255),   # Yellow
}

COMMAND_ICONS = {
    GestureCommand.STOP:          "STOP  [FIST / OPEN PALM]",
    GestureCommand.FORWARD:       "FWD   [INDEX UP]",
    GestureCommand.FORWARD_FAST:  "FAST  [3 FINGERS]",
    GestureCommand.BACKWARD:      "BACK  [INDEX DOWN]",
    GestureCommand.BACKWARD_SLOW: "SLOW- [PEACE SIGN]",
    GestureCommand.TURN_LEFT:     "LEFT  [INDEX LEFT]",
    GestureCommand.TURN_RIGHT:    "RIGHT [INDEX RIGHT]",
}


@dataclass
class GestureResult:
    """Result of gesture classification."""
    command: str
    confidence: float          # 0.0 – 1.0
    finger_states: List[bool]  # [thumb, index, middle, ring, pinky]
    index_angle_deg: float     # angle of index finger vector
    label: str                 # human-readable label
    color: Tuple[int, int, int]


# A quick helper function to figure out which fingers are open or closed
def _get_finger_states(lm) -> List[bool]:
    """
    Returns [thumb, index, middle, ring, pinky] as True (open) / False (closed).
    Works for both left and right hands (uses relative x for thumb).
    lm: list of landmark objects with .x, .y, .z attributes (normalized 0-1).
    """
    fingers = []

    # Thumb: compare tip x to IP x — determine hand side from wrist vs index MCP
    thumb_tip = lm[4]
    thumb_ip  = lm[3]
    wrist     = lm[0]
    index_mcp = lm[5]

    if wrist.x < index_mcp.x:
        # Right hand: thumb tip should be to the LEFT of IP to be open
        fingers.append(thumb_tip.x < thumb_ip.x)
    else:
        # Left hand: thumb tip to the RIGHT of IP
        fingers.append(thumb_tip.x > thumb_ip.x)

    # Index, Middle, Ring, Pinky: tip.y < pip.y (smaller y = higher in image)
    tip_ids = [8,  12, 16, 20]
    pip_ids = [6,  10, 14, 18]
    for tip_id, pip_id in zip(tip_ids, pip_ids):
        fingers.append(lm[tip_id].y < lm[pip_id].y)

    return fingers  # [thumb, index, middle, ring, pinky]


# This helper checks which direction the index finger is pointing
def _get_index_direction(lm) -> Tuple[str, float]:
    """
    Returns (direction_str, angle_deg) for the index finger vector.
    Vector: index MCP (5) → index TIP (8).
    Angle 0° = right, 90° = up, 180°/-180° = left, -90° = down.
    """
    dx = lm[8].x - lm[5].x
    dy = lm[8].y - lm[5].y  # image y increases downward → negate for math

    angle = math.degrees(math.atan2(-dy, dx))  # standard math angle

    if -45 <= angle <= 45:
        direction = "RIGHT"
    elif 45 < angle <= 135:
        direction = "UP"
    elif angle > 135 or angle < -135:
        direction = "LEFT"
    else:  # -135 < angle < -45
        direction = "DOWN"

    return direction, angle


# This just counts how many fingers (excluding the thumb) are open
def _openness_score(fingers: List[bool]) -> int:
    """Number of open non-thumb fingers (index through pinky)."""
    return sum(fingers[1:])


# This is the main function that looks at the hand landmarks and decides what command to issue
def classify_gesture(lm) -> GestureResult:
    """
    Classify hand landmarks into a robot GestureCommand.

    Args:
        lm: list of 21 MediaPipe NormalizedLandmark objects

    Returns:
        GestureResult with command, confidence, and display info
    """
    fingers = _get_finger_states(lm)
    thumb, index, middle, ring, pinky = fingers

    direction, angle = _get_index_direction(lm)
    n_open = _openness_score(fingers)

    command    = GestureCommand.STOP
    confidence = 0.9

    # Let's go through the possible gestures in order of priority

    # 1. All 4 non-thumb fingers open (open palm) → STOP  [safe default]
    if n_open == 4:
        command    = GestureCommand.STOP
        confidence = 0.95

    # 2. All fingers closed (fist) → STOP
    elif n_open == 0:
        command    = GestureCommand.STOP
        confidence = 0.95

    # 3. Only index finger open → direction-based command
    elif index and not middle and not ring and not pinky:
        if direction == "UP":
            command = GestureCommand.FORWARD
        elif direction == "DOWN":
            command = GestureCommand.BACKWARD
        elif direction == "LEFT":
            command = GestureCommand.TURN_LEFT
        elif direction == "RIGHT":
            command = GestureCommand.TURN_RIGHT
        confidence = 0.85

    # 4. Peace sign (index + middle, others closed) → BACKWARD SLOW
    elif index and middle and not ring and not pinky:
        command    = GestureCommand.BACKWARD_SLOW
        confidence = 0.85

    # 5. Three fingers (index + middle + ring) → FORWARD FAST
    elif index and middle and ring and not pinky:
        command    = GestureCommand.FORWARD_FAST
        confidence = 0.80

    # 6. Pinky only → TURN RIGHT (alternative mapping)
    elif pinky and not index and not middle and not ring:
        command    = GestureCommand.TURN_RIGHT
        confidence = 0.75

    # Default
    else:
        command    = GestureCommand.STOP
        confidence = 0.5

    return GestureResult(
        command=command,
        confidence=confidence,
        finger_states=fingers,
        index_angle_deg=angle,
        label=COMMAND_ICONS.get(command, command),
        color=COMMAND_COLORS.get(command, (200, 200, 200)),
    )


# A quick lookup to translate the command string into physical robot velocities
def command_to_velocity(command: str) -> Tuple[float, float]:
    """
    Convert a GestureCommand string to (linear_x, angular_z) velocities.

    Returns:
        (linear_x m/s, angular_z rad/s)
    """
    return COMMAND_VELOCITIES.get(command, (0.0, 0.0))


# We use this to smooth out the commands over a few frames so the robot doesn't jitter
class GestureSmoother:
    """
    Temporal smoothing using a sliding window majority vote.
    Prevents flickering between gestures on noisy frames.
    """

    def __init__(self, window_size: int = 10):
        self.window_size = window_size
        self._history: List[str] = []

    def update(self, command: str) -> str:
        """Add new command and return the smoothed (majority) command."""
        self._history.append(command)
        if len(self._history) > self.window_size:
            self._history.pop(0)

        counts: dict = {}
        for c in self._history:
            counts[c] = counts.get(c, 0) + 1
        return max(counts, key=counts.get)

    def reset(self):
        self._history.clear()


# These functions draw the nice visual overlay on top of the webcam feed
def draw_gesture_overlay(frame, result: GestureResult, smoothed_cmd: str,
                          linear_x: float, angular_z: float):
    """
    Draw a HUD overlay on the OpenCV frame showing gesture info and velocities.
    """
    import cv2

    color   = result.color
    s_color = COMMAND_COLORS.get(smoothed_cmd, color)

    # Draw a dark, semi-transparent background panel so the text is easy to read
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (380, 220), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    # Display the raw gesture that we detected
    cv2.putText(frame, f"Gesture: {result.label}",
                (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    # Display the smoothed, final command in a larger font
    cv2.putText(frame, f"CMD: {smoothed_cmd}",
                (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 1.0, s_color, 3)

    # Show the current linear and angular speeds
    cv2.putText(frame, f"Linear X : {linear_x:+.2f} m/s",
                (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
    cv2.putText(frame, f"Angular Z: {angular_z:+.2f} rad/s",
                (10, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)

    # Draw a small bar for each finger to show if it's currently open or closed
    finger_names = ["T", "I", "M", "R", "P"]
    for i, (name, state) in enumerate(zip(finger_names, result.finger_states)):
        x = 15 + i * 60
        bar_color = (0, 255, 0) if state else (80, 80, 80)
        cv2.rectangle(frame, (x, 160), (x + 45, 185), bar_color, -1)
        cv2.putText(frame, name, (x + 14, 178),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    # Draw a progress bar showing how confident we are in the current gesture
    bar_w = int(result.confidence * 180)
    cv2.rectangle(frame, (10, 200), (190, 215), (60, 60, 60), -1)
    cv2.rectangle(frame, (10, 200), (10 + bar_w, 215), s_color, -1)
    cv2.putText(frame, f"Conf: {result.confidence:.0%}",
                (200, 214), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    # Draw a little compass arrow showing the robot's movement direction
    h, w = frame.shape[:2]
    _draw_direction_arrow(frame, smoothed_cmd, w - 90, 90, 60)

    return frame


def _draw_direction_arrow(frame, command: str, cx: int, cy: int, size: int):
    """Draw a direction indicator compass in the top-right corner."""
    import cv2

    cv2.circle(frame, (cx, cy), size + 10, (40, 40, 40), -1)
    cv2.circle(frame, (cx, cy), size + 10, (100, 100, 100), 2)

    arrow_map = {
        GestureCommand.FORWARD:       (cx, cy - size, cx, cy + size // 3),
        GestureCommand.FORWARD_FAST:  (cx, cy - size, cx, cy + size // 3),
        GestureCommand.BACKWARD:      (cx, cy + size, cx, cy - size // 3),
        GestureCommand.BACKWARD_SLOW: (cx, cy + size, cx, cy - size // 3),
        GestureCommand.TURN_LEFT:     (cx - size, cy, cx + size // 3, cy),
        GestureCommand.TURN_RIGHT:    (cx + size, cy, cx - size // 3, cy),
        GestureCommand.STOP:          None,
    }

    color = COMMAND_COLORS.get(command, (200, 200, 200))

    if command == GestureCommand.STOP:
        cv2.putText(frame, "STOP", (cx - 25, cy + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    else:
        arrow = arrow_map.get(command)
        if arrow:
            cv2.arrowedLine(frame, (arrow[0], arrow[1]),
                            (arrow[2], arrow[3]), color, 4, tipLength=0.4)

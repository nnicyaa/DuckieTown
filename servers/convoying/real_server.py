import sys
import os
import signal
import threading
import time
import queue
import socket

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, '..', '..')
sys.path.insert(0, project_root)

import cv2
from flask import Flask, Response, render_template_string, jsonify, request

from tasks.object_detection.packages.agent import ObjectDetectionAgent
from tasks.visual_lane_servoing.packages.agent import LaneServoingAgent
from tasks.convoying.packages.target_tracker_activity import TargetTracker
from tasks.convoying.packages.convoy_controller_statemachine import ConvoyController
from tasks.convoying.packages.red_line_gate import RedLineGate
from tasks.convoying.packages.follow_types import SEARCH, FOLLOWING, STOPPED, LOST_TARGET, TOO_CLOSE_STATE

from servers.convoying.visualization import create_convoying_visualization
from servers.templates.convoying import CONVOYING_TEMPLATE as HTML_TEMPLATE

from duckiebot.camera_driver import CameraDriver
from duckiebot.wheel_driver import DaguWheelsDriver
from duckiebot.wheel_driver.wheels_driver_abs import WheelPWMConfiguration
from launcher.ports import find_available_port
from servers.common import make_frame_generator, shutdown_cleanup, suppress_http_logs


app = Flask(__name__)

camera = None
wheels = None
object_agent = None
lane_agent = None
tracker = None
convoy_ctrl = None
red_line_gate = None

running = False
stop_event = threading.Event()

# Async detection — keeps the video stream smooth while YOLO runs.
_frame_queue = queue.Queue(maxsize=1)
_detection_lock = threading.Lock()
_last_detections = []  # scaled to full camera resolution

_last_target = None
_last_valid_target = None
_last_command = None
_last_lane_left = 0.0
_last_lane_right = 0.0
_last_red_line_state = None

# Prints only when lane mode changes.
_last_lane_disabled = False

# Prints only when follower state changes.
_last_follower_state = SEARCH

# ---------------------------------------------------------------------------
# Manual driving state
# ---------------------------------------------------------------------------
# When manual_mode is True, the autonomous pipeline (detection, marker
# tracking, lane servoing, convoy state machine) keeps running every frame
# exactly as before -- this is what feeds the visualization/debug panel --
# but its computed command is NOT sent to the wheels. Instead the wheels
# are driven from _manual_left/_manual_right, set by POST /manual/drive
# (arrow keys / on-screen D-pad in the browser).
manual_mode = False
_manual_lock = threading.Lock()
_manual_left = 0.0
_manual_right = 0.0

MANUAL_MAX_SPEED = 0.30
MANUAL_TURN_RATIO = 0.6


# ---------------------------------------------------------------------------
# Background detection thread
# ---------------------------------------------------------------------------

def _detection_loop():
    """Pull frames from the queue, run YOLO, store scaled detections."""
    global _last_detections

    while not stop_event.is_set():
        if object_agent is None or not object_agent.model_loaded:
            time.sleep(0.1)
            continue

        try:
            frame_rgb, orig_h, orig_w = _frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        result = object_agent.detect(frame_rgb)
        if result is None:
            continue

        # Scale boxes from model input resolution back to full camera size.
        sx = orig_w / float(object_agent.img_size)
        sy = orig_h / float(object_agent.img_size)

        scaled = [
            ((int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy)), score, class_id)
            for (x1, y1, x2, y2), score, class_id in result
        ]

        with _detection_lock:
            _last_detections = scaled


# ---------------------------------------------------------------------------
# Frame pipeline (called for every MJPEG frame)
# ---------------------------------------------------------------------------

def visualize(frame_bgr):
    """
    frame_bgr: BGR frame from the real camera (CameraDriver.read() returns BGR).

    Pipeline:
        camera frame (BGR)
        -> frame_rgb derived for red-line gate / lane servoing (which
           expect RGB, per their existing implementations)
        -> object detection (async thread, operates on frame_rgb)
        -> target + marker-bracket tracking (operates on frame_bgr --
           MarkerBracketDetector's color thresholds are BGR-based; passing
           frame_rgb here would swap channels during its internal
           cvtColor and produce a slightly wrong grayscale/HSV read)
        -> lane servoing if allowed (fallback steering source)
        -> convoy state machine (SEARCH/FOLLOWING/STOPPED/LOST_TARGET/
           TOO_CLOSE_STATE), using the marker as primary steering signal
           and lane servoing as fallback when the marker isn't trusted
        -> wheel speeds (autonomous command, OR manual override)
        -> browser visualization
    """
    global _last_target
    global _last_valid_target
    global _last_command
    global _last_lane_left
    global _last_lane_right
    global _last_red_line_state
    global _last_lane_disabled
    global _last_follower_state

    if frame_bgr is None:
        return frame_bgr

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    image_height, image_width = frame_bgr.shape[:2]

    # Red line detection.
    # If close red line is detected, lane following is disabled for 5 seconds.
    if red_line_gate is not None:
        red_line_state = red_line_gate.update(frame_rgb)
    else:
        red_line_state = None

    _last_red_line_state = red_line_state
    lane_disabled = bool(red_line_state and red_line_state.lane_disabled)

    if lane_disabled != _last_lane_disabled:
        if lane_disabled:
            remaining = getattr(red_line_state, "disabled_remaining", 0.0)
            red_ratio = getattr(red_line_state, "red_ratio", 0.0)
            red_row_ratio = getattr(red_line_state, "red_row_ratio", 0.0)

            print(
                "[Convoying] LANE DETECTION OFF -> LEADER-ONLY FOLLOWING ON "
                f"| remaining={remaining:.2f}s "
                f"| red_ratio={red_ratio:.4f} "
                f"| red_row_ratio={red_row_ratio:.4f}"
            )
        else:
            print("[Convoying] LANE DETECTION ON -> normal lane + leader following resumed")

        _last_lane_disabled = lane_disabled

    # Feed a downscaled copy to the detection thread (non-blocking).
    if object_agent is not None and object_agent.model_loaded:
        small = cv2.resize(frame_rgb, (object_agent.img_size, object_agent.img_size))
        try:
            _frame_queue.put_nowait((small, image_height, image_width))
        except queue.Full:
            pass

    # Grab latest detections. They are already scaled to full camera resolution.
    with _detection_lock:
        detections = list(_last_detections)

    # Target + marker-bracket tracking.
    # frame_bgr (NOT frame_rgb) is passed here -- see the pipeline note in
    # this function's docstring for why that matters.
    if tracker is not None:
        target = tracker.update(
            detections=detections,
            image_height=image_height,
            image_width=image_width,
            frame_bgr=frame_bgr,
        )
    else:
        target = _last_target

    if target is not None and target.found:
        _last_valid_target = target

    # Lane servoing.
    # Used as a fallback steering source by the state machine (SEARCH /
    # LOST_TARGET) when the marker bracket isn't currently trusted. Still
    # skipped entirely when lane_disabled (red-line window), same as
    # before -- the state machine treats "no lane signal" as a reason to
    # stop rather than guess, if the marker is also unavailable then.
    if lane_agent is not None and not lane_disabled:
        try:
            lane_left, lane_right = lane_agent.compute_commands(frame_rgb)
        except Exception as e:
            print(f"[Convoying] Lane servoing error: {e}")
            lane_left, lane_right = 0.0, 0.0
    else:
        lane_left, lane_right = 0.0, 0.0

    # Convoy state machine.
    # Always runs (even in manual mode) so the debug panel stays live.
    if convoy_ctrl is not None and target is not None:
        command = convoy_ctrl.decide(
            target=target,
            lane_left=lane_left,
            lane_right=lane_right,
            image_width=image_width,
            image_height=image_height,
            lane_disabled=lane_disabled,
        )
    else:
        command = _last_command

    follower_state = command.state if command is not None else SEARCH
    if follower_state != _last_follower_state:
        print(f"[Convoying] STATE {_last_follower_state} -> {follower_state}")
        _last_follower_state = follower_state

    # Apply wheel commands only when running.
    # In manual mode, the autonomous command above is computed (for the
    # debug panel) but ignored here -- wheels are driven from the
    # browser-controlled _manual_left/_manual_right instead.
    if wheels is not None:
        if not running:
            wheels.set_wheels_speed(0.0, 0.0)
        elif manual_mode:
            with _manual_lock:
                wheels.set_wheels_speed(_manual_left, _manual_right)
        elif command is not None:
            wheels.set_wheels_speed(command.left_speed, command.right_speed)
        else:
            wheels.set_wheels_speed(0.0, 0.0)

    _last_target = target
    _last_command = command
    _last_lane_left = lane_left
    _last_lane_right = lane_right

    return create_convoying_visualization(
        image_bgr=frame_bgr,
        lane_debug_info=lane_agent.last_debug_info if lane_agent else {},
        target=target,
        command=command,
        lane_left=lane_left,
        lane_right=lane_right,
        detections=detections,
    )


# Real camera returns BGR frames — rgb=False.
generate_frames = make_frame_generator(lambda: camera, visualize, quality=50, rgb=False)


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, hostname=socket.gethostname())


@app.route('/video')
def video():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
    )


@app.route('/start', methods=['POST'])
def start():
    global running

    running = True
    print("[Convoying] Started")
    return jsonify({'status': 'running'})


@app.route('/stop', methods=['POST'])
def stop():
    global running

    running = False

    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)

    print("[Convoying] Stopped")
    return jsonify({'status': 'stopped'})


@app.route('/reset', methods=['POST'])
def reset():
    global running
    global _last_detections
    global _last_target
    global _last_valid_target
    global _last_command
    global _last_red_line_state
    global _last_lane_disabled
    global _last_follower_state
    global _manual_left, _manual_right

    running = False

    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)

    if tracker is not None:
        tracker.reset()

    if convoy_ctrl is not None:
        convoy_ctrl.reset()

    if red_line_gate is not None:
        red_line_gate.reset()

    with _detection_lock:
        _last_detections = []

    with _manual_lock:
        _manual_left = 0.0
        _manual_right = 0.0

    _last_target = None
    _last_valid_target = None
    _last_command = None
    _last_red_line_state = None
    _last_lane_disabled = False
    _last_follower_state = SEARCH

    print("[Convoying] Reset")
    return jsonify({'status': 'reset'})


@app.route('/running')
def get_running():
    return jsonify({'running': running})


# ---------------------------------------------------------------------------
# Manual driving routes
# ---------------------------------------------------------------------------

@app.route('/manual/enable', methods=['POST'])
def manual_enable():
    global manual_mode, _manual_left, _manual_right

    manual_mode = True

    with _manual_lock:
        _manual_left = 0.0
        _manual_right = 0.0

    print("[Convoying] MANUAL MODE ENABLED — autonomous commands ignored")
    return jsonify({'manual_mode': True})


@app.route('/manual/disable', methods=['POST'])
def manual_disable():
    global manual_mode, _manual_left, _manual_right

    manual_mode = False

    with _manual_lock:
        _manual_left = 0.0
        _manual_right = 0.0

    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)

    print("[Convoying] MANUAL MODE DISABLED — autonomous control resumed")
    return jsonify({'manual_mode': False})


@app.route('/manual/drive', methods=['POST'])
def manual_drive():
    """
    Body: {"forward": -1..1, "turn": -1..1}
    Combinable, e.g. forward=1, turn=0.5 drives forward while turning right.
    Ignored unless manual_mode is currently enabled.
    """
    global _manual_left, _manual_right

    if not manual_mode:
        return jsonify({'ok': False, 'reason': 'manual_mode_not_enabled'}), 400

    data = request.json or {}

    forward = float(data.get('forward', 0.0))
    turn = float(data.get('turn', 0.0))

    forward = max(-1.0, min(1.0, forward))
    turn = max(-1.0, min(1.0, turn))

    base = forward * MANUAL_MAX_SPEED
    steer = turn * MANUAL_MAX_SPEED * MANUAL_TURN_RATIO

    left = base + steer
    right = base - steer

    left = max(-MANUAL_MAX_SPEED, min(MANUAL_MAX_SPEED, left))
    right = max(-MANUAL_MAX_SPEED, min(MANUAL_MAX_SPEED, right))

    with _manual_lock:
        _manual_left = left
        _manual_right = right

    return jsonify({'ok': True, 'left_speed': left, 'right_speed': right})


@app.route('/status')
def status():
    with _manual_lock:
        manual_left = _manual_left
        manual_right = _manual_right

    return jsonify({
        'running': running,
        'model_loaded': bool(getattr(object_agent, 'model_loaded', False)) if object_agent else False,
        'model_load_error': getattr(object_agent, 'load_error', None) if object_agent else None,
        'trt_building': getattr(object_agent, 'trt_building', False) if object_agent else False,
        'lane_frame_count': getattr(lane_agent, 'frame_count', 0) if lane_agent else 0,
        'target': _target_to_dict(_last_target),
        'command': _command_to_dict(_last_command),
        'lane_left': _last_lane_left,
        'lane_right': _last_lane_right,
        'lane_disabled': _last_lane_disabled,
        'red_line': _red_line_to_dict(_last_red_line_state),
        'follower_state': _last_follower_state,
        'manual_mode': manual_mode,
        'manual_left': manual_left,
        'manual_right': manual_right,
    })


@app.route('/update_config', methods=['POST'])
def update_config():
    """Tune convoy controller multipliers live from the browser."""
    data = request.json or {}

    if convoy_ctrl is not None:
        if 'close_multiplier' in data:
            convoy_ctrl.close_multiplier = float(data['close_multiplier'])
        if 'good_multiplier' in data:
            convoy_ctrl.good_multiplier = float(data['good_multiplier'])
        if 'far_multiplier' in data:
            convoy_ctrl.far_multiplier = float(data['far_multiplier'])
        if 'max_speed' in data:
            convoy_ctrl.max_speed = float(data['max_speed'])
        if 'steering_gain' in data:
            convoy_ctrl.steering_gain = float(data['steering_gain'])
        if 'steering_sign' in data:
            convoy_ctrl.steering_sign = float(data['steering_sign'])
        if 'search_speed' in data:
            convoy_ctrl.search_speed = float(data['search_speed'])
        if 'stop_detection_timeout' in data:
            convoy_ctrl.stop_detection_timeout = float(data['stop_detection_timeout'])
        if 'lost_target_timeout' in data:
            convoy_ctrl.lost_target_timeout = float(data['lost_target_timeout'])

    return jsonify({
        'close_multiplier': convoy_ctrl.close_multiplier if convoy_ctrl else None,
        'good_multiplier': convoy_ctrl.good_multiplier if convoy_ctrl else None,
        'far_multiplier': convoy_ctrl.far_multiplier if convoy_ctrl else None,
        'max_speed': convoy_ctrl.max_speed if convoy_ctrl else None,
        'steering_gain': convoy_ctrl.steering_gain if convoy_ctrl else None,
        'steering_sign': convoy_ctrl.steering_sign if convoy_ctrl else None,
        'search_speed': convoy_ctrl.search_speed if convoy_ctrl else None,
        'stop_detection_timeout': convoy_ctrl.stop_detection_timeout if convoy_ctrl else None,
        'lost_target_timeout': convoy_ctrl.lost_target_timeout if convoy_ctrl else None,
    })


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _target_to_dict(target):
    if target is None:
        return None

    return {
        'found': target.found,
        'bbox': target.bbox,
        'center_x': target.center_x,
        'center_y': target.center_y,
        'bottom_y': target.bottom_y,
        'area': target.area,
        'score': target.score,
        'class_id': target.class_id,
        'distance_state': target.distance_state,
        'reason': target.reason,
        'dot_count': target.dot_count,
        'marker_bbox': target.marker_bbox,
        'marker_center_x': target.marker_center_x,
        'marker_center_y': target.marker_center_y,
    }


def _command_to_dict(command):
    if command is None:
        return None

    return {
        'should_move': command.should_move,
        'left_speed': command.left_speed,
        'right_speed': command.right_speed,
        'speed_multiplier': command.speed_multiplier,
        'reason': command.reason,
        'state': command.state,
    }


def _red_line_to_dict(red_line_state):
    if red_line_state is None:
        return None

    return {
        'red_line_close': red_line_state.red_line_close,
        'lane_disabled': red_line_state.lane_disabled,
        'disabled_remaining': red_line_state.disabled_remaining,
        'red_ratio': red_line_state.red_ratio,
        'red_row_ratio': red_line_state.red_row_ratio,
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    global camera
    global wheels
    global object_agent
    global lane_agent
    global tracker
    global convoy_ctrl
    global red_line_gate

    import argparse
    ap = argparse.ArgumentParser(description="Real Convoying Server")
    ap.add_argument('--port', type=int, default=5000)
    args = ap.parse_args()

    suppress_http_logs()

    print("=" * 60)
    print("CONVOYING — REAL ROBOT SERVER (marker-bracket state machine)")
    print("=" * 60)

    def _init_wheels():
        global wheels
        wheels = DaguWheelsDriver(WheelPWMConfiguration(), WheelPWMConfiguration())
        print("[Init] Wheels ready")

    def _init_camera():
        global camera
        cam = CameraDriver()
        cam.start()
        camera = cam
        print("[Init] Camera ready")

    def _init_agents():
        global object_agent
        global lane_agent
        global tracker
        global convoy_ctrl
        global red_line_gate

        lane_agent = LaneServoingAgent()
        print(f"[Init] Lane agent ready (base_speed={lane_agent.base_speed})")

        object_agent = ObjectDetectionAgent()
        if object_agent.model_loaded:
            print(f"[Init] Detection model ready ({object_agent.img_size}px)")
        elif getattr(object_agent, 'trt_building', False):
            print("[Init] TensorRT engine building in background — will be ready shortly")
        else:
            print(f"[Init] Detection model: {object_agent.load_error}")

        tracker = TargetTracker()
        convoy_ctrl = ConvoyController()
        red_line_gate = RedLineGate(disable_seconds=5.0)

        print("[Init] Convoy state machine + marker tracker ready")
        print("[Init] Red-line gate ready: lane disabled for 5 seconds after close red line")

    threading.Thread(target=_init_wheels, daemon=True).start()
    threading.Thread(target=_init_camera, daemon=True).start()
    threading.Thread(target=_init_agents, daemon=True).start()
    threading.Thread(target=_detection_loop, daemon=True).start()

    def _shutdown(signum, frame):
        shutdown_cleanup(wheels, camera, stop_event)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    web_port = find_available_port(args.port)
    print(f"\nWeb Interface: http://{socket.gethostname()}.local:{web_port}")
    print("=" * 60 + "\n")

    try:
        app.run(host='0.0.0.0', port=web_port, debug=False, threaded=True)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        shutdown_cleanup(wheels, camera, stop_event)


if __name__ == '__main__':
    sys.exit(main())
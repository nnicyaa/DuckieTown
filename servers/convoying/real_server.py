import sys
import os
import signal
import threading
import time
import queue
import socket

script_dir   = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, '', '..')
sys.path.insert(0, project_root)

import cv2
from dataclasses import replace
from flask import Flask, Response, render_template_string, jsonify, request

from tasks.object_detection.packages.agent import ObjectDetectionAgent
from tasks.visual_lane_servoing.packages.agent import LaneServoingAgent
from tasks.convoying.packages.target_tracker_activity import TargetTracker
from tasks.convoying.packages.convoy_controller_activity import ConvoyController
from tasks.convoying.packages.red_line_stop import RedLineGate
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

running         = False
stop_event      = threading.Event()

_frame_queue    = queue.Queue(maxsize=1)
_detection_lock = threading.Lock()
_last_detections = []
_last_target     = None
_last_command    = None
_last_lane_left  = 0.0
_last_lane_right = 0.0
# FIX: this was read via `global` inside visualize() and status() but never
# initialized here, which raised NameError on first use (e.g. hitting
# /status before any frame had been processed).
_last_red_line_state = None


# Background detection thread
# Runs object detection independently from the video-stream loop so that
# slow inference doesn't drop frames.

def _detection_loop():
    global _last_detections

    while not stop_event.is_set():
        if object_agent is None or not object_agent.model_loaded:
            time.sleep(0.1)
            continue

        try:
            frame_rgb = _frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        result = object_agent.detect(frame_rgb)
        if result is not None:
            with _detection_lock:
                _last_detections = result


# Frame pipeline (called for every MJPEG frame)

def visualize(frame_bgr):
    """
    frame_bgr: BGR frame from the real camera (CameraDriver.read() returns BGR).

    Full pipeline:
        camera frame
        -> object detection (async thread)
        -> target tracking
        -> red-line gate (intersection window timing)
        -> lane servoing (always computed, even during the red-line window)
        -> convoy controller decides steering source:
             normal:                lane steering, leader-distance speed
             red-line + straight:   leader-only steering
             red-line + turning:    lane steering (smooth turn), leader-distance speed
        -> wheel speeds
        -> browser visualization
    """
    global _last_target, _last_command, _last_lane_left, _last_lane_right, _last_red_line_state

    if frame_bgr is None:
        return frame_bgr

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    image_height, image_width = frame_bgr.shape[:2]

    if red_line_gate is not None:
        red_line_state = red_line_gate.update(frame_rgb)
        # FIX: previously `_last_red_line_state = None` was set right here,
        # immediately discarding the value we just computed before it could
        # be used anywhere (status endpoint, visualization panel). The real
        # assignment now happens once, at the bottom, alongside the other
        # _last_* globals.
    else:
        red_line_state = None

    lane_disabled = bool(
        red_line_state and red_line_state.lane_disabled
    )

    # Feed a downscaled copy to the detection thread (non-blocking).
    if object_agent is not None and object_agent.model_loaded:
        try:
            small = cv2.resize(frame_rgb, (object_agent.img_size, object_agent.img_size))
            _frame_queue.put_nowait(small)
        except queue.Full:
            pass

    # Grab latest detections (produced by the detection thread).
    with _detection_lock:
        detections = list(_last_detections)

    # Scale bounding boxes from the model's input resolution back to the
    # full camera frame so the tracker sees pixel-accurate coordinates.
    if object_agent is not None and object_agent.model_loaded and detections:
        sx = image_width  / float(object_agent.img_size)
        sy = image_height / float(object_agent.img_size)
        scaled_detections = [
            ((int(x1*sx), int(y1*sy), int(x2*sx), int(y2*sy)), score, class_id)
            for (x1, y1, x2, y2), score, class_id in detections
        ]
    else:
        scaled_detections = detections

    # Target tracking.
    if tracker is not None:
        target = tracker.update(
            detections=scaled_detections,
            image_height=image_height,
            image_width=image_width,
        )
    else:
        target = _last_target

    # Lane servoing for steering.
    # FIX: previously this was skipped (forced to 0.0, 0.0) whenever
    # lane_disabled was True. That broke the leader-turn fallback in
    # ConvoyController: if the leader turns during the red-line window, the
    # controller now wants to steer using lane_left/lane_right, but they'd
    # have been zeroed out here -- meaning the robot would just drive
    # straight with zero steering instead of turning. Lane servoing now
    # always runs; ConvoyController.decide() is responsible for choosing
    # whether to actually use these values this frame.
    if lane_agent is not None:
        try:
            lane_left, lane_right = lane_agent.compute_commands(frame_rgb)
        except Exception as e:
            print(f"[Convoying] Lane servoing error: {e}")
            lane_left, lane_right = 0.0, 0.0
    else:
        lane_left, lane_right = 0.0, 0.0

    # Convoy controller combines distance state, leader position, and lane
    # speeds into the final command.
    if convoy_ctrl is not None and target is not None:
        command = convoy_ctrl.decide(
            target=target,
            lane_left=lane_left,
            lane_right=lane_right,
            image_width=image_width,
            lane_disabled=lane_disabled,
        )
    else:
        command = _last_command

    # Apply wheel commands only when running.
    if wheels is not None:
        if running and command is not None and command.should_move:
            wheels.set_wheels_speed(command.left_speed, command.right_speed)
        else:
            wheels.set_wheels_speed(0.0, 0.0)

    _last_target          = target
    _last_command         = command
    _last_lane_left        = lane_left
    _last_lane_right       = lane_right
    _last_red_line_state   = red_line_state

    return create_convoying_visualization(
        image_bgr=frame_bgr,
        lane_debug_info=lane_agent.last_debug_info if lane_agent else {},
        target=target,
        command=command,
        lane_left=lane_left,
        lane_right=lane_right,
        detections=scaled_detections,
        red_line_state=red_line_state,
    )


# Real camera returns BGR frames — rgb=False.
generate_frames = make_frame_generator(lambda: camera, visualize, quality=50, rgb=False)

def _red_line_to_dict(red_line_state):
    if red_line_state is None:
        return None

    return {
        'red_line_close': red_line_state.red_line_close,
        'lane_disabled': red_line_state.lane_disabled,
        'disabled_remaining': red_line_state.disabled_remaining,
        'mode': red_line_state.mode,
        'red_ratio': red_line_state.red_ratio,
        'red_row_ratio': red_line_state.red_row_ratio,
    }

# Flask routes

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
    running = False
    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)
    if tracker is not None:
        tracker.reset()
    print("[Convoying] Reset")
    return jsonify({'status': 'reset'})


@app.route('/running')
def get_running():
    return jsonify({'running': running})


@app.route('/status')
def status():
    return jsonify({
        'running':           running,
        'model_loaded':      bool(getattr(object_agent, 'model_loaded', False)) if object_agent else False,
        'model_load_error':  getattr(object_agent, 'load_error', None) if object_agent else None,
        'lane_frame_count':  getattr(lane_agent, 'frame_count', 0) if lane_agent else 0,
        'target':            _target_to_dict(_last_target),
        'command':           _command_to_dict(_last_command),
        'lane_left':         _last_lane_left,
        'lane_right':        _last_lane_right,
        'red_line': _red_line_to_dict(_last_red_line_state),
    })


@app.route('/update_config', methods=['POST'])
def update_config():
    """Tune convoying controller multipliers at runtime."""
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
    return jsonify({
        'close_multiplier': convoy_ctrl.close_multiplier if convoy_ctrl else None,
        'good_multiplier':  convoy_ctrl.good_multiplier  if convoy_ctrl else None,
        'far_multiplier':   convoy_ctrl.far_multiplier   if convoy_ctrl else None,
        'max_speed':        convoy_ctrl.max_speed        if convoy_ctrl else None,
    })

# Helpers
def _target_to_dict(target):
    if target is None:
        return None
    return {
        'found':          target.found,
        'bbox':           target.bbox,
        'center_x':       target.center_x,
        'center_y':       target.center_y,
        'bottom_y':       target.bottom_y,
        'area':           target.area,
        'score':          target.score,
        'class_id':       target.class_id,
        'distance_state': target.distance_state,
        'reason':         target.reason,
    }


def _command_to_dict(command):
    if command is None:
        return None
    return {
        'should_move':      command.should_move,
        'left_speed':       command.left_speed,
        'right_speed':      command.right_speed,
        'speed_multiplier': command.speed_multiplier,
        'reason':           command.reason,
    }

# Entry point
def main():
    global camera, wheels, object_agent, lane_agent, tracker, convoy_ctrl

    import argparse
    ap = argparse.ArgumentParser(description="Real Convoying Server")
    ap.add_argument('--port', type=int, default=5000)
    args = ap.parse_args()

    suppress_http_logs()

    print("=" * 60)
    print("CONVOYING — REAL ROBOT SERVER")
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
        global object_agent, lane_agent, tracker, convoy_ctrl, red_line_gate
        lane_agent  = LaneServoingAgent()
        print(f"[Init] Lane agent ready (base_speed={lane_agent.base_speed})")
        object_agent = ObjectDetectionAgent()
        if object_agent.model_loaded:
            print(f"[Init] Detection model ready ({object_agent.img_size}px)")
        else:
            print(f"[Init] Detection model: {object_agent.load_error}")
        tracker     = TargetTracker()
        convoy_ctrl = ConvoyController()
        red_line_gate = RedLineGate(disable_seconds=7.0)
        print("[Init] Convoy tracker and controller ready")

    threading.Thread(target=_init_wheels,    daemon=True).start()
    threading.Thread(target=_init_camera,    daemon=True).start()
    threading.Thread(target=_init_agents,    daemon=True).start()
    threading.Thread(target=_detection_loop, daemon=True).start()

    def _shutdown(signum, frame):
        shutdown_cleanup(wheels, camera, stop_event)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT,  _shutdown)

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
import json
import os
import threading
import time
from urllib import error as url_error
from urllib import request as url_request

import cv2
from flask import Flask, Response, request, jsonify
try:
    from flask_cors import CORS
except ImportError:
    def CORS(app):
        return app
from PCA9685 import PCA9685
from Detection.line_follower import LineFollower

app = Flask(__name__)
CORS(app)

HOST_IP = '0.0.0.0'
PORT = 8080
API_TELEMETRY_URL = os.environ.get('API_TELEMETRY_URL', '').strip()
MODE_AUTONOMOUS = 'autonomous'
MODE_MANUAL = 'manual'
MODE_STOPPED = 'stopped'
VALID_MODES = {MODE_AUTONOMOUS, MODE_MANUAL, MODE_STOPPED}
MISSION_MESSAGE_LIFE_FOUND = 'WE ARE NOT ALONE'
AUTONOMOUS_LOOP_DELAY = 0.08
current_mode = MODE_STOPPED
manual_command = None
auto_command = None
chosen_command = 'stop'
mode_version = 0
life_detected = False
mission_message = ''
last_telemetry = {}
state_lock = threading.Lock()

# === МОТОРЫ ===
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)


def motors(left, right):
    pwm.setDutycycle(0, int(abs(left) * 100))
    pwm.setLevel(1, 0 if left >= 0 else 1)
    pwm.setLevel(2, 1 if left >= 0 else 0)
    pwm.setDutycycle(5, int(abs(right) * 100))
    pwm.setLevel(3, 1 if right >= 0 else 0)
    pwm.setLevel(4, 0 if right >= 0 else 1)


def motors_stop():
    pwm.setDutycycle(0, 0)
    pwm.setDutycycle(5, 0)


def set_mode(mode):
    global current_mode, mode_version, chosen_command
    with state_lock:
        current_mode = mode
        mode_version += 1
        if mode == MODE_STOPPED:
            chosen_command = 'stop'


def status_snapshot():
    with state_lock:
        telemetry = dict(last_telemetry)
        telemetry.update({
            'mode': current_mode,
            'command': chosen_command,
            'manual_command': manual_command,
            'auto_command': auto_command,
            'chosen_command': chosen_command,
            'life_detected': life_detected,
            'mission_message': mission_message,
        })
        telemetry.setdefault('path_detected', False)
        telemetry.setdefault('path_center_x', None)
        telemetry.setdefault('steering_error', None)
        telemetry.setdefault('hazard_detected', False)
        telemetry.setdefault('autonomous_state', telemetry.get('state'))
        telemetry.setdefault('mission_state', telemetry.get('state', current_mode))
        return telemetry


def log_rover_status(mode, manual, auto, telemetry=None, action=None):
    telemetry = telemetry or {}
    state = telemetry.get('autonomous_state') or telemetry.get('state')
    confidence = telemetry.get('confidence')
    error = telemetry.get('steering_error')
    chosen_action = action if action is not None else telemetry.get('action') or chosen_command
    life = telemetry.get('life_detected', life_detected)
    confidence_text = 'none' if confidence is None else f'{float(confidence):.2f}'
    print(
        f"mode={mode} manual={manual} auto={auto} state={state} "
        f"confidence={confidence_text} error={error} action={chosen_action} life={life}",
        flush=True,
    )


def post_telemetry_if_configured(telemetry):
    if not API_TELEMETRY_URL:
        return
    try:
        body = json.dumps(telemetry).encode('utf-8')
        req = url_request.Request(
            API_TELEMETRY_URL,
            data=body,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with url_request.urlopen(req, timeout=0.25):
            pass
    except (OSError, url_error.URLError, TimeoutError):
        pass


# === WATCHDOG: если хост умер — стоп ===
last_command_time = time.time()


def watchdog():
    while True:
        if time.time() - last_command_time > 1.0:
            motors_stop()
        time.sleep(0.1)


threading.Thread(target=watchdog, daemon=True).start()

# === КАМЕРА ===
last_frame = None
last_processed_frame = None
lock = threading.Lock()


def camera_loop():
    global last_frame
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        ret, frame = cap.read()
        if ret:
            with lock:
                last_frame = frame
        else:
            time.sleep(0.05)


threading.Thread(target=camera_loop, daemon=True).start()


def generate_frames(processed=False):
    while True:
        with lock:
            if last_frame is None:
                time.sleep(0.05)
                continue
            if processed and last_processed_frame is not None:
                frame = last_processed_frame.copy()
            else:
                frame = last_frame.copy()
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
               + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


def autonomous_loop():
    global last_command_time, last_processed_frame, last_telemetry
    global auto_command, chosen_command, life_detected, mission_message

    follower = None
    while True:
        with state_lock:
            mode = current_mode
            version = mode_version
            life = life_detected or mission_message == MISSION_MESSAGE_LIFE_FOUND

        if life:
            motors_stop()
            time.sleep(AUTONOMOUS_LOOP_DELAY)
            continue

        if mode != MODE_AUTONOMOUS:
            time.sleep(AUTONOMOUS_LOOP_DELAY)
            continue

        with lock:
            frame = None if last_frame is None else last_frame.copy()
        if frame is None:
            time.sleep(0.03)
            continue

        if follower is None:
            follower = LineFollower(frame_width=frame.shape[1])

        left, right, telemetry = follower.update(frame)
        telemetry['mode'] = MODE_AUTONOMOUS
        telemetry['manual_command'] = manual_command
        telemetry['auto_command'] = telemetry.get('action')
        telemetry['chosen_command'] = telemetry.get('action')
        telemetry['autonomous_state'] = telemetry.get('state')
        telemetry['mission_state'] = telemetry.get('state')

        if telemetry.get('life_detected') is True or telemetry.get('mission_message') == MISSION_MESSAGE_LIFE_FOUND:
            with state_lock:
                life_detected = True
                mission_message = MISSION_MESSAGE_LIFE_FOUND
                chosen_command = 'life_found_stop'
                last_telemetry = telemetry
            motors_stop()
            post_telemetry_if_configured(status_snapshot())
            log_rover_status(MODE_STOPPED, manual_command, telemetry.get('action'), telemetry, 'life_found_stop')
            time.sleep(AUTONOMOUS_LOOP_DELAY)
            continue

        with state_lock:
            still_autonomous = current_mode == MODE_AUTONOMOUS and mode_version == version
            if still_autonomous:
                auto_command = telemetry.get('action')
                chosen_command = auto_command
                last_telemetry = telemetry

        if still_autonomous:
            motors(left, right)
            last_command_time = time.time()
            overlay = follower.draw_overlay(frame, telemetry)
            with lock:
                last_processed_frame = overlay
            post_telemetry_if_configured(status_snapshot())
            log_rover_status(MODE_AUTONOMOUS, manual_command, auto_command, telemetry)
        else:
            log_rover_status(current_mode, manual_command, telemetry.get('action'), telemetry)

        time.sleep(AUTONOMOUS_LOOP_DELAY)


threading.Thread(target=autonomous_loop, daemon=True).start()


# === ROUTES ===
@app.post('/move_pwm')
def move_pwm():
    global last_command_time, manual_command, auto_command, chosen_command, last_telemetry
    data = request.get_json(silent=True) or {}
    source = data.get('source', MODE_MANUAL)
    command = data.get('command') or 'manual_pwm'
    left = float(data.get('left', 0))
    right = float(data.get('right', 0))

    telemetry = data.get('telemetry')
    if isinstance(telemetry, dict):
        last_telemetry = telemetry

    with state_lock:
        life = life_detected or mission_message == MISSION_MESSAGE_LIFE_FOUND
    if life:
        motors_stop()
        return jsonify({
            'accepted': False,
            'mode': MODE_STOPPED,
            'reason': 'life_detected',
        }), 409

    with state_lock:
        mode = current_mode

    if source != MODE_AUTONOMOUS:
        set_mode(MODE_MANUAL)
        manual_command = command
        chosen_command = command
        auto_command = None
    elif mode != MODE_AUTONOMOUS:
        return jsonify({
            'accepted': False,
            'mode': mode,
            'reason': 'autonomous_disabled',
        }), 409
    else:
        auto_command = command
        chosen_command = command

    with state_lock:
        mode = current_mode
    if mode == MODE_STOPPED:
        motors_stop()
        return jsonify({
            'accepted': False,
            'mode': mode,
            'reason': 'stopped',
        }), 409

    last_command_time = time.time()
    motors(left, right)
    log_rover_status(mode, manual_command, auto_command, last_telemetry, chosen_command)
    return jsonify({
        'accepted': True,
        'mode': mode,
        'left': left,
        'right': right,
    })


@app.post('/move/stop')
def move_stop():
    global last_command_time, manual_command, auto_command, chosen_command
    data = request.get_json(silent=True) or {}
    source = data.get('source', MODE_MANUAL)
    with state_lock:
        mode = current_mode
    if source == MODE_AUTONOMOUS and mode != MODE_AUTONOMOUS:
        return jsonify({
            'accepted': False,
            'mode': mode,
            'reason': 'autonomous_disabled',
        }), 409
    next_mode = MODE_MANUAL if source == MODE_MANUAL else MODE_STOPPED
    set_mode(next_mode)
    manual_command = 'stop' if next_mode == MODE_MANUAL else None
    auto_command = None
    chosen_command = 'stop'
    last_command_time = time.time()
    motors_stop()
    log_rover_status(next_mode, manual_command, auto_command, last_telemetry, chosen_command)
    return jsonify({'accepted': True, 'mode': next_mode, 'command': 'stop'})


@app.route('/mode', methods=['GET', 'POST', 'PUT'])
def mode_route():
    global last_command_time, manual_command, auto_command, chosen_command
    if request.method == 'GET':
        return jsonify({'mode': status_snapshot()['mode']})

    data = request.get_json(silent=True) or {}
    mode = data.get('mode')
    if mode not in VALID_MODES:
        return jsonify({
            'accepted': False,
            'valid_modes': sorted(VALID_MODES),
        }), 400

    set_mode(mode)
    last_command_time = time.time()
    if mode == MODE_AUTONOMOUS:
        manual_command = None
        auto_command = 'autonomous'
        chosen_command = 'autonomous'
    else:
        manual_command = 'stop' if mode == MODE_MANUAL else None
        auto_command = None
        chosen_command = 'stop'
        motors_stop()
    log_rover_status(mode, manual_command, auto_command, last_telemetry, chosen_command)
    return jsonify({'accepted': True, 'mode': mode})


@app.get('/telemetry')
def telemetry_route():
    return jsonify(status_snapshot())


@app.delete('/mission')
def reset_mission():
    global life_detected, mission_message
    with state_lock:
        life_detected = False
        mission_message = ''
    return jsonify(status_snapshot())


@app.put('/mission')
def set_mission():
    global life_detected, mission_message, manual_command, auto_command, chosen_command
    data = request.get_json(silent=True) or {}
    life_detected = bool(data.get('life_detected'))
    mission_message = data.get('mission_message') or ''
    if life_detected:
        mission_message = MISSION_MESSAGE_LIFE_FOUND
        set_mode(MODE_STOPPED)
        manual_command = 'stop'
        auto_command = None
        chosen_command = 'life_found_stop'
        motors_stop()
    return jsonify(status_snapshot())


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(processed=True),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed_raw')
def video_feed_raw():
    return Response(generate_frames(processed=False),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/frame')
def frame():
    with lock:
        frame_copy = None if last_frame is None else last_frame.copy()
    if frame_copy is None:
        return jsonify({'error': 'no_frame'}), 503
    ret, buffer = cv2.imencode('.jpg', frame_copy)
    if not ret:
        return jsonify({'error': 'encode_failed'}), 500
    return Response(buffer.tobytes(), mimetype='image/jpeg')


@app.route('/ping')
def ping():
    return jsonify({'ok': True, 'mode': status_snapshot()['mode']})


if __name__ == '__main__':
    try:
        app.run(host=HOST_IP, port=PORT, threaded=True)
    finally:
        motors_stop()

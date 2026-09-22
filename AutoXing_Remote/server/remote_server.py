"""AutoXing Remote Controller

ESP32 -> HTTP POST /button -> this server -> AutoXing Task API -> Robot
"""
import threading
from flask import Flask, jsonify, request

from config import BUTTON_POI, ROBOT_ID, SERVER_HOST, SERVER_PORT, config
from autoxing_sdk.token import TokenManager
from autoxing_sdk.robot import RobotManager
from autoxing_sdk.map_info import MapInfoManager
from autoxing_sdk.task import TaskManager, TaskBuilder, TaskPoint

app = Flask(__name__)

_token_manager = TokenManager()
_lock = threading.Lock()
_pois_by_name = {}
_last_task_id = None
_last_button = None


def get_token():
    ok, token = _token_manager.getToken()
    if not ok or not token:
        raise RuntimeError("Unable to obtain AutoXing token. Check config.py credentials.")
    return token


def load_pois():
    """Load POIs for the configured robot and index them by exact name."""
    token = get_token()
    manager = MapInfoManager(token)
    ok, pois = manager.getPoiList(None, ROBOT_ID, None)
    if not ok or pois is None:
        raise RuntimeError("Unable to get POI list from AutoXing.")

    _pois_by_name.clear()
    for poi in pois:
        name = poi.get("name")
        if name:
            _pois_by_name[name] = poi
    return pois


def refresh_if_needed():
    if not _pois_by_name:
        load_pois()


def resolve_poi(button_number):
    refresh_if_needed()
    poi_name = BUTTON_POI.get(button_number)
    if not poi_name:
        raise ValueError(f"Button {button_number} is not configured.")
    poi = _pois_by_name.get(poi_name)
    if poi is None:
        available = ", ".join(sorted(_pois_by_name.keys()))
        raise ValueError(
            f'POI "{poi_name}" was not found for robot {ROBOT_ID}. '
            f"Available POIs: {available}"
        )
    return poi


def go_to_poi(button_number):
    """Create and execute one AutoXing task for the selected button."""
    global _last_task_id, _last_button

    with _lock:
        token = get_token()
        poi = resolve_poi(button_number)

        task_manager = TaskManager(token)
        task = TaskBuilder(f"Remote Button {button_number}", ROBOT_ID)
        task.addTaskPt(TaskPoint(poi))

        ok, task_id = task_manager.newTask(task.getTask())
        if not ok or not task_id:
            raise RuntimeError("AutoXing task creation failed.")

        if not task_manager.executeTask(task_id):
            raise RuntimeError(f"Task {task_id} was created but could not be executed.")

        _last_task_id = task_id
        _last_button = button_number
        return task_id, poi.get("name", "")


@app.get("/health")
def health():
    return jsonify({"ok": True, "service": "autoxing-remote"})


@app.get("/pois")
def pois():
    try:
        loaded = load_pois()
        return jsonify({
            "ok": True,
            "robotId": ROBOT_ID,
            "count": len(loaded),
            "pois": loaded,
            "buttonMap": BUTTON_POI,
        })
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.get("/status")
def status():
    try:
        token = get_token()
        ok, state = RobotManager(token).getRobotState(ROBOT_ID)
        return jsonify({
            "ok": ok,
            "robotId": ROBOT_ID,
            "robotState": state,
            "lastButton": _last_button,
            "lastTaskId": _last_task_id,
        }), (200 if ok else 502)
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.post("/reload-pois")
def reload_pois():
    try:
        loaded = load_pois()
        return jsonify({"ok": True, "count": len(loaded)})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.post("/button")
def button():
    data = request.get_json(silent=True) or {}
    raw_button = data.get("button")

    try:
        button_number = int(raw_button)
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "button must be 1, 2, or 3"}), 400

    if button_number not in BUTTON_POI:
        return jsonify({"ok": False, "error": "Unsupported button"}), 400

    try:
        task_id, poi_name = go_to_poi(button_number)
        return jsonify({
            "ok": True,
            "button": button_number,
            "poi": poi_name,
            "taskId": task_id,
        })
    except Exception as exc:
        return jsonify({"ok": False, "button": button_number, "error": str(exc)}), 502


if __name__ == "__main__":
    print(f"AutoXing Remote Server: http://{SERVER_HOST}:{SERVER_PORT}")
    print(f"Robot ID: {ROBOT_ID}")
    print(f"Button map: {BUTTON_POI}")
    app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False)

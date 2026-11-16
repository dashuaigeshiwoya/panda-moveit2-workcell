
# ECE9013 - Panda Mission Control
# AI disclosure: OpenAI ChatGPT assisted in writing
# and explaining this program.

import json
from datetime import datetime
from pathlib import Path
from threading import Event, Lock, Thread

from flask import (
    Flask,
    redirect,
    render_template_string,
    request,
    url_for,
)

from pymoveit2 import connect
from mission_v3 import MISSIONS, execute_step

app = Flask(__name__)

status_lock = Lock()
stop_requested = Event()

LOG_FILE = Path(__file__).with_name("mission_history.jsonl")

status = {
    "running": False,
    "cancel_requested": False,
    "mission": "None",
    "step": 0,
    "total": 0,
    "result": "Idle",
    "message": "Choose a mission."
}

PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Panda Mission Control</title>

    {% if view.running %}
    <meta http-equiv="refresh" content="2">
    {% endif %}
</head>

<body>
    <h1>Panda Mission Control</h1>

    <form action="{{ url_for('start_mission') }}"
          method="post">

        <label>Select Mission:</label>

        <select name="mission"
                {% if view.running %}disabled{% endif %}>
            {% for name in missions %}
            <option value="{{ name }}">{{ name }}</option>
            {% endfor %}
        </select>

        <button type="submit"
                {% if view.running %}disabled{% endif %}>
            Start Mission
        </button>
    </form>

    <form action="{{ url_for('request_stop') }}"
          method="post">

        <button type="submit"
                {% if not view.running or view.cancel_requested %}
                disabled
                {% endif %}>
            Stop After Current Step
        </button>
    </form>

    <h2>Mission Status</h2>

    <p>Mission: {{ view.mission }}</p>
    <p>Result: {{ view.result }}</p>
    <p>Step: {{ view.step }} / {{ view.total }}</p>
    <p>Message: {{ view.message }}</p>

</body>
</html>
"""

def update_status(changes):
    with status_lock:
        status.update(changes)

def write_log(mission, event, step=0, detail=""):
    record = {
        "time": datetime.now().astimezone().isoformat(
            timespec="seconds"
        ),
        "mission": mission,
        "event": event,
        "step": step,
        "detail": detail
    }

    with LOG_FILE.open("a", encoding="utf-8") as file:
        file.write(json.dumps(record) + "\n")

def run_mission(name):
    result = "Succeeded"
    message = "All steps completed."
    last_completed_step = 0

    try:
        write_log(name, "started")

        with connect() as robot:
            steps = MISSIONS[name]

            for index, step in enumerate(steps, start=1):

                if stop_requested.is_set():
                    result = "Stopped"
                    message = "Stopped before the next step."
                    break

                update_status({
                    "step": index,
                    "message": f"Executing: {step['type']}"
                })

                execute_step(robot, step)

                last_completed_step = index

                write_log(
                    name,
                    "step_completed",
                    index,
                    step["type"]
                )

                if stop_requested.is_set():
                    result = "Stopped"
                    message = "Stopped after the current step."
                    break

    except Exception as error:
        result = "Failed"
        message = f"{type(error).__name__}: {error}"

    finally:
        try:
            write_log(
                name,
                "finished",
                last_completed_step,
                result
            )
        except OSError as error:
            result = "Failed"
            message = f"Could not save log: {error}"

        update_status({
            "running": False,
            "cancel_requested": False,
            "result": result,
            "message": message
        })

@app.get("/")
def index():
    with status_lock:
        view = status.copy()

    return render_template_string(
        PAGE,
        view=view,
        missions=MISSIONS.keys()
    )

@app.post("/start")
def start_mission():
    name = request.form.get("mission", "")

    if name not in MISSIONS:
        return "Unknown mission", 400

    with status_lock:
        if status["running"]:
            return redirect(url_for("index"))

        stop_requested.clear()

        status.update({
            "running": True,
            "cancel_requested": False,
            "mission": name,
            "step": 0,
            "total": len(MISSIONS[name]),
            "result": "Running",
            "message": "Connecting to MoveIt..."
        })

    worker = Thread(
        target=run_mission,
        args=(name,),
        daemon=True
    )

    worker.start()

    return redirect(url_for("index"))

@app.post("/stop")
def request_stop():
    with status_lock:
        if status["running"]:
            stop_requested.set()

            status.update({
                "cancel_requested": True,
                "message": "Stop requested. Waiting for current step."
            })

    return redirect(url_for("index"))

if __name__ == "__main__":
    app.run(
        host="127.0.0.1",
        port=5000,
        debug=False,
        use_reloader=False
    )
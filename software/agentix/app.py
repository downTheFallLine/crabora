#!/usr/bin/env python3
"""
agentix -- Flask API for the CRABORA chat UI.

Serves /api for the web front end. nginx is expected to serve the static
files in web/ in production; this also serves them directly so the thing
runs standalone during development.

Endpoints:

    GET  /api/health        is the server up, is the bus open
    GET  /api/status        robot health summary (cheap, for polling)
    GET  /api/tools         what the agent can call, and what each does
    POST /api/tools/<name>  call one tool directly, bypassing the model
    POST /api/chat          natural language in, answer out
    POST /api/stop          EMERGENCY STOP -- drops torque on everything

/api/stop does NOT go through the agent. It is a direct call to the
arbiter, so it works when the model is slow, confused, unreachable, or
out of credit. That is the whole point of it being a button.

PHASE 1 IS READ ONLY. No tool here moves a servo. See tools.py.

Usage:
    python3 agentix/app.py
    python3 agentix/app.py --port 8080 --host 0.0.0.0
    python3 agentix/app.py --no-bus          # UI development, no robot

Requires: pip install flask
"""

import argparse
import os
import sys
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

try:
    from flask import Flask, jsonify, request, send_from_directory
except ImportError:
    print("flask is not installed:  python3 -m pip install flask")
    sys.exit(1)

from arbiter import Arbiter, ArbiterError, Busy      # noqa: E402
from tools import READ_ONLY_TOOLS, TOOL_HELP         # noqa: E402

WEB_DIR = os.path.join(_HERE, "web")

app = Flask(__name__, static_folder=None)
ARB = None
AGENT = None


def _fail(message, code=500, **extra):
    return jsonify({"ok": False, "error": message, **extra}), code


# =============================================================
# STATIC (nginx does this in production)
# =============================================================
@app.get("/")
def index():
    return send_from_directory(WEB_DIR, "index.html")


@app.get("/<path:name>")
def static_file(name):
    return send_from_directory(WEB_DIR, name)


# =============================================================
# API
# =============================================================
@app.get("/api/health")
def health():
    return jsonify({
        "ok": True,
        "bus": {"available": ARB.available, "detail": ARB.reason},
        "agent": AGENT.describe() if AGENT else
                 {"enabled": False, "reason": "no model configured"},
        "phase": "read-only",
    })


@app.get("/api/status")
def status():
    if not ARB.available:
        return _fail(f"bus unavailable: {ARB.reason}", 503)
    try:
        return jsonify({"ok": True, "result": READ_ONLY_TOOLS["robot_status"](ARB)})
    except Busy as e:
        return _fail(str(e), 409)
    except ArbiterError as e:
        return _fail(str(e), 503)


@app.get("/api/tools")
def list_tools():
    return jsonify({"ok": True, "phase": "read-only",
                    "tools": [{"name": n, "description": TOOL_HELP.get(n, "")}
                              for n in sorted(READ_ONLY_TOOLS)]})


@app.post("/api/tools/<name>")
def call_tool(name):
    """Call a tool directly. Useful for the control strip and for
    debugging without burning model tokens."""
    fn = READ_ONLY_TOOLS.get(name)
    if fn is None:
        return _fail(f"no such tool: {name}", 404,
                     available=sorted(READ_ONLY_TOOLS))
    if not ARB.available and name != "imu_state":
        return _fail(f"bus unavailable: {ARB.reason}", 503)
    kwargs = request.get_json(silent=True) or {}
    try:
        return jsonify({"ok": True, "tool": name, "result": fn(ARB, **kwargs)})
    except TypeError as e:
        return _fail(f"bad arguments for {name}: {e}", 400)
    except Busy as e:
        return _fail(str(e), 409)
    except ArbiterError as e:
        return _fail(str(e), 503)
    except Exception as e:                            # noqa: BLE001
        traceback.print_exc()
        return _fail(f"{type(e).__name__}: {e}")


@app.post("/api/chat")
def chat():
    data = request.get_json(silent=True) or {}
    prompt = (data.get("prompt") or "").strip()
    if not prompt:
        return _fail("empty prompt", 400)
    if AGENT is None:
        return jsonify({
            "ok": True,
            "reply": ("No model is configured, so I can't answer in prose "
                      "yet — but the tools work. Try the buttons, or POST "
                      "to /api/tools/<name>."),
            "tools_used": [],
            "degraded": True,
        })
    try:
        return jsonify({"ok": True, **AGENT.ask(prompt)})
    except Busy as e:
        return _fail(str(e), 409)
    except Exception as e:                            # noqa: BLE001
        traceback.print_exc()
        return _fail(f"{type(e).__name__}: {e}")


@app.post("/api/stop")
def stop():
    """EMERGENCY STOP. Deliberately not routed through the agent: it has
    to work when the model is slow, confused or unreachable."""
    if not ARB.available:
        return _fail(f"bus unavailable: {ARB.reason} -- CUT THE SERVO POWER", 503)
    try:
        r = ARB.emergency_stop()
    except ArbiterError as e:
        return _fail(f"{e} -- CUT THE SERVO POWER", 503)
    ok = not r["failed"]
    return jsonify({
        "ok": ok,
        "result": r,
        "message": (f"All {len(r['released'])} servos released."
                    if ok and not r["missing"] else
                    f"Released {len(r['released'])}; "
                    f"{len(r['failed'])} still holding, "
                    f"{len(r['missing'])} not answering. "
                    f"CUT THE SERVO POWER if anything is still driven."),
    }), (200 if ok else 500)


# =============================================================
# MAIN
# =============================================================
def main():
    global ARB, AGENT
    ap = argparse.ArgumentParser(description="CRABORA agentix API")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--ports", nargs="+", help="serial ports (default: auto)")
    ap.add_argument("--no-bus", action="store_true",
                    help="do not open the servo bus (UI work with no robot)")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    ARB = Arbiter(ports=args.ports)
    if args.no_bus:
        ARB.available = False
        ARB.reason = "started with --no-bus"
    else:
        ARB.start()
    print(f"  servo bus: {'open' if ARB.available else 'UNAVAILABLE'} "
          f"-- {ARB.reason}")

    try:
        from agent import build_agent
        AGENT = build_agent(ARB)
        print(f"  agent: {AGENT.describe()['model']}")
    except Exception as e:                            # noqa: BLE001
        AGENT = None
        print(f"  agent: disabled ({type(e).__name__}: {e})")
        print("         the UI and tools still work; chat replies will say so")

    print(f"\n  http://{args.host}:{args.port}/\n")
    try:
        app.run(host=args.host, port=args.port, debug=args.debug,
                threaded=True)
    finally:
        ARB.stop()


if __name__ == "__main__":
    main()

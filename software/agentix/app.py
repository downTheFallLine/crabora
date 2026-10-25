#!/usr/bin/env python3
"""
agentix -- Flask API for the CRABORA chat UI.

Serves /api for the web front end. nginx is expected to serve the static
files in web/ in production; this also serves them directly so the thing
runs standalone during development.

Endpoints:

    GET  /api/health        is the server up, is the bus open
    GET  /api/status        robot health summary (cheap, for polling)
    GET  /api/imu           pitch and roll, for the tilt widget (polled)
    GET  /api/tools         what the agent can call, and what each does
    POST /api/tools/<name>  call one tool directly, bypassing the model
    POST /api/chat          natural language in, answer out
    POST /api/stop          EMERGENCY STOP -- drops torque on everything

/api/stop does NOT go through the agent. It is a direct call to the
arbiter, so it works when the model is slow, confused, unreachable, or
out of credit. That is the whole point of it being a button.

Motion is gated by arming: POST /api/arm opens a time window, and
nothing moves outside one. The agent cannot call /api/arm -- that route
exists for the human on the control strip. See safety.py and motion.py.

Usage:
    python3 agentix/app.py
    python3 agentix/app.py --port 8080 --host 0.0.0.0
    python3 agentix/app.py --no-bus          # UI development, no robot

Requires: pip install flask
"""

import argparse
import logging
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

from arbiter import Arbiter, ArbiterError, Busy, Cancelled  # noqa: E402
from motion import MOTION_HELP, MOTION_TOOLS, Unsafe   # noqa: E402
from safety import NotArmed, Safety                    # noqa: E402
from tools import READ_ONLY_TOOLS, TOOL_HELP         # noqa: E402

WEB_DIR = os.path.join(_HERE, "web")

app = Flask(__name__, static_folder=None)
ARB = None
AGENT = None
# The arming gate. Lives here, beside the HTTP routes, because arming is
# a human action over a route the tool layer cannot call.
SAFE = Safety()
# Why the agent is off, in the words of whatever actually failed. Without
# this the reason reached only the Pi's console at startup, so a browser
# on another machine could not tell a missing API key from a missing
# dependency -- both just showed "no agent".
AGENT_REASON = "not started"


class _QuietPolls(logging.Filter):
    """Drop the access-log line for /api/imu. The tilt widget polls it
    every second; logged, that is 86,400 journal lines a day burying the
    ones worth reading."""
    def filter(self, record):
        return "/api/imu " not in record.getMessage()


logging.getLogger("werkzeug").addFilter(_QuietPolls())


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
                 {"enabled": False, "reason": AGENT_REASON},
        "safety": SAFE.state(),
        "phase": "motion (armed)" if SAFE.state()["armed"] else "read-only",
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


@app.get("/api/imu")
def imu():
    # Not under ARB: the IMU is on I2C, not the servo bus, so tilt still
    # reads with the bus down or a motion holding it.
    return jsonify({"ok": True, "result": READ_ONLY_TOOLS["imu_state"]()})


@app.get("/api/tools")
def list_tools():
    return jsonify({
        "ok": True,
        "safety": SAFE.state(),
        "tools": [{"name": n, "description": TOOL_HELP.get(n, ""),
                   "moves": False} for n in sorted(READ_ONLY_TOOLS)] +
                 [{"name": n, "description": MOTION_HELP.get(n, ""),
                   "moves": True} for n in sorted(MOTION_TOOLS)],
    })


# =============================================================
# ARMING -- human only
# =============================================================
# These routes are the whole reason the agent cannot move the robot by
# itself. Nothing in tools.py, motion.py or agent.py can reach them: the
# model can ask to move, and a person decides whether that is allowed.
@app.post("/api/arm")
def arm():
    if not ARB.available:
        return _fail(f"bus unavailable: {ARB.reason}", 503)
    body = request.get_json(silent=True) or {}
    state = SAFE.arm(who=str(body.get("who", "operator"))[:40])
    ARB.clear_stop()
    return jsonify({"ok": True, "safety": state})


@app.post("/api/disarm")
def disarm():
    return jsonify({"ok": True,
                    "safety": SAFE.disarm("disarmed from the control strip")})


@app.post("/api/motion/<name>")
def call_motion(name):
    """Run a motion directly, for the control strip buttons.

    Same gates as the agent path -- the buttons are a shortcut for the
    human, not a way round the arming window.
    """
    fn = MOTION_TOOLS.get(name)
    if fn is None:
        return _fail(f"no such motion: {name}", 404,
                     available=sorted(MOTION_TOOLS))
    if not ARB.available:
        return _fail(f"bus unavailable: {ARB.reason}", 503)
    kwargs = request.get_json(silent=True) or {}
    try:
        return jsonify({"ok": True, "motion": name,
                        "result": fn(ARB, SAFE, **kwargs),
                        "safety": SAFE.state()})
    except NotArmed as e:
        return _fail(str(e), 403, safety=SAFE.state())
    except Unsafe as e:
        return _fail(str(e), 409)
    except Cancelled as e:
        return _fail(str(e), 409)
    except TypeError as e:
        return _fail(f"bad arguments for {name}: {e}", 400)
    except Busy as e:
        return _fail(str(e), 409)
    except ArbiterError as e:
        return _fail(str(e), 503)
    except Exception as e:                            # noqa: BLE001
        traceback.print_exc()
        return _fail(f"{type(e).__name__}: {e}")


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
    to work when the model is slow, confused or unreachable.

    Disarms as well as releasing torque. Someone who hits STOP wants the
    robot to stop, not to stop and then be free to move again the moment
    the next sentence is typed. Re-arming is a deliberate act."""
    SAFE.disarm("emergency stop")
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
def _lan_ip():
    """This machine's address on the LAN, or None.

    Opens a UDP socket toward an off-subnet address and asks the kernel
    which local interface it would use. Nothing is sent -- UDP connect()
    only sets the default peer -- so this needs no network and no DNS.
    Beats gethostname() lookups, which on a stock Pi answer 127.0.1.1.
    """
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 1))      # TEST-NET-1, guaranteed unrouted
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def _browsable_urls(host, port):
    """URLs you can actually open, given a bind address you may not.

    0.0.0.0 means "every interface" and :: means the same for IPv6.
    Neither is a destination: a browser pointed at one reports the
    address as unreachable while the server is sitting there working.
    """
    if host not in ("0.0.0.0", "::", ""):
        return [f"http://{host}:{port}/"]
    urls = [f"http://127.0.0.1:{port}/   (on this machine)"]
    ip = _lan_ip()
    if ip:
        urls.insert(0, f"http://{ip}:{port}/   (from another machine)")
    return urls


def main():
    global ARB, AGENT, AGENT_REASON
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
        AGENT = build_agent(ARB, SAFE)
        print(f"  agent: {AGENT.describe()['model']}")
    except Exception as e:                            # noqa: BLE001
        AGENT = None
        AGENT_REASON = f"{type(e).__name__}: {e}"
        print(f"  agent: disabled ({AGENT_REASON})")
        print("         the UI and tools still work; chat replies will say so")

    # NOT args.host -- that defaults to 0.0.0.0, which is a bind address
    # ("every interface"), not somewhere a browser can go. Printing it
    # sends you to ERR_ADDRESS_INVALID while the server is working fine.
    for url in _browsable_urls(args.host, args.port):
        print(f"\n  {url}")
    print()
    try:
        app.run(host=args.host, port=args.port, debug=args.debug,
                threaded=True)
    finally:
        ARB.stop()


if __name__ == "__main__":
    main()

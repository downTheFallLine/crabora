# agentix — chat UI and API

An agentic layer over the robot: a web UI served by nginx, a Flask API,
and tools the model can call.

## Phase 1 is READ ONLY

Nothing the agent can call moves a servo. Not because motion is hard to
wire up, but because the plumbing — nginx, Flask, tool dispatch, the UI
— should be shaken out against a robot that cannot be damaged by it.
Two tibias have already snapped from motions that were geometrically
wrong, and a model choosing parameters is a new source of wrong
parameters.

Motion gets added in order of blast radius: dance and turn (feet
planted, recoverable) before stand (the only thing that puts weight on
legs). When it is, tools get **constrained schemas** —
`turn(degrees: 90|180|270|360)` — not raw passthrough to the CLI, so
the safe range lives where the model cannot argue with it.

## The STOP button is not a tool

It posts straight to `/api/stop`, which calls the arbiter directly. No
model, no tool dispatch, no LangChain. It has to work when the model is
slow, confused, unreachable or out of credit — which is the entire
reason it is a button and not a sentence.

Same reasoning for the status lights: they poll a cheap endpoint rather
than asking the model how the robot is.

## Layout

| | |
|---|---|
| `arbiter.py` | the only thing that touches the servo bus. One lock, ports held open, and `emergency_stop()` |
| `tools.py` | what the agent may call. Read-only in phase 1 |
| `app.py` | Flask API, and serves `web/` during development |
| `web/` | the UI: chat pane plus a fixed control strip |

`arbiter.py` exists because the bus is half-duplex and shared: two
concurrent requests interleave their packets and both get garbage. A
single lock makes that impossible rather than unlikely.

## Running it

```bash
python3 -m pip install flask
python3 agentix/app.py                 # http://localhost:5000
python3 agentix/app.py --no-bus        # UI work with no robot attached
```

The agent is optional. With no model configured the UI, the control
strip and every tool still work; only `/api/chat` degrades, and it says
so rather than failing.

## API

| | |
|---|---|
| `GET /api/health` | server up, bus open, agent configured |
| `GET /api/status` | robot health summary — cheap, for polling |
| `GET /api/tools` | what the agent can call |
| `POST /api/tools/<name>` | call one tool directly, no model involved |
| `POST /api/chat` | natural language in, answer out |
| `POST /api/stop` | **emergency stop** |

## The agent

`agent.py` is LangChain + `ChatAnthropic`, the same shape as NORA.

| | |
|---|---|
| model | `claude-opus-5-5` |
| thinking | adaptive — it cannot be disabled on this model, and `budget_tokens` is rejected |
| effort | `low` — these are small lookups over small tool results, and the UI should feel quick. Raise it in `agent.py` if answers get thin |
| tool choice | `auto` — Opus 5.5 rejects forced `any`/`tool` |

A **manual tool loop** rather than LangGraph: five tools and one
decision, LangGraph is another dependency on a Pi, and executing the
tools here is where the safety boundary lives. Every call goes through
the arbiter, and nothing the model emits reaches a servo except through
a function listed in `tools.py`.

Set `ANTHROPIC_API_KEY` before starting the server. Without it
`build_agent()` raises, `app.py` catches it, and the UI reports chat as
unavailable while the control strip and every tool keep working.

```bash
export ANTHROPIC_API_KEY=sk-ant-...
python3 agentix/app.py
```

An **org-level** key is not tied to a workspace, and the API rejects it
with a 400 asking for a workspace. Either use a workspace-scoped key, or
name one:

```bash
export ANTHROPIC_WORKSPACE_ID=wrkspc_...
```

That goes out as the `anthropic-workspace-id` header on every request,
and is only attached when set. `/api/health` reports which is in play.

Note the failure lands late: a key being *present* is all `build_agent`
checks, so startup looks healthy and the 400 arrives on the first chat.

## Running at boot

```bash
cd ~/crabora/software/agentix && ./install_service.sh
```

Installs `crabora-agentix.service`, enables it at boot and starts it.
Re-runnable. It resolves this machine's paths and interpreter rather
than hardcoding them, checks `flask` is importable for that interpreter,
offers to add you to `dialout` (without which the bus comes up
unavailable while the UI looks fine), and prompts for the credentials.

**Credentials live in `/etc/crabora/agentix.env`**, root-owned and 0600,
outside the repo — a key inside a working tree is one `git add -A` away
from being published. The unit references it with a leading `-`, so a
missing file is not a startup failure: the UI and all tools still work,
only chat is disabled.

```
logs      sudo journalctl -u crabora-agentix -f
restart   sudo systemctl restart crabora-agentix
disable   sudo systemctl disable --now crabora-agentix
```

**Stop the service before running a gait program by hand.** They cannot
share the bus, and the gait programs now *refuse to start* while it is
up rather than warning and continuing.

The confusing part, worth stating once: **the service holds the serial
ports whether or not the chat agent is enabled.** The arbiter opens all
three URTs at startup and keeps them open — that is what makes the UI
answer quickly. So a UI showing "no agent" because no API key is set is
still a process that owns the bus. "The agent is not running" and "the
service is not running" are different statements, and only the second
one frees the ports.

Tool failures are handed back to the model as observations rather than
raised — it can usually recover, and "leg 3 isn't answering" is a better
answer than a 500.

## Still to do

- nginx config for serving `web/` and proxying `/api`. The unit
  currently binds 0.0.0.0:5000 directly; nginx should take 80 and
  proxy, at which point the bind can move to 127.0.0.1.
- Motion tools, in blast-radius order, with constrained schemas.
- The live model path is **untested** — there were no Anthropic
  credentials on the machine it was written on. Tool wiring, schemas and
  the degraded path are verified; the first real request is not.

## Motion

Motion is gated. Three different kinds of check stand between a sentence
in the chat box and a servo moving:

1. **ARMED** (`safety.py`) — a human pressed ARM, opening a time window.
   The agent cannot arm itself: `/api/arm` is a route the tool layer has
   no way to call. The window expires on idle (5 min), has a hard
   session ceiling (30 min), and is dropped immediately by STOP.
2. **PREFLIGHT** (`motion.py`) — bus up, nothing latched, nothing over
   65 °C, supply not sagging below 6.2 V. A brownout reads exactly like
   a wiring fault, so it is worth refusing rather than debugging.
3. **SHAPE** — the move is one of a fixed set with bounded arguments.
   `turn` takes 90/180/270/360 and nothing else; `nudge_joint` is capped
   at ±20°. The schemas state the bounds so the model usually asks for
   something legal; `motion.py` enforces them because "usually" is not a
   safety property. Servo firmware limits clamp underneath all of it.

**Nothing here invents a trajectory.** Two tibias have snapped from
motions that looked geometrically reasonable. Every sequence is the one
debugged against hardware — `stand.py`'s phase ordering, `turn.py`'s
planted-foot sweep, `dance.py`'s frames — called as a library. Where the
sequence was buried in a `main()`, it was lifted out (`turn.run_turn`)
rather than copied, so the CLI and the agent run the same code.

| tool | blast radius |
|---|---|
| `relax` | drops torque; a standing robot sits down |
| `nudge_joint` | one joint, ±20°, relative |
| `lay_down` | splay flat — all joints centred, femurs last |
| `sit` | compact pose, held under torque |
| `stand_up` | full recovery: collapse, untangle, stand |
| `turn` | planted-foot, clockwise only |
| `walk` | planted-foot, stop-and-wait, any of four directions |
| `scurry` | walk's gait streamed at 50 Hz, open loop; period 2–6 s, refused if the femurs cannot follow; stops itself on a supply sag |
| `dance` | highest frame rate, most servos at once |

`lay_down` is `pose_splay()`, which is every joint at centre — that is
how the conventions were chosen. It moves coxa, then tibia, then femur:
femurs last is the lesson from the tibia that snapped, since the other
order drives a flexed knee into the floor.

### Interrupting

Long motions poll `arbiter.stopping` between steps. STOP sets that flag
*without* taking the bus lock — otherwise a 30-second dance holding the
lock would make STOP wait for it, which is the one thing it must never
do. The motion drops out at its next step, then STOP takes the bus and
releases torque.

## Voice

**Talk** — tap it, speak, and the transcript is sent exactly as if you
had typed it. It stops on its own at a pause. **Voice** reads replies
aloud. STOP cancels both, because an emergency stop should not be
competing with a voice finishing the previous answer.

### The microphone needs a secure context

Speech recognition is refused outside https or localhost. Served over
plain http from the Pi's LAN address, `http://192.168.0.131:5000`, the
browser blocks the microphone and the button shows **Mic blocked** with
the reason — the failure is detected up front rather than surfacing as a
bare `not-allowed` on first tap.

The easy way round is an SSH tunnel, because **localhost counts as a
secure context**:

```bash
ssh -N -L 5050:localhost:5000 ubuntu@<pi>
```

Then open `http://localhost:5050/`. Use 5050 rather than 5000: AirPlay
Receiver owns 5000 on macOS. This also sidesteps the macOS Local Network
privacy permission, which Chrome needs to reach a LAN address at all.

Reading replies aloud has no such requirement and works either way.

Recognition works in Chrome and Safari; Firefox has none. Note that
browsers refuse an insecure page differently -- Chrome exposes
`webkitSpeechRecognition` and fails at `start()`, Safari does not define
it at all -- so the context is checked *before* the constructor. Testing
the other way round reports "no such feature" on a browser that has the
feature and is only objecting to http, which sends you looking for the
wrong fix.

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

## Still to do

- `agent.py` — LangChain wiring. `app.py` imports it if present and
  degrades cleanly if not.
- nginx config for serving `web/` and proxying `/api`.
- Motion tools, in blast-radius order, with constrained schemas.

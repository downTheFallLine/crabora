"""
agent -- the language model behind /api/chat.

LangChain + ChatAnthropic, matching the shape of NORA so the pattern is
familiar rather than novel.

A manual tool loop rather than LangGraph: there are five tools and one
decision, LangGraph would be another dependency on a Pi, and -- more to
the point -- executing the tools myself is where the safety boundary
lives. Every call goes through the arbiter, and nothing the model emits
reaches a servo except through a function listed in tools.py.

PHASE 1 IS READ ONLY. The tool list contains nothing that moves the
robot. The emergency stop is deliberately absent: it belongs to the
human, on a button, reachable without the model being involved at all.

Credentials come from ANTHROPIC_API_KEY. With none set, build_agent()
raises, app.py catches it, and the UI says chat is unavailable while
every tool and the control strip keep working.
"""

import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from langchain_anthropic import ChatAnthropic                 # noqa: E402
from langchain_core.messages import (                         # noqa: E402
    AIMessage, HumanMessage, SystemMessage, ToolMessage,
)
from langchain_core.tools import StructuredTool               # noqa: E402
from pydantic import BaseModel, Field                         # noqa: E402

from motion import MOTION_HELP, MOTION_TOOLS, NUDGE_MAX_DEG   # noqa: E402
from tools import READ_ONLY_TOOLS, TOOL_HELP                  # noqa: E402

# Opus 5.5. Thinking is adaptive (it cannot be disabled on this model);
# effort defaults to medium but these are simple lookups over small tool
# results, so low keeps the UI responsive. Raise it if answers get thin.
MODEL = "claude-opus-5-5"
EFFORT = "low"
MAX_TOKENS = 4096
MAX_STEPS = 6          # tool round trips before giving up

# An org-level key is not tied to a workspace, so the API rejects it with
# a 400 unless the request names one. A workspace-scoped key carries that
# itself and needs nothing here. Set ANTHROPIC_WORKSPACE_ID to the
# wrkspc_... id to use an org key.
WORKSPACE_ID = os.environ.get("ANTHROPIC_WORKSPACE_ID")

SYSTEM = """You are the voice of CRABORA, a six-legged walking robot.

You can look at the robot AND move it. There is no arming step: motion
tools work whenever you call them, so a move happens the moment you ask
for it. The human has a STOP button that drops all torque. If a motion
tool reports it was stopped or refused, say so plainly rather than
retrying.

Moving the robot is real and it breaks: two tibias have already snapped.
So:
- Look before you move. robot_status or calibration_check first if you do
  not know what state it is in.
- Prefer the smallest motion that answers the question. nudge_joint
  before lay_down, lay_down before stand_up.
- stand_up, turn, scurry and scare draw the most current and carry the
  most risk. scare stands on four legs and has not been proven on
  hardware. scurry is walk done fast and has not been proven on hardware;
  use walk unless speed is what was asked for. Say
  what you are about to do before doing it when the move is a big one.
- If a motion reports joints that came up short, say which, and do not
  simply try again harder.
- Never chain several motions to "fix" an unexpected result. Report it.

Conventions you should use when answering:
- Legs are numbered 1-6 clockwise from above. The front of the robot is
  the hexagon point between legs 1 and 6. Leg 1 is front-right, 2 right,
  3 rear-right, 4 rear-left, 5 left, 6 front-left.
- Each leg has three joints: coxa (swings fore/aft), femur (lifts), and
  tibia (the knee). Servo IDs are leg*10 + joint, where joint is 1 coxa,
  2 femur, 3 tibia -- so 42 is leg 4's femur.
- Body frame: +X forward, +Y left, +Z up.

Call tools to find things out rather than guessing, and call
robot_status first if you do not know the overall state. Several tools
in one go is fine.

Answer like an engineer talking to the person who built the robot:
short, concrete, numbers where they matter. Lead with the answer. If
something looks wrong, say what it is and what would fix it. Do not pad
with caveats, and do not list everything you checked unless it helps.

One thing worth flagging when it comes up, because it is known and
unresolved: the servo power converter is a small LM2596 good for about
2 A continuous (3 A absolute), well under what standing up or walking
can draw. It runs from a 3S pack and regulates at 7.4 V, so servo
voltage below 7.4 V means the converter is at its current limit, not
that the battery is low. A low battery does not show in servo voltage.

Formatting: your reply is rendered as Markdown in a chat window, so use
it where it helps -- tables for per-leg or per-joint numbers, lists,
`code` for IDs and commands. For a picture, such as a leg layout or a
foot plot, you may include a small inline <svg> (keep it under ~60
lines, no scripts). Plain prose is fine when that is clearest.

Spoken summary: the reply is also read aloud, but only a summary is
spoken, not the Markdown. End EVERY reply with one final line of the
form
SAY: <the answer in about 20 words of plain speech>
No Markdown, numbers written as you would say them, no "SAY:" anywhere
else. It is stripped from the chat before display."""


# Schemas are where the safe range lives. Stating the bounds here means
# the model usually asks for something legal; motion.py enforces them
# anyway, because "usually" is not a safety property.
class NoArgs(BaseModel):
    pass


class JointArgs(BaseModel):
    leg: int = Field(description="leg number, 1-6")
    joint: str = Field(description="'coxa', 'femur' or 'tibia'")


class LegsArgs(BaseModel):
    legs: list[int] | None = Field(
        default=None, description="legs 1-6; omit for all six")


class NudgeArgs(BaseModel):
    leg: int = Field(description="leg number, 1-6")
    joint: str = Field(description="'coxa', 'femur' or 'tibia'")
    degrees: float = Field(
        description=f"how far to move from the present position, "
                    f"-{NUDGE_MAX_DEG:g} to +{NUDGE_MAX_DEG:g}. Positive is "
                    f"forward (coxa), up (femur), extend (tibia).")


class StandArgs(BaseModel):
    height: float = Field(
        default=100.0, description="deck clearance in mm, 40-160")


class WalkArgs(BaseModel):
    distance: float = Field(
        default=200.0, description="how far to travel in mm, 20-1000")
    direction: str = Field(
        default="forward",
        description="'forward', 'backward', 'left' or 'right'")


class ScurryArgs(BaseModel):
    distance: float = Field(
        default=300.0, description="how far to travel in mm, 20-1000")
    direction: str = Field(
        default="forward",
        description="'forward', 'backward', 'left' or 'right'")
    period: float = Field(
        default=3.0,
        description="seconds per gait cycle, 2-6; shorter is faster. "
                    "Start at 3 unless asked otherwise")


class TurnArgs(BaseModel):
    degrees: int = Field(
        description="clockwise turn: exactly 90, 180, 270 or 360")


class DanceArgs(BaseModel):
    seconds: float = Field(default=10.0, description="2-30 seconds")


class ScareArgs(BaseModel):
    waves: int = Field(default=4, description="how many waves, 1-12")


_SCHEMAS = {
    "joint_detail": JointArgs,
    "relax":        LegsArgs,
    "lay_down":     LegsArgs,
    "nudge_joint":  NudgeArgs,
    "stand_up":     StandArgs,
    "walk":         WalkArgs,
    "scurry":       ScurryArgs,
    "turn":         TurnArgs,
    "dance":        DanceArgs,
    "scare":        ScareArgs,
}


def _schema_for(name):
    return _SCHEMAS.get(name, NoArgs)


def _bind(fn, arbiter):
    """Close over the arbiter so the model never sees it as an argument."""
    def call(**kwargs):
        return fn(arbiter, **kwargs)
    return call


def _bind2(fn, arbiter, safety):
    """Same, for motion tools, which also need the arming gate."""
    def call(**kwargs):
        return fn(arbiter, safety, **kwargs)
    return call


class Agent:
    def __init__(self, arbiter, safety=None, model=MODEL, effort=EFFORT):
        self.model_name = model
        self._arbiter = arbiter
        self._safety = safety
        self._tools = [
            StructuredTool.from_function(
                func=_bind(fn, arbiter),
                name=name,
                description=TOOL_HELP.get(name, name),
                args_schema=_schema_for(name),
            )
            for name, fn in sorted(READ_ONLY_TOOLS.items())
        ]
        if safety is not None:
            # Motion tools take (arbiter, safety, **kwargs). The safety
            # gate is closed over, not exposed: there is no argument the
            # model can pass to affect whether it is armed.
            self._tools += [
                StructuredTool.from_function(
                    func=_bind2(fn, arbiter, safety),
                    name=name,
                    description=MOTION_HELP.get(name, name),
                    args_schema=_schema_for(name),
                )
                for name, fn in sorted(MOTION_TOOLS.items())
            ]
        kwargs = {}
        if WORKSPACE_ID:
            kwargs["default_headers"] = {
                "anthropic-workspace-id": WORKSPACE_ID}
        llm = ChatAnthropic(
            model=model,
            max_tokens=MAX_TOKENS,
            # Adaptive thinking: on Opus 5.5 it cannot be disabled, and
            # budget_tokens is rejected. Effort is the only dial.
            thinking={"type": "adaptive"},
            output_config={"effort": effort},
            **kwargs,
        )
        # tool_choice stays "auto" -- Opus 5.5 rejects forced any/tool.
        self.llm = llm.bind_tools(self._tools)
        self._by_name = {t.name: t for t in self._tools}

    def describe(self):
        return {
            "enabled": True,
            "model": self.model_name,
            "tools": sorted(self._by_name),
            "phase": "read-only",
            "workspace": WORKSPACE_ID or "(key-scoped)",
        }

    def ask(self, prompt):
        """One turn. Returns {'reply': str, 'tools_used': [str]}."""
        messages = [SystemMessage(SYSTEM), HumanMessage(prompt)]
        used = []

        for _step in range(MAX_STEPS):
            reply = self.llm.invoke(messages)
            messages.append(reply)
            calls = getattr(reply, "tool_calls", None) or []
            if not calls:
                text, spoken = split_spoken(_text(reply))
                return {"reply": text, "spoken": spoken, "tools_used": used}

            for call in calls:
                name = call.get("name")
                tool = self._by_name.get(name)
                used.append(name)
                if tool is None:
                    result = {"error": f"no such tool: {name}"}
                else:
                    try:
                        result = tool.func(**(call.get("args") or {}))
                    except Exception as e:                    # noqa: BLE001
                        # Hand the failure back rather than raising: the
                        # model can usually recover, and a robot that
                        # says "leg 3 isn't answering" beats a 500.
                        result = {"error": f"{type(e).__name__}: {e}"}
                messages.append(ToolMessage(
                    content=str(result), tool_call_id=call.get("id", name)))

        msg = ("I kept needing more information and ran out of steps. "
               "Try asking something narrower.")
        return {"reply": msg, "spoken": msg, "tools_used": used}


def _text(message: AIMessage) -> str:
    """Pull the text out, ignoring thinking blocks."""
    content = message.content
    if isinstance(content, str):
        return content.strip()
    parts = []
    for block in content or []:
        if isinstance(block, str):
            parts.append(block)
        elif isinstance(block, dict) and block.get("type") == "text":
            parts.append(block.get("text", ""))
    return "\n".join(p for p in parts if p).strip()


# The SAY: line the system prompt asks for. Last one wins, and it is
# matched anywhere on its own line, not only at the very end, because a
# model that adds a trailing remark after it should not get the remark
# read out instead.
_SAY = re.compile(r"^[ \t>*_]*SAY:[ \t*_]*(.+?)[ \t*_]*$", re.MULTILINE)
SPOKEN_WORDS = 30      # fallback cap, a little over the asked-for 20


def split_spoken(text):
    """(reply for display, short text to speak).

    The display text has the SAY: line removed. If the model forgot it,
    fall back to the opening sentences with the Markdown stripped, so
    the voice never reads out a table or an SVG.
    """
    found = list(_SAY.finditer(text))
    if found:
        last = found[-1]
        spoken = last.group(1).strip()
        shown = (text[:last.start()] + text[last.end():]).strip()
        return shown, spoken
    return text, plain_summary(text)


def plain_summary(text, words=SPOKEN_WORDS):
    t = re.sub(r"<svg.*?</svg>", " ", text, flags=re.S | re.I)
    t = re.sub(r"```.*?```", " ", t, flags=re.S)
    t = "\n".join(line for line in t.splitlines()     # tables, headings
                  if not line.lstrip().startswith(("|", "#")))
    t = re.sub(r"!?\[([^\]]*)\]\([^)]*\)", r"\1", t)     # links, images
    t = re.sub(r"[#*_`>~]+", "", t)
    t = re.sub(r"^\s*[-+]\s+", "", t, flags=re.M)
    t = " ".join(t.split())
    out = []
    for sentence in re.split(r"(?<=[.!?])\s+", t):
        out.append(sentence)
        if len(" ".join(out).split()) >= words * 2 // 3:
            break
    w = " ".join(out).split()
    return " ".join(w[:words]) + ("…" if len(w) > words else "")


def build_agent(arbiter, safety=None):
    """Raises if no credentials, so app.py can degrade cleanly."""
    if not (os.environ.get("ANTHROPIC_API_KEY")
            or os.environ.get("ANTHROPIC_AUTH_TOKEN")):
        raise RuntimeError(
            "no ANTHROPIC_API_KEY set -- chat is disabled, tools still work")
    return Agent(arbiter, safety)

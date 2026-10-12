/* CRABORA chat UI.
 *
 * Two independent paths on purpose:
 *   - the composer talks to /api/chat, which goes through the model
 *   - the control strip talks to /api/tools/* and /api/stop directly
 *
 * STOP must work when the model is slow, confused or unreachable, so it
 * never touches the agent. Same reason the status lights poll a cheap
 * endpoint rather than asking the model how the robot is.
 */

const $ = (id) => document.getElementById(id);
const log = $("log");

function el(tag, cls, text) {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text !== undefined) n.textContent = text;
  return n;
}

function say(who, cls, build) {
  const msg = el("div", `msg ${cls}`);
  msg.appendChild(el("div", "who", who));
  const body = el("div", "body");
  build(body);
  msg.appendChild(body);
  log.appendChild(msg);
  log.scrollTop = log.scrollHeight;
  return msg;
}

const sayText = (who, cls, text) =>
  say(who, cls, (b) => b.appendChild(el("p", null, text)));

const sayData = (who, data) =>
  say(who, "bot", (b) =>
    b.appendChild(el("pre", null, JSON.stringify(data, null, 2))));

async function api(path, body) {
  const res = await fetch(path, {
    method: body === undefined ? "GET" : "POST",
    headers: { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  let data;
  try { data = await res.json(); }
  catch { throw new Error(`${res.status} ${res.statusText}`); }
  if (!res.ok || data.ok === false) {
    throw new Error(data.error || `${res.status} ${res.statusText}`);
  }
  return data;
}

/* ---------------- status lights ---------------- */
function setLight(id, state, label) {
  const n = $(id);
  n.className = `light ${state}`;
  if (label) n.textContent = label;
}

async function refreshLights() {
  try {
    const h = await api("/api/health");
    setLight("light-bus", h.bus.available ? "good" : "bad",
             h.bus.available ? "bus" : "no bus");
    setLight("light-agent", h.agent && h.agent.enabled ? "good" : "warn",
             h.agent && h.agent.enabled ? "agent" : "no agent");
    setArmed(h.safety);
    if (!h.bus.available) {
      setLight("light-servos", "", "servos");
      setLight("light-faults", "", "faults");
      return;
    }
  } catch {
    setLight("light-bus", "bad", "offline");
    return;
  }

  try {
    const s = (await api("/api/status")).result;
    const all = s.answering === s.expected;
    setLight("light-servos", all ? "good" : "warn",
             `${s.answering}/${s.expected}`);
    const n = s.faults.length;
    setLight("light-faults", n ? "bad" : "good",
             n ? `${n} fault${n > 1 ? "s" : ""}` : "no faults");
  } catch {
    setLight("light-servos", "warn", "servos");
  }
}

/* ---------------- arming ---------------- */
/* The server decides whether motion is allowed; this just reflects it and
 * keeps the buttons honest. Never assume armed state locally -- a window
 * can expire between polls, and a disabled button is a courtesy, not a
 * guarantee. */
let armedUntil = 0;

function setArmed(safety) {
  const armed = !!(safety && safety.armed);
  armedUntil = armed ? Date.now() + safety.seconds_left * 1000 : 0;
  $("arm").textContent = armed ? `ARMED ${safety.seconds_left}s` : "ARM";
  $("arm").classList.toggle("on", armed);
  $("phase").textContent = armed ? "motion armed" : "read-only";
  document.querySelectorAll("[data-motion]").forEach((b) => {
    b.disabled = !armed;
  });
}

function tickArm() {
  if (!armedUntil) return;
  const left = Math.max(0, Math.round((armedUntil - Date.now()) / 1000));
  $("arm").textContent = left ? `ARMED ${left}s` : "ARM";
  if (!left) { armedUntil = 0; refreshLights(); }
}
setInterval(tickArm, 1000);

$("arm").addEventListener("click", async () => {
  const armed = armedUntil > Date.now();
  try {
    const r = await api(armed ? "/api/disarm" : "/api/arm", {});
    setArmed(r.safety);
    sayText("crabora", "bot", armed
      ? "Disarmed. Nothing will move until you arm again."
      : `Armed for ${r.safety.seconds_left}s. Motion is allowed until it expires.`);
  } catch (e) {
    sayText("crabora", "bot bad", e.message);
  }
});

/* ---------------- motion buttons ---------------- */
document.querySelectorAll("[data-motion]").forEach((btn) => {
  btn.addEventListener("click", async () => {
    const name = btn.dataset.motion;
    const label = btn.textContent;
    document.querySelectorAll("[data-motion]").forEach((b) => (b.disabled = true));
    btn.textContent = "…";
    sayText("you", "me", label);
    try {
      const r = await api(`/api/motion/${name}`, {});
      sayData("crabora", r.result);
      if (r.safety) setArmed(r.safety);
    } catch (e) {
      sayText("crabora", "bot bad", e.message);
    } finally {
      btn.textContent = label;
      refreshLights();
    }
  });
});

/* ---------------- control strip ---------------- */
document.querySelectorAll("[data-tool]").forEach((btn) => {
  btn.addEventListener("click", async () => {
    const name = btn.dataset.tool;
    btn.disabled = true;
    sayText("you", "me", name.replace(/_/g, " "));
    try {
      const r = await api(`/api/tools/${name}`, {});
      sayData("crabora", r.result);
    } catch (e) {
      sayText("crabora", "bot bad", e.message);
    } finally {
      btn.disabled = false;
      refreshLights();
    }
  });
});

$("stop").addEventListener("click", async () => {
  // Straight to the arbiter. No model, no tool dispatch, no waiting on
  // anything that could be slow.
  const btn = $("stop");
  btn.disabled = true;
  sayText("you", "me", "STOP");
  try {
    const res = await fetch("/api/stop", { method: "POST" });
    const data = await res.json();
    sayText("crabora", data.ok ? "bot" : "bot bad",
            data.message || data.error || "no response");
  } catch (e) {
    sayText("crabora", "bot bad",
            `Could not reach the robot: ${e.message}. CUT THE SERVO POWER.`);
  } finally {
    btn.disabled = false;
    refreshLights();
  }
});

/* ---------------- chat ---------------- */
$("composer").addEventListener("submit", async (ev) => {
  ev.preventDefault();
  const input = $("prompt");
  const text = input.value.trim();
  if (!text) return;
  input.value = "";
  sayText("you", "me", text);

  const send = $("send");
  send.disabled = true;
  const pending = sayText("crabora", "bot thinking", "thinking…");

  try {
    const r = await api("/api/chat", { prompt: text });
    pending.remove();
    say("crabora", "bot", (b) => {
      (r.reply || "").split(/\n{2,}/).forEach((p) =>
        b.appendChild(el("p", null, p)));
      if (r.tools_used && r.tools_used.length) {
        b.appendChild(el("div", "tools",
          `looked at: ${r.tools_used.join(", ")}`));
      }
    });
  } catch (e) {
    pending.remove();
    sayText("crabora", "bot bad", e.message);
  } finally {
    send.disabled = false;
    input.focus();
    refreshLights();
  }
});

refreshLights();
setInterval(refreshLights, 10000);
$("prompt").focus();

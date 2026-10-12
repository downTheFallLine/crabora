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

/* ---------------- rendering results ----------------
 * Tool and motion results arrive as plain dicts. Dumping them as JSON is
 * fine for debugging and bad for reading: a six-element leg list becomes
 * six lines of punctuation. So format them, and keep the raw JSON one
 * click away rather than deleting it.
 */

/* Keys whose value carries a unit, and what to show. */
const UNITS = {
  height_mm: "mm", moved_deg: "\u00b0", asked_deg: "\u00b0",
  turned_deg: "\u00b0", degrees: "\u00b0", seconds: "s",
  volts_min: "V", temp_c: "\u00b0C", reach_mm: "mm",
  x_mm: "mm", y_mm: "mm", z_mm: "mm", apart_mm: "mm",
};

/* A one-line summary per action, so the first thing read is the outcome
 * rather than the first key alphabetically. */
function headline(d) {
  switch (d.action) {
    case "stand":
      return `Stood up to ${d.height_mm} mm` +
             (d.excluded && d.excluded.length
               ? ` \u2014 without leg${d.excluded.length > 1 ? "s" : ""} ${d.excluded.join(", ")}`
               : " \u2014 all six legs");
    case "lay_down":   return "Laid out flat \u2014 all joints centred";
    case "sit":        return "Sat down and holding";
    case "relax":      return d.was_bearing_weight
                              ? "Limp \u2014 it was holding a pose, so it has sat down"
                              : "Limp \u2014 it was not bearing weight";
    case "nudge":      return `${d.joint}: moved ${d.moved_deg}\u00b0` +
                              (d.clamped ? " (clamped by its travel limit)" : "");
    case "turn":       return `Turned ${d.turned_deg}\u00b0 clockwise` +
                              (d.cancelled ? " \u2014 stopped early" : "");
    case "dance":      return `Danced ${d.seconds}s, ${d.frames} frames` +
                              (d.cancelled ? " \u2014 stopped early" : "");
    default:           return null;
  }
}

/* Keys the headline already covers, per action. Repeating them under a
 * sentence that just said them is noise, and it buries the rows that
 * actually carry new information. */
const CONSUMED = {
  stand:    ["height_mm", "legs", "excluded"],
  lay_down: ["pose", "legs"],
  sit:      [],
  relax:    ["was_bearing_weight", "note", "legs"],
  nudge:    ["joint", "moved_deg", "clamped", "note"],
  turn:     ["turned_deg", "degrees", "clockwise", "cancelled"],
  dance:    ["seconds", "frames", "cancelled"],
};

function fmtValue(key, v) {
  if (v === null || v === undefined) return null;      // drop empty rows
  if (Array.isArray(v)) {
    if (!v.length) return "none";
    if (v.every((x) => typeof x !== "object")) return v.join(", ");
    return null;                                        // handled as a block
  }
  if (typeof v === "boolean") return v ? "yes" : "no";
  if (typeof v === "object") return null;
  const u = UNITS[key] || "";
  return `${v}${u}`;
}

const prettyKey = (k) =>
  k.replace(/_(mm|deg|c)$/, "").replace(/_/g, " ");

function renderInto(box, data, depth, skip) {
  skip = skip || [];
  Object.entries(data).forEach(([k, v]) => {
    if (k === "action") return;                         // in the headline
    if (depth === 0 && skip.indexOf(k) !== -1) return;  // ditto
    const flat = fmtValue(k, v);
    if (flat !== null) {
      const row = el("div", "kv");
      row.appendChild(el("span", "k", prettyKey(k)));
      row.appendChild(el("span", "v", flat));
      box.appendChild(row);
      return;
    }
    if (Array.isArray(v)) {                             // array of objects
      box.appendChild(el("div", "k block", prettyKey(k)));
      v.forEach((item) => {
        const sub = el("div", "sub");
        if (typeof item === "object" && item !== null) renderInto(sub, item, depth + 1);
        else sub.appendChild(el("span", "v", String(item)));
        box.appendChild(sub);
      });
      return;
    }
    if (v && typeof v === "object") {
      box.appendChild(el("div", "k block", prettyKey(k)));
      const sub = el("div", "sub");
      renderInto(sub, v, depth + 1);
      box.appendChild(sub);
    }
  });
}

const sayData = (who, data) =>
  say(who, "bot", (b) => {
    if (data === null || typeof data !== "object") {
      b.appendChild(el("p", null, String(data)));
      return;
    }
    const head = headline(data);
    if (head) b.appendChild(el("p", "headline", head));
    const box = el("div", "kvs");
    renderInto(box, data, 0, head ? (CONSUMED[data.action] || []) : []);
    if (box.children.length) b.appendChild(box);
    if (!head && !box.children.length) {
      b.appendChild(el("p", "muted", "nothing to report"));
    }
    // Raw JSON stays available -- this is a debugging UI as much as a
    // chat one, and the formatter will not know every future shape.
    const det = el("details", "raw");
    det.appendChild(el("summary", null, "raw"));
    det.appendChild(el("pre", null, JSON.stringify(data, null, 2)));
    b.appendChild(det);
  });

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
  // Stop talking and stop listening: an emergency stop should not be
  // competing with a voice reading out the previous answer.
  if (canSpeak) window.speechSynthesis.cancel();
  if (listening && recog) recog.stop();
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


/* ---------------- voice ----------------
 * Tap Talk, speak, and the transcript is sent as if typed. Voice reads
 * replies back.
 *
 * SECURE CONTEXT: speech recognition needs https or localhost. Served
 * over plain http from the Pi's LAN address, the browser refuses the
 * microphone outright -- so detect that up front and say so, rather than
 * letting the first tap fail with a bare "not-allowed". An SSH tunnel
 * (localhost) is the easy way round it, and it happens to be a secure
 * context for free.
 *
 * Speech SYNTHESIS has no such requirement, so Voice works either way.
 */
const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
const canHear = !!SR && window.isSecureContext;
const canSpeak = "speechSynthesis" in window;

let recog = null, listening = false, speakReplies = false;

function micState(state, label) {
  const b = $("mic");
  b.classList.toggle("listening", state === "listening");
  $("mic-label").textContent = label;
}

// Check the CONTEXT before the constructor. Browsers differ in how they
// refuse an insecure page: Chrome exposes webkitSpeechRecognition and
// fails at start(), Safari does not define it at all. Testing for the
// constructor first therefore reports "no such feature" on a browser
// that has the feature and is only objecting to http -- the wrong
// diagnosis, pointing at the wrong fix.
if (!window.isSecureContext) {
  $("mic").disabled = true;
  micState("", "Mic blocked");
  $("mic").title =
    "The microphone needs https or localhost -- this page is plain http. " +
    "Reach it through an SSH tunnel: " +
    "ssh -N -L 5050:localhost:5000 ubuntu@<pi>, then http://localhost:5050/";
} else if (!SR) {
  $("mic").disabled = true;
  micState("", "No mic");
  $("mic").title =
    "This browser has no speech recognition. Chrome and Safari both do; " +
    "Firefox does not.";
}

function startListening() {
  recog = new SR();
  recog.lang = "en-US";
  recog.interimResults = true;
  recog.continuous = false;          // stops on its own at a pause
  recog.maxAlternatives = 1;
  // Safari requires start() to happen inside the user gesture that asked
  // for it, and will not honour continuous. Both are satisfied here: this
  // runs synchronously from the click handler.

  const input = $("prompt");
  let finalText = "";

  recog.onstart = () => { listening = true; micState("listening", "Listening"); };

  recog.onresult = (ev) => {
    let interim = "";
    for (let i = ev.resultIndex; i < ev.results.length; i++) {
      const r = ev.results[i];
      if (r.isFinal) finalText += r[0].transcript;
      else interim += r[0].transcript;
    }
    input.value = (finalText + interim).trim();
  };

  recog.onerror = (ev) => {
    const why = {
      "not-allowed": "Microphone permission was refused. Allow it in the " +
                     "address bar, then try again.",
      "service-not-allowed": "The browser blocked speech recognition. This " +
                     "usually means the page is not on https or localhost.",
      "no-speech": "I did not hear anything.",
      "audio-capture": "No microphone was found.",
      "network": "Speech recognition could not reach its network service.",
    }[ev.error] || `Speech recognition failed: ${ev.error}`;
    sayText("crabora", "bot bad", why);
  };

  recog.onend = () => {
    listening = false;
    micState("", "Talk");
    const text = input.value.trim();
    // Send it the same way typing would, so there is one path into chat.
    if (text) $("composer").dispatchEvent(new Event("submit", {cancelable: true}));
  };

  try { recog.start(); }
  catch { listening = false; micState("", "Talk"); }
}

$("mic").addEventListener("click", () => {
  if (listening) { recog && recog.stop(); return; }
  if (canSpeak) window.speechSynthesis.cancel();   // do not talk over the user
  startListening();
});

/* --- speaking --- */
$("speak").addEventListener("click", () => {
  if (!canSpeak) {
    sayText("crabora", "bot bad", "This browser cannot speak.");
    return;
  }
  speakReplies = !speakReplies;
  $("speak").textContent = speakReplies ? "Voice on" : "Voice off";
  $("speak").classList.toggle("on", speakReplies);
  $("speak").setAttribute("aria-pressed", String(speakReplies));
  if (!speakReplies) window.speechSynthesis.cancel();
});

function speak(text) {
  if (!speakReplies || !canSpeak || !text) return;
  window.speechSynthesis.cancel();
  const u = new SpeechSynthesisUtterance(text);
  u.rate = 1.0;
  u.pitch = 0.9;          // a shade below default; it is a robot
  window.speechSynthesis.speak(u);
}

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
    speak(r.reply);
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

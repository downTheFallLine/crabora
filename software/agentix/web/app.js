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
  asked_mm: "mm", travelled_mm: "mm", period_s: "s",
  volts_at_rest: "V", volts_lowest: "V",
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
    case "walk":       return `Walked ${d.travelled_mm} mm ${d.direction}` +
                              (d.cancelled ? " \u2014 stopped early" : "");
    case "scurry":     return `Scurried ${d.travelled_mm} mm ${d.direction}` +
                              (d.speed_mm_s ? ` at ${d.speed_mm_s} mm/s` : "") +
                              (d.stopped ? ` \u2014 stopped: ${d.stopped}`
                               : d.cancelled ? " \u2014 stopped early" : "");
    case "turn":       return `Turned ${d.turned_deg}\u00b0 clockwise` +
                              (d.cancelled ? " \u2014 stopped early" : "");
    case "dance":      return `Danced ${d.seconds}s, ${d.frames} frames` +
                              (d.cancelled ? " \u2014 stopped early" : "");
    case "scare":      return `Reared up and waved ${d.waves} time${d.waves === 1 ? "" : "s"}` +
                              (d.stopped ? ` \u2014 stopped: ${d.stopped}`
                               : d.cancelled ? " \u2014 stopped early" : "");
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
  walk:     ["travelled_mm", "direction", "cancelled"],
  scurry:   ["travelled_mm", "direction", "speed_mm_s", "cancelled", "stopped"],
  dance:    ["seconds", "frames", "cancelled"],
  scare:    ["waves", "cancelled", "stopped"],
};

/* Bus retries deserve to be noticed, not folded into a nested block. */
function busWarning(d) {
  if (!d.bus) return null;
  const r = d.bus.retries || 0, f = d.bus.failures || 0;
  return `bus needed ${r} re-send${r === 1 ? "" : "s"}` +
         (f ? ` and ${f} failed outright` : "") +
         " \u2014 likely the supply sagging under load";
}

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
    const warn = busWarning(data);
    if (warn) b.appendChild(el("p", "warn-line", warn));
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

/* ---------------- tilt ----------------
 * Polls /api/imu once a second while the page is visible. The IMU is on
 * I2C, not the servo bus, so this keeps working with the bus down or a
 * motion running. With no IMU fitted it backs off to every 10 s, so
 * plugging one in shows up without a reload.
 *
 * Signs, from imu_lib: roll is about +X, positive = LEFT side up; pitch
 * is about +Y, positive = nose up. Both gauges are drawn so that is what
 * you see: from behind, left side rising; from the right, nose rising.
 * SVG rotate() is clockwise-positive with y down, hence the minus.
 */
const TILT_MS = 1000, TILT_IDLE_MS = 10000;
const TILT_WARN = 5, TILT_BAD = 15;          // degrees, either axis
let tiltTimer = null;

const deg = (v) => `${v > 0 ? "+" : ""}${v.toFixed(1)}\u00b0`;

function showTilt(r) {
  const t = $("tilt");
  $("roll-body").style.transform = `rotate(${-r.roll_deg}deg)`;
  $("pitch-body").style.transform = `rotate(${-r.pitch_deg}deg)`;
  $("roll-val").textContent = deg(r.roll_deg);
  $("pitch-val").textContent = deg(r.pitch_deg);
  const worst = Math.max(Math.abs(r.roll_deg), Math.abs(r.pitch_deg));
  t.dataset.level = worst >= TILT_BAD ? "bad" : worst >= TILT_WARN ? "warn" : "good";
  // Tilt comes from the accelerometer, so it is only true when gravity
  // is the only force on it. Mid-step it is not; say so. Judged on g
  // alone, NOT r.still: that also tests the gyro, and imu_state never
  // zeroes the gyro, so its raw bias reads as "moving" on a robot sat
  // on the bench.
  const steady = Math.abs(r.g_total - 1) < 0.08;
  t.classList.toggle("moving", !steady);
  $("tilt-state").textContent = worst >= TILT_BAD ? "Tilted"
                              : worst >= TILT_WARN ? "Leaning" : "Level";
  $("tilt-detail").textContent =
    `${r.g_total.toFixed(2)} g · ${steady ? "steady" : "moving, approximate"}` +
    ` · ${r.temp_c.toFixed(0)}\u00b0C`;
  t.title = "Roll: + is left side up. Pitch: + is nose up.";
}

function noTilt(state, detail) {
  const t = $("tilt");
  t.dataset.level = "none";
  t.classList.remove("moving");
  $("roll-val").textContent = "—";
  $("pitch-val").textContent = "—";
  $("tilt-state").textContent = state;
  $("tilt-detail").textContent = detail;
  t.title = detail;
}

async function refreshTilt() {
  clearTimeout(tiltTimer);
  let next = TILT_MS;
  if (!document.hidden) {
    try {
      const r = (await api("/api/imu")).result;
      if (r.fitted) showTilt(r);
      else { noTilt("No IMU", r.error || "not fitted"); next = TILT_IDLE_MS; }
    } catch (e) {
      noTilt("IMU offline", e.message);
      next = TILT_IDLE_MS / 2;
    }
  }
  tiltTimer = setTimeout(refreshTilt, next);
}
document.addEventListener("visibilitychange", () => {
  if (!document.hidden) refreshTilt();
});
refreshTilt();

/* ---------------- leg animation ----------------
 * Polls /api/leg/<n> four times a second while the page is visible and
 * eases the drawing toward each reading, so a gait reads as motion
 * rather than a slideshow. The server answers with MEASURED angles when
 * the bus is free and the gait's COMMANDED goals while a motion holds
 * it; the tag under the picker says which.
 *
 * Geometry is pose.leg_fk's, in mm: femur 100, tibia 120, femur angle
 * up from level, tibia flex 0 = straight. Top view puts the front up,
 * legs at their bearings (30 + (n-1)*60 degrees, clockwise from the
 * front), coxa angle + = counter-clockwise from above.
 */
const LEG_POLL_MS = 250, LEG_IDLE_MS = 2000;
const FEMUR_MM = 100, TIBIA_MM = 120;
const HIP_R = 85, COXA_MM = 55;        // top view, drawn not to scale
const LEG_KEY = "crabora.leg";
let legTimer = null, legTarget = null, legShown = null, legRaf = null;

const rad = (d) => d * Math.PI / 180;
const fmt = (v) => `${v > 0 ? "+" : ""}${v.toFixed(0)}\u00b0`;

// Body frame (X forward, Y left) -> screen (front up).
const topXY = (x, y) => [-y, -x];

(function drawBody() {
  const hex = [];
  for (let n = 1; n <= 6; n++) {
    // Hexagon POINTS sit between legs (the front point is between 1 and
    // 6), so the vertices are at the legs' bearings minus 30.
    const b = rad(-(n - 1) * 60);
    hex.push(topXY(HIP_R * 1.12 * Math.cos(b), HIP_R * 1.12 * Math.sin(b)).join(","));
  }
  $("leg-hex").setAttribute("points", hex.join(" "));
})();

function drawLegs(leg, coxaDeg) {
  const g = $("leg-others");
  g.textContent = "";
  for (let n = 1; n <= 6; n++) {
    const b = rad(-(30 + (n - 1) * 60));
    const [hx, hy] = topXY(HIP_R * Math.cos(b), HIP_R * Math.sin(b));
    if (n === leg) {
      const phi = b + rad(coxaDeg);
      const [ex, ey] = topXY(HIP_R * Math.cos(b) + COXA_MM * Math.cos(phi),
                             HIP_R * Math.sin(b) + COXA_MM * Math.sin(phi));
      const c = $("leg-coxa");
      c.setAttribute("x1", hx); c.setAttribute("y1", hy);
      c.setAttribute("x2", ex); c.setAttribute("y2", ey);
      continue;
    }
    const [ex, ey] = topXY((HIP_R + COXA_MM * 0.8) * Math.cos(b),
                           (HIP_R + COXA_MM * 0.8) * Math.sin(b));
    const l = document.createElementNS("http://www.w3.org/2000/svg", "line");
    l.setAttribute("class", "other");
    l.setAttribute("x1", hx); l.setAttribute("y1", hy);
    l.setAttribute("x2", ex); l.setAttribute("y2", ey);
    g.appendChild(l);
  }
}

function drawSide(femurUp, flex) {
  const beta = rad(femurUp), gamma = beta - rad(flex);
  const kx = FEMUR_MM * Math.cos(beta), ky = -FEMUR_MM * Math.sin(beta);
  const fx = kx + TIBIA_MM * Math.cos(gamma), fy = ky - TIBIA_MM * Math.sin(gamma);
  const set = (id, a) => Object.entries(a).forEach(([k, v]) => $(id).setAttribute(k, v));
  set("leg-femur", { x2: kx, y2: ky });
  set("leg-tibia", { x1: kx, y1: ky, x2: fx, y2: fy });
  set("leg-knee", { cx: kx, cy: ky });
  set("leg-foot", { cx: fx, cy: fy });
}

function legFrame() {
  legRaf = null;
  if (!legTarget) return;
  const keys = ["coxa_deg", "femur_up_deg", "tibia_flex_deg"];
  if (!legShown || legShown.leg !== legTarget.leg) legShown = { ...legTarget };
  let moving = false;
  for (const k of keys) {
    const d = legTarget[k] - legShown[k];
    legShown[k] = Math.abs(d) < 0.05 ? legTarget[k] : legShown[k] + d * 0.3;
    if (legShown[k] !== legTarget[k]) moving = true;
  }
  drawSide(legShown.femur_up_deg, legShown.tibia_flex_deg);
  drawLegs(legShown.leg, legShown.coxa_deg);
  if (moving) legRaf = requestAnimationFrame(legFrame);
}

const LEG_SOURCE = {
  "measured": "measured",
  "commanded": "commanded · moving",
  "last commanded": "last commanded",
};

async function refreshLeg() {
  clearTimeout(legTimer);
  const leg = Number($("leg-pick").value);
  let next = LEG_POLL_MS;
  if (!document.hidden) {
    try {
      const r = (await api(`/api/leg/${leg}`)).result;
      $("legview").dataset.source = r.source;
      if (r.source === "none") {
        $("leg-src").textContent = r.reason || "no reading";
        next = LEG_IDLE_MS;
      } else {
        legTarget = r;
        $("leg-femur-val").textContent = fmt(r.femur_up_deg);
        $("leg-tibia-val").textContent = fmt(r.tibia_flex_deg);
        $("leg-coxa-val").textContent = fmt(r.coxa_forward_deg);
        $("leg-src").textContent = LEG_SOURCE[r.source] || r.source;
        if (!legRaf) legRaf = requestAnimationFrame(legFrame);
      }
    } catch (e) {
      $("legview").dataset.source = "none";
      $("leg-src").textContent = `offline: ${e.message}`;
      next = LEG_IDLE_MS;
    }
  } else {
    next = LEG_IDLE_MS;
  }
  legTimer = setTimeout(refreshLeg, next);
}

try { const v = localStorage.getItem(LEG_KEY); if (v) $("leg-pick").value = v; }
catch { /* blocked */ }
$("leg-pick").addEventListener("change", () => {
  try { localStorage.setItem(LEG_KEY, $("leg-pick").value); } catch { /* blocked */ }
  legTarget = null; legShown = null;
  refreshLeg();
});
document.addEventListener("visibilitychange", () => {
  if (!document.hidden) refreshLeg();
});
drawSide(0, 90);
drawLegs(Number($("leg-pick").value), 0);
refreshLeg();

/* ---------------- servo current ----------------
 * Polls /api/current twice a second. Measured from every servo when the
 * bus is free; during a motion, whatever that motion's own polling read
 * most recently (scurry reads one servo every 0.05 s, so each reading is
 * up to about a second old there). The total is a sum of readings taken
 * one at a time -- an estimate of the draw, not a simultaneous one.
 *
 * BUCK_A is the servo converter's comfortable continuous rating: the
 * SELOKY LM2596 today (doc/power.md). Raise it when the bigger buck goes
 * in. SERVO_A is where an STS3215 starts its over-current trip (>2 A for
 * 2 s).
 */
const POWER_POLL_MS = 500, POWER_IDLE_MS = 3000;
const BUCK_A = 2.0, SERVO_A = 2.0;
const JOINTS = ["coxa", "femur", "tibia"];
let powerTimer = null, powerPeak = 0, powerWasMoving = false;

(function buildAmps() {
  const body = $("amps-body");
  for (let n = 1; n <= 6; n++) {
    const tr = el("tr");
    tr.appendChild(el("th", null, `L${n}`));
    for (const j of JOINTS) {
      const td = el("td", null, "—");
      td.id = `amps-${n}-${j}`;
      tr.appendChild(td);
    }
    body.appendChild(tr);
  }
})();

function showPower(r) {
  const p = $("power");
  p.dataset.source = r.source;
  if (r.source === "none") {
    p.dataset.level = "none";
    $("power-src").textContent = r.reason || "no reading";
    return;
  }
  for (let n = 1; n <= 6; n++) {
    for (const j of JOINTS) {
      const td = $(`amps-${n}-${j}`), c = (r.legs[n] || {})[j];
      td.className = "";
      if (!c) { td.textContent = "—"; continue; }
      td.textContent = c.amps.toFixed(2);
      if (!c.fresh) td.classList.add("stale");
      else if (c.amps >= SERVO_A) td.classList.add("bad");
      else if (c.amps >= SERVO_A / 2) td.classList.add("warn");
    }
  }
  // Peak covers one motion: it resets when a motion starts, and holds
  // after it ends so the number is still there to read.
  const moving = r.source === "during motion";
  if (moving && !powerWasMoving) powerPeak = 0;
  powerWasMoving = moving;
  powerPeak = Math.max(powerPeak, r.total_amps);
  const t = r.total_amps;
  p.dataset.level = t >= BUCK_A ? "bad" : t >= BUCK_A * 0.75 ? "warn" : "good";
  $("power-total").textContent = `${t.toFixed(2)} A`;
  $("power-fill").style.width = `${Math.min(100, (t / BUCK_A) * 100)}%`;
  $("power-peak").textContent =
    `peak ${powerPeak.toFixed(2)} A · buck ~${BUCK_A} A`;
  $("power-src").textContent =
    `${moving ? "during motion" : "measured"} · ${r.servos} servos`;
  p.title = r.note || "";
}

async function refreshPower() {
  clearTimeout(powerTimer);
  let next = POWER_POLL_MS;
  if (!document.hidden) {
    try {
      const r = (await api("/api/current")).result;
      showPower(r);
      if (r.source === "none") next = POWER_IDLE_MS;
    } catch (e) {
      $("power").dataset.source = "none";
      $("power").dataset.level = "none";
      $("power-src").textContent = `offline: ${e.message}`;
      next = POWER_IDLE_MS;
    }
  } else {
    next = POWER_IDLE_MS;
  }
  powerTimer = setTimeout(refreshPower, next);
}
document.addEventListener("visibilitychange", () => {
  if (!document.hidden) refreshPower();
});
refreshPower();

/* ---------------- motion enable ----------------
 * There is no ARM button: the server runs always-armed (safety.py,
 * ALWAYS_ARMED in app.py). The motion buttons still follow what the
 * server reports rather than assuming it, so turning the gate back on
 * server-side greys them out again without touching this file. */
function setArmed(safety) {
  const armed = !!(safety && safety.armed);
  document.querySelectorAll("[data-motion]").forEach((b) => {
    b.disabled = !armed;
    b.title = armed ? (b.dataset.tip || "") : (safety && safety.reason) || "";
  });
}
document.querySelectorAll("[data-motion]").forEach((b) => {
  b.dataset.tip = b.title;
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
  endConversation();
  if (canSpeak) window.speechSynthesis.cancel();
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
 * Tap Talk once to start a CONVERSATION: it listens, sends what you said
 * as if typed, reads the reply aloud, then listens again -- no tapping
 * per turn. Tap Talk again (or STOP) to end it. It also ends itself
 * after a few silent turns, so a forgotten tab does not keep a live
 * microphone.
 *
 * It never listens while it is speaking: the microphone would hear the
 * reply and send it back as your next question. So each turn is
 * listen -> think -> speak -> listen, strictly in that order.
 *
 * Replies are spoken in a conversation whatever the Voice button says;
 * Voice only decides whether TYPED questions get spoken answers.
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
const canSpeak = "speechSynthesis" in window;

// Silent listens in a row before a conversation gives up. Chrome ends a
// silent listen with "no-speech" after about 8 s, so 3 is roughly 25 s.
const QUIET_TURNS = 3;

let recog = null, speakReplies = false;
let talkMode = false, quietTurns = 0;
// Bumped by every start. A loop left awaiting a slow reply when its
// conversation was ended must not wake up inside the NEXT one and run
// beside it, so each loop checks it is still the current conversation.
let convoId = 0;

// Chrome fires "start" at once but its speech service can take several
// seconds (measured: ~4.8 s on Tom's Mac) before "audiostart", when it
// is actually taking sound. So Wait... until audiostart, then Speak now
// -- lighting up on "start" told you to talk into a microphone that was
// not open yet. In a conversation the button also says what the robot
// is doing between your turns.
function micState(state, label) {
  const b = $("mic");
  b.classList.toggle("convo", talkMode);
  b.classList.toggle("starting", state === "starting");
  b.classList.toggle("listening", state === "listening");
  $("mic-label").textContent = label;
  b.title = talkMode ? "Tap to end the conversation" : "Start a conversation";
}

// Why the mic cannot work here, or null if it can. Checked on every tap
// and said in the chat, not only in a hover title nobody reads -- a
// greyed-out button that does nothing when tapped looks like a bug.
//
// Check the CONTEXT before the constructor. Browsers differ in how they
// refuse an insecure page: Chrome exposes webkitSpeechRecognition and
// fails at start(), Safari does not define it at all. Testing for the
// constructor first therefore reports "no such feature" on a browser
// that has the feature and is only objecting to http -- the wrong
// diagnosis, pointing at the wrong fix.
const micBlocked =
  !window.isSecureContext
    ? "The microphone needs https or localhost, and this page is plain " +
      `http (${location.host}). Reach it through an SSH tunnel instead: ` +
      "ssh -N -L 5050:localhost:5000 ubuntu@<pi>, then open " +
      "http://localhost:5050/"
  : !SR
    ? "This browser has no speech recognition. Chrome and Safari both " +
      "do; Firefox does not."
  : null;

if (micBlocked) {
  $("mic").classList.add("blocked");
  micState("", window.isSecureContext ? "No mic" : "Mic blocked");
  $("mic").title = micBlocked;
}

const RECOG_ERRORS = {
  "not-allowed": "Microphone permission was refused. Allow it from the " +
                 "icon in the address bar, then try again. On a Mac, " +
                 "Chrome also needs System Settings → Privacy & Security " +
                 "→ Microphone. Safari will not reopen the microphone " +
                 "without a tap, so a conversation there lasts one turn.",
  "service-not-allowed": "The browser blocked speech recognition. This " +
                 "usually means the page is not on https or localhost.",
  "audio-capture": "No microphone was found, or another app has it.",
  "network": "Speech recognition could not reach its network service " +
             "(Chrome sends the audio to Google to transcribe).",
};

/* One listen: resolves with what was heard ("" for nothing), or rejects
 * with a message worth showing. Ending the conversation aborts it, which
 * resolves null. */
function listenOnce() {
  return new Promise((resolve, reject) => {
    const r = new SR();
    recog = r;
    r.lang = navigator.language || "en-US";
    r.interimResults = true;
    r.continuous = false;            // stops on its own at a pause
    r.maxAlternatives = 1;

    const input = $("prompt");
    const typed = input.value;
    let finalText = "", interim = "", error = null;

    r.onstart = () => micState("starting", "Wait…");
    r.onaudiostart = () => micState("listening", "Speak now");

    r.onresult = (ev) => {
      interim = "";
      for (let i = ev.resultIndex; i < ev.results.length; i++) {
        const res = ev.results[i];
        if (res.isFinal) finalText += res[0].transcript;
        else interim += res[0].transcript;
      }
      input.value = (finalText + interim).trim();
    };

    r.onerror = (ev) => { error = ev.error; };

    r.onend = () => {
      if (recog === r) recog = null;
      // Only what was actually heard. Without this a silent listen would
      // send whatever had been typed before tapping Talk.
      const heard = (finalText || (error ? "" : interim)).trim();
      if (!heard) input.value = typed;
      if (error === "aborted") return resolve(null);
      if (error && error !== "no-speech") {
        return reject(new Error(RECOG_ERRORS[error] ||
          `Speech recognition failed: ${error}` +
          (error === "language-not-supported" ? ` (${r.lang})` : "")));
      }
      resolve(heard);
    };

    // Safari requires start() inside the user gesture that asked for it.
    // The first listen is; later turns are not, and Safari refuses them
    // with "not-allowed" -- reported above. Chrome does not mind.
    try { r.start(); }
    catch (e) {
      recog = null;
      reject(new Error(`Could not start the microphone: ${e.message}`));
    }
  });
}

async function conversation(id) {
  const live = () => talkMode && id === convoId;
  quietTurns = 0;
  while (live()) {
    let heard;
    try { heard = await listenOnce(); }
    catch (e) { if (live()) endConversation(e.message); return; }
    if (!live() || heard === null) return;

    if (!heard) {
      if (++quietTurns >= QUIET_TURNS) {
        endConversation("I have not heard anything for a while, so I " +
                        "stopped listening. Tap Talk to carry on.");
        return;
      }
      continue;
    }
    quietTurns = 0;

    micState("", "Thinking…");
    $("prompt").value = "";
    const reply = await sendChat(heard);
    if (!live()) return;
    if (reply === null) {               // the chat failed; it said why
      endConversation();
      return;
    }
    micState("", "Replying…");
    await speak(reply.spoken);
  }
}

function startConversation() {
  talkMode = true;
  if (canSpeak) window.speechSynthesis.cancel();   // do not talk over the user
  conversation(++convoId);
}

// Safe to call any time, including when no conversation is running.
function endConversation(why) {
  const was = talkMode;
  talkMode = false;
  if (recog) recog.abort();
  if (was && canSpeak) window.speechSynthesis.cancel();
  if (!micBlocked) micState("", "Talk");
  if (why) sayText("crabora", "bot bad", why);
}

$("mic").addEventListener("click", () => {
  if (micBlocked) { sayText("crabora", "bot bad", micBlocked); return; }
  if (talkMode) endConversation();
  else startConversation();
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
  if (!speakReplies && !talkMode) window.speechSynthesis.cancel();
});

/* --- voice picker ---
 * Every voice the browser has, grouped so the fun ones are easy to find.
 * Chrome on a Mac lists ~200, most of them other languages; English
 * comes first, and the macOS novelty voices get their own groups.
 * Picking one says a line in it. The choice is remembered per browser
 * (localStorage) -- a convenience, so it fails quietly where storage is
 * blocked. Voices load asynchronously in Chrome, hence voiceschanged.
 */
const VOICE_KEY = "crabora.voice";
const ROBOT_VOICES = ["Zarvox", "Trinoids", "Fred", "Ralph", "Albert",
                      "Junior", "Kathy"];
const FUN_VOICES = ["Whisper", "Bubbles", "Bells", "Boing", "Organ",
                    "Cellos", "Good News", "Bad News", "Jester", "Superstar",
                    "Wobble", "Bahh", "Grandma", "Grandpa"];
const SAMPLE_LINES = [
  "Hello. I am Crabora. All six legs reporting.",
  "Servos warm. Feet planted. Awaiting orders.",
  "I have six legs and no regrets.",
  "Scuttling is just walking with confidence.",
];
// A preset is a voice plus a manner of speaking. `prefer` is tried in
// order, since which voices exist depends on the machine; the first one
// present wins. Stored in the picker as "preset:<key>".
const PRESETS = {
  hal: {
    label: "HAL · calm, slow, low",
    prefer: ["Daniel", "Reed", "Google UK English Male", "Eddy", "Rocko"],
    rate: 0.85,
    pitch: 0.85,
    samples: [
      "Good evening. All six legs are functioning normally.",
      "I am watching the servo bus very closely.",
      "I think we should let the femurs cool before standing.",
    ],
  },
};

// What speak() uses: {voice, rate, pitch}, or null for the default.
let voices = [], chosen = null;

const baseName = (v) => v.name.replace(/\s*\(.*\)$/, "");
const inList = (list, v) => list.includes(baseName(v));

function loadVoices() {
  if (!canSpeak) return;
  voices = window.speechSynthesis.getVoices();
  if (!voices.length) return;

  let saved = null;
  try { saved = localStorage.getItem(VOICE_KEY); } catch { /* blocked */ }

  const pick = $("voice-pick");
  pick.textContent = "";
  pick.appendChild(new Option("Default voice", ""));
  const presets = document.createElement("optgroup");
  presets.label = "Presets";
  for (const [key, p] of Object.entries(PRESETS)) {
    if (presetVoice(p)) presets.appendChild(new Option(p.label, `preset:${key}`));
  }
  if (presets.children.length) pick.appendChild(presets);
  const english = voices.filter((v) => v.lang.startsWith("en"));
  const groups = [
    ["Robot", english.filter((v) => inList(ROBOT_VOICES, v))],
    ["Speaking", english.filter((v) => !inList(ROBOT_VOICES, v) &&
                                       !inList(FUN_VOICES, v))],
    ["Silly", english.filter((v) => inList(FUN_VOICES, v))],
    ["Other languages", voices.filter((v) => !v.lang.startsWith("en"))
                              .sort((a, b) => a.lang.localeCompare(b.lang))],
  ];
  for (const [label, list] of groups) {
    if (!list.length) continue;
    const g = document.createElement("optgroup");
    g.label = label;
    for (const v of list) {
      const tag = v.lang.startsWith("en") ? v.lang.slice(3) : v.lang;
      g.appendChild(new Option(
        // "Eddy (English (United States))" -> "Eddy · US": the tag
        // already says which, and the long form crowds the menu.
        `${baseName(v)}${tag ? ` · ${tag}` : ""}${v.localService ? "" : " ☁"}`,
        v.voiceURI));
    }
    pick.appendChild(g);
  }
  chosen = resolveChoice(saved);
  pick.value = chosen ? saved : "";
  pick.hidden = false;
}

function presetVoice(p) {
  for (const name of p.prefer) {
    const v = voices.find((x) => x.lang.startsWith("en") &&
                                 (x.name === name || baseName(x) === name));
    if (v) return v;
  }
  return null;
}

// Picker value -> {voice, rate, pitch, samples}, or null for the default.
function resolveChoice(value) {
  if (!value) return null;
  if (value.startsWith("preset:")) {
    const p = PRESETS[value.slice(7)];
    const v = p && presetVoice(p);
    return v ? { voice: v, rate: p.rate, pitch: p.pitch, samples: p.samples }
             : null;
  }
  const v = voices.find((x) => x.voiceURI === value);
  return v ? { voice: v, rate: 1.0, pitch: 1.0 } : null;
}

if (canSpeak) {
  loadVoices();
  window.speechSynthesis.addEventListener("voiceschanged", loadVoices);
  $("voice-pick").addEventListener("change", () => {
    const value = $("voice-pick").value;
    chosen = resolveChoice(value);
    try {
      if (value) localStorage.setItem(VOICE_KEY, value);
      else localStorage.removeItem(VOICE_KEY);
    } catch { /* blocked */ }
    // Do not talk over a conversation turn; otherwise, a sample.
    const lines = (chosen && chosen.samples) || SAMPLE_LINES;
    if (!talkMode) speak(lines[Math.floor(Math.random() * lines.length)]);
  });
}

/* Speak text; resolves when it has finished (or been cancelled), so the
 * conversation knows when it may listen again.
 *
 * Queued a sentence at a time: Chrome silently stops a single long
 * utterance after about 15 s and then never fires its end event, which
 * would leave a conversation waiting forever. The poll is the backstop
 * for the same bug -- if nothing is speaking or queued, it is done. */
function speak(text) {
  if (!canSpeak || !text) return Promise.resolve();
  window.speechSynthesis.cancel();
  const parts = text.replace(/\s+/g, " ").match(/[^.!?]+[.!?]*["')\]]*\s*/g)
                || [text];
  return new Promise((resolve) => {
    let done = false;
    const finish = () => {
      if (done) return;
      done = true;
      clearInterval(poll);
      resolve();
    };
    parts.forEach((p, i) => {
      const u = new SpeechSynthesisUtterance(p.trim());
      if (chosen) {
        u.voice = chosen.voice;
        u.lang = chosen.voice.lang;
        u.rate = chosen.rate;
        u.pitch = chosen.pitch;
      } else {
        u.rate = 1.0;
        u.pitch = 0.9;      // a shade below default; it is a robot
      }
      if (i === parts.length - 1) { u.onend = finish; u.onerror = finish; }
      window.speechSynthesis.speak(u);
    });
    const poll = setInterval(() => {
      if (!window.speechSynthesis.speaking && !window.speechSynthesis.pending)
        finish();
    }, 500);
  });
}

/* ---------------- markdown ----------------
 * Bot replies are Markdown, with inline SVG allowed for small diagrams.
 * Sanitised before it touches the DOM: the model's output is untrusted
 * HTML like any other, and an SVG can carry script. */
const canMarkdown = !!(window.marked && window.DOMPurify);
if (canMarkdown) {
  marked.setOptions({ gfm: true, breaks: false });
  // Links open in a new tab so following one does not lose the chat.
  DOMPurify.addHook("afterSanitizeAttributes", (n) => {
    if (n.tagName === "A" && n.getAttribute("href")) {
      n.setAttribute("target", "_blank");
      n.setAttribute("rel", "noopener noreferrer");
    }
  });
}

function renderMarkdown(body, text) {
  if (!canMarkdown) {
    (text || "").split(/\n{2,}/).forEach((p) => body.appendChild(el("p", null, p)));
    return;
  }
  const wrap = el("div", "md");
  wrap.innerHTML = DOMPurify.sanitize(marked.parse(text || ""), {
    USE_PROFILES: { html: true, svg: true, svgFilters: true },
  });
  // Wide tables scroll inside the bubble rather than stretching the page.
  wrap.querySelectorAll("table").forEach((t) => {
    const s = el("div", "table-scroll");
    t.replaceWith(s);
    s.appendChild(t);
  });
  body.appendChild(wrap);
}

/* ---------------- chat ----------------
 * One way in for typed and spoken questions alike. Resolves with
 * {reply, spoken}, or null if it failed (and has already said why).
 * `spoken` is the ~20-word summary the server split off the reply; only
 * that is read aloud, never the full Markdown. */
async function sendChat(text) {
  sayText("you", "me", text);

  const send = $("send");
  send.disabled = true;
  const pending = sayText("crabora", "bot thinking", "thinking…");

  try {
    const r = await api("/api/chat", { prompt: text });
    pending.remove();
    say("crabora", "bot", (b) => {
      renderMarkdown(b, r.reply);
      if (r.tools_used && r.tools_used.length) {
        b.appendChild(el("div", "tools",
          `looked at: ${r.tools_used.join(", ")}`));
      }
    });
    return { reply: r.reply || "", spoken: r.spoken || r.reply || "" };
  } catch (e) {
    pending.remove();
    sayText("crabora", "bot bad", e.message);
    return null;
  } finally {
    send.disabled = false;
    refreshLights();
  }
}

$("composer").addEventListener("submit", async (ev) => {
  ev.preventDefault();
  const input = $("prompt");
  const text = input.value.trim();
  if (!text) return;
  input.value = "";
  const reply = await sendChat(text);
  // In a conversation the loop does the speaking for its own turns; a
  // typed question in the middle of one is answered aloud too.
  if (reply && (speakReplies || talkMode)) speak(reply.spoken);
  input.focus();
});

refreshLights();
setInterval(refreshLights, 10000);
$("prompt").focus();

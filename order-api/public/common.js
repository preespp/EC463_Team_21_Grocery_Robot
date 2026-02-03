const API = "http://localhost:3000";

function setSession(user) {
  sessionStorage.setItem("user", JSON.stringify(user));
}
function getSession() {
  const raw = sessionStorage.getItem("user");
  return raw ? JSON.parse(raw) : null;
}
function clearSession() {
  sessionStorage.removeItem("user");
}

async function apiGet(path) {
  const res = await fetch(API + path);
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}
async function apiPost(path, body) {
  const res = await fetch(API + path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}

function requireLogin() {
  const user = getSession();
  if (!user) window.location.href = "login.html";
  return user;
}

function idleTimeout(ms = 30000) {
  let t = null;
  const reset = () => {
    if (t) clearTimeout(t);
    t = setTimeout(() => (window.location.href = "idle.html"), ms);
  };
  ["click","mousemove","touchstart","keydown"].forEach(e => document.addEventListener(e, reset, {passive:true}));
  reset();
}

function idleTimeoutWithWarnings({
  totalMs,
  warnMs,
  warnText = "Still there?",
  finalText = "Session timed out. Logging out…",
  finalPopupMs = 5000,
  onTimeout = () => (window.location.href = "idle.html"),
}) {
  let idleTimer = null;
  let warnTimer = null;
  let finalTimer = null;

  // overlay element
  let overlay = document.getElementById("idleOverlay");
  if (!overlay) {
    overlay = document.createElement("div");
    overlay.id = "idleOverlay";
    overlay.style.cssText = `
      position:fixed; inset:0; display:none; align-items:center; justify-content:center;
      background:rgba(0,0,0,0.65); z-index:9999; padding:22px; text-align:center;
    `;
    overlay.innerHTML = `
      <div style="
        max-width:720px; width:92%;
        background:rgba(255,255,255,0.08);
        border:1px solid rgba(255,255,255,0.14);
        border-radius:18px; padding:18px;
      ">
        <div id="idleOverlayText" style="font-size:28px; font-weight:900;"></div>
        <div class="muted" style="margin-top:8px;">Touch the screen to continue.</div>
      </div>
    `;
    document.body.appendChild(overlay);
  }
  const overlayText = overlay.querySelector("#idleOverlayText");

  function hideOverlay() {
    overlay.style.display = "none";
  }
  function showOverlay(text) {
    overlayText.textContent = text;
    overlay.style.display = "flex";
  }

  function clearAll() {
    if (idleTimer) clearTimeout(idleTimer);
    if (warnTimer) clearTimeout(warnTimer);
    if (finalTimer) clearTimeout(finalTimer);
    idleTimer = warnTimer = finalTimer = null;
  }

  function armTimers() {
    clearAll();
    hideOverlay();

    // Warn at warnMs (ex: 4 min)
    warnTimer = setTimeout(() => {
      // light warning overlay (doesn't log out)
      showOverlay(warnText);
    }, warnMs);

    // Timeout at totalMs (ex: 5 min)
    idleTimer = setTimeout(() => {
      showOverlay(finalText);
      finalTimer = setTimeout(() => {
        onTimeout();
      }, finalPopupMs);
    }, totalMs);
  }

  function activity() {
    armTimers();
  }

  ["click", "mousemove", "touchstart", "keydown"].forEach((e) =>
    document.addEventListener(e, activity, { passive: true })
  );

  armTimers();

  // return a tiny controller if you ever want it
  return { reset: armTimers, hideOverlay };
}

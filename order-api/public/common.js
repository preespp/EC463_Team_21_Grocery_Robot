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

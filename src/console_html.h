#pragma once

static const char s_consoleHtml[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-S3-DSP Live Console</title>
<link rel="icon" type="image/svg+xml" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'%3E%3Crect width='32' height='32' rx='7' fill='%232E3440'/%3E%3Cpath d='M4 16 C 8 6, 12 6, 16 16 C 20 26, 24 26, 28 16' fill='none' stroke='%2388C0D0' stroke-width='3' stroke-linecap='round'/%3E%3C/svg%3E">
<style>
:root {
  --bg: #1A1E25;
  --card-bg: #232831;
  --card-header: #2B313C;
  --border: #353C49;
  --border-strong: #4A5262;
  --text: #ECEFF4;
  --muted: #9AA5BA;
  --accent: #88C0D0;
  --on-accent: #1A1E25;
  --aurora-red: #BF616A;
  --aurora-orange: #D08770;
  --aurora-yellow: #EBCB8B;
  --aurora-green: #A3BE8C;
  --aurora-purple: #B48EAD;
  --mono: ui-monospace, "SF Mono", "Cascadia Code", "Fira Code", "Liberation Mono", Menlo, Monaco, Consolas, monospace;
}
* { box-sizing: border-box; }
::selection { background: var(--accent); color: var(--on-accent); }
body { font-family: var(--mono); font-variant-numeric: tabular-nums; background: var(--bg); color: var(--text); margin: 0; padding: 14px 16px; display: flex; flex-direction: column; height: 100vh; }
.header { display: flex; justify-content: space-between; align-items: center; padding: 10px 14px; background: var(--card-bg); border: 1px solid var(--border); border-radius: 6px; flex-shrink: 0; flex-wrap: wrap; gap: 10px; }
.title { font-size: 13px; font-weight: 800; letter-spacing: 0.04em; text-transform: uppercase; color: var(--text); display: flex; align-items: center; gap: 10px; }
.controls { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
button, .btn-link { font-family: var(--mono); background: var(--card-header); color: var(--text); border: 1px solid var(--border-strong); padding: 5px 11px; font-size: 11px; font-weight: 700; border-radius: 4px; cursor: pointer; transition: background-color 0.15s ease, border-color 0.15s ease, color 0.15s ease; text-decoration: none; display: inline-flex; align-items: center; }
button:hover, .btn-link:hover { background: #353C49; border-color: var(--accent); }
button:focus-visible, .btn-link:focus-visible, .footer a:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
button.active { background: var(--accent); border-color: var(--accent); color: var(--on-accent); }
button.danger:hover { background: var(--aurora-red); border-color: var(--aurora-red); color: #ECEFF4; }
.btn-link.ota { background: var(--card-header); border-color: var(--accent); color: var(--accent); }
.btn-link.ota:hover { background: var(--accent); color: var(--on-accent); }
.badge { font-size: 10.5px; font-weight: 700; padding: 2px 8px; border-radius: 999px; background: var(--card-header); border: 1px solid var(--border-strong); color: var(--muted); text-transform: none; letter-spacing: 0; }
.badge.green { color: var(--aurora-green); border-color: rgba(163, 190, 140, 0.4); background: rgba(163, 190, 140, 0.12); }
.badge.red { color: var(--aurora-red); border-color: rgba(191, 97, 106, 0.4); background: rgba(191, 97, 106, 0.12); }
.terminal { flex: 1; background: #000000; border: 1px solid var(--border); border-radius: 6px; margin-top: 12px; padding: 12px 14px; font-family: var(--mono); font-size: 12px; line-height: 1.5; overflow-y: auto; white-space: pre-wrap; word-break: break-all; color: #D8DEE9; scrollbar-width: thin; scrollbar-color: var(--border-strong) var(--card-bg); }
.terminal::-webkit-scrollbar { width: 8px; height: 8px; }
.terminal::-webkit-scrollbar-track { background: var(--card-bg); border-radius: 4px; }
.terminal::-webkit-scrollbar-thumb { background: var(--border-strong); border-radius: 4px; }
.terminal::-webkit-scrollbar-thumb:hover { background: var(--accent); }
.log-line { margin: 0; padding: 1px 0; }
.log-line.boot { color: var(--accent); }
.log-line.audio { color: var(--aurora-purple); }
.log-line.slim { color: var(--aurora-green); }
.log-line.warn { color: var(--aurora-yellow); }
.log-line.err { color: var(--aurora-red); }
.footer { display: flex; justify-content: space-between; align-items: center; margin-top: 10px; font-size: 11px; color: var(--muted); flex-shrink: 0; }
.footer code { color: var(--text); }
.footer a { color: var(--accent); text-decoration: none; font-weight: 700; text-underline-offset: 3px; }
.footer a:hover { text-decoration: underline; }
</style>
</head>
<body>
<div class="header">
  <div class="title">
    <span>ESP32-S3-DSP Live Console</span>
    <span id="wsStatus" class="badge red" role="status" aria-live="polite">Connecting...</span>
  </div>
  <div class="controls">
    <button type="button" id="scrollBtn" class="active" onclick="toggleScroll()" aria-pressed="true">Auto-scroll: ON</button>
    <button type="button" onclick="clearLogs()">Clear</button>
    <button type="button" id="copyBtn" onclick="copyLogs()">Copy</button>
    <button type="button" id="rebootBtn" class="danger" onclick="rebootEsp()">Reboot</button>
    <a class="btn-link ota" href="/?ota=1">OTA Update</a>
  </div>
</div>

<div class="terminal" id="term"></div>

<div class="footer">
  <div>Telnet mirror: <code>port 23</code> &bull; WebSocket: <code>/ws</code></div>
  <div><a class="btn-link" href="/">&larr; Return to DSP Dashboard</a></div>
</div>

<script>
let autoScroll = true;
const term = document.getElementById('term');
const statusBadge = document.getElementById('wsStatus');

function toggleScroll() {
  autoScroll = !autoScroll;
  const btn = document.getElementById('scrollBtn');
  btn.innerText = 'Auto-scroll: ' + (autoScroll ? 'ON' : 'OFF');
  btn.className = autoScroll ? 'active' : '';
  btn.setAttribute('aria-pressed', autoScroll ? 'true' : 'false');
}

function clearLogs() {
  term.innerHTML = '';
}

function flashCopyBtn(msg) {
  const btn = document.getElementById('copyBtn');
  if (!btn) return;
  btn.innerText = msg;
  btn.classList.add('active');
  clearTimeout(flashCopyBtn._t);
  flashCopyBtn._t = setTimeout(() => {
    btn.innerText = 'Copy';
    btn.classList.remove('active');
  }, 1500);
}

function copyLogs() {
  const text = term.innerText;
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(text).then(() => flashCopyBtn('Copied!')).catch(() => fallbackCopy(text));
  } else {
    fallbackCopy(text);
  }
}

function fallbackCopy(text) {
  try {
    const ta = document.createElement('textarea');
    ta.value = text;
    document.body.appendChild(ta);
    ta.select();
    document.execCommand('copy');
    document.body.removeChild(ta);
    flashCopyBtn('Copied!');
  } catch (e) {
    flashCopyBtn('Copy failed');
  }
}

let rebootArmed = false;
let rebootArmTimer = null;
function rebootEsp() {
  const btn = document.getElementById('rebootBtn');
  if (!rebootArmed) {
    rebootArmed = true;
    if (btn) {
      btn.innerText = 'Confirm Reboot?';
      btn.style.background = 'var(--aurora-red)';
      btn.style.borderColor = 'var(--aurora-red)';
      btn.style.color = '#ECEFF4';
    }
    clearTimeout(rebootArmTimer);
    rebootArmTimer = setTimeout(() => {
      rebootArmed = false;
      if (btn) {
        btn.innerText = 'Reboot';
        btn.style.background = '';
        btn.style.borderColor = '';
        btn.style.color = '';
      }
    }, 3500);
    return;
  }
  clearTimeout(rebootArmTimer);
  rebootArmed = false;
  if (btn) {
    btn.disabled = true;
    btn.innerText = 'Rebooting...';
  }
  appendLog('[BOOT] Sending reboot command to ESP32-S3...');
  fetch('/api/reboot', { method: 'POST' }).then(res => {
    if (res.ok) {
      statusBadge.className = 'badge red';
      statusBadge.innerText = 'Rebooting (5s)...';
      setTimeout(() => location.reload(), 5000);
    } else {
      if (btn) { btn.disabled = false; btn.innerText = 'Reboot'; btn.style.background = ''; btn.style.borderColor = ''; btn.style.color = ''; }
      appendLog('[ERR] Reboot failed: HTTP ' + res.status);
    }
  }).catch(err => {
    if (btn) { btn.disabled = false; btn.innerText = 'Reboot'; btn.style.background = ''; btn.style.borderColor = ''; btn.style.color = ''; }
    appendLog('[ERR] Reboot network error: ' + err);
  });
}

function appendLog(line) {
  if (!line || !line.trim()) return;
  const el = document.createElement('div');
  el.className = 'log-line';
  if (line.includes('[SLIMPROTO]')) el.classList.add('slim');
  else if (line.includes('[AUDIO]')) el.classList.add('audio');
  else if (line.includes('[BOOT]') || line.includes('[NET]')) el.classList.add('boot');
  else if (line.includes('WARN') || line.includes('warning')) el.classList.add('warn');
  else if (line.includes('ERR') || line.includes('error') || line.includes('FAIL') || line.includes('FATAL')) el.classList.add('err');
  el.innerText = line;
  term.appendChild(el);
  while (term.childElementCount > 1000) {
    term.removeChild(term.firstElementChild);
  }
  if (autoScroll) {
    term.scrollTop = term.scrollHeight;
  }
}

// Fetch historical buffer first
fetch('/api/logs').then(r => {
  if (!r.ok) throw new Error('HTTP ' + r.status);
  return r.text();
}).then(txt => {
  txt.split('\n').forEach(appendLog);
}).catch(err => {
  appendLog('[WARN] Failed to load log history: ' + err.message);
});

// Setup WebSocket
let ws;
function connectWs() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  ws = new WebSocket(protocol + '//' + window.location.host + '/ws');

  ws.onopen = () => {
    statusBadge.className = 'badge green';
    statusBadge.innerText = 'Live';
  };

  ws.onmessage = (e) => {
    try {
      const data = JSON.parse(e.data);
      if (data.t === 'log') {
        appendLog(data.msg);
      }
    } catch(err) {}
  };

  ws.onclose = () => {
    statusBadge.className = 'badge red';
    statusBadge.innerText = 'Disconnected';
    setTimeout(connectWs, 2000);
  };
}
connectWs();
</script>
</body>
</html>
)rawhtml";


#pragma once

static const char s_consoleHtml[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-S3-DSP Live Console</title>
<style>
body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0c0c0e; color: #e1e1e6; margin: 0; padding: 15px; display: flex; flex-direction: column; height: 100vh; box-sizing: border-box; }
.header { display: flex; justify-content: space-between; align-items: center; padding-bottom: 12px; border-bottom: 1px solid #27272a; flex-shrink: 0; flex-wrap: wrap; gap: 10px; }
.title { font-size: 18px; font-weight: 700; color: #a855f7; display: flex; align-items: center; gap: 8px; }
.controls { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
button { background: #27272a; color: #e4e4e7; border: 1px solid #3f3f46; padding: 6px 12px; font-size: 13px; font-weight: 500; border-radius: 6px; cursor: pointer; transition: 0.2s; }
button:hover { background: #3f3f46; }
button.active { background: #7c3aed; border-color: #a855f7; color: #fff; }
button.danger:hover { background: #dc2626; border-color: #ef4444; color: #fff; }
.badge { font-size: 12px; padding: 3px 8px; border-radius: 999px; background: #18181b; border: 1px solid #27272a; color: #a1a1aa; }
.badge.green { color: #4ade80; border-color: #166534; background: #052e16; }
.badge.red { color: #f87171; border-color: #991b1b; background: #450a0a; }
.terminal { flex: 1; background: #141417; border: 1px solid #27272a; border-radius: 8px; margin-top: 12px; padding: 12px; font-family: "JetBrains Mono", Consolas, "Courier New", monospace; font-size: 13px; line-height: 1.45; overflow-y: auto; white-space: pre-wrap; word-break: break-all; color: #d4d4d8; }
.log-line { margin: 0; padding: 1px 0; }
.log-line.boot { color: #38bdf8; }
.log-line.audio { color: #c084fc; }
.log-line.slim { color: #4ade80; }
.log-line.warn { color: #fbbf24; }
.log-line.err { color: #f87171; }
.footer { display: flex; justify-content: space-between; align-items: center; margin-top: 10px; font-size: 12px; color: #71717a; flex-shrink: 0; }
.footer a { color: #a855f7; text-decoration: none; }
.footer a:hover { text-decoration: underline; }
</style>
</head>
<body>
<div class="header">
  <div class="title">
    <span>ESP32-S3-DSP Live Console</span>
    <span id="wsStatus" class="badge red">Connecting...</span>
  </div>
  <div class="controls">
    <button id="scrollBtn" class="active" onclick="toggleScroll()">Auto-scroll: ON</button>
    <button onclick="clearLogs()">Clear</button>
    <button onclick="copyLogs()">Copy</button>
    <button class="danger" onclick="rebootEsp()">Reboot</button>
    <a href="/update"><button style="background: #9333ea; border-color: #a855f7; color: #fff;">OTA Update</button></a>
  </div>
</div>

<div class="terminal" id="term"></div>

<div class="footer">
  <div>Telnet mirror: <code>port 23</code> &bull; WebSocket: <code>/ws</code></div>
  <div><a href="/">&larr; Return to DSP Dashboard</a> &bull; <a href="/update">Firmware Update (OTA) &rarr;</a></div>
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
}

function clearLogs() {
  term.innerHTML = '';
}

function copyLogs() {
  navigator.clipboard.writeText(term.innerText).then(() => alert('Logs copied to clipboard!'));
}

function rebootEsp() {
  if (confirm('Reboot the ESP32-S3 audio processor?')) {
    fetch('/api/reboot', { method: 'POST' }).then(() => {
      alert('Reboot command sent. Reconnecting in 5s...');
      setTimeout(() => location.reload(), 5000);
    });
  }
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
  if (autoScroll) {
    term.scrollTop = term.scrollHeight;
  }
}

// Fetch historical buffer first
fetch('/api/logs').then(r => r.text()).then(txt => {
  txt.split('\n').forEach(appendLog);
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


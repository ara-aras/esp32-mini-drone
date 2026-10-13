#pragma once
#include <Arduino.h>

// Single-page dashboard. Plain HTML/JS, no external resources (works offline on the AP).
const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Drone Bench</title>
<style>
:root{--bg:#101418;--card:#1a2027;--fg:#e6edf3;--mut:#8b949e;--ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff}
*{box-sizing:border-box}body{margin:0;font:14px system-ui,sans-serif;background:var(--bg);color:var(--fg)}
header{position:sticky;top:0;background:#0b0f13;padding:10px 12px;display:flex;gap:8px;align-items:center;flex-wrap:wrap;z-index:2}
h1{font-size:16px;margin:0 auto 0 0}.pill{padding:3px 9px;border-radius:99px;font-weight:600;font-size:12px}
main{padding:10px;display:grid;gap:10px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.card{background:var(--card);border-radius:10px;padding:12px}.card h2{font-size:13px;margin:0 0 8px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}.kv{display:flex;justify-content:space-between;padding:2px 0}.kv b{font-variant-numeric:tabular-nums}
button,a.btn{background:#263040;color:var(--fg);border:1px solid #334155;border-radius:8px;padding:9px 6px;font-size:13px;cursor:pointer;text-align:center;text-decoration:none;display:block}
button:active{transform:scale(.97)}button.danger{background:#5a1d1d;border-color:#8b2a2a}button.pri{background:#1f3a5f;border-color:#2f5d99}
button:disabled{opacity:.35}.btns{display:grid;grid-template-columns:repeat(auto-fill,minmax(95px,1fr));gap:6px}
.motor{background:#11161c;border-radius:8px;padding:6px;text-align:center}.bar{height:70px;background:#0b0f13;border-radius:6px;position:relative;overflow:hidden;margin-top:4px}
.bar i{position:absolute;bottom:0;left:0;right:0;background:var(--acc)}
pre{background:#0b0f13;border-radius:8px;padding:8px;height:260px;overflow:auto;margin:0;font-size:12px;white-space:pre-wrap}
input{background:#0b0f13;color:var(--fg);border:1px solid #334155;border-radius:6px;padding:6px;width:100%}
table{width:100%;border-collapse:collapse}td{padding:2px 4px}td:first-child{color:var(--mut)}
label.safe{display:flex;gap:8px;align-items:center;padding:8px;border-radius:8px;background:#3a2a0a;margin-bottom:8px;font-weight:600}
canvas{width:100%;height:150px;background:#0b0f13;border-radius:8px}.row{display:flex;gap:6px;margin-top:6px}
</style></head><body>
<header><h1>ESP32 Drone Bench</h1><span id="st" class="pill">…</span><span id="lk" class="pill">link</span>
<button class="danger" onclick="cmd('stop')">STOP</button></header>
<main>
<section class="card"><h2>Attitude &amp; system</h2><div class="grid2">
<div><div class="kv">Roll<b id="roll">-</b></div><div class="kv">Pitch<b id="pitch">-</b></div>
<div class="kv">Gyro X<b id="gx">-</b></div><div class="kv">Gyro Y<b id="gy">-</b></div><div class="kv">Gyro Z<b id="gz">-</b></div></div>
<div><div class="kv">Battery<b id="vb">-</b></div><div class="kv">Loop<b id="hz">-</b></div><div class="kv">Compute<b id="wk">-</b></div>
<div class="kv">IMU err<b id="ie">-</b></div><div class="kv">Radio<b id="rx">-</b></div><div class="kv">Throttle<b id="th">-</b></div></div></div></section>

<section class="card"><h2>Motors (top view)</h2><div class="grid2" id="motors"></div></section>

<section class="card"><h2>Tests</h2>
<label class="safe"><input type="checkbox" id="safe" style="width:auto" onchange="safety(this.checked)"> Propellers are REMOVED <span id="ul" style="margin-left:auto;font-weight:400"></span></label>
<div class="btns">
<button class="pri" onclick="cmd('selftest')">Self-test</button>
<button class="pri mot" onclick="cmd('selftest motors')">Self-test + motors</button>
<button onclick="cmd('orient')">Orientation</button>
<button onclick="cmd('cal')">Calibrate gyro</button>
<button onclick="cmd('i2c')">I2C scan</button>
<button onclick="cmd('status')">Status</button>
<button class="mot" onclick="cmd('t1')">M1 FL</button><button class="mot" onclick="cmd('t2')">M2 FR</button>
<button class="mot" onclick="cmd('t4')">M4 RL</button><button class="mot" onclick="cmd('t3')">M3 RR</button>
<button class="mot" onclick="cmd('mall')">All in order</button><button class="mot" onclick="cmd('ramp all')">Ramp all</button>
<button class="mot" onclick="cmd('mix roll')">Mix roll</button><button class="mot" onclick="cmd('mix pitch')">Mix pitch</button>
<button class="mot" onclick="cmd('mix yaw')">Mix yaw</button>
</div></section>

<section class="card"><h2>PID live view</h2>
<div class="row"><button onclick="ax=0">Roll</button><button onclick="ax=1">Pitch</button><button onclick="ax=2">Yaw</button>
<span id="axn" style="margin:auto"></span></div>
<canvas id="cv" width="600" height="150"></canvas>
<div style="font-size:12px;color:var(--mut)"><span style="color:#f0883e">■ setpoint</span> <span style="color:#58a6ff">■ gyro</span> <span style="color:#3fb950">■ PID out ×1000</span> (moves only when armed)</div></section>

<section class="card"><h2>Tuning</h2><table id="tune"></table>
<div class="row"><button class="pri" onclick="applyTune()">Apply</button><button onclick="cmd('save')">Save</button>
<button onclick="cmd('load').then(loadTune)">Load</button><button onclick="cmd('defaults').then(loadTune)">Defaults</button></div></section>

<section class="card"><h2>Flight recorder</h2>
<div style="font-size:12px;color:var(--mut)" id="bbs">records 14 s at 100 Hz while armed; fault disarms are kept in flash</div>
<div class="btns"><a class="btn" id="bbdl" href="/api/blackbox" download>Download last flight</a>
<a class="btn" id="bbsv" href="/api/blackbox?saved=1" download>Download saved fault log</a>
<button onclick="cmd('bb clear')">Delete saved</button></div></section>

<section class="card" style="grid-column:1/-1"><h2>Log</h2><pre id="log"></pre>
<div class="row"><input id="ci" placeholder="command (help)" onkeydown="if(event.key=='Enter'){cmd(this.value);this.value=''}">
<button onclick="document.getElementById('log').textContent=''">Clear</button></div></section>
</main>
<script>
const $=id=>document.getElementById(id);let cur=0,ax=0,hist=[],armed=false,unlock=0,tuneVals={};
const names=['M1 FL CW','M2 FR CCW','M4 RL CCW','M3 RR CW'],order=[0,1,3,2];
$('motors').innerHTML=order.map((k,i)=>`<div class="motor">${names[i]}<div class="bar"><i id="mb${k}"></i></div><b id="mv${k}">0%</b></div>`).join('');
async function cmd(c){if(!c)return;try{await fetch('/api/cmd',{method:'POST',body:c});}catch(e){}pollLog();}
function safety(on){cmd(on?'props off':'lock');}
function pill(el,txt,col){el.textContent=txt;el.style.background=col;}
async function poll(){try{const t=await (await fetch('/api/telem')).json();armed=!!t.armed;unlock=t.unlock;$('bbdl').style.opacity=t.bb?1:.35;$('bbsv').style.opacity=t.bbSaved?1:.35;
$('roll').textContent=t.roll.toFixed(1)+'°';$('pitch').textContent=t.pitch.toFixed(1)+'°';
['gx','gy','gz'].forEach((k,i)=>$(k).textContent=t.g[i].toFixed(1));
$('vb').textContent=t.vbat.toFixed(2)+' V';$('hz').textContent=t.hz+' Hz';$('wk').textContent=t.workUs+' µs';
$('ie').textContent=t.imuErr;$('th').textContent=(t.thr*100).toFixed(0)+'%';
const ok=t.rxAge<500;$('rx').textContent=ok?`${t.src} ${t.rxAge} ms`:'none';pill($('lk'),ok?t.src+' linked':'no controller',ok?'#1f4d2b':'#5a1d1d');
pill($('st'),t.armed?'ARMED':(t.bench?'TESTING':'disarmed'),t.armed?'#8b2a2a':(t.bench?'#6b4d0f':'#263040'));
for(let k=0;k<4;k++){const v=t.m[k];$('mb'+k).style.height=(v*100)+'%';$('mv'+k).textContent=(v*100).toFixed(0)+'%';}
$('ul').textContent=unlock?`unlocked ${unlock}s`:'locked';if(!unlock&&$('safe').checked)$('safe').checked=false;
document.querySelectorAll('.mot').forEach(b=>b.disabled=!unlock||armed);
hist.push([t.sp[ax],t.g[ax],t.pid[ax]*1000]);if(hist.length>150)hist.shift();draw();
}catch(e){pill($('st'),'no connection','#5a1d1d');}}
function draw(){const c=$('cv'),g=c.getContext('2d'),W=c.width,H=c.height;g.clearRect(0,0,W,H);
$('axn').textContent=['Roll','Pitch','Yaw'][ax]+' rate';let m=50;hist.forEach(r=>r.forEach(v=>m=Math.max(m,Math.abs(v))));
g.strokeStyle='#30363d';g.beginPath();g.moveTo(0,H/2);g.lineTo(W,H/2);g.stroke();
['#f0883e','#58a6ff','#3fb950'].forEach((col,j)=>{g.strokeStyle=col;g.beginPath();
hist.forEach((r,i)=>{const x=i*W/149,y=H/2-r[j]/m*(H/2-4);i?g.lineTo(x,y):g.moveTo(x,y);});g.stroke();});}
async function pollLog(){try{const r=await fetch('/api/log?since='+cur);const c=r.headers.get('X-Cursor');if(c)cur=+c;
const txt=await r.text();if(txt){const l=$('log');const bottom=l.scrollTop+l.clientHeight>=l.scrollHeight-5;
l.textContent+=txt;if(l.textContent.length>20000)l.textContent=l.textContent.slice(-15000);if(bottom)l.scrollTop=l.scrollHeight;}}catch(e){}}
async function loadTune(){try{tuneVals=await (await fetch('/api/settings')).json();
$('tune').innerHTML=Object.entries(tuneVals).map(([k,v])=>`<tr><td>${k}</td><td><input id="k_${k}" value="${v}" inputmode="decimal"></td></tr>`).join('');}catch(e){}}
async function applyTune(){for(const k in tuneVals){const v=$('k_'+k).value.trim();if(v!==String(tuneVals[k]))await cmd(`set ${k} ${v}`);}loadTune();}
setInterval(poll,150);setInterval(pollLog,600);loadTune();cmd('help');
</script></body></html>)HTML";

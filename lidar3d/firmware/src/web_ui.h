// Die Seite, die der Scanner selbst ausliefert.
//
// Sie laeuft im Safari am iPhone, das im Accesspoint des ESP32 haengt - also
// OHNE Internet. Deshalb steckt alles hier drin: kein CDN, keine
// Bibliothek, keine Schriftdatei. three.js waere fuer eine Punktwolke ohnehin
// ueberdimensioniert; ein Punkt braucht einen Vertex- und einen
// Fragment-Shader, sonst nichts.
//
// Die Punkte kommen als binaere WebSocket-Frames, gebuendelt. Ein Textframe
// je Punkt - wie in manchen Beispielen zu sehen - waeren bei 5000 Messungen/s
// 5000 Frames pro Sekunde; das haelt weder der ESP32 noch das WLAN durch.
#pragma once

#include <pgmspace.h>

namespace nwl {

static const char kIndexHtml[] PROGMEM = R"HTML(<!doctype html>
<html lang="de"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover,user-scalable=no">
<title>LiDAR 3D</title>
<style>
:root{--ground:#0B1016;--surface:#131C24;--line:#263543;--ink:#E4EAF0;
--muted:#8395A6;--accent:#E8A33D;--warn:#E2574C;--ok:#6FB3A8}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;overflow:hidden;background:var(--ground);
color:var(--ink);font:14px/1.5 -apple-system,BlinkMacSystemFont,sans-serif}
canvas{display:block;width:100%;height:100%;touch-action:none}
.panel{position:fixed;background:rgba(19,28,36,.88);border:1px solid var(--line);
border-radius:3px;backdrop-filter:blur(12px)}
#hud{top:max(12px,env(safe-area-inset-top));left:12px;padding:10px 14px;min-width:170px}
#hud .eyebrow{font:10px/1 ui-monospace,monospace;letter-spacing:.14em;
text-transform:uppercase;color:var(--accent);margin-bottom:8px}
#hud dl{display:grid;grid-template-columns:auto 1fr;gap:2px 12px;margin:0;
font:11px/1.6 ui-monospace,monospace;font-variant-numeric:tabular-nums}
#hud dt{color:var(--muted)}#hud dd{margin:0;text-align:right}
#bar{left:50%;transform:translateX(-50%);bottom:max(12px,env(safe-area-inset-bottom));
display:flex;gap:10px;padding:9px 12px;align-items:center}
button{font:11px/1 ui-monospace,monospace;letter-spacing:.06em;color:var(--ink);
background:#1B2732;border:1px solid var(--line);border-radius:3px;
padding:9px 15px;cursor:pointer}
button:active{background:#243342}
button.on{background:var(--accent);border-color:var(--accent);color:#0B1016}
#note{top:max(12px,env(safe-area-inset-top));right:12px;max-width:min(320px,60vw);
padding:10px 14px;border-left:2px solid var(--warn);display:none}
#note b{display:block;font-size:12px;margin-bottom:3px}
#note span{font-size:12px;color:var(--muted)}
</style></head><body>
<canvas id="c"></canvas>
<div class="panel" id="hud">
  <div class="eyebrow" id="link">verbinde …</div>
  <dl>
    <dt>Punkte</dt><dd id="n">0</dd>
    <dt>Ebenen</dt><dd id="pl">0</dd>
    <dt>Gierwinkel</dt><dd id="yaw">–</dd>
    <dt>Zustand</dt><dd id="st">–</dd>
  </dl>
</div>
<div class="panel" id="note"><b id="nt"></b><span id="nx"></span></div>
<div class="panel" id="bar">
  <button id="go">Sweep</button>
  <button id="stop">Stop</button>
  <button id="clr">Leeren</button>
  <button id="top">Von oben</button>
</div>
<script>
"use strict";
const MAX = 400000;                 // Punkte, danach wird nichts mehr angehaengt
const pos = new Float32Array(MAX*3);
let count = 0, planes = 0;

// -- WebGL -------------------------------------------------------------------
const cv = document.getElementById("c");
const gl = cv.getContext("webgl", {antialias:true, alpha:false});
const VS = `
attribute vec3 p; uniform mat4 mvp; uniform float ps;
uniform float lo, hi; varying float t;
void main(){ gl_Position = mvp*vec4(p,1.0); gl_PointSize = ps;
  t = clamp((p.z-lo)/max(hi-lo,0.001), 0.0, 1.0); }`;
// Farbverlauf ueber die Hoehe: tiefes Blau unten, warmes Gelb oben. Kein
// Regenbogen - der erfindet Kanten, wo keine sind.
const FS = `
precision mediump float; varying float t;
void main(){
  vec3 a = vec3(0.16,0.33,0.55), b = vec3(0.42,0.70,0.62), c = vec3(0.91,0.64,0.24);
  vec3 col = t < 0.5 ? mix(a,b,t*2.0) : mix(b,c,(t-0.5)*2.0);
  gl_FragColor = vec4(col,1.0); }`;
function sh(type,src){const s=gl.createShader(type);gl.shaderSource(s,src);
  gl.compileShader(s);return s}
const prog = gl.createProgram();
gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS));
gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS));
gl.linkProgram(prog); gl.useProgram(prog);
const buf = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, buf);
gl.bufferData(gl.ARRAY_BUFFER, pos.byteLength, gl.DYNAMIC_DRAW);
const aP = gl.getAttribLocation(prog,"p");
gl.enableVertexAttribArray(aP); gl.vertexAttribPointer(aP,3,gl.FLOAT,false,0,0);
const uMvp = gl.getUniformLocation(prog,"mvp"), uPs = gl.getUniformLocation(prog,"ps");
const uLo = gl.getUniformLocation(prog,"lo"), uHi = gl.getUniformLocation(prog,"hi");
gl.clearColor(0.043,0.063,0.086,1); gl.enable(gl.DEPTH_TEST);

// -- Kamera ------------------------------------------------------------------
let yaw = 0.9, pitch = 0.35, dist = 8, dirty = 0;
function resize(){const d=Math.min(devicePixelRatio,2);
  cv.width=innerWidth*d; cv.height=innerHeight*d; gl.viewport(0,0,cv.width,cv.height)}
addEventListener("resize", resize); resize();

function mvp(){
  const a = cv.width/cv.height, f = 1/Math.tan(0.5*1.0), n = 0.05, fa = 200;
  const P = [f/a,0,0,0, 0,f,0,0, 0,0,(fa+n)/(n-fa),-1, 0,0,2*fa*n/(n-fa),0];
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  // Kameraposition auf einer Kugel um den Ursprung, Z ist oben.
  const ex = dist*cp*cy, ey = dist*cp*sy, ez = dist*sp;
  let fx=-ex, fy=-ey, fz=-ez; const fl=Math.hypot(fx,fy,fz); fx/=fl;fy/=fl;fz/=fl;
  let sx=fy*1-fz*0, sy2=fz*0-fx*1, sz=fx*0-fy*0;   // f x (0,0,1)
  sx=fy; sy2=-fx; sz=0; const sl=Math.hypot(sx,sy2,sz)||1; sx/=sl;sy2/=sl;sz/=sl;
  const ux=sy2*fz-sz*fy, uy=sz*fx-sx*fz, uz=sx*fy-sy2*fx;
  const V=[sx,ux,-fx,0, sy2,uy,-fy,0, sz,uz,-fz,0,
    -(sx*ex+sy2*ey+sz*ez), -(ux*ex+uy*ey+uz*ez), (fx*ex+fy*ey+fz*ez), 1];
  const M=new Float32Array(16);
  for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;
    for(let k=0;k<4;k++) s+=P[k*4+j]*V[i*4+k]; M[i*4+j]=s}
  return M;
}

let lo=-1, hi=2;
function draw(){
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  if(count){
    gl.uniformMatrix4fv(uMvp,false,mvp());
    gl.uniform1f(uPs, Math.max(1.5, 3*Math.min(devicePixelRatio,2)));
    gl.uniform1f(uLo, lo); gl.uniform1f(uHi, hi);
    gl.drawArrays(gl.POINTS, 0, count);
  }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

// -- Bedienung ---------------------------------------------------------------
let last=null, pinch=0;
cv.addEventListener("touchstart",e=>{
  if(e.touches.length===1) last={x:e.touches[0].clientX,y:e.touches[0].clientY};
  else if(e.touches.length===2) pinch=Math.hypot(
    e.touches[0].clientX-e.touches[1].clientX, e.touches[0].clientY-e.touches[1].clientY);
},{passive:true});
cv.addEventListener("touchmove",e=>{
  e.preventDefault();
  if(e.touches.length===1 && last){
    yaw -= (e.touches[0].clientX-last.x)*0.007;
    pitch = Math.max(-1.5, Math.min(1.5, pitch+(e.touches[0].clientY-last.y)*0.007));
    last={x:e.touches[0].clientX,y:e.touches[0].clientY};
  } else if(e.touches.length===2){
    const d=Math.hypot(e.touches[0].clientX-e.touches[1].clientX,
                       e.touches[0].clientY-e.touches[1].clientY);
    if(pinch) dist=Math.max(0.5, Math.min(60, dist*pinch/d));
    pinch=d;
  }
},{passive:false});
cv.addEventListener("touchend",()=>{last=null;pinch=0},{passive:true});
cv.addEventListener("mousedown",e=>last={x:e.clientX,y:e.clientY});
addEventListener("mousemove",e=>{if(!last)return;
  yaw-=(e.clientX-last.x)*0.007;
  pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-last.y)*0.007));
  last={x:e.clientX,y:e.clientY}});
addEventListener("mouseup",()=>last=null);
cv.addEventListener("wheel",e=>{e.preventDefault();
  dist=Math.max(0.5,Math.min(60,dist*(1+e.deltaY*0.0015)))},{passive:false});

// -- Verbindung --------------------------------------------------------------
const $=id=>document.getElementById(id);
let ws=null;
function connect(){
  ws = new WebSocket("ws://"+location.hostname+":81/");
  ws.binaryType = "arraybuffer";
  ws.onopen = ()=>{ $("link").textContent = "verbunden"; };
  ws.onclose = ()=>{ $("link").textContent = "getrennt"; setTimeout(connect, 1000); };
  ws.onmessage = ev=>{
    if(typeof ev.data === "string"){ status(JSON.parse(ev.data)); return; }
    const v = new DataView(ev.data);
    if(v.getUint8(0) !== 1) return;
    const n = v.getUint16(2, true);
    for(let i=0; i<n && count<MAX; i++){
      const o = 4 + i*12;
      pos[count*3]   = v.getFloat32(o,   true);
      pos[count*3+1] = v.getFloat32(o+4, true);
      pos[count*3+2] = v.getFloat32(o+8, true);
      const z = pos[count*3+2];
      if(count===0){ lo=z; hi=z; } else { if(z<lo)lo=z; if(z>hi)hi=z; }
      count++;
    }
    // Nur den neuen Abschnitt hochladen, nicht die ganze Wolke.
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferSubData(gl.ARRAY_BUFFER, (count-n)*12, pos.subarray((count-n)*3, count*3));
    $("n").textContent = count;
  };
}
function status(s){
  $("st").textContent = s.state;
  $("pl").textContent = s.planes;
  $("yaw").textContent = s.yaw.toFixed(1)+"°";
  planes = s.planes;
  if(s.fault){ $("note").style.display="block"; $("nt").textContent="Scanner meldet";
    $("nx").textContent = s.fault; }
  else $("note").style.display="none";
  $("go").classList.toggle("on", s.state === "Sweep");
}
const send=t=>{ if(ws && ws.readyState===1) ws.send(t) };
$("go").onclick   = ()=>send("S");
$("stop").onclick = ()=>send("X");
$("clr").onclick  = ()=>{ count=0; lo=-1; hi=2; $("n").textContent="0" };
$("top").onclick  = ()=>{ pitch=1.45; yaw=0.9; dist=8 };
connect();
</script></body></html>)HTML";

}  // namespace nwl

#pragma once

const char armPage[] = R"ARMUI(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Arm Pose Controller</title>
<style>
:root{color-scheme:light;--ink:#183832;--muted:#63736d;--paper:#f3f5ef;--panel:#fff;--line:#d6dfd8;--green:#18765b;--lime:#d4e85c}
*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:16px/1.45 "Trebuchet MS",Arial,sans-serif}
header{background:var(--ink);color:#fff;padding:18px max(20px,calc((100vw - 1040px)/2));display:flex;align-items:center;justify-content:space-between;gap:16px}
h1{font-size:22px;margin:0;font-weight:700}.network{font-size:13px;color:#d8e8d9}main{max-width:1040px;margin:0 auto;padding:24px 20px 48px}
.notice{border-left:4px solid var(--lime);padding:10px 14px;margin-bottom:22px;background:#e9eee4;font-size:14px}
h2{font-size:17px;margin:0 0 14px}.layout{display:grid;grid-template-columns:minmax(240px,320px) 1fr;gap:28px}
.board{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.cell{aspect-ratio:1;border:1px solid var(--line);background:var(--panel);color:var(--ink);font-size:22px;font-weight:700;cursor:pointer;border-radius:4px}
.cell[aria-pressed="true"]{background:var(--green);color:#fff;border-color:var(--green)}.cell.saved::after{content:"  •";color:var(--lime)}
.pickup{width:100%;aspect-ratio:auto;font-size:16px;padding:12px;margin-top:12px}
.section{border-bottom:1px solid var(--line);padding:0 0 22px;margin-bottom:22px}.section:last-child{border-bottom:0}
button{font:inherit}.action{border:0;border-radius:4px;background:var(--green);color:#fff;padding:11px 16px;cursor:pointer;font-weight:700}.action:disabled{opacity:.45;cursor:not-allowed}.secondary{border:1px solid var(--line);border-radius:4px;background:#fff;color:var(--ink);padding:7px 11px;cursor:pointer}
.arm-row{display:flex;align-items:center;gap:14px;flex-wrap:wrap}.state{color:var(--muted);font-size:14px}.motor{display:grid;grid-template-columns:72px 1fr 42px 34px 34px;align-items:center;gap:10px;padding:10px 0;border-bottom:1px solid var(--line)}
.motor:last-child{border-bottom:0}.motor-name{font-weight:700}.motor input{width:100%;accent-color:var(--green)}.angle{text-align:right;font-variant-numeric:tabular-nums}.nudge{width:34px;height:34px;padding:0;font-size:20px}
.test{grid-column:2/-1;justify-self:start}.stop{background:#a52c2c}.secondary:disabled,.cell:disabled{opacity:.45;cursor:not-allowed}
.table-wrap{overflow:auto}table{border-collapse:collapse;width:100%;min-width:580px;font-size:13px}th,td{border-bottom:1px solid var(--line);padding:8px;text-align:center}th:first-child,td:first-child{text-align:left}td.empty{color:#9aa6a0}
textarea{width:100%;min-height:150px;margin-top:12px;padding:12px;border:1px solid var(--line);border-radius:4px;background:#fbfcf9;color:var(--ink);font:12px/1.4 Consolas,monospace}
@media(max-width:720px){header{padding:16px 20px}main{padding:18px 14px 32px}.layout{grid-template-columns:1fr;gap:22px}.board{max-width:340px}.motor{grid-template-columns:62px 1fr 38px 32px 32px;gap:7px}.notice{font-size:13px}}
</style>
</head>
<body>
<header><h1>Arm Pose Controller</h1><div class="network" id="network">Connecting…</div></header>
<main>
<div class="notice">Keep the work area clear. Arm the servos only when ready. Stop if a joint strains. Saved poses are held in RAM and will be lost when the board resets.</div>
<div class="layout">
<section>
<div class="section"><h2>Mapping target</h2><div class="board" id="board"></div><button class="cell pickup" id="pickup" aria-pressed="false">Pickup (outside board)</button><p class="state" id="selected">Selected cell 1</p><p class="state">Select a target, adjust the motors, then save. Selection does not move the arm. Pickup is separate from the existing resting/home pose; it does not automatically grip or place pieces.</p></div>
<div class="section"><h2>Servo control</h2><div class="arm-row"><button class="action" id="arm">Arm servos</button><button class="action stop" id="stop">Stop motion</button><span class="state" id="armState">Disarmed</span></div><p class="state">Arming sets every motor to 90° (50% of the commanded 0–180° range). Initial alignment can be sudden. Support the arm and clear its entire travel first.</p></div>
<div><h2>Saved poses</h2><div class="table-wrap"><table id="poseTable"></table></div><button class="secondary" id="copy">Copy C++ table</button><textarea id="export" readonly aria-label="Exported angle table"></textarea></div>
</section>
<section><div class="section"><h2>Motor angles <span class="state">(0 = D0 … 5 = D5)</span></h2><p class="state">Release a slider to send its angle. Test sweeps one motor slowly to 0°, then 180°, then back to its starting angle (about 29 seconds from 90°). Other motors hold position. Stop if anything strains; keep this page open during the test. Stop motion retains holding torque and depends on Wi-Fi; disconnect servo power if a stop cannot reach the controller. Angles are commands, not measured feedback.</p><p class="state" id="motionState">Idle</p><div id="motors"></div></div>
<div class="arm-row"><button class="action" id="save" disabled>Save this target pose</button><span class="state" id="message">Select a target, then arm the servos.</span></div></section>
</div>
</main>
<script>
const board=document.getElementById('board'),motorList=document.getElementById('motors');
for(let i=1;i<=9;i++){const b=document.createElement('button');b.className='cell';b.textContent=i;b.setAttribute('aria-pressed',i===1);b.onclick=()=>call('/api/select?box='+i);board.appendChild(b)}
document.getElementById('pickup').onclick=()=>call('/api/select?box=10');
function targetName(box){return box===10?'Pickup (outside board)':'cell '+box}
for(let i=0;i<6;i++){motorList.insertAdjacentHTML('beforeend',`<div class="motor"><span class="motor-name">Motor ${i}</span><input id="slider${i}" type="range" min="0" max="180" value="90" disabled><span class="angle" id="angle${i}">90°</span><button class="secondary nudge" data-delta="-5" data-motor="${i}" disabled aria-label="Decrease motor ${i}">−</button><button class="secondary nudge" data-delta="5" data-motor="${i}" disabled aria-label="Increase motor ${i}">+</button><button class="secondary test" data-test="${i}" disabled>Test motor ${i}: 0–180°</button></div>`)}
function status(text){document.getElementById('message').textContent=text}
let requestQueue=Promise.resolve(),pendingRequests=0,controllerState=null,connected=false,pollTimer=null,requestVersion=0,renderedVersion=0;
function updateControls(){const busy=pendingRequests>0,moving=controllerState?.movingMotor>=0,armed=connected&&controllerState?.armed;document.getElementById('arm').disabled=busy||!connected||armed;document.getElementById('save').disabled=busy||moving||!armed;document.querySelectorAll('.cell').forEach(b=>b.disabled=busy||moving||!connected);document.querySelectorAll('#motors input,.nudge,.test').forEach(b=>b.disabled=busy||moving||!armed);if(armed&&!busy&&!moving)document.querySelectorAll('.nudge').forEach(b=>{const angle=controllerState.angles[Number(b.dataset.motor)];b.disabled=Number(b.dataset.delta)>0?angle>=180:angle<=0})}
function call(path,priority=false,background=false){
  clearTimeout(pollTimer);
  if(!background)pendingRequests++;
  updateControls();
  const execute=async()=>{
    const version=++requestVersion;
    const abort=new AbortController(),timeout=setTimeout(()=>abort.abort(),10000);
    try{
      const url=typeof path==='function'?path():path;
      if(!connected&&url!=='/api/state'&&url!=='/api/stop')throw new Error('Reload to reconnect before sending more commands');
      if(controllerState?.movingMotor>=0&&url!=='/api/state'&&url!=='/api/stop')throw new Error('Wait for motion to finish or stop it first');
      if(url!=='/api/state')status('Sending command; waiting for controller...');
      const r=await fetch(url,{cache:'no-store',signal:abort.signal}),data=await r.json();
      if(!r.ok)throw new Error(data.error||'Request failed');
      const jog=url.startsWith('/api/jog?')?new URLSearchParams(url.split('?')[1]):null;
      if(jog&&(!data.armed||data.movingMotor!==Number(jog.get('motor'))||data.targetAngle!==Number(jog.get('angle'))))throw new Error('Controller did not acknowledge the requested motor angle');
      if(version>=renderedVersion){renderedVersion=version;connected=true;render(data);if(jog)status('Motor '+jog.get('motor')+': controller accepted target '+jog.get('angle')+' degrees. Watch live commanded angle.');}
      return data;
    }catch(e){
      if(version>=renderedVersion){renderedVersion=version;connected=false;clearTimeout(pollTimer);document.getElementById('network').textContent='Command failed; reload to reconnect';status(e.name==='AbortError'?'Controller timed out. Motion may have occurred; use Stop motion and check the arm before continuing.':e.message);}
      throw e;
    }finally{
      clearTimeout(timeout);if(!background)pendingRequests--;updateControls();
    }
  };
  const request=priority?execute():requestQueue.then(execute);
  if(!priority)requestQueue=request.catch(()=>{});
  return request;
}
function render(s){controllerState=s;clearTimeout(pollTimer);pollTimer=setTimeout(()=>{call('/api/state',false,true).catch(()=>{})},s.movingMotor>=0?500:1500);document.getElementById('motionState').textContent=s.movingMotor>=0?(s.testing?'Testing':'Moving')+' motor '+s.movingMotor+': '+s.angles[s.movingMotor]+'° → '+s.targetAngle+'°':s.motionStatus;document.getElementById('network').textContent='Device online';document.getElementById('selected').textContent='Selected '+targetName(s.box);document.getElementById('armState').textContent=s.armed?'Armed':'Disarmed';document.getElementById('arm').textContent=s.armed?'Servos armed':'Arm servos';document.querySelectorAll('.cell').forEach((b,i)=>{b.setAttribute('aria-pressed',i+1===s.box);b.classList.toggle('saved',s.saved[i])});
for(let i=0;i<6;i++){const slider=document.getElementById('slider'+i);if(document.activeElement!==slider||s.movingMotor>=0||!s.armed){slider.value=s.angles[i];document.getElementById('angle'+i).textContent=s.angles[i]+'°'}}
let html='<thead><tr><th>Target</th>';for(let m=0;m<6;m++)html+='<th>M'+m+'</th>';html+='</tr></thead><tbody>';for(let b=0;b<10;b++){html+='<tr><th>'+targetName(b+1)+'</th>';for(let m=0;m<6;m++)html+='<td class="'+(s.saved[b]?'':'empty')+'">'+(s.saved[b]?s.poses[b][m]:'—')+'</td>';html+='</tr>'}html+='</tbody>';document.getElementById('poseTable').innerHTML=html;
const poseValues=i=>(s.saved[i]?s.poses[i]:Array(6).fill(-1)).join(', ');
let rows=s.poses.slice(0,9).map((row,i)=>'  {'+poseValues(i)+'}'+(i<8?',':''));document.getElementById('export').value='// Unsaved poses use -1; map and save them before use.\nconst int boxAngles[9][6] = {\n'+rows.join('\n')+'\n};\n\n// Outside-board pickup point; keep the resting/home pose separate.\nconst int pickupAngles[6] = {'+poseValues(9)+'};';status(s.armed?'Adjust one motor at a time, then save the selected target.':'Select a target, then arm the servos.');updateControls()}
document.getElementById('arm').onclick=async()=>{if(!confirm('Arm all six servos at 90 degrees? Initial movement can be sudden. Support the arm and clear its entire travel first.'))return;try{await call('/api/arm')}catch(e){}};
document.getElementById('stop').onclick=async()=>{try{await call('/api/stop',true)}catch(e){}};
document.querySelectorAll('.test').forEach(b=>b.onclick=async()=>{const m=Number(b.dataset.test);if(!confirm('Test motor '+m+' through its FULL 0-180 degree range and return to its starting angle? Clear the whole travel and support the arm. Other motors hold. Stop immediately if a joint strains.'))return;try{await call('/api/test?motor='+m)}catch(e){}});
document.getElementById('save').onclick=async()=>{try{const s=await call('/api/save');status('Pose saved to '+targetName(s.box))}catch(e){}};
for(let i=0;i<6;i++){const slider=document.getElementById('slider'+i);slider.oninput=e=>{document.getElementById('angle'+i).textContent=e.target.value+'° (pending)'};slider.onchange=async e=>{const angle=Number(e.target.value);try{await call('/api/jog?motor='+i+'&angle='+angle)}catch(err){}}}
document.querySelectorAll('.nudge').forEach(b=>b.onclick=async()=>{const m=Number(b.dataset.motor),d=Number(b.dataset.delta);try{await call(()=>'/api/jog?motor='+m+'&angle='+Math.max(0,Math.min(180,controllerState.angles[m]+d)))}catch(e){}});
document.getElementById('copy').onclick=async()=>{try{await navigator.clipboard.writeText(document.getElementById('export').value);status('C++ angle table copied.')}catch(e){document.getElementById('export').select();status('Select and copy the angle table.')}};
call('/api/state').catch(()=>{document.getElementById('network').textContent='Controller not responding';status('Reload this page to reconnect.')});
</script>
</body>
</html>)ARMUI";
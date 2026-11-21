const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const path=require('node:path');

const source=fs.readFileSync(path.join(__dirname,'../apps/arm-dashboard/src/arm_dashboard/static/panel.js'),'utf8');
const page=fs.readFileSync(path.join(__dirname,'../apps/arm-dashboard/src/arm_dashboard/static/panel.html'),'utf8');
const css=fs.readFileSync(path.join(__dirname,'../apps/arm-dashboard/src/arm_dashboard/static/panel.css'),'utf8');

class Element {
  constructor(id='') {
    this.id=id; this.children=[]; this.parentElement=null; this.nextElementSibling=null;
    this.dataset={}; this.style={}; this.attributes={}; this.listeners={}; this.options=[];
    this.hidden=false; this.disabled=false; this.open=false; this.value=''; this.textContent='';
    this.classNames=new Set();this.srcWrites=0;
    this.classList={
      add:(...names)=>names.forEach(name=>this.classNames.add(name)),
      remove:(...names)=>names.forEach(name=>this.classNames.delete(name)),
      contains:name=>this.classNames.has(name),
      toggle:(name,force)=>{const next=force===undefined?!this.classNames.has(name):Boolean(force); next?this.classNames.add(name):this.classNames.delete(name);return next;}
    };
    this._queries={};
  }
  set className(value){this.classNames=new Set(String(value).split(/\s+/).filter(Boolean));}
  get className(){return [...this.classNames].join(' ');}
  set textContent(value){this._textContent=String(value);this.textWrites=(this.textWrites||0)+1;}
  get textContent(){return this._textContent||'';}
  set hidden(value){this._hidden=Boolean(value);this.hiddenWrites=(this.hiddenWrites||0)+1;}
  get hidden(){return this._hidden;}
  set src(value){this.attributes.src=String(value);this.srcWrites++;}
  get src(){return this.attributes.src||'';}
  set innerHTML(value){
    this._html=value; this._queries={}; this.children=[];
    const part=(selector,cls='')=>{const child=new Element(); if(cls)child.className=cls; this._queries[selector]=child; return child;};
    const head=part('.servo-head','servo-head'),name=part('.servo-name','servo-name'),measured=part('.measured','measured');
    head.append(name,measured);
    const row=part('.servo-row','servo-row'),minus=part('.minus','motion minus'),input=part('input','motion target-range'),plus=part('.plus','motion plus');
    input.type='range';this._queries['.target-range']=input;
    row.append(minus,input,plus);this.append(head,row);
  }
  get innerHTML(){return this._html||'';}
  append(...items){for(const item of items){this.children.push(item);if(item&&typeof item==='object')item.parentElement=this;} }
  insertBefore(item,before){const index=this.children.indexOf(before);if(index<0)this.children.unshift(item);else this.children.splice(index,0,item);item.parentElement=this;}
  replaceChildren(...items){this.children=[];this.append(...items);}
  querySelector(selector){if(this._queries[selector])return this._queries[selector];return this.children.find(child=>child.matches?.(selector))||null;}
  querySelectorAll(selector){const found=[];const walk=node=>{for(const child of node.children||[]){if(child.matches?.(selector))found.push(child);walk(child);}};walk(this);return found;}
  matches(selector){if(selector.startsWith('.'))return this.classList.contains(selector.slice(1));if(selector==='input')return this.type==='range';if(selector==='[data-joint]')return Boolean(this.dataset.joint);return false;}
  addEventListener(name,fn){(this.listeners[name]??=[]).push(fn);}
  dispatch(name,event={}){for(const fn of this.listeners[name]||[])fn(event);}
  setPointerCapture(id){this.capturedPointer=id;}
  setAttribute(name,value){this.attributes[name]=String(value);}
  getAttribute(name){return this.attributes[name]??null;}
  hasAttribute(name){return Object.hasOwn(this.attributes,name);}
  removeAttribute(name){delete this.attributes[name];}
  contains(item){if(item===this)return true;return this.children.some(child=>child.contains?.(item));}
  add(option){this.options.push(option);}
  focus(){}
}
function setup(host='100.85.56.14'){
 const objectURLs=[],revokedURLs=[];
 const TestURL=class extends URL {}; TestURL.createObjectURL=()=>{const url='blob:test-'+objectURLs.length;objectURLs.push(url);return url;}; TestURL.revokeObjectURL=url=>revokedURLs.push(url);
 const ids=new Map(),requests=[],timers=[],intervals=[],documentEvents=new Element(),windowEvents=new Element();
 for(const [,id] of page.matchAll(/id="([^"]+)"/g))if(!ids.has(id))ids.set(id,new Element(id));
 for(const [,attributes] of page.matchAll(/<[a-z][a-z0-9-]*\s+([^>]+)>/g)){
  const id=attributes.match(/\bid="([^"]+)"/)?.[1];
  if(id) {
   const node=ids.get(id);
   for(const [,name,value] of attributes.matchAll(/\b(aria-[a-z-]+|role)="([^"]+)"/g))node.setAttribute(name,value);
   node.className=attributes.match(/\bclass="([^"]+)"/)?.[1]||'';
   node.dataset.command=attributes.match(/\bdata-command="([^"]+)"/)?.[1];
  }
 }
 const directions=[...page.matchAll(/data-direction="([^"]+)"/g)].map(([,direction])=>{const button=new Element();button.dataset.direction=direction;return button;});
 ids.get('notice').hidden=true;
 ids.get('cartesian-speed').value='slow';ids.get('cartesian-keyboard').checked=false;ids.get('joint-controls').hidden=true;
 const nav=['overhead','wrist','observer','twin'].map(view=>{const e=new Element();e.dataset.view=view;return e;});
 const document={hidden:false,activeElement:null,body:{dataset:{}},getElementById:id=>ids.get(id)||null,
   createElement:tag=>new Element(),createTextNode:text=>({textContent:text}),addEventListener:(name,fn)=>documentEvents.addEventListener(name,fn),
   querySelector:selector=>selector==='.workbench'?ids.get('primary-camera').parentElement||new Element('workbench'):null,
   querySelectorAll:selector=>{
     if(selector==='[data-view]')return nav;
     if(selector==='[data-direction]')return directions;
     const all=[];const seen=new Set();const walk=node=>{if(!node||seen.has(node))return;seen.add(node);if(node.matches?.(selector))all.push(node);for(const child of node.children||[])walk(child);};
     for(const node of ids.values())walk(node);
     return all;
   }};
 ids.get('primary-camera').parentElement=new Element('workbench');
 ids.get('viewer').parentElement=new Element('viewer-host');
 for(const slot of ['primary','secondary','observer'])ids.get(slot+'-image').nextElementSibling=new Element(slot+'-empty');
 const location={hostname:host,protocol:'http:',href:'http://'+host+':8780/'},media={matches:false,addEventListener(){}};
 const context=vm.createContext({document,window:{location,addEventListener:(name,fn)=>windowEvents.addEventListener(name,fn)},location,URL:TestURL,AbortController,
   matchMedia:()=>media,ResizeObserver:class{observe(){}},
   AbortSignal:{timeout:()=>({})},Option:function(text,value){this.text=text;this.value=value;},
   setTimeout:(fn,ms)=>{timers.push({fn,ms});return timers.length;},setInterval:(fn,ms)=>{intervals.push({fn,ms});return intervals.length;},clearTimeout:id=>{if(timers[id-1])timers[id-1].cancelled=true;},
   fetch:(url,options)=>new Promise((resolve,reject)=>requests.push({url,options,resolve,reject})),confirm:()=>true});
 vm.runInContext(source,context);
 return {context,ids,requests,timers,intervals,nav,directions,media,document,documentEvents,windowEvents,objectURLs,revokedURLs,run:code=>vm.runInContext(code,context)};
}
const response=(request,body,status=200)=>request.resolve({ok:status===200,status,json:async()=>body});
const robotState={
 connected:true,loop_ok:true,age_s:.01,torque:true,speed:30,loop_hz:20,robot:'so101',port:'/dev/ttyACM0',
 joints:['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'],gripper:'gripper',
 limits:{shoulder_pan:[-105,105],shoulder_lift:[-95,95],elbow_flex:[-92,92],wrist_flex:[-95,95],wrist_roll:[-150,150],gripper:[0,100]},
 measured:{shoulder_pan:0,shoulder_lift:0,elbow_flex:0,wrist_flex:0,wrist_roll:0,gripper:0},
 target:{shoulder_pan:0,shoulder_lift:0,elbow_flex:0,wrist_flex:0,wrist_roll:0,gripper:0},
 control:{owner_id:null},faults:[],cameras:{},viewer:{available:true,proxy_url:'/viewer/'},
 operator_commands:[
   'control/claim','control/heartbeat','control/release','stop','emergency-stop','emergency-stop/reset',
   'jog','target','home','speed','torque','cameras','view','cartesian/start','cartesian/renew','cartesian/stop'
 ].map(command=>'/api/'+command),
 emergency_stop:{latched:false,generation:7,hold_status:'sent',hold_error:null}
};
function renderReady(s,overrides={}){
 const state={...robotState,...overrides};
 s.run('state = ' + JSON.stringify(state) + '; online=true; render()');
}
function rootByJoint(s,name){return s.ids.get('joint-list').children.find(root=>root.dataset.joint===name);}

test('build renders all six joint sliders and keeps gripper Open/Close controls',()=>{
 const s=setup();renderReady(s);
 const roots=s.ids.get('joint-list').children;
 assert.equal(roots.length,6);
 assert.deepEqual(roots.map(root=>root.dataset.joint),robotState.joints);
 for(const root of roots)assert.ok(root.querySelector('input'));
 const actions=rootByJoint(s,'gripper').querySelector('.gripper-actions');
 assert.deepEqual(actions.children.map(button=>button.textContent),['Close','Open']);
});

test('the panel disables unsupported commands without sending them',async()=>{
 const s=setup();
 const unsupported=['/api/target','/api/home','/api/cartesian/renew','/api/view'];
 renderReady(s,{operator_commands:robotState.operator_commands.filter(command=>!unsupported.includes(command))});
 s.run("token='t';ownerId='owner';state.control={owner_id:'owner'};enable()");
 const count=s.requests.length;
 const joint=rootByJoint(s,'wrist_roll');
 assert.equal(joint.querySelector('input').disabled,true);
 assert.equal(s.ids.get('home').disabled,true);
 assert.ok(s.directions.every(button=>button.disabled));
 joint.querySelector('input').dispatch('change');
 joint.querySelector('.servo-name').onclick();
 await s.ids.get('home').onclick();
 assert.equal(s.requests.length,count);
});

test('missing lease commands block claim while STOP remains available',async()=>{
 const s=setup();renderReady(s,{operator_commands:['/api/emergency-stop']});
 const count=s.requests.length;
 assert.equal(s.ids.get('claim').disabled,true);
 await s.ids.get('claim').onclick();
 assert.equal(s.requests.length,count);
 const stopping=s.ids.get('stop').onclick();
 assert.equal(s.requests.at(-1).url,'/api/emergency-stop');
 response(s.requests.at(-1),{emergency_stop:{latched:true,generation:8}});
 await stopping;
 assert.equal(s.ids.get('emergency-reset').disabled,true);
});

test('without the local video relay the panel uses same-origin snapshots',()=>{
 const s=setup('localhost');
 renderReady(s,{cameras:{overhead:{status:'live',url:'/cameras/overhead.mjpg'}}});
 assert.equal(s.ids.get('primary-image').src,'http://localhost:8781/cameras/overhead.mjpg');
 s.ids.get('primary-image').dispatch('error');
 assert.equal(s.requests.at(-1).url,'http://localhost:8780/cameras/overhead.jpg');
});

test('the camera center column orders wrist, measured-pose viewer, then observer',()=>{
 const wrist=page.indexOf('id="secondary-camera"'),viewer=page.indexOf('id="twin"'),observer=page.indexOf('id="observer-camera"');
 assert.ok(wrist>=0 && wrist<viewer && viewer<observer);
});

test('selecting a joint updates the read-only highlight request',()=>{
 const s=setup();renderReady(s);
 const button=rootByJoint(s,'elbow_flex').querySelector('.servo-name');
 button.onclick();
 assert.equal(s.requests.at(-1).url,'/api/view');
 assert.deepEqual(JSON.parse(s.requests.at(-1).options.body),{joint:'elbow_flex'});
 assert.equal(button.getAttribute('aria-pressed'),'true');
 assert.equal(rootByJoint(s,'shoulder_pan').querySelector('.servo-name').getAttribute('aria-pressed'),'false');
});

test('slider release sends its joint target through the existing API',()=>{
 const s=setup();renderReady(s,{control:{owner_id:'owner'},emergency_stop:{latched:false,generation:7,hold_status:'sent',hold_error:null}});
 s.run("token='t';ownerId='owner';state.control={owner_id:'owner'};enable()");
 const slider=rootByJoint(s,'wrist_roll').querySelector('input');slider.value='12';
 slider.dispatch('change');
 assert.equal(s.requests.at(-1).url,'/api/target');
 assert.deepEqual(JSON.parse(s.requests.at(-1).options.body),{joint:'wrist_roll',value:12,token:'t'});
});

test('gripper Open and Close send the calibrated endpoint values',()=>{
 const s=setup();renderReady(s,{control:{owner_id:'owner'},emergency_stop:{latched:false,generation:7,hold_status:'sent',hold_error:null}});
 s.run("token='t';ownerId='owner';state.control={owner_id:'owner'};enable()");
 const buttons=rootByJoint(s,'gripper').querySelector('.gripper-actions').children;
 buttons[0].onclick();buttons[1].onclick();
 const close=JSON.parse(s.requests.at(-2).options.body),open=JSON.parse(s.requests.at(-1).options.body);
 assert.equal(close.value,0);assert.equal(open.value,100);
 assert.equal(close.joint,'gripper');assert.equal(open.joint,'gripper');
});

test('claim carries the current generation and a delayed claim cannot undo STOP',async()=>{
 const s=setup();renderReady(s);
 const claim=s.ids.get('claim').onclick();
 assert.equal(s.requests.at(-1).url,'/api/control/claim');
 assert.equal(JSON.parse(s.requests.at(-1).options.body).emergency_stop_generation,7);
 const delayed=s.requests.at(-1);
 const stop=s.ids.get('stop').onclick();
 const stopRequest=s.requests.at(-1);
 assert.equal(stopRequest.url,'/api/emergency-stop');
 assert.deepEqual(JSON.parse(stopRequest.options.body),{});
 response(stopRequest,{emergency_stop:{latched:true,generation:8,hold_status:'sent',hold_error:null}});
 await stop;
 response(delayed,{token:'late',owner_id:'late-owner'});
 await claim;
 assert.equal(s.run('token'),null);
 assert.equal(s.run('state.emergency_stop.latched'),true);
});

test('a delayed reset cannot clear a newer repeated STOP',async()=>{
 const s=setup();renderReady(s,{emergency_stop:{latched:true,generation:8,hold_status:'sent',hold_error:null}});
 const reset=s.ids.get('emergency-reset').onclick();
 const resetRequest=s.requests.at(-1);
 assert.equal(resetRequest.url,'/api/emergency-stop/reset');
 assert.deepEqual(JSON.parse(resetRequest.options.body),{emergency_stop_generation:8});
 const stop=s.ids.get('stop').onclick();
 const stopRequest=s.requests.at(-1);
 assert.equal(stopRequest.url,'/api/emergency-stop');
 response(resetRequest,{emergency_stop:{latched:false,generation:9,hold_status:'sent',hold_error:null}});
 await reset;
 response(stopRequest,{emergency_stop:{latched:true,generation:10,hold_status:'sent',hold_error:null}});
 await stop;
 assert.equal(s.run('state.emergency_stop.latched'),true);
 assert.equal(s.run('state.emergency_stop.generation'),10);
});

test('the localhost dashboard keeps camera frames on the separate 8781 relay',()=>{
 const s=setup('localhost');
 renderReady(s,{cameras:{overhead:{status:'live',url:'/cameras/overhead.mjpg',fps:15},wrist:{status:'unconfigured'},observer:{status:'unconfigured'}},viewer:{available:false,proxy_url:'/viewer/'}});
 assert.equal(s.ids.get('primary-image').src,'http://localhost:8781/cameras/overhead.mjpg');
});

test('direct Tailscale panel camera feeds stay same-origin',()=>{
 const s=setup('100.85.56.14');
 renderReady(s,{cameras:{overhead:{status:'live',url:'/cameras/overhead.mjpg',fps:15},wrist:{status:'unconfigured'},observer:{status:'unconfigured'}},viewer:{available:false,port:8766}});
 assert.equal(s.ids.get('primary-image').src,'/cameras/overhead.mjpg');
});

const cartesianCapability={active:false,motion_id:null,sequence:0,available:true,spatial_available:true,
 profiles:{slow:{linear_m_s:.01,angular_deg_s:5,joint_deg_s:8,gripper_units_s:10},medium:{linear_m_s:.025,angular_deg_s:10,joint_deg_s:15,gripper_units_s:20},fast:{linear_m_s:.05,angular_deg_s:20,joint_deg_s:25,gripper_units_s:35}}};
function cartesianReady(overrides={}){
 const s=setup();renderReady(s,{sequence:42,cartesian:cartesianCapability,...overrides});
 s.run("token='t';ownerId='owner';state.control={owner_id:'owner'};enable()");return s;
}
const direction=(s,name)=>s.directions.find(button=>button.dataset.direction===name);
const pointer=(id=1)=>({button:0,pointerId:id,preventDefault(){}});
const key=(name,options={})=>({key:name,preventDefault(){},...options});
const motions=s=>s.requests.filter(request=>request.url.startsWith('/api/cartesian/'));
const flush=()=>new Promise(resolve=>setImmediate(resolve));
async function acceptStart(s){response(motions(s).at(-1),{cartesian:{active:true,motion_id:'move-1'}});await flush();}

test('Cartesian is the default, keyboard starts off, and rates come from controller metadata',()=>{
 const s=cartesianReady();
 assert.equal(s.ids.get('joint-controls').hidden,true);
 assert.equal(s.ids.get('cartesian-keyboard').checked,false);
 assert.equal(s.directions.length,12);
 assert.match(s.ids.get('cartesian-rates').textContent,/10 mm\/s.*yaw 5.*gripper 10/);
 s.ids.get('cartesian-speed').value='fast';s.run('enable()');
 assert.match(s.ids.get('cartesian-rates').textContent,/50 mm\/s.*yaw 20.*gripper 35/);
 s.ids.get('joint-mode').onclick();
 assert.equal(s.ids.get('joint-controls').hidden,false);
 assert.equal(s.ids.get('cartesian-controls').hidden,true);
 assert.equal(s.ids.get('joint-list').children.length,6);
});

test('pointer hold captures one direction and release fences a pending start',async()=>{
 const s=cartesianReady(),forward=direction(s,'forward');
 forward.dispatch('pointerdown',pointer(8));
 const start=motions(s)[0],payload=JSON.parse(start.options.body);
 assert.deepEqual(payload,{token:'t',sequence:1,direction:'forward',profile:'slow',mode:'normal',observation_sequence:42});
 assert.equal(forward.capturedPointer,8);
 direction(s,'right').dispatch('pointerdown',pointer(9));
 assert.equal(motions(s).length,1);
 forward.dispatch('pointerup',pointer(8));
 const stop=motions(s)[1];assert.equal(stop.url,'/api/cartesian/stop');
 assert.deepEqual(JSON.parse(stop.options.body),{token:'t',sequence:2});
 response(start,{cartesian:{active:true,motion_id:'late'}});await flush();
 assert.equal(s.timers.filter(timer=>timer.ms===100).length,0);
 assert.equal(forward.getAttribute('aria-pressed'),'false');
});

test('renewals are bounded to one request and release cancels an in-flight renewal',async()=>{
 const s=cartesianReady(),up=direction(s,'up');up.dispatch('pointerdown',pointer());await acceptStart(s);
 const renewal=s.timers.find(timer=>timer.ms===100);renewal.fn();renewal.fn();
 assert.equal(motions(s).length,2);
 const renew=motions(s)[1];assert.equal(renew.url,'/api/cartesian/renew');
 assert.deepEqual(JSON.parse(renew.options.body),{token:'t',sequence:2,motion_id:'move-1'});
 up.dispatch('pointerup',pointer());
 assert.deepEqual(JSON.parse(motions(s)[2].options.body),{token:'t',sequence:3,motion_id:'move-1'});
 response(renew,{cartesian:{active:true,motion_id:'move-1'}});await flush();
 assert.equal(s.timers.filter(timer=>timer.ms===100).length,1);
});

for(const cancellation of ['pointercancel','lostpointercapture','blur','hidden','pagehide','joint-mode','model-tab','speed','keyboard-off','ownership-loss','STOP']){
 test(cancellation+' cancels pending Cartesian motion and a late start cannot renew',async()=>{
  const s=cartesianReady(),button=direction(s,'forward');button.dispatch('pointerdown',pointer());
  const start=motions(s)[0];
  if(['pointercancel','lostpointercapture'].includes(cancellation))button.dispatch(cancellation,pointer());
  else if(cancellation==='blur'||cancellation==='pagehide')s.windowEvents.dispatch(cancellation);
  else if(cancellation==='hidden'){s.document.hidden=true;s.documentEvents.dispatch('visibilitychange');}
  else if(cancellation==='speed')s.ids.get('cartesian-speed').onchange();
  else if(cancellation==='keyboard-off')s.ids.get('cartesian-keyboard').onchange();
  else if(cancellation==='ownership-loss')s.run("state.control={owner_id:'other'};enable()");
  else if(cancellation==='STOP')s.ids.get('stop').onclick();
  else s.ids.get(cancellation).onclick();
  assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
  assert.ok(JSON.parse(motions(s).at(-1).options.body).sequence>JSON.parse(start.options.body).sequence);
  response(start,{cartesian:{active:true,motion_id:'late'}});await flush();
  assert.equal(s.timers.filter(timer=>timer.ms===100).length,0);
 });
}

test('keyboard requires opt-in, ignores typing/modifiers/repeats, and stops on key release',async()=>{
 const s=cartesianReady();s.documentEvents.dispatch('keydown',key('w'));assert.equal(motions(s).length,0);
 s.ids.get('cartesian-keyboard').checked=true;
 for(const options of [{repeat:true},{ctrlKey:true},{altKey:true},{metaKey:true},{shiftKey:true},{target:{closest:()=>({})}}])s.documentEvents.dispatch('keydown',key('w',options));
 assert.equal(motions(s).length,0);
 for(const [name,expected] of Object.entries({w:'forward',s:'back',a:'left',d:'right',r:'up',f:'down',q:'open',e:'close'})){
  s.documentEvents.dispatch('keydown',key(name));
  assert.equal(JSON.parse(motions(s).at(-1).options.body).direction,expected);
  await acceptStart(s);
  s.documentEvents.dispatch('keyup',key(name));assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
 }
 s.documentEvents.dispatch('keydown',key('Escape'));
 assert.equal(s.requests.at(-1).url,'/api/emergency-stop');
});

test('claim then enable keyboard works while the checkbox retains focus',async()=>{
 const s=setup();renderReady(s,{sequence:42,cartesian:cartesianCapability});
 const claim=s.ids.get('claim').onclick();
 response(s.requests.at(-1),{token:'t',owner_id:'owner'});await claim;
 const checkbox=s.ids.get('cartesian-keyboard');
 checkbox.checked=true;checkbox.closest=()=>checkbox;
 s.document.activeElement=checkbox;checkbox.onchange();
 let prevented=false;
 s.documentEvents.dispatch('keydown',key('w',{target:checkbox,preventDefault(){prevented=true;}}));
 assert.equal(motions(s).length,1);
 assert.equal(JSON.parse(motions(s)[0].options.body).direction,'forward');
 assert.equal(prevented,true);
 await acceptStart(s);
 s.documentEvents.dispatch('keyup',key('w',{target:checkbox}));
 assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
 for(const name of [' ','Tab']){
  prevented=false;
  s.documentEvents.dispatch('keydown',key(name,{target:checkbox,preventDefault(){prevented=true;}}));
  assert.equal(prevented,false,'native checkbox navigation must remain available');
 }
 assert.equal(motions(s).length,2);
});

test('keyboard checkbox exception does not enable movement while editing other fields',()=>{
 const s=cartesianReady();s.ids.get('cartesian-keyboard').checked=true;
 for(const type of ['text','number','range','checkbox','textarea','select','contenteditable']){
  const field=new Element();field.type=type;field.closest=()=>field;
  s.document.activeElement=field;
  s.documentEvents.dispatch('keydown',key('w',{target:field}));
 }
 assert.equal(motions(s).length,0);
});

test('spatial controls fail closed without solver/floor while gripper remains available',()=>{
 const s=cartesianReady({cartesian:{...cartesianCapability,spatial_available:false,reason:'Configure the workspace floor.'}});
 for(const name of ['forward','back','up','down','left','right']){
  assert.equal(direction(s,name).disabled,true);direction(s,name).dispatch('pointerdown',pointer());
 }
 assert.equal(motions(s).length,0);assert.equal(direction(s,'open').disabled,false);
 direction(s,'open').dispatch('pointerdown',pointer());assert.equal(motions(s).length,1);
});

test('expired or failed renewals end the hold without automatically restarting',async()=>{
 for(const failure of ['expired','network']){
  const s=cartesianReady();direction(s,'forward').dispatch('pointerdown',pointer());await acceptStart(s);
  s.timers.find(timer=>timer.ms===100).fn();const renew=motions(s).at(-1);
  if(failure==='expired')response(renew,{cartesian:{active:false,motion_id:null}});else renew.reject(new Error('lost'));
  await flush();assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
  assert.equal(motions(s).filter(request=>request.url.endsWith('/start')).length,1);
  assert.equal(s.timers.filter(timer=>timer.ms===100).length,1);
 }
});

test('start timeout sends a tombstone; stale polls do not cancel an accepted hold',async()=>{
 const failed=cartesianReady();direction(failed,'forward').dispatch('pointerdown',pointer());
 motions(failed)[0].reject(new Error('timeout'));await flush();
 assert.equal(motions(failed).at(-1).url,'/api/cartesian/stop');
 assert.equal(failed.timers.filter(timer=>timer.ms===100).length,0);
 const s=cartesianReady();direction(s,'forward').dispatch('pointerdown',pointer());await acceptStart(s);
 s.run('enable()');assert.equal(motions(s).length,1);
 s.run('state.cartesian={...state.cartesian,sequence:1,active:false};enable()');
 assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
});

async function claimSession(s,name){
 const claim=s.ids.get('claim').onclick();
 response(s.requests.at(-1),{token:name,owner_id:name+'-owner'});await claim;
}
function releaseSession(s){
 s.ids.get('release').onclick();
 s.run('state.control={owner_id:null};enable()');
}
const heartbeats=s=>s.requests.filter(request=>request.url==='/api/control/heartbeat');

for(const outcome of ['403','timeout','success']){
 test('a stale heartbeat '+outcome+' cannot affect the new owner or its pending heartbeat',async()=>{
  const s=setup();renderReady(s);await claimSession(s,'old');
  const oldHeartbeat=heartbeats(s)[0];
  releaseSession(s);await claimSession(s,'new');
  assert.deepEqual(heartbeats(s).map(request=>JSON.parse(request.options.body).token),['old','new'],'new claim starts its heartbeat immediately');
  const newHeartbeat=heartbeats(s)[1];
  const status=s.ids.get('notice-message');s.run("notice('Current session message','motion')");
  if(outcome==='timeout')oldHeartbeat.reject(new Error('request timed out'));
  else response(oldHeartbeat,outcome==='403'?{error:'old token expired'}:{},outcome==='403'?403:200);
  await flush();
  assert.equal(s.run('token'),'new');assert.equal(s.run('online'),true);
  assert.equal(status.textContent,'Current session message');assert.equal(s.ids.get('notice').hidden,false);
  const tick=s.intervals.find(timer=>timer.ms===400).fn;
  tick();assert.equal(heartbeats(s).length,2,'old finally must not clear the new heartbeat in flight');
  response(newHeartbeat,{});await flush();
  tick();assert.equal(heartbeats(s).length,3);
  assert.equal(JSON.parse(heartbeats(s)[2].options.body).token,'new');
 });
}

for(const outcome of ['401','403','409','timeout','success']){
 test('an authenticated manual POST '+outcome+' from a released session cannot change the current session',async()=>{
  const s=setup();renderReady(s);await claimSession(s,'same-token');
  const oldCommand=s.ids.get('home').onclick(),request=s.requests.at(-1);
  assert.equal(request.url,'/api/home');
  releaseSession(s);await claimSession(s,'same-token');
  const status=s.ids.get('notice-message');s.run("notice('New session message','motion')");
  if(outcome==='timeout')request.reject(new Error('request timed out'));
  else response(request,outcome==='success'?{}:{error:'stale command'},outcome==='success'?200:Number(outcome));
  await oldCommand;
  assert.equal(s.run('token'),'same-token');assert.equal(s.run('online'),true);
  assert.equal(status.textContent,'New session message');assert.equal(s.ids.get('notice').hidden,false);
 });
}

test('a current session heartbeat failure still revokes local control',async()=>{
 const s=setup();renderReady(s);await claimSession(s,'current');
 heartbeats(s)[0].reject(new Error('connection lost'));await flush();
 assert.equal(s.run('token'),null);assert.equal(s.run('online'),false);
 assert.match(s.ids.get('notice-message').textContent,/Connection lost/);
});

const readyRanges={status:'ready',observation_sequence:42,age_s:.1,orientation:'fixed_tilt',resolution_m:.0005,horizon_m:.4,
 directions:{
  back:{allowed:true,distance_m:.1,limiting_reason:'Joint limit',limiting_joint:'elbow_flex',at_least:false},
  forward:{allowed:true,distance_m:.4,limiting_reason:null,limiting_joint:null,at_least:true},
  down:{allowed:false,distance_m:0,limiting_reason:'Configured TCP floor.',limiting_joint:null,at_least:false},
  up:{allowed:true,distance_m:.0255,limiting_reason:'Local solver / position-orientation boundary.',limiting_joint:null,at_least:false}
 }};
function rangeReady(ranges=readyRanges){return cartesianReady({cartesian:{...cartesianCapability,ranges:structuredClone(ranges)}});}

test('travel axes show current-tilt distances, scale, horizon and actual boundary reasons',()=>{
 const s=rangeReady();
 assert.equal(s.ids.get('range-back').textContent,'~100 mm');
 assert.equal(s.ids.get('range-forward').textContent,'≥400 mm');
 assert.equal(s.ids.get('range-up').textContent,'~25.5 mm');
 assert.equal(s.ids.get('range-down').textContent,'~0 mm');
 assert.equal(s.ids.get('range-back-fill').style.width,'12.5%');
 assert.equal(s.ids.get('range-forward-fill').style.width,'50%');
 assert.equal(s.ids.get('range-down-fill').style.width,'0%');
 assert.match(s.ids.get('range-back-reason').textContent,/Joint limit.*Elbow flex/);
 assert.equal(s.ids.get('range-down-reason').textContent,'Minimum tip height.');
 assert.equal(s.ids.get('range-down-reason').title,'Configured TCP floor.');
 assert.equal(s.ids.get('range-up-reason').textContent,'No further path found at this tilt.');
 assert.equal(s.ids.get('range-up-reason').title,'Local solver / position-orientation boundary.');
 assert.match(s.ids.get('cartesian-range-context').textContent,/Tip travel estimate/);
 assert.match(s.ids.get('cartesian-range-scale').textContent,/400 mm/);
 assert.match(s.ids.get('range-reach-track').getAttribute('aria-label'),/Back ~100 mm; Forward ≥400 mm/);
 assert.equal(s.ids.get('range-down-side').dataset.boundary,'blocked');
 assert.equal(direction(s,'down').disabled,false,'estimate cannot replace the guarded motion controller');
});

for(const status of ['pending','stale','unavailable']){
 test(status+' travel estimate clears previous distances and bars',()=>{
  const s=rangeReady();
  s.run('state.cartesian.ranges.status='+JSON.stringify(status)+';enable()');
  for(const name of ['back','forward','up','down']){
   assert.equal(s.ids.get('range-'+name).textContent,'—');
   assert.equal(s.ids.get('range-'+name+'-fill').style.width,'0%');
   assert.equal(s.ids.get('range-'+name+'-reason').textContent,'—');
  }
  assert.notEqual(s.ids.get('cartesian-range-status').textContent,'Distances from the current arm pose.');
 });
}

test('absent travel estimate and stale feedback never display previous numbers as current',()=>{
 const s=cartesianReady();assert.equal(s.ids.get('range-back').textContent,'—');
 assert.match(s.ids.get('cartesian-range-status').textContent,/Waiting/);
 const ready=rangeReady();ready.run('state.age_s=2;enable()');
 assert.equal(ready.ids.get('range-forward').textContent,'—');
 assert.match(ready.ids.get('cartesian-range-status').textContent,/Fresh arm feedback/);
 const old=rangeReady();old.run('state.cartesian.ranges.age_s=.8;enable()');
 assert.equal(old.ids.get('range-forward').textContent,'—');
 assert.match(old.ids.get('cartesian-range-status').textContent,/Updating/);
});

test('malformed or incompatible range values cannot overflow axes or appear as a usable distance',()=>{
 const s=rangeReady();
 for(const value of [-.1,.5,null]){
  s.run('state.cartesian.ranges.directions.forward.distance_m='+JSON.stringify(value)+';enable()');
  assert.equal(s.ids.get('range-forward').textContent,'—');
  assert.equal(s.ids.get('range-forward-fill').style.width,'0%');
 }
 s.run('state.cartesian.ranges.horizon_m=0;enable()');
 assert.equal(s.ids.get('range-back').textContent,'—');
 const other=rangeReady();other.run('state.cartesian.ranges.orientation="free_tilt";enable()');
 assert.equal(other.ids.get('range-back').textContent,'—');
});

const cameraDelivery={default_fps:2,available_fps:[2,10,15],max_fps:15};
const liveCameras=Object.fromEntries(['overhead','wrist','observer'].map(name=>[name,{status:'live',fps:25,url:'/cameras/'+name+'.mjpg?consumer=operator'}]));
function choosePreview(s,fps){s.ids.get('video-preview').value=String(fps);s.ids.get('video-preview').onchange();}

test('preview setting requires server metadata and defaults to the existing 2fps route',()=>{
 const direct=setup();renderReady(direct,{cameras:liveCameras});
 assert.equal(direct.ids.get('video-preview-setting').hidden,true);
 assert.equal(direct.ids.get('primary-image').src,'/cameras/overhead.mjpg?consumer=operator');
 assert.match(direct.ids.get('primary-status').textContent,/capture 25 fps/);
 choosePreview(direct,15);assert.equal(direct.ids.get('primary-image').srcWrites,1);
 const dashboard=setup();renderReady(dashboard,{cameras:liveCameras,camera_delivery:cameraDelivery});
 assert.equal(dashboard.ids.get('video-preview-setting').hidden,false);
 assert.equal(dashboard.ids.get('video-preview').value,'2');
 assert.equal(dashboard.ids.get('primary-image').src,'/cameras/overhead.mjpg?consumer=operator');
 assert.match(dashboard.ids.get('primary-status').textContent,/target 2 fps/);
});

test('preview selection preserves URL parameters, is per browser, and never reassigns cameras',()=>{
 const s=setup(),other=setup();
 for(const browser of [s,other])renderReady(browser,{cameras:liveCameras,camera_delivery:cameraDelivery});
 choosePreview(s,10);
 assert.equal(s.ids.get('primary-image').src,'/cameras/overhead.mjpg?consumer=operator&fps=10');
 assert.equal(other.ids.get('primary-image').src,'/cameras/overhead.mjpg?consumer=operator');
 assert.match(s.ids.get('primary-status').textContent,/target 10 fps/);
 assert.match(s.ids.get('primary-status').title,/Capture: 25 fps.*Preview target: 10 fps/);
 for(let i=0;i<3;i++)s.run('render()');
 choosePreview(s,10);assert.equal(s.ids.get('primary-image').srcWrites,2,'unchanged choice and polling must not reconnect');
 choosePreview(s,15);assert.match(s.ids.get('primary-image').src,/fps=15/);
 choosePreview(s,2);assert.match(s.ids.get('primary-image').src,/fps=2/);
 assert.equal(s.requests.filter(request=>request.options?.method==='POST').length,0);
});

test('localhost preview keeps the dedicated 8781 relay and replaces an existing fps parameter',()=>{
 const s=setup('localhost');
 renderReady(s,{cameras:{...liveCameras,overhead:{...liveCameras.overhead,url:'/cameras/overhead.mjpg?fps=2&consumer=operator'}},camera_delivery:cameraDelivery,viewer:{available:false,proxy_url:'/viewer/'}});
 choosePreview(s,15);
 const url=new URL(s.ids.get('primary-image').src);
 assert.equal(url.origin,'http://localhost:8781');
 assert.equal(url.searchParams.get('consumer'),'operator');
 assert.deepEqual(url.searchParams.getAll('fps'),['15']);
});

test('enlarging the viewer pauses hidden camera streams and restoring reconnects each once',()=>{
 const s=setup();renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery});
 const primary=s.ids.get('primary-image'),secondary=s.ids.get('secondary-image'),observer=s.ids.get('observer-image');
 s.ids.get('enlarge-viewer').onclick();
 assert.equal(primary.hasAttribute('src'),false);assert.equal(secondary.hasAttribute('src'),false);
 assert.equal(observer.hasAttribute('src'),true);
 for(let i=0;i<3;i++)s.run('render()');
 assert.equal(primary.srcWrites,1);assert.equal(secondary.srcWrites,1);
 choosePreview(s,10);assert.equal(primary.hasAttribute('src'),false);
 s.ids.get('enlarge-viewer').onclick();
 assert.equal(primary.srcWrites,2);assert.equal(secondary.srcWrites,2);
 assert.match(primary.src,/fps=10/);
 s.run('render()');assert.equal(primary.srcWrites,2);
 assert.equal(s.requests.filter(request=>request.options?.method==='POST').length,0);
});

test('mobile camera navigation subscribes only to the visible view',()=>{
 const s=setup();s.media.matches=true;renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery});
 const primary=s.ids.get('primary-image'),secondary=s.ids.get('secondary-image'),observer=s.ids.get('observer-image');
 assert.equal(primary.hasAttribute('src'),true);assert.equal(secondary.hasAttribute('src'),false);assert.equal(observer.hasAttribute('src'),false);
 s.nav.find(button=>button.dataset.view==='wrist').onclick();
 assert.equal(primary.hasAttribute('src'),false);assert.equal(secondary.hasAttribute('src'),true);
 s.nav.find(button=>button.dataset.view==='wrist').onclick();assert.equal(secondary.srcWrites,1);
 s.nav.find(button=>button.dataset.view==='observer').onclick();
 assert.equal(secondary.hasAttribute('src'),false);assert.equal(observer.hasAttribute('src'),true);
 s.nav.find(button=>button.dataset.view==='twin').onclick();
 assert.equal(observer.hasAttribute('src'),false);
 assert.equal(s.requests.filter(request=>request.options?.method==='POST').length,0);
});

const previousPoseRanges={...readyRanges,status:'stale',stale_reason:'stale_pose',age_s:1.5,computed_age_s:1.2,
 source_pose:{...robotState.measured},source_observation_sequence:41,latest_pose_delta_deg:.0879};

test('completed previous-pose ranges remain visibly advisory with age and source identity',()=>{
 const s=rangeReady(previousPoseRanges);
 assert.equal(s.ids.get('range-back').textContent,'~100 mm');
 assert.equal(s.ids.get('range-forward').textContent,'≥400 mm');
 assert.equal(s.ids.get('range-back-fill').style.width,'12.5%');
 assert.equal(s.ids.get('cartesian-ranges').dataset.state,'previous');
 assert.match(s.ids.get('cartesian-range-status').textContent,/Updating — estimate from previous pose, 1.2s ago/);
 assert.match(s.ids.get('cartesian-range-context').textContent,/sampled tilt/);
 assert.match(s.ids.get('cartesian-range-context').title,/Source observation 41.*Shoulder pan/);
 assert.match(s.ids.get('range-reach-track').getAttribute('aria-label'),/^Previous pose:/);
 assert.equal(s.ids.get('range-down-side').dataset.boundary,'clear','old estimate cannot claim the current direction is blocked');
 assert.equal(direction(s,'down').disabled,false);
 s.run('state.cartesian.ranges.computed_age_s=3;enable()');
 assert.equal(s.ids.get('range-back').textContent,'~100 mm');
 s.run('state.cartesian.ranges.computed_age_s=3.01;enable()');
 assert.equal(s.ids.get('range-back').textContent,'—');
});

test('previous-pose display requires trusted source metadata and an explicit pose-only stale reason',()=>{
 for(const override of [{stale_reason:'stale_feedback'},{stale_reason:null},{source_pose:null},{source_pose:{shoulder_pan:0}},{source_observation_sequence:null},{computed_age_s:null},{computed_age_s:-1},{computed_age_s:3.1}]){
  const s=rangeReady({...previousPoseRanges,...override});
  assert.equal(s.ids.get('range-forward').textContent,'—');
  assert.equal(s.ids.get('range-forward-fill').style.width,'0%');
  assert.equal(s.ids.get('cartesian-ranges').dataset.state,'unavailable');
 }
});

test('stale robot feedback or lost network clears even an otherwise recent prior-pose estimate',()=>{
 for(const change of ['state.age_s=2','state.connected=false','online=false']){
  const s=rangeReady(previousPoseRanges);s.run(change+';enable()');
  assert.equal(s.ids.get('range-forward').textContent,'—');
  assert.equal(s.ids.get('range-forward-fill').style.width,'0%');
  assert.doesNotMatch(s.ids.get('cartesian-range-status').textContent,/estimate from previous pose/);
 }
});

test('an exact fresh estimate returns to normal current-pose styling',()=>{
 const s=rangeReady(previousPoseRanges);
 s.run('state.cartesian.ranges='+JSON.stringify(readyRanges)+';enable()');
 assert.equal(s.ids.get('cartesian-ranges').dataset.state,'current');
 assert.equal(s.ids.get('cartesian-range-status').textContent,'Distances from the current arm pose.');
 assert.equal(s.ids.get('range-down-side').dataset.boundary,'blocked');
 assert.equal(s.ids.get('cartesian-range-context').title,'');
});

for(const outcome of ['timeout','403','success']){
 test('a cancelled claim '+outcome+' cannot invalidate a later successful claim',async()=>{
  const s=setup();renderReady(s);
  const oldClaim=s.ids.get('claim').onclick(),oldRequest=s.requests.at(-1);
  releaseSession(s);await claimSession(s,'new');
  const status=s.ids.get('notice-message');s.run("notice('Current session message','motion')");
  if(outcome==='timeout')oldRequest.reject(new Error('claim timed out'));
  else response(oldRequest,outcome==='success'?{token:'old',owner_id:'old-owner'}:{error:'claim expired'},outcome==='success'?200:403);
  await oldClaim;
  assert.equal(s.run('token'),'new');assert.equal(s.run('online'),true);
  assert.equal(status.textContent,'Current session message');assert.equal(s.ids.get('notice').hidden,false);
 });
}

const recoveryCapability={supported:true,max_soft_violation_deg:1,max_tip_drift_m:.002,max_pitch_change_deg:3,
 phase:null,pitch_change_deg:null,tip_drift_m:null,reason:null};
function recoveryReady(){return cartesianReady({cartesian:{...cartesianCapability,recovery:recoveryCapability,ranges:structuredClone(readyRanges)}});}

test('Recovery is an explicit default-off switch that requires backend support',()=>{
 const unsupported=cartesianReady();
 assert.equal(unsupported.ids.get('cartesian-recovery-setting').hidden,true);
 unsupported.ids.get('cartesian-recovery').onclick();
 assert.equal(unsupported.ids.get('cartesian-recovery').getAttribute('aria-checked'),'false');
 const s=recoveryReady();
 assert.equal(s.ids.get('cartesian-recovery-setting').hidden,false);
 assert.equal(s.ids.get('cartesian-recovery').getAttribute('aria-checked'),'false');
 assert.match(page, /id="cartesian-recovery"[^>]*role="switch"[^>]*aria-checked="false"/);
 assert.match(s.ids.get('cartesian-recovery-help').textContent,/3° total tilt.*2 mm.*1° joint overshoot.*selected speed/);
});

test('explicit Recovery uses the selected speed, while pan and gripper remain normal',async()=>{
 for(const [directionName,expectedMode] of [['forward','recovery'],['up','recovery'],['left','normal'],['open','normal']]){
  const s=recoveryReady();s.ids.get('cartesian-recovery').onclick();
  s.ids.get('cartesian-speed').value='fast';s.ids.get('cartesian-speed').onchange();
  direction(s,directionName).dispatch('pointerdown',pointer());
  const body=JSON.parse(motions(s)[0].options.body);
  assert.equal(body.mode,expectedMode);assert.equal(body.profile,'fast');
  assert.match(s.ids.get('cartesian-range-context').textContent,/Normal mode at current tilt/);
  await acceptStart(s);
 }
});

test('Recovery switch cancels pending motion and never resumes after a delayed response',async()=>{
 const s=recoveryReady();direction(s,'forward').dispatch('pointerdown',pointer());
 const pending=motions(s)[0];s.ids.get('cartesian-recovery').onclick();
 assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
 assert.equal(s.ids.get('cartesian-recovery').getAttribute('aria-checked'),'true');
 response(pending,{cartesian:{active:true,motion_id:'late'}});await flush();
 assert.equal(s.timers.filter(timer=>timer.ms===100).length,0);
 assert.equal(motions(s).filter(request=>request.url.endsWith('/start')).length,1);
 direction(s,'forward').dispatch('pointerdown',pointer());
 assert.equal(JSON.parse(motions(s).at(-1).options.body).mode,'recovery');
});

test('Recovery shows ingress, tracking deviations and the controller stop reason',async()=>{
 const s=recoveryReady();s.ids.get('cartesian-recovery').onclick();
 direction(s,'up').dispatch('pointerdown',pointer());
 response(motions(s)[0],{cartesian:{...cartesianCapability,active:true,motion_id:'move-1',mode:'recovery',phase:'ingress',recovery:{...recoveryCapability,phase:'ingress',pitch_change_deg:.2,tip_drift_m:.0005}}});await flush();
 assert.match(s.ids.get('cartesian-recovery-status').textContent,/Returning joint inside limits/);
 assert.match(s.ids.get('cartesian-recovery-progress').textContent,/0.2°.*0.5 mm/);
 s.run('state.cartesian='+JSON.stringify({...cartesianCapability,active:true,motion_id:'move-1',sequence:1,mode:'recovery',phase:'tracking',recovery:{...recoveryCapability,phase:'tracking',pitch_change_deg:1.5,tip_drift_m:.001}})+';enable()');
 assert.match(s.ids.get('cartesian-recovery-status').textContent,/Recovery moving/);
 s.run('state.cartesian.active=false;state.cartesian.phase=null;state.cartesian.recovery.reason="Total pitch budget reached.";enable()');
 assert.match(s.ids.get('cartesian-recovery-status').textContent,/Total pitch budget reached/);
 assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
 assert.equal(motions(s).filter(request=>request.url.endsWith('/start')).length,1);
});

test('Recovery remains usable with opted-in keyboard while its switch is focused',async()=>{
 const s=recoveryReady();s.ids.get('cartesian-keyboard').checked=true;
 const toggle=s.ids.get('cartesian-recovery');toggle.onclick();toggle.closest=()=>null;
 s.document.activeElement=toggle;
 s.documentEvents.dispatch('keydown',key('w',{target:toggle}));
 assert.equal(JSON.parse(motions(s)[0].options.body).mode,'recovery');
 await acceptStart(s);
 s.documentEvents.dispatch('keyup',key('w',{target:toggle}));
 assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
});

const directCameraDelivery={...cameraDelivery,direct_hosts:['100.85.56.14','ramion-server-01'],direct_default_fps:10};

for(const host of ['100.85.56.14','ramion-server-01','RaMiOn-SeRvEr-01']){
 test('known direct host '+host+' starts once at Smooth 10fps',()=>{
  const s=setup(host);renderReady(s,{cameras:liveCameras,camera_delivery:directCameraDelivery});
  assert.equal(s.ids.get('video-preview').value,'10');
  assert.equal(s.ids.get('primary-image').src,'/cameras/overhead.mjpg?consumer=operator&fps=10');
  assert.equal(s.ids.get('primary-image').srcWrites,1,'first metadata must select target before subscribing');
  assert.match(s.ids.get('primary-status').textContent,/target 10 fps/);
  s.run('render()');assert.equal(s.ids.get('primary-image').srcWrites,1);
 });
}

for(const host of ['localhost','127.0.0.1','100.85.56.15','unknown-host','ramion-server-01.example']){
 test('unlisted or local host '+host+' retains low-bandwidth default',()=>{
  const s=setup(host);renderReady(s,{cameras:liveCameras,camera_delivery:directCameraDelivery,viewer:{available:false}});
  assert.equal(s.ids.get('video-preview').value,'2');
  assert.equal(s.ids.get('primary-image').src,'/cameras/overhead.mjpg?consumer=operator');
  assert.equal(s.ids.get('primary-image').srcWrites,1);
 });
}

test('manual preview selection persists across direct-host metadata polls',()=>{
 const s=setup('100.85.56.14');renderReady(s,{cameras:liveCameras,camera_delivery:directCameraDelivery});
 choosePreview(s,2);
 for(let index=0;index<3;index++)renderReady(s,{cameras:liveCameras,camera_delivery:structuredClone(directCameraDelivery)});
 assert.equal(s.ids.get('video-preview').value,'2');
 assert.match(s.ids.get('primary-image').src,/fps=2/);
 assert.equal(s.ids.get('primary-image').srcWrites,2);
 choosePreview(s,15);s.run('render()');
 assert.equal(s.ids.get('video-preview').value,'15');
 assert.match(s.ids.get('primary-image').src,/fps=15/);
});

test('a direct host requires explicit supported metadata for its automatic default',()=>{
 for(const metadata of [cameraDelivery,{...directCameraDelivery,direct_default_fps:30}]){
  const s=setup('100.85.56.14');renderReady(s,{cameras:liveCameras,camera_delivery:metadata});
  assert.equal(s.ids.get('video-preview').value,'2');
  assert.equal(s.ids.get('primary-image').src,'/cameras/overhead.mjpg?consumer=operator');
 }
});

test('unrecognized range reasons retain their supplied wording',()=>{
 const ranges=structuredClone(readyRanges);
 ranges.directions.up.limiting_reason='custom boundary: wait for review';
 const s=rangeReady(ranges);
 assert.equal(s.ids.get('range-up-reason').textContent,'custom boundary: wait for review');
 assert.equal(s.ids.get('range-up-reason').title,'custom boundary: wait for review');
});

const orientationCapability={supported:true,tilt:{supported:true,available:true,reason:null},roll:{supported:true,available:true,reason:null},
 angular_rate_key:'angular_deg_s',joint_rate_key:'joint_deg_s',tilt_tip_compensated:false,tilt_view:'tip_up_down',tilt_up_joint_sign:-1,tilt_down_joint_sign:1,roll_tip_compensated:false,roll_view:'from_wrist_toward_tip',roll_left_joint_sign:1,roll_right_joint_sign:-1};
const orientationDirections=['tilt_up','tilt_down','roll_left','roll_right'];
function orientationReady(overrides={}){return cartesianReady({cartesian:{...cartesianCapability,recovery:recoveryCapability,orientation_controls:structuredClone(orientationCapability),...overrides}});}

test('Orientation exposes four hold controls with distinct tilt and wrist-axis behavior',()=>{
 const s=orientationReady();
 for(const name of orientationDirections)assert.equal(direction(s,name).disabled,false);
 assert.match(page,/Tilt and roll turn the wrist axes\. The tip may move in an arc\./);
 assert.doesNotMatch(page,/Tilt holds the tip in place/);
 assert.match(page,/title="Viewed from the wrist toward the tip"/);
 assert.match(s.ids.get('cartesian-orientation-rates').textContent,/Tilt 5°\/s.*roll 5°\/s/);
 assert.doesNotMatch(s.ids.get('cartesian-orientation-rates').textContent,/roll 8°\/s/,'joint rate is a cap, not the requested roll rate');
 s.ids.get('cartesian-speed').value='medium';s.run('enable()');
 assert.match(s.ids.get('cartesian-orientation-rates').textContent,/Tilt 10°\/s.*roll 10°\/s/);
 assert.doesNotMatch(s.ids.get('cartesian-orientation-rates').textContent,/roll 15°\/s/);
 s.ids.get('cartesian-speed').value='fast';s.run('enable()');
 assert.match(s.ids.get('cartesian-orientation-rates').textContent,/Tilt 20°\/s.*roll 20°\/s/);
 assert.doesNotMatch(s.ids.get('cartesian-orientation-rates').textContent,/roll 25°\/s/);
});

test('older controllers fail closed for Orientation without changing existing controls',()=>{
 const s=cartesianReady();s.ids.get('cartesian-keyboard').checked=true;
 for(const name of orientationDirections){assert.equal(direction(s,name).disabled,true);direction(s,name).dispatch('pointerdown',pointer());}
 for(const name of ['i','k','j','l'])s.documentEvents.dispatch('keydown',key(name));
 assert.equal(motions(s).length,0);
 assert.equal(direction(s,'forward').disabled,false);assert.equal(direction(s,'open').disabled,false);
 assert.match(s.ids.get('cartesian-orientation-status').textContent,/unavailable on this controller/);
});

test('tilt and roll each respect their advertised availability and reason',()=>{
 const s=orientationReady({orientation_controls:{...orientationCapability,tilt:{supported:true,available:false,reason:'Configure the workspace floor.'}}});
 assert.equal(direction(s,'tilt_up').disabled,true);assert.equal(direction(s,'tilt_down').disabled,true);
 assert.equal(direction(s,'roll_left').disabled,false);
 direction(s,'tilt_up').dispatch('pointerdown',pointer());assert.equal(motions(s).length,0);
 assert.match(s.ids.get('cartesian-orientation-status').textContent,/Tilt: Configure the workspace floor/);
 s.run('state.cartesian.orientation_controls.roll={supported:false,available:true,reason:"Wrist axis unavailable."};enable()');
 assert.equal(direction(s,'roll_right').disabled,true);
});

for(const focus of ['keyboard-toggle','recovery-switch']){
 test('I K J L work with keyboard opt-in while '+focus+' retains focus',async()=>{
  const s=orientationReady(),toggle=s.ids.get('cartesian-keyboard');
  s.documentEvents.dispatch('keydown',key('i'));assert.equal(motions(s).length,0);
  toggle.checked=true;toggle.onchange();toggle.closest=()=>toggle;
  const target=focus==='keyboard-toggle'?toggle:s.ids.get('cartesian-recovery');
  if(focus==='recovery-switch'){target.onclick();target.closest=()=>null;}
  s.document.activeElement=target;
  for(const [name,expected] of Object.entries({i:'tilt_up',k:'tilt_down',j:'roll_left',l:'roll_right'})){
   s.documentEvents.dispatch('keydown',key(name,{target}));
   const body=JSON.parse(motions(s).at(-1).options.body);
   assert.equal(body.direction,expected);assert.equal(body.mode,'normal');
   await acceptStart(s);
   s.documentEvents.dispatch('keyup',key(name,{target}));
   assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
  }
 });
}

for(const name of orientationDirections){
 test(name+' captures, renews once and cancels without a late renewal restart',async()=>{
  const s=orientationReady();s.ids.get('cartesian-recovery').onclick();s.ids.get('cartesian-speed').value='fast';
  const button=direction(s,name);button.dispatch('pointerdown',pointer(17));
  const body=JSON.parse(motions(s)[0].options.body);
  assert.equal(body.direction,name);assert.equal(body.mode,'normal');assert.equal(body.profile,'fast');
  assert.equal(button.capturedPointer,17);
  direction(s,'forward').dispatch('pointerdown',pointer(18));assert.equal(motions(s).length,1);
  await acceptStart(s);
  const timer=s.timers.find(item=>item.ms===100);timer.fn();timer.fn();
  const renew=motions(s).at(-1);assert.equal(renew.url,'/api/cartesian/renew');assert.equal(motions(s).length,2);
  button.dispatch('pointercancel',pointer(17));
  assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
  assert.equal(button.getAttribute('aria-pressed'),'false');
  response(renew,{cartesian:{active:true,motion_id:'move-1'}});await flush();
  assert.equal(s.timers.filter(item=>item.ms===100).length,1);
  assert.equal(motions(s).filter(request=>request.url.endsWith('/start')).length,1);
 });
}

test('Orientation keyboard still ignores fields, modifiers and repeats and keeps Q/E gripper mapping',async()=>{
 const s=orientationReady();s.ids.get('cartesian-keyboard').checked=true;
 for(const options of [{repeat:true},{ctrlKey:true},{shiftKey:true},{altKey:true},{metaKey:true},{target:{closest:()=>({})}}])s.documentEvents.dispatch('keydown',key('i',options));
 assert.equal(motions(s).length,0);
 for(const [name,expected] of [['q','open'],['e','close']]){
  s.documentEvents.dispatch('keydown',key(name));assert.equal(JSON.parse(motions(s).at(-1).options.body).direction,expected);
  await acceptStart(s);s.documentEvents.dispatch('keyup',key(name));
 }
});

test('losing orientation capability cancels a hold and a delayed start cannot reactivate it',async()=>{
 const s=orientationReady();direction(s,'tilt_up').dispatch('pointerdown',pointer());const start=motions(s)[0];
 s.run('state.cartesian.orientation_controls.tilt.available=false;enable()');
 assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
 response(start,{cartesian:{active:true,motion_id:'late'}});await flush();
 assert.equal(s.timers.filter(item=>item.ms===100).length,0);
});

test('notification overlay is outside document flow and keeps a polite message with explicit dismissal',()=>{
 const start=page.indexOf('<header'),end=page.indexOf('</header>'),notice=page.indexOf('id="notice"');
 assert.ok(start<notice && notice<end,'notice must anchor inside the header');
 const rule=css.match(/#notice\{([^}]+)\}/)?.[1];
 assert.match(rule,/position:absolute/);assert.match(rule,/top:calc\(100% \+ 8px\)/);
 assert.match(rule,/width:min\(480px,calc\(100vw - 40px\)\)/);
 assert.match(css,/@media\(max-height:500px\) and \(max-width:1000px\)\{#notice\{position:fixed;top:68px/,'landscape notice must stay visible below fixed STOP while the header scrolls away');
 assert.match(page,/id="notice-message"[^>]*role="status"[^>]*aria-live="polite"/);
 assert.match(page,/id="notice-dismiss"[^>]*aria-label="Dismiss message"/);
});

test('motion error survives successful heartbeat, view and unrelated camera POSTs until a relevant new action',async()=>{
 const s=cartesianReady();direction(s,'forward').dispatch('pointerdown',pointer());
 response(motions(s)[0],{error:'Minimum tip height reached.'},409);await flush();
 const message=s.ids.get('notice-message'),host=s.ids.get('notice');
 assert.equal(message.textContent,'Minimum tip height reached.');assert.equal(host.hidden,false);
 const heartbeat=s.run('heartbeat()');response(heartbeats(s).at(-1),{});await heartbeat;
 const view=rootByJoint(s,'wrist_flex').querySelector('.servo-name');view.onclick();
 response(s.requests.at(-1),{});await flush();
 s.ids.get('camera-save').onclick();response(s.requests.at(-1),{});await flush();
 assert.equal(message.textContent,'Minimum tip height reached.');assert.equal(host.hidden,false);
 direction(s,'back').dispatch('pointerdown',pointer());
 assert.equal(host.hidden,true);assert.equal(message.textContent,'');
 assert.equal(motions(s).at(-1).url,'/api/cartesian/start');
});

test('identical notices do not churn text, visibility or motion while held',async()=>{
 const s=orientationReady();s.ids.get('cartesian-keyboard').checked=true;
 const checkbox=s.ids.get('cartesian-keyboard');checkbox.closest=()=>checkbox;
 s.documentEvents.dispatch('keydown',key('i',{target:checkbox}));
 response(motions(s)[0],{error:'Wrist tilt limit reached.'},409);await flush();
 const message=s.ids.get('notice-message'),host=s.ids.get('notice');
 const textWrites=message.textWrites,hiddenWrites=host.hiddenWrites;
 for(let index=0;index<3;index++){
  s.run("notice('Wrist tilt limit reached.','motion');enable()");
  s.documentEvents.dispatch('keydown',key('i',{target:checkbox,repeat:true}));
 }
 assert.equal(message.textWrites,textWrites);assert.equal(host.hiddenWrites,hiddenWrites);
 assert.equal(motions(s).filter(request=>request.url.endsWith('/start')).length,1);
 assert.equal(s.timers.filter(item=>item.ms===100).length,0);
 s.ids.get('notice-dismiss').onclick();
 assert.equal(host.hidden,true);assert.equal(s.run('token'),'t');
});

test('an explicit manual joint action clears its previous motion error before its response',()=>{
 const s=cartesianReady();s.ids.get('joint-mode').onclick();s.ids.get('step').value='5';s.run("notice('Previous joint motion failed.','motion')");
 rootByJoint(s,'wrist_flex').querySelector('.plus').onclick();
 assert.equal(s.ids.get('notice').hidden,true);
 assert.equal(s.requests.at(-1).url,'/api/jog');
});

test('a current acknowledged hold shows the actual backend stop reason from polling',async()=>{
 const s=orientationReady();direction(s,'tilt_up').dispatch('pointerdown',pointer());await acceptStart(s);
 s.run('state.cartesian={...state.cartesian,sequence:1,active:false,motion_id:null,stopped_motion_id:"move-1",stop_reason:"Wrist tilt reached its configured limit."};enable()');
 assert.equal(s.ids.get('notice-message').textContent,'Wrist tilt reached its configured limit.');
 assert.equal(s.ids.get('notice').hidden,false);
 assert.equal(motions(s).at(-1).url,'/api/cartesian/stop');
 const textWrites=s.ids.get('notice-message').textWrites;
 s.run('enable();enable()');assert.equal(s.ids.get('notice-message').textWrites,textWrites);
});

test('renewal-ended response uses its current stop reason instead of a generic message',async()=>{
 const s=orientationReady();direction(s,'roll_left').dispatch('pointerdown',pointer());await acceptStart(s);
 s.timers.find(item=>item.ms===100).fn();
 response(motions(s).at(-1),{error:'Hold-jog has ended; press again to move.',cartesian:{sequence:2,active:false,stopped_motion_id:'move-1',stop_reason:'Wrist roll reached its lower limit.'}},409);await flush();
 assert.equal(s.ids.get('notice-message').textContent,'Wrist roll reached its lower limit.');
});

test('new holds never borrow old clip or stop reasons over their current validation error',async()=>{
 const s=orientationReady({sequence:5,active:false,stop_reason:'An older unrelated hold failed.'});
 s.run('state.clip="An old clip must stay historical."');
 direction(s,'tilt_down').dispatch('pointerdown',pointer());
 response(motions(s)[0],{error:'Current wrist tilt is unavailable.'},409);await flush();
 assert.equal(s.ids.get('notice-message').textContent,'Current wrist tilt is unavailable.');
 const second=orientationReady({sequence:0,stop_reason:'An older unrelated hold failed.'});
 direction(second,'tilt_up').dispatch('pointerdown',pointer());await acceptStart(second);
 second.timers.find(item=>item.ms===100).fn();
 response(motions(second).at(-1),{error:'Hold-jog has ended; press again to move.'},409);await flush();
 assert.equal(second.ids.get('notice-message').textContent,'Hold-jog has ended; press again to move.');
});

test('ownership revocation preserves the actual stop reason only for the matching held motion',async()=>{
 for(const [stoppedId,expected] of [['move-1','Cartesian tracking error reached its bound.'],['unrelated-motion','']]){
  const s=orientationReady();direction(s,'tilt_up').dispatch('pointerdown',pointer());await acceptStart(s);
  const next={...robotState,sequence:43,control:{owner_id:null},cartesian:{...cartesianCapability,sequence:1,active:false,motion_id:null,stopped_motion_id:stoppedId,stop_reason:'Cartesian tracking error reached its bound.'}};
  response(s.requests.find(request=>request.url==='/api/state'),next);await flush();
  assert.equal(s.run('token'),null);
  assert.equal(s.ids.get('notice-message').textContent,expected);
  assert.equal(s.ids.get('notice').hidden,!expected);
 }
});


test('state transport failure frees streaming media, drops control, and recovers with sticky finite previews',async()=>{
 const s=setup();renderReady(s,{cameras:liveCameras,camera_delivery:directCameraDelivery,control:{owner_id:'owner'}});
 s.run("token='secret';ownerId='owner'");
 assert.ok(s.ids.get('primary-image').src.includes('.mjpg'));
 assert.ok(s.ids.get('viewer').src);
 s.requests[0].reject(new Error('timeout'));await flush();
 assert.equal(s.run('online'),false);assert.equal(s.run('token'),null);
 assert.equal(s.ids.get('primary-image').src,'');assert.equal(s.ids.get('viewer').src,'');
 assert.equal(s.run('snapshotPreview'),true);
 const retry=s.timers.findLast(t=>!t.cancelled&&t.ms===250);retry.fn();
 response(s.requests.at(-1),{...robotState,cameras:liveCameras,camera_delivery:directCameraDelivery});await flush();
 assert.equal(s.run('online'),true);assert.equal(s.run('token'),null);
 const snapshots=s.requests.filter(r=>r.url.includes('.jpg'));
 assert.equal(snapshots.length,3);assert.equal(snapshots[0].url,'http://100.85.56.14:8780/cameras/overhead.jpg?consumer=operator&fps=10');
 assert.equal(s.ids.get('primary-image').src,'');
 snapshots[0].resolve({ok:true,blob:async()=>({})});await flush();
 assert.equal(s.ids.get('primary-image').src,'blob:test-0');
 assert.equal(s.run('snapshotPreview'),true);
 assert.equal(s.requests.filter(r=>r.url.includes('control/claim')).length,0);
});

test('finite preview serializes frames, retains completed frame, and revokes replaced URLs',async()=>{
 const s=setup();s.run('snapshotPreview=true');renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery});
 const first=s.requests.find(r=>r.url.includes('overhead.jpg'));
 const initialCount=s.requests.length;
 for(let index=0;index<4;index++)s.run('render()');
 assert.equal(s.requests.length,initialCount);
 first.resolve({ok:true,blob:async()=>({})});await flush();
 const previous=s.ids.get('primary-image').src;
 assert.equal(previous,'blob:test-0');
 const timer=s.timers.findLast(t=>!t.cancelled&&t.ms>0&&t.ms<=500);assert.ok(timer);timer.fn();
 const next=s.requests.at(-1);assert.ok(next.url.includes('overhead.jpg'));
 assert.equal(s.ids.get('primary-image').src,previous);
 for(let index=0;index<4;index++)s.run('render()');
 assert.equal(s.requests.at(-1),next);
 next.resolve({ok:true,blob:async()=>({})});await flush();
 assert.equal(s.ids.get('primary-image').src,'blob:test-1');
 assert.deepEqual(s.revokedURLs,[previous]);
});

test('hidden media cancels finite fetches and fences late completions and timers',async()=>{
 const s=setup();s.run('snapshotPreview=true');renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery});
 const snapshots=s.requests.filter(r=>r.url.includes('.jpg'));
 s.document.hidden=true;s.documentEvents.dispatch('visibilitychange');
 for(const request of snapshots)assert.equal(request.options.signal.aborted,true);
 for(const request of snapshots)request.resolve({ok:true,blob:async()=>({})});
 await flush();assert.equal(s.objectURLs.length,0);assert.equal(s.run('cameraJobs.size'),0);
 s.run('updateCameras()');assert.equal(s.requests.filter(r=>r.url.includes('.jpg')).length,3);
 s.document.hidden=false;s.documentEvents.dispatch('visibilitychange');
 assert.equal(s.requests.filter(r=>r.url.includes('.jpg')).length,6);
});

test('finite localhost snapshots use CORS-safe dashboard and preserve selection and query',()=>{
 const s=setup('localhost');s.run('snapshotPreview=true');
 renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery,viewer:{available:true,proxy_url:'/viewer/'}});
 choosePreview(s,15);
 const request=s.requests.filter(r=>r.url.includes('overhead.jpg')).at(-1);
 assert.equal(request.url,'http://localhost:8780/cameras/overhead.jpg?consumer=operator&fps=15');
 assert.equal(s.ids.get('video-preview').value,'15');
 assert.equal(s.requests.find(r=>r.url.includes('overhead.jpg')).options.signal.aborted,true);
});

test('snapshot failures back off and deadlines abort without relaxing state deadline',async()=>{
 const s=setup();s.run('snapshotPreview=true');renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery});
 const request=s.requests.find(r=>r.url.includes('overhead.jpg'));
 const deadline=s.timers.find(t=>!t.cancelled&&t.ms===1200);deadline.fn();
 assert.equal(request.options.signal.aborted,true);
 request.reject(new Error('timeout'));await flush();
 assert.ok(s.timers.some(t=>!t.cancelled&&t.ms===1000));
 assert.equal(s.run('online'),true);
});

test('rendering failure fails closed without activating transport fallback',async()=>{
 const s=setup();s.run("render=()=>{throw new Error('render bug')};token='secret';ownerId='owner'");
 response(s.requests[0],robotState);await flush();
 assert.equal(s.run('online'),false);assert.equal(s.run('token'),null);assert.equal(s.run('snapshotPreview'),false);
});


test('selected snapshot cadence survives recovery and suspension revokes displayed URLs',async()=>{
 const s=setup();s.run('snapshotPreview=true');renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery});
 choosePreview(s,15);
 const request=s.requests.filter(r=>r.url.includes('overhead.jpg')).at(-1);
 request.resolve({ok:true,blob:async()=>({})});await flush();
 const cadence=s.timers.findLast(t=>!t.cancelled&&t.ms>=0&&t.ms<=1000/15);
 assert.ok(cadence,'the selected 15fps is the next-frame target');
 const displayed=s.ids.get('primary-image').src;
 s.requests[0].reject(new Error('state timeout'));await flush();
 assert.equal(s.run('online'),false);assert.equal(s.run('cameraJobs.size'),0);
 assert.ok(s.revokedURLs.includes(displayed));assert.equal(cadence.cancelled,true);
 const count=s.requests.length;cadence.fn();s.run('updateCameras()');
 assert.equal(s.requests.length,count,'old timer and offline render cannot resume previews');
});


test('failed snapshots retain the last frame with a persistent warning until retry succeeds',async()=>{
 const s=setup();s.run('snapshotPreview=true');renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery});
 const first=s.requests.find(r=>r.url.includes('overhead.jpg'));
 first.resolve({ok:true,blob:async()=>({})});await flush();
 const displayed=s.ids.get('primary-image').src;
 s.timers.findLast(t=>!t.cancelled&&t.ms>0&&t.ms<=500).fn();
 s.requests.at(-1).reject(new Error('camera unavailable'));await flush();
 for(let index=0;index<3;index++)s.run('render()');
 assert.equal(s.ids.get('primary-image').src,displayed);
 assert.match(s.ids.get('primary-status').textContent,/Preview interrupted.*retrying/);
 const overlay=s.ids.get('primary-media').querySelector('.camera-stale');
 assert.equal(overlay.hidden,false);assert.match(overlay.textContent,/Preview interrupted/);
 s.timers.findLast(t=>!t.cancelled&&t.ms===1000).fn();
 s.requests.at(-1).resolve({ok:true,blob:async()=>({})});await flush();
 assert.match(s.ids.get('primary-status').textContent,/Live/);assert.equal(overlay.hidden,true);
 assert.notEqual(s.ids.get('primary-image').src,displayed);
});

test('late snapshot from a swapped camera cannot replace the current source',async()=>{
 const s=setup();s.run('snapshotPreview=true');renderReady(s,{cameras:liveCameras,camera_delivery:cameraDelivery});
 const old=s.requests.find(r=>r.url.includes('overhead.jpg'));
 s.run('swap()');assert.equal(old.options.signal.aborted,true);
 const current=s.requests.filter(r=>r.url.includes('wrist.jpg')).at(-1);
 current.resolve({ok:true,blob:async()=>({})});await flush();
 const displayed=s.ids.get('primary-image').src;
 old.resolve({ok:true,blob:async()=>({})});await flush();
 assert.equal(s.ids.get('primary-image').src,displayed);
 assert.equal(s.objectURLs.length,1);
});

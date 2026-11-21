// Optional browser regression: RAMION_PLAYWRIGHT_MODULE may point to an installed
// Playwright package; RAMION_CHROMIUM_EXECUTABLE may select an existing Chromium.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const http=require('node:http');
const path=require('node:path');
let chromium;
try { ({chromium}=require(process.env.RAMION_PLAYWRIGHT_MODULE || 'playwright')); } catch {}
const jpeg=Buffer.from('/9j/2wBDAAYEBQYFBAYGBQYHBwYIChAKCgkJChQODwwQFxQYGBcUFhYaHSUfGhsjHBYWICwgIyYnKSopGR8tMC0oMCUoKSj/2wBDAQcHBwoIChMKChMoGhYaKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCj/wAARCAAIAAgDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAf/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/8QAFAEBAAAAAAAAAAAAAAAAAAAABf/EABQRAQAAAAAAAAAAAAAAAAAAAAD/2gAMAwEAAhEDEQA/AIkAaEP/2Q==','base64');

test('state polling recovers with finite previews while another tab keeps three MJPEG streams', {
  skip:!chromium && 'Install Playwright or set RAMION_PLAYWRIGHT_MODULE',
  timeout:20000,
},async()=>{
  const root=path.join(__dirname,'../apps/arm-dashboard/src/arm_dashboard/static');
  const state={robot:'so101',joints:[],gripper:'gripper',limits:{},measured:{},target:{},
    connected:false,preview_only:true,loop_ok:true,age_s:0,control:{owner_id:null},faults:[],
    emergency_stop:{latched:true,generation:1},viewer:{available:false},
    cameras:Object.fromEntries(['overhead','wrist','observer'].map(name=>[name,{status:'live',url:'/cameras/'+name+'.mjpg'}])),
    camera_delivery:{default_fps:2,available_fps:[2,10,15],max_fps:15}};
  let streams=0,states=0,snapshots=0;
  const sockets=new Set();
  const server=http.createServer((req,res)=>{
    const route=new URL(req.url,'http://localhost').pathname;
    if(route==='/api/state'){
      states++;
      res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'});
      res.end(JSON.stringify(state));
    } else if(route.endsWith('.mjpg') || route==='/stream'){
      streams++;
      res.writeHead(200,{'Content-Type':'multipart/x-mixed-replace; boundary=frame','Cache-Control':'no-store','Connection':'close'});
      res.flushHeaders();
      const send=()=>res.write('--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+jpeg.length+'\r\n\r\n') && res.write(jpeg) && res.write('\r\n');
      send();
      const interval=setInterval(send,200);
      res.on('close',()=>{clearInterval(interval);streams--;});
    } else if(route.endsWith('.jpg')){
      snapshots++;
      res.writeHead(200,{'Content-Type':'image/jpeg','Cache-Control':'no-store','Content-Length':jpeg.length});
      res.end(jpeg);
    } else if(route==='/background'){
      res.end('<html>Other camera consumer</html>');
    } else {
      const file=route==='/'?'panel.html':route.slice(1);
      if(!['panel.html','panel.css','panel.js'].includes(file)){res.writeHead(404);res.end();return;}
      res.writeHead(200,{'Content-Type':file.endsWith('.js')?'text/javascript':file.endsWith('.css')?'text/css':'text/html'});
      res.end(fs.readFileSync(path.join(root,file)));
    }
  });
  server.on('connection',socket=>{sockets.add(socket);socket.on('close',()=>sockets.delete(socket));});
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  let browser;
  try {
    browser=await chromium.launch({headless:true,...(process.env.RAMION_CHROMIUM_EXECUTABLE?{executablePath:process.env.RAMION_CHROMIUM_EXECUTABLE}:{})});
    const context=await browser.newContext({viewport:{width:1440,height:900}});
    const base='http://127.0.0.1:'+server.address().port;
    const other=await context.newPage();
    await other.goto(base+'/background');
    await other.evaluate(()=>{
      window.cameraConsumers=Array.from({length:3},(_,i)=>{
        const image=new Image();image.src='/stream?i='+i;document.body.append(image);return image;
      });
    });
    const app=await context.newPage();
    await app.goto(base,{waitUntil:'domcontentloaded'});
    await app.waitForFunction(()=>online && ['primary','secondary','observer'].every(slot=>
      document.getElementById(slot+'-image').getAttribute('src')?.startsWith('blob:')),null,{timeout:10000});
    const recoveredStates=states,recoveredSnapshots=snapshots;
    await app.waitForTimeout(1600);
    assert.equal(await app.evaluate(()=>online),true);
    assert.ok(states>recoveredStates+2,'state polls keep completing after media resumes');
    assert.ok(snapshots>recoveredSnapshots,'all camera previews keep refreshing');
    assert.equal(streams,3,'only the other tab retains long-lived connections');
    assert.equal(await app.evaluate(()=>token),null,'recovery does not claim robot control');
  } finally {
    if(browser)await browser.close();
    for(const socket of sockets)socket.destroy();
    await new Promise(resolve=>server.close(resolve));
  }
});

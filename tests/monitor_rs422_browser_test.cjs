// Isolated real ZMQ publisher + monitor HTTP + browser. No production bus or hardware.
const {chromium} = require(process.env.AVIATOR_PLAYWRIGHT || 'playwright-core');
const {spawn} = require('child_process');
const fs=require('fs'),os=require('os'),path=require('path'),net=require('net'),assert=require('assert');
const root=path.resolve(__dirname,'..');
const binary=process.argv[2] || path.join(root,'build/bin/aviator_monitor');
async function port() { return new Promise(resolve=>{const server=net.createServer();server.listen(0,'127.0.0.1',()=>{const p=server.address().port;server.close(()=>resolve(p));});}); }
const publisher = `
import json, pathlib, socket, struct, sys, time, zmq
pub=zmq.Context().socket(zmq.PUB)
pub.bind(sys.argv[1])
clock=socket.gethostname()+'-'+pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
sequence=0
saved=None
request=None
while True:
    sequence+=1
    mode=pathlib.Path(sys.argv[2]).read_text().strip()
    now=time.monotonic_ns()//1000
    message=dict(msg_type='FlightCommand',version='1.0',sequence=sequence,timestamp=time.time_ns()//1000,
        sample_mono_us=now,clock_id=clock,publisher_id='flight_gateway',session_id='rs422-browser',
        valid=mode!='stale',source='JOYSTICK',control=dict(roll=.125,pitch=-.25))
    if mode!='joystick':
        operation=16 if mode=='next' else (32 if mode in ['serial','stale','unknown','malformed'] else 0)
        raw=struct.pack('<BBBBhh',235,144,sequence%256,operation,125,-250)+bytes([165])+bytes(22)
        raw+=bytes([sum(raw[2:])&255])
        message['source']='FLIGHT'
        if operation:
            saved=dict(raw_bytes=list(raw),received_mono_us=now,received_utc_us=time.time_ns()//1000,request_id=None)
        if mode in ['queued','rejected'] and saved:
            if request is None:
                request=dict(msg_type='ServiceRequest',version='1.0',request_id='8c99db33-25c2-4bb7-901a-cfd9a7268e40',
                    client_id='flight_gateway',client_session_id='rs422-browser',target='aviator_core',
                    operation='grasp_wheel',timestamp=time.time_ns()//1000,issued_mono_us=now,clock_id=clock,
                    deadline_ms=10000,parameters=dict(source='FLIGHT',operation_code=32),gateway_observation='QUEUED')
            saved['request_id']=request['request_id']
            pub.send_multipart([b'record.service.request',json.dumps(request).encode()])
            if mode=='rejected':
                reply=dict(msg_type='ServiceReply',version='1.0',request_id=request['request_id'],
                    client_id='flight_gateway',client_session_id='rs422-browser',server_id='aviator_core',
                    server_session_id='core',status='REJECTED',error_code=1,timestamp=time.time_ns()//1000,
                    result=dict(reason='INVALID_STATE',state='READY'))
                pub.send_multipart([b'record.service.reply',json.dumps(reply).encode()])
        message['rs422']=dict(format='aviator-downlink-32-v1',selected_channel='A',blocked=False,
            receive_timeout_ms=50,channels=[dict(channel=c,raw_bytes=list(raw) if c=='A' else [],
                received_mono_us=now-(200000 if mode=='stale' else 0) if c=='A' else 0,
                healthy=c=='A',accepted_frames=sequence if c=='A' else 0,rejected_bytes=7,
                last_operation=saved if c=='A' else None) for c in ['A','B']])
        if mode=='unknown': message['clock_id']='other-clock'
        if mode=='malformed': message['rs422']['channels'][0]['raw_bytes'][0]=256
    pub.send_multipart([b'flight.command',json.dumps(message).encode()])
    time.sleep(.0125)
`;
async function main() {
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'aviator-rs422-ui-'));
  const mode=path.join(dir,'mode');fs.writeFileSync(mode,'joystick');
  const http=await port(), bus=await port(), endpoint=`tcp://127.0.0.1:${bus}`;
  const server=spawn(binary,['--bind','127.0.0.1','--port',String(http),'--subscribe',endpoint,'--preview','off']);
  const fixture=spawn(process.env.AVIATOR_TEST_PYTHON || 'python3',['-u','-c',publisher,endpoint,mode]);
  let browser,logs='';for (const child of [server,fixture]) child.stderr.on('data',b=>logs+=b);
  try {
    for (let i=0;i<100;i++) {try{if((await fetch(`http://127.0.0.1:${http}/api/state`)).ok)break;}catch{}await new Promise(r=>setTimeout(r,50));}
    browser=await chromium.launch({executablePath:process.env.AVIATOR_CHROME || '/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
    const context=await browser.newContext({viewport:{width:1920,height:1080},permissions:['clipboard-read','clipboard-write']});
    const page=await context.newPage(),errors=[];page.on('pageerror',e=>errors.push(e.message));
    await page.goto(`http://127.0.0.1:${http}/`,{waitUntil:'domcontentloaded'});
    await page.click('#tab-rs422');
    await page.waitForFunction(()=>document.querySelector('#rs422-note').textContent.includes('暂无 RS422 报文'));
    fs.writeFileSync(mode,'serial');
    await page.waitForFunction(()=>document.querySelector('#rs422-channels .rs422-hex')?.textContent.includes('7D 00 06 FF A5'));
    assert((await page.locator('#rs422-channels .rs422-frame').first().innerText()).includes('-250 → -25.0%'));
    assert((await page.locator('#rs422-channels .rs422-frame').first().innerText()).includes('握盘 · grasp_wheel'));
    assert((await page.locator('#rs422-channels .rs422-frame').first().innerText()).includes('通过 · 字节 2–30'));
    assert((await page.locator('#rs422-channels .rs422-frame').last().innerText()).includes('尚未接收到可用报文'));
    await page.click('#tab-messages');
    fs.writeFileSync(mode,'idle');
    await page.waitForTimeout(250);
    await page.click('#tab-rs422');
    await page.waitForFunction(()=>document.querySelector('#rs422-channels').textContent.includes('不发起新操作'));
    assert((await page.locator('#rs422-operations').innerText()).includes('握盘 · grasp_wheel'));
    assert((await page.locator('#rs422-operations').innerText()).includes('尚无关联的 Core 请求'));
    assert.match(await page.locator('#rs422-operations').innerText(), /接收时间 \d{4}\/\d{2}\/\d{2} \d{2}:\d{2}:\d{2}\.\d{3}/);
    const retainedHex=await page.locator('#rs422-operations pre').first().textContent();
    const liveHex=await page.locator('#rs422-channels pre').first().textContent();
    await page.waitForTimeout(250);
    assert.equal(await page.locator('#rs422-operations pre').first().textContent(),retainedHex);
    assert.notEqual(await page.locator('#rs422-channels pre').first().textContent(),liveHex);
    await page.reload({waitUntil:'domcontentloaded'});
    await page.click('#tab-rs422');
    await page.waitForFunction(()=>document.querySelector('#rs422-operations').textContent.includes('握盘'));
    assert.equal(await page.locator('#rs422-operations pre').first().textContent(),retainedHex);
    fs.writeFileSync(mode,'queued');
    await page.waitForFunction(()=>document.querySelector('#rs422-operations').textContent.includes('等待应答'));
    fs.writeFileSync(mode,'rejected');
    await page.waitForFunction(()=>document.querySelector('#rs422-operations').textContent.includes('Core 已拒绝'));
    assert((await page.locator('#rs422-operations').innerText()).includes('INVALID_STATE'));
    await page.locator('#rs422-operations button').first().click();
    assert.equal(await page.evaluate(()=>navigator.clipboard.readText()),retainedHex.replace(/^00  /,'').replace('\n10  ',' '));
    await page.click('#rs422-pause');
    const raw=await page.locator('#rs422-channels .rs422-hex').first().textContent();
    await page.waitForTimeout(250);
    assert.equal(await page.locator('#rs422-channels .rs422-hex').first().textContent(),raw);
    assert((await page.locator('#rs422-note').innerText()).includes('历史快照'));
    await page.locator('#rs422-channels .rs422-frame > button').first().click();
    const copied=await page.evaluate(()=>navigator.clipboard.readText());
    assert.equal(copied.split(' ').length,32); assert(copied.startsWith('EB 90'));
    const artifacts=process.env.AVIATOR_TEST_ARTIFACTS || path.join(root,'build/artifacts');fs.mkdirSync(artifacts,{recursive:true});
    await page.screenshot({path:path.join(artifacts,'monitor-rs422.png'),fullPage:true});
    await page.setViewportSize({width:900,height:900});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.click('#rs422-pause');
    fs.writeFileSync(mode,'next');
    await page.waitForFunction(()=>document.querySelector('#rs422-operations').textContent.includes('进入待机'));
    assert(!(await page.locator('#rs422-operations').innerText()).includes('Core 已拒绝'));
    fs.writeFileSync(mode,'stale');
    await page.waitForFunction(()=>document.querySelector('#rs422-channels .rs422-frame .badge')?.textContent==='报文已过期');
    fs.writeFileSync(mode,'unknown');
    await page.waitForFunction(()=>document.querySelector('#rs422-channels .rs422-frame .badge')?.textContent==='时钟域未知');
    fs.writeFileSync(mode,'serial');
    await page.waitForFunction(()=>document.querySelector('#rs422-channels .rs422-frame .badge')?.textContent==='报文新鲜');
    // A healthy 50 ms serial lease must not expire merely because HTTP delivery
    // and the next web refresh take longer. The response still reports its observation.
    await page.route('**/api/state',async route=>{
      const response=await route.fetch();
      await new Promise(resolve=>setTimeout(resolve,120));
      await route.fulfill({response});
    });
    const observations=await page.evaluate(async()=>{
      const samples=[];
      for(let i=0;i<60;i++) {
        samples.push({badge:document.querySelector('#rs422-channels .badge').textContent,
          note:document.querySelector('#rs422-note').textContent});
        await new Promise(resolve=>setTimeout(resolve,20));
      }
      return samples;
    });
    assert(observations.every(s=>s.badge==='报文新鲜' && s.note.includes('控制有效')),
      JSON.stringify(observations));
    await page.unrouteAll({behavior:'wait'});
    await page.route('**/api/state',route=>route.abort());
    await page.waitForFunction(()=>document.querySelector('#rs422-channels .rs422-frame .badge')?.textContent==='网页数据未更新');
    assert((await page.locator('#rs422-note').innerText()).includes('当前控制状态未知'));
    await page.unroute('**/api/state');
    await page.waitForFunction(()=>document.querySelector('#rs422-channels .rs422-frame .badge')?.textContent==='报文新鲜');
    fs.writeFileSync(mode,'malformed');
    await page.waitForFunction(()=>document.querySelector('#rs422-note').textContent.includes('格式不受支持'));
    fs.writeFileSync(mode,'joystick');
    await page.waitForFunction(()=>document.querySelector('#rs422-note').textContent.includes('暂无 RS422 报文'));
    await page.click('#tab-messages');
    await page.waitForFunction(()=>document.querySelector('#rows').textContent.includes('flight.command'));
    assert.equal(errors.length,0,errors.join('\n'));
    console.log('PASS: optional RS422, actual ZMQ bytes, fields, retained operation across idle/reload/tab changes, exact Core outcome, pause/copy, stale clock and malformed data, responsive layout');
  } catch(error) {console.error(logs);throw error;}
  finally {
    await browser?.close();
    for (const child of [fixture,server]) child.kill('SIGTERM');
    await Promise.all([fixture,server].map(child=>new Promise(resolve=>{if(child.exitCode!==null)return resolve();child.once('exit',resolve);})));
    fs.rmSync(dir,{recursive:true,force:true});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});

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
while True:
    sequence+=1
    mode=pathlib.Path(sys.argv[2]).read_text().strip()
    now=time.monotonic_ns()//1000
    message=dict(msg_type='FlightCommand',version='1.0',sequence=sequence,timestamp=time.time_ns()//1000,
        sample_mono_us=now,clock_id=clock,publisher_id='flight_gateway',session_id='rs422-browser',
        valid=mode!='stale',source='JOYSTICK',control=dict(roll=.125,pitch=-.25))
    if mode!='joystick':
        raw=struct.pack('<BBBBhh',235,144,sequence%256,32,125,-250)+bytes([165])+bytes(22)
        raw+=bytes([sum(raw[2:])&255])
        message['source']='FLIGHT'
        message['rs422']=dict(format='aviator-downlink-32-v1',selected_channel='A',blocked=False,
            receive_timeout_ms=100,channels=[dict(channel=c,raw_bytes=list(raw) if c=='A' else [],
                received_mono_us=now-(200000 if mode=='stale' else 0) if c=='A' else 0,
                healthy=c=='A',accepted_frames=sequence if c=='A' else 0,rejected_bytes=7) for c in ['A','B']])
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
    await page.waitForFunction(()=>document.querySelector('.rs422-hex')?.textContent.includes('7D 00 06 FF A5'));
    assert((await page.locator('.rs422-frame').first().innerText()).includes('-250 → -25.0%'));
    assert((await page.locator('.rs422-frame').first().innerText()).includes('握盘 · grasp_wheel'));
    assert((await page.locator('.rs422-frame').first().innerText()).includes('通过 · 字节 2–30'));
    assert((await page.locator('.rs422-frame').last().innerText()).includes('尚未接收到可用报文'));
    await page.click('#rs422-pause');
    const raw=await page.locator('.rs422-hex').first().textContent();
    await page.waitForTimeout(250);
    assert.equal(await page.locator('.rs422-hex').first().textContent(),raw);
    assert((await page.locator('#rs422-note').innerText()).includes('历史快照'));
    await page.locator('.rs422-frame > button').first().click();
    const copied=await page.evaluate(()=>navigator.clipboard.readText());
    assert.equal(copied.split(' ').length,32); assert(copied.startsWith('EB 90'));
    const artifacts=process.env.AVIATOR_TEST_ARTIFACTS || path.join(root,'build/artifacts');fs.mkdirSync(artifacts,{recursive:true});
    await page.screenshot({path:path.join(artifacts,'monitor-rs422.png'),fullPage:true});
    await page.setViewportSize({width:900,height:900});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.click('#rs422-pause');
    fs.writeFileSync(mode,'stale');
    await page.waitForFunction(()=>document.querySelector('.rs422-frame .badge')?.textContent==='报文已过期');
    fs.writeFileSync(mode,'unknown');
    await page.waitForFunction(()=>document.querySelector('.rs422-frame .badge')?.textContent==='时钟域未知');
    fs.writeFileSync(mode,'serial');
    await page.waitForFunction(()=>document.querySelector('.rs422-frame .badge')?.textContent==='报文新鲜');
    await page.route('**/api/state',route=>route.abort());
    await page.waitForFunction(()=>document.querySelector('.rs422-frame .badge')?.textContent==='报文已过期');
    await page.unroute('**/api/state');
    fs.writeFileSync(mode,'malformed');
    await page.waitForFunction(()=>document.querySelector('#rs422-note').textContent.includes('格式不受支持'));
    fs.writeFileSync(mode,'joystick');
    await page.waitForFunction(()=>document.querySelector('#rs422-note').textContent.includes('暂无 RS422 报文'));
    await page.click('#tab-messages');
    await page.waitForFunction(()=>document.querySelector('#rows').textContent.includes('flight.command'));
    assert.equal(errors.length,0,errors.join('\n'));
    console.log('PASS: optional RS422, actual ZMQ bytes, fields, pause/copy, stale clock and malformed data, responsive layout');
  } catch(error) {console.error(logs);throw error;}
  finally {
    await browser?.close();
    for (const child of [fixture,server]) child.kill('SIGTERM');
    await Promise.all([fixture,server].map(child=>new Promise(resolve=>{if(child.exitCode!==null)return resolve();child.once('exit',resolve);})));
    fs.rmSync(dir,{recursive:true,force:true});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});

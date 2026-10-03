// Node >=18 + playwright-core. Simulates a remote HTTP hostname and a slow link.
const {chromium} = require(process.env.AVIATOR_PLAYWRIGHT || 'playwright-core');
const {spawn} = require('child_process');
const fs = require('fs'), os = require('os'), path = require('path'), net = require('net'), assert = require('assert');
const root = path.resolve(__dirname, '..');
async function port() { return new Promise(resolve => { const s=net.createServer(); s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));}); }); }
async function main() {
  const directory=fs.mkdtempSync(path.join(os.tmpdir(),'aviator-monitor-remote-'));
  const http=await port(), base=`http://monitor.test:${http}`;
  const web=spawn(process.argv[2] || path.join(root,'build/communication/bin/aviator_monitor'),
    ['--bind','127.0.0.1','--port',String(http),'--subscribe','tcp://127.0.0.1:1','--preview','off']);
  let stderr=''; web.stderr.on('data',b=>stderr+=b);
  let browser;
  try {
    for(let i=0;i<100;i++){try{if((await fetch(`http://127.0.0.1:${http}/api/state`)).ok)break;}catch{}await new Promise(r=>setTimeout(r,30));}
    browser=await chromium.launch({executablePath:process.env.AVIATOR_CHROME || '/usr/bin/google-chrome',headless:true,
      args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader','--no-proxy-server','--host-resolver-rules=MAP monitor.test 127.0.0.1']});
    const page=await browser.newPage({viewport:{width:1440,height:1000}}), errors=[], resources=new Set();
    page.on('pageerror',e=>errors.push(e.message));
    page.on('request',request=>resources.add(request.url()));
    const cdp=await page.context().newCDPSession(page);
    await cdp.send('Network.enable');
    await cdp.send('Network.emulateNetworkConditions',{offline:false,latency:40,downloadThroughput:2*1024*1024,uploadThroughput:1024*1024});
    await page.goto(base,{waitUntil:'domcontentloaded'});
    assert.equal(await page.evaluate(()=>window.isSecureContext),false);
    // Observe the real viewer's geometry, without a production-only testing API.
    await page.evaluate(async()=>{
      const {Viewer}=await import('/assets/viewer.js'), update=Viewer.prototype.updateHelpers;
      Viewer.prototype.updateHelpers=function(){window.testViewer=this;return update.call(this);};
    });
    await page.waitForFunction(()=>{
      const note=document.getElementById('model-loading');
      return note.hidden || note.classList.contains('failed');
    },null,{timeout:120000});
    assert(await page.locator('#model-loading').evaluate(e=>e.hidden),await page.locator('#model-loading').textContent());
    await page.waitForFunction(()=>window.testViewer?.robot);
    const geometry=await page.evaluate(()=>{
      const v=window.testViewer; let meshes=0, vertices=0;
      v.robot.traverse(o=>{if(o.isMesh){meshes++;vertices+=o.geometry.attributes.position.count;}});
      return {meshes,vertices,links:Object.keys(v.robot.links).length};
    });
    assert(geometry.meshes>=80 && geometry.vertices>100000 && geometry.links>=80,JSON.stringify(geometry));
    assert([...resources].every(url=>new URL(url).origin===base),[...resources].join('\n'));
    assert([...resources].some(url=>url.includes('/models/urdf/aviator.urdf')));
    assert([...resources].filter(url=>/\.stl$/i.test(url)).length>=70);
    assert.equal(await page.locator('#connection').textContent(),'监控服务在线');
    assert.deepStrictEqual(errors,[]); assert.equal(stderr,'');
    await page.screenshot({path:path.join(directory,'remote-model.png')});
    console.log('Remote HTTP model loaded at 2 MiB/s with 40 ms latency:',geometry);
    console.log('Screenshot:',path.join(directory,'remote-model.png'));
  } finally {
    if(browser)await browser.close();
    web.kill('SIGTERM');
    await new Promise(resolve=>{if(web.exitCode!==null)resolve();else web.once('exit',resolve);});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});

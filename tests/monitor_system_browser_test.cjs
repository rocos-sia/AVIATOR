// Node >=18 + playwright-core; no bus or hardware required.
const {chromium} = require(process.env.AVIATOR_PLAYWRIGHT || 'playwright-core');
const {spawn} = require('child_process');
const fs = require('fs'), os = require('os'), path = require('path'), net = require('net'), assert = require('assert');
async function port() { return new Promise(resolve => { const s=net.createServer(); s.listen(0,'127.0.0.1',()=>{const p=s.address().port; s.close(()=>resolve(p));}); }); }
(async()=>{
  const directory = fs.mkdtempSync(path.join(os.tmpdir(),'monitor-system-browser-'));
  const http=await port(), bus=await port(), base=`http://127.0.0.1:${http}`;
  const web=spawn(process.argv[2] || 'build/bin/aviator_monitor',['--bind','127.0.0.1','--port',String(http),'--subscribe',`tcp://127.0.0.1:${bus}`,'--preview','off']);
  web.stdout.resume(); web.stderr.resume();
  let browser;
  try {
    for (let i=0;i<100;i++) { try { if((await fetch(base+'/api/system')).ok) break; } catch {} await new Promise(r=>setTimeout(r,50)); }
    browser=await chromium.launch({executablePath:process.env.AVIATOR_CHROME || '/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
    const page=await browser.newPage({viewport:{width:1440,height:1100}}), errors=[], polls=[];
    page.on('pageerror',e=>errors.push(e.message));
    page.on('request',r=>{if(r.url().endsWith('/api/system')) polls.push(Date.now());});
    await page.goto(base,{waitUntil:'domcontentloaded'});
    await page.waitForFunction(()=>document.querySelector('#system-summary strong').textContent.includes('%'));
    assert.match(await page.locator('#system-uptime').textContent(),/\d+ 天 \d{2}:\d{2}:\d{2}/);
    assert.match(await page.locator('#program-uptime').textContent(),/0 天 00:00:\d{2}/);
    const beforeReload = await (await fetch(base+'/api/system')).json();
    await page.reload({waitUntil:'domcontentloaded'});
    await page.waitForFunction(()=>document.getElementById('program-uptime').textContent !== '—');
    const afterReload = await (await fetch(base+'/api/system')).json();
    assert(afterReload.program_uptime_seconds >= beforeReload.program_uptime_seconds,'page reload does not reset program uptime');
    assert(afterReload.program_uptime_seconds < afterReload.uptime_seconds,'program and boot uptimes are distinct');
    for (const width of [1920,1440,1100,700,390]) {
      await page.setViewportSize({width,height:1100});
      const layout = await page.evaluate(()=>{
        const values=[...document.querySelectorAll('.status-strip strong,#notice,.resource-summary strong')];
        const original=values.map(e=>e.textContent);
        const bounds=()=>[...document.querySelectorAll('.status-strip>span,.resource-summary')].map(e=>{
          const r=e.getBoundingClientRect(); return [r.x,r.y,r.width,r.height];
        });
        values.forEach(e=>e.textContent='—'); const short=JSON.stringify(bounds());
        values.forEach(e=>e.textContent='99999 天 23:59:59 / LONG_STATUS_VALUE 1023.9 MiB/s');
        const long=JSON.stringify(bounds());
        values.forEach((e,i)=>e.textContent=original[i]);
        return {short,long,fits:document.documentElement.scrollWidth<=innerWidth};
      });
      assert.equal(layout.short,layout.long,`changing status content must not move fields at ${width}px`);
      assert(layout.fits,`status fields fit at ${width}px`);
    }
    await page.setViewportSize({width:1440,height:1100});
    assert.equal(await page.locator('.resource-summary svg').count(),5);
    assert.deepEqual(await page.locator('.resource-summary>span').allTextContents(),['CPU','GPU','内存','硬盘','网络']);
    const gpuValue = page.locator('.resource-summary strong').nth(1);
    await page.route('**/api/system',async route=>{
      const data = await (await route.fetch()).json();
      data.gpu = {percent:37.5};
      await route.fulfill({json:data});
    });
    await page.waitForFunction(()=>document.querySelectorAll('.resource-summary strong')[1].textContent === '37.5%');
    await page.screenshot({path:path.join(directory,'overview.png')});
    await page.unroute('**/api/system');
    await page.route('**/api/system',async route=>{
      const data = await (await route.fetch()).json();
      data.gpu = {percent:null};
      await route.fulfill({json:data});
    });
    await page.waitForFunction(()=>document.querySelectorAll('.resource-summary strong')[1].textContent === '—');
    await page.unroute('**/api/system');
    assert.deepEqual(await page.locator('[role=tab]').allTextContents(),['直观监测','系统消息','系统状态','日志','配置']);
    await page.locator('#tab-messages').focus(); await page.keyboard.press('ArrowRight');
    assert.equal(await page.locator('#tab-resources').getAttribute('aria-selected'),'true');
    assert(await page.locator('#resources').isVisible());
    assert.equal(await page.locator('.resource-panel').count(),4);
    await page.waitForFunction(()=>[...document.querySelectorAll('.resource-plot path')].some(p=>p.getAttribute('d').includes('L')));
    const start=polls.length;
    await new Promise(r=>setTimeout(r,2200));
    assert(polls.length-start>=2 && polls.length-start<=3,'one resource request per second');
    await page.screenshot({path:path.join(directory,'system.png')});
    for (const width of [700,390]) {
      await page.setViewportSize({width,height:900});
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'resource page fits viewport');
      await page.locator('#tab-overview').click();
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'overview fits viewport');
      await page.locator('#tab-resources').click();
    }
    await page.route('**/api/system',route=>route.fulfill({status:503,body:'{}'}));
    await page.waitForFunction(()=>document.getElementById('resource-status').textContent.includes('旧快照'));
    assert.equal(await page.locator('#system-uptime').textContent(),'—');
    assert.equal(await page.locator('#program-uptime').textContent(),'—');
    assert.equal(await page.locator('.resource-summary strong').first().textContent(),'—');
    assert.equal(await gpuValue.textContent(),'—');
    await page.unroute('**/api/system');
    await page.waitForFunction(()=>document.querySelector('#system-summary strong').textContent.includes('%'));
    assert.deepEqual(errors,[]);
    console.log(`system browser passed; screenshots: ${directory}`);
  } finally {
    if(browser) await browser.close();
    web.kill('SIGTERM');
    await new Promise(resolve=>web.exitCode!==null ? resolve() : web.once('exit',resolve));
  }
})().catch(error=>{console.error(error);process.exitCode=1;});

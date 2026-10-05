// Node >=18 + playwright-core; isolated log files/ports, no hardware.
const {chromium} = require(process.env.AVIATOR_PLAYWRIGHT || 'playwright-core');
const {spawn} = require('child_process');
const fs = require('fs'), os = require('os'), path = require('path'), net = require('net'), assert = require('assert');
const root = path.resolve(__dirname, '..');
const binary = process.argv[2] || path.join(root, 'build/bin/aviator_monitor');
async function port() { return new Promise(resolve => { const s = net.createServer(); s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));}); }); }
async function main() {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'aviator-monitor-logs-ui-'));
  const core = path.join(directory, 'core.log');
  const stamp = '[2026-10-05 12:34:56.123] [aviator]';
  fs.writeFileSync(core, `${stamp} [info] ready\n${stamp} [warning] waiting\n${stamp} [error] failed\nexternal output\n<script>alert(1)</script>\n`);
  fs.writeFileSync(path.join(directory, 'camera.log'), 'camera ready\n');
  const http = await port(), bus = await port(), base = `http://127.0.0.1:${http}`;
  const web = spawn(binary, ['--bind','127.0.0.1','--port',String(http),'--subscribe',`tcp://127.0.0.1:${bus}`,'--preview','off','--log-dir',directory]);
  let stderr = ''; web.stderr.on('data', b => stderr += b);
  web.stdout.resume();
  let browser;
  try {
    for (let i=0;i<100;i++) { try { if((await fetch(base+'/api/logs')).ok) break; } catch {} await new Promise(r=>setTimeout(r,50)); }
    browser = await chromium.launch({executablePath:process.env.AVIATOR_CHROME || '/usr/bin/google-chrome',headless:true,
      args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
    const page = await browser.newPage({viewport:{width:1440,height:1000}}), errors=[], polls=[];
    page.on('pageerror', error=>errors.push(error.message));
    page.on('request', request=>{if(request.url().includes('/api/logs?file=')) polls.push(Date.now());});
    page.on('dialog',()=>{throw new Error('log text executed as HTML');});
    await page.goto(base, {waitUntil:'domcontentloaded'});
    await page.locator('#tab-logs').click();
    await page.waitForFunction(()=>document.getElementById('log-content').textContent.includes('camera ready'));
    assert((await page.locator('#log-directory').textContent()).includes(directory));
    await page.getByRole('button',{name:'core',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('log-content').textContent.includes('external output'));
    assert.equal(await page.locator('#log-content .log-line').count(),5);
    assert.equal(await page.locator('#log-content script').count(),0);
    await page.locator('#log-level').selectOption('info');
    assert.equal(await page.locator('#log-content .log-line').count(),3);
    assert((await page.locator('#log-content').textContent()).includes('external output'));
    await page.locator('#log-level').selectOption('warn');
    assert.equal(await page.locator('#log-content .log-line').count(),1);
    assert((await page.locator('#log-content').textContent()).includes('waiting'));
    fs.appendFileSync(core, `${stamp} [warning] appended warning\n`);
    await page.waitForFunction(()=>document.getElementById('log-content').textContent.includes('appended warning'));
    assert.equal(await page.locator('#log-content .log-line').count(),2);
    const count = polls.length;
    await page.waitForFunction(()=>document.getElementById('log-status').textContent.includes('2 / 6'));
    await new Promise(r=>setTimeout(r,2200));
    assert(polls.length-count >= 2 && polls.length-count <= 3, 'poll about once per second');
    await page.locator('#log-level').selectOption('debug');
    assert((await page.locator('#log-status').textContent()).includes('没有匹配'));
    await page.locator('#log-level').selectOption('');
    fs.writeFileSync(core,'reset after truncation\n');
    await page.waitForFunction(()=>document.getElementById('log-content').textContent==='reset after truncation\n');
    fs.writeFileSync(path.join(directory,'logger.log'),'late node\n');
    await page.getByRole('button',{name:'logger',exact:true}).waitFor();
    await page.getByRole('button',{name:'logger',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('log-content').textContent.includes('late node'));
    fs.unlinkSync(path.join(directory,'logger.log'));
    await page.waitForFunction(()=>document.getElementById('log-content').textContent.includes('camera ready'));
    await page.getByRole('button',{name:'core',exact:true}).click();
    fs.writeFileSync(core, Array.from({length:2100},(_,i)=>`line ${i}\n`).join(''));
    await page.waitForFunction(()=>document.getElementById('log-status').textContent.includes('最新 2000 行'));
    await page.locator('#log-follow').uncheck();
    await page.locator('#log-content').evaluate(element=>element.scrollTop=0);
    fs.appendFileSync(core,'new tail\n');
    await page.waitForFunction(()=>document.getElementById('log-content').textContent.includes('new tail'));
    assert.equal(await page.locator('#log-content').evaluate(element=>element.scrollTop),0);
    await page.screenshot({path:path.join(directory,'logs.png')});
    await page.setViewportSize({width:700,height:900});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth));
    await page.setViewportSize({width:390,height:844});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth));
    await page.screenshot({path:path.join(directory,'logs-mobile.png')});
    await page.locator('#tab-logs').focus(); await page.keyboard.press('ArrowRight');
    assert(await page.locator('#settings').isVisible());
    const stopped = polls.length;
    await new Promise(r=>setTimeout(r,1200));
    assert.equal(polls.length,stopped,'log polling stops outside the log tab');
    assert.deepStrictEqual(errors,[]); assert.equal(stderr,'');
    console.log('Monitor logs browser tests passed. Screenshots:',directory);
  } finally {
    if(browser) await browser.close();
    web.kill('SIGTERM');
    await new Promise(resolve=>{if(web.exitCode!==null)resolve();else web.once('exit',resolve);});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});

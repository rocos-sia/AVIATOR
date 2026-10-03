// Node >=18 + playwright-core; isolated temporary ports/configuration, no hardware.
const {chromium} = require(process.env.AVIATOR_PLAYWRIGHT || 'playwright-core');
const {spawn} = require('child_process');
const fs = require('fs'), os = require('os'), path = require('path'), net = require('net'), assert = require('assert');
const root = path.resolve(__dirname, '..');
const binary = process.argv[2] || path.join(root, 'build/communication/bin/aviator_monitor');
async function port() { return new Promise(resolve => { const s = net.createServer(); s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));}); }); }
async function main() {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'aviator-monitor-config-ui-'));
  const configPath = path.join(directory, 'monitor.yaml');
  fs.writeFileSync(configPath, "version: 1\npreview:\n  endpoint: ''\n");
  const http = await port(), bus = await port(), base = `http://127.0.0.1:${http}`;
  const web = spawn(binary, ['--bind','127.0.0.1','--port',String(http),'--subscribe',`tcp://127.0.0.1:${bus}`,'--config',configPath]);
  let stderr = ''; web.stderr.on('data', b => stderr += b);
  let browser;
  try {
    for (let i=0;i<100;i++) { try { if((await fetch(base+'/api/config')).ok) break; } catch {} await new Promise(r=>setTimeout(r,50)); }
    browser = await chromium.launch({executablePath:process.env.AVIATOR_CHROME || '/usr/bin/google-chrome',headless:true,
      args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
    const page = await browser.newPage({viewport:{width:1440,height:1000}}), errors=[];
    page.on('pageerror', error=>errors.push(error.message));
    await page.goto(base, {waitUntil:'domcontentloaded'});
    await page.locator('#tab-settings').click();
    await page.waitForFunction(()=>!document.getElementById('config-fields').disabled);
    assert(await page.locator('#settings').isVisible());
    assert(!(await page.locator('#overview').isVisible()));
    await page.locator('[name="sources.arm.state"]').fill('simulation');
    await page.locator('[name="sources.hand.state"]').fill('simulation');
    await page.locator('[name="timeouts_ms.arm.state"]').fill('600');
    await page.locator('#config-save').click();
    await page.waitForFunction(()=>document.getElementById('config-status').textContent.includes('已保存到'));
    let data = await (await fetch(base+'/api/config')).json();
    assert.equal(data.config.sources['arm.state'],'simulation');
    assert.equal(data.config.timeouts_ms['arm.state'],600);
    assert(fs.readFileSync(configPath,'utf8').includes('simulation'));
    await page.screenshot({path:path.join(directory,'settings.png')});
    await page.locator('#config-mode').selectOption('yaml');
    await page.locator('#config-yaml').fill('version: 1\ntimeouts_ms: {arm.state: -1}\n');
    await page.locator('#config-save').click();
    await page.waitForFunction(()=>document.getElementById('config-status').textContent.includes('保存失败'));
    assert.equal((await (await fetch(base+'/api/config')).json()).revision,data.revision);
    await page.locator('#config-yaml').fill('version: 1\nsources: {arm.state: yaml_source}\npreview: {endpoint: ""}\n');
    await page.locator('#config-save').click();
    await page.waitForFunction(()=>document.getElementById('config-status').textContent.includes('已保存到'));
    await page.locator('#config-mode').selectOption('form');
    assert.equal(await page.locator('[name="sources.arm.state"]').inputValue(),'yaml_source');
    data = await (await fetch(base+'/api/config')).json();
    data.config.sources['arm.state']='another_editor';
    assert((await fetch(base+'/api/config',{method:'PUT',headers:{'Content-Type':'application/json','X-Monitor-Config':'1'},body:JSON.stringify({revision:data.revision,config:data.config})})).ok);
    await page.locator('[name="sources.arm.state"]').fill('stale_editor');
    await page.locator('#config-save').click();
    await page.waitForFunction(()=>document.getElementById('config-status').textContent.includes('配置已被更新'));
    await page.locator('#config-reload').click();
    await page.waitForFunction(()=>document.querySelector('[name="sources.arm.state"]').value==='another_editor');
    await page.locator('#tab-settings').focus(); await page.keyboard.press('ArrowRight');
    assert(await page.locator('#overview').isVisible());
    await page.keyboard.press('End'); assert(await page.locator('#settings').isVisible());
    await page.setViewportSize({width:700,height:900});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth));
    await page.screenshot({path:path.join(directory,'settings-mobile.png')});
    assert.deepStrictEqual(errors,[]); assert.equal(stderr,'');
    console.log('Monitor configuration browser tests passed. Screenshots:',directory);
  } finally {
    if(browser) await browser.close();
    web.kill('SIGTERM');
    await new Promise(resolve=>{if(web.exitCode!==null)resolve();else web.once('exit',resolve);});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});

// Run with Node >=18 and playwright-core. Uses an installed Chrome executable.
const {chromium} = require(process.env.AVIATOR_PLAYWRIGHT || 'playwright-core');
const {spawn} = require('child_process');
const fs = require('fs'), os = require('os'), path = require('path'), net = require('net'), assert = require('assert');
const root = path.resolve(__dirname,'..');
const binary = process.argv[2] || path.join(root,'build/communication/bin/aviator_monitor');
const python = process.env.AVIATOR_TEST_PYTHON || 'python3';
const children = [];
async function port() { return new Promise(resolve => {const server=net.createServer(); server.listen(0,'127.0.0.1',()=>{const p=server.address().port;server.close(()=>resolve(p));});}); }
async function main() {
  const directory=fs.mkdtempSync(path.join(os.tmpdir(),'aviator-monitor-ui-'));
  const [http,bus,rgb]=await Promise.all([port(),port(),port()]);
  assert(new Set([http,bus,rgb]).size===3);
  const identity={position_m:[0,0,0],quaternion_xyzw:[0,0,0,1]};
  const config={yoke_calibration:{id:'BROWSER-TEST-ONLY',aircraft_camera:identity,tag_yoke:identity,aircraft_yoke_zero:identity,
    roll_axis:[0,0,1],pitch_axis:[0,0,1],pitch_zero_mm:85,min_confidence:.5,max_rotation_residual_deg:2,max_translation_residual_mm:2,
    model:{roll_sign:1,roll_offset_rad:0,pitch_sign:-1,pitch_offset_m:0}}};
  const configPath=path.join(directory,'monitor.json'),controlPath=path.join(directory,'fixture.json');
  fs.writeFileSync(configPath,JSON.stringify(config)); fs.writeFileSync(controlPath,'{}');
  const web=spawn(binary,['--port',String(http),'--subscribe',`tcp://127.0.0.1:${bus}`,'--preview',`tcp://127.0.0.1:${rgb}`,'--config',configPath]);
  children.push(web); let serverErrors=''; web.stderr.on('data',chunk=>serverErrors+=chunk);
  const fixture=spawn(python,[path.join(__dirname,'monitor_fixture.py'),`tcp://127.0.0.1:${bus}`,`tcp://127.0.0.1:${rgb}`,controlPath]);
  children.push(fixture); fixture.stderr.on('data',chunk=>serverErrors+=chunk);
  let browser;
  try {
    for (let i=0;i<100;i++) {try{if((await fetch(`http://127.0.0.1:${http}/api/state`)).ok)break;}catch{}await new Promise(r=>setTimeout(r,50));}
    browser=await chromium.launch({executablePath:process.env.AVIATOR_CHROME || '/usr/bin/google-chrome',headless:true,
      args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
    const page=await browser.newPage({viewport:{width:1920,height:1080}}),errors=[];
    page.setDefaultTimeout(30000);
    page.on('pageerror',e=>errors.push(e.message));
    await page.goto(`http://127.0.0.1:${http}/`,{waitUntil:'domcontentloaded'});
    await page.waitForFunction(()=>document.getElementById('command-roll').textContent.includes('+35.0'),{timeout:15000});
    await page.waitForFunction(()=>document.getElementById('model-loading').hidden,{timeout:30000});
    await page.waitForFunction(()=>document.getElementById('actual-pitch').textContent==='73.1 mm');
    await page.waitForFunction(()=>!document.getElementById('camera-image').hidden && document.getElementById('camera-image').naturalWidth>0);
    await page.waitForFunction(()=>!document.getElementById('command-marker').hasAttribute('hidden') && !document.getElementById('measured-marker').hasAttribute('hidden'));
    // Capture the active viewer through its render hook without adding a production debug API.
    await page.evaluate(async()=>{
      const {Viewer}=await import('/assets/viewer.js'),update=Viewer.prototype.updateHelpers;
      Viewer.prototype.updateHelpers=function(){window.testViewer=this;return update.call(this);};
    });
    await page.waitForFunction(()=>window.testViewer?.robot);
    // Verify camera-derived roll and the full negative pitch stroke in the real
    // URDF loader, including the endpoint outside the original 165 mm limit.
    for (const [travel,expected] of [[-.085,0],[0,-.085],[.085,-.170]]) {
      fs.writeFileSync(controlPath,JSON.stringify({wheel_translation:travel,wheel_roll:-.3}));
      await page.waitForFunction(expected=>{
        const v=window.testViewer,g=v.data.yoke_observation;
        return v.isLive(g) && g.current?.pose_mapping==='CAMERA_STEERING_WHEEL' &&
          !v.invalidGroups.has('yoke_observation') &&
          Math.abs(v.robot.joints.pitch_input_joint.angle-expected)<1e-5 &&
          Math.abs(v.robot.joints.roll_input_joint.angle-.3)<1e-5;
      },expected);
    }
    // Valid camera motion must reach the real URDF joints despite an axis
    // warning, nominal limit overshoot, old capture time, or another preview ID.
    fs.writeFileSync(controlPath,JSON.stringify({wheel_translation:.12,wheel_roll:1.2,
      wheel_axis_match:false,camera_id:'different-from-preview',camera_sample_delay_us:1000000}));
    await page.waitForFunction(()=>{
      const v=window.testViewer,g=v.data.yoke_observation;
      return v.isLive(g) && !v.invalidGroups.has('yoke_observation') &&
        Math.abs(v.robot.joints.pitch_input_joint.angle+.205)<1e-5 &&
        Math.abs(v.robot.joints.roll_input_joint.angle+1.2)<1e-5 &&
        !document.getElementById('model-state').textContent.includes('驾驶盘：');
    });
    fs.writeFileSync(controlPath,JSON.stringify({wheel_valid:false}));
    await page.waitForFunction(()=>window.testViewer.data.yoke_observation.measurement_state==='INVALID');
    assert(await page.evaluate(()=>Math.abs(window.testViewer.robot.joints.pitch_input_joint.angle+.205)<1e-5));
    fs.writeFileSync(controlPath,JSON.stringify({legacy_camera:true}));
    await page.waitForFunction(()=>{
      const v=window.testViewer;
      return v.data.yoke_observation.current &&
        !v.data.yoke_observation.current.pose_mapping &&
        !v.robot.joints.pitch_input_joint.ignoreLimits && !v.robot.joints.roll_input_joint.ignoreLimits &&
        Math.abs(v.robot.joints.pitch_input_joint.angle+.0731)<1e-5;
    });
    fs.writeFileSync(controlPath,'{}');
    await page.waitForFunction(()=>window.testViewer.data.yoke_observation.current?.pose_mapping==='CAMERA_STEERING_WHEEL');
    await page.screenshot({path:path.join(directory,'camera-wheel.png')});
    console.log('Camera wheel checks passed: roll, pitch endpoints/midpoint, axis warning, limit overshoot, receive freshness, invalid feedback, legacy fallback.');
    // Exercise the real HandState -> overview -> URDF path, including the
    // endpoints, distinct channels, and recursively coupled mimic joints.
    async function handPose(positions,sides=['left','right']) {
      await page.waitForFunction(({positions,sides})=>{
        const v=window.testViewer,channels=['thumb_1','thumb_2','index_1','middle_1','ring_1','little_1'];
        return sides.every(side=>{
          const group=v.data.hands[side];
          return group.current && v.isLive(group) &&
            channels.every((channel,i)=>{
              const j=v.robot.joints[`${side}_${channel}_joint`];
              return Math.abs(j.angle-(j.limit.lower+(1-positions[i])*(j.limit.upper-j.limit.lower)))<1e-5;
            }) && Object.values(v.robot.joints).filter(j=>j.name.startsWith(`${side}_`) && j.mimicJoint).every(j=>{
              const expected=v.robot.joints[j.mimicJoint].angle*j.multiplier+j.offset;
              return Math.abs(j.angle-Math.min(j.limit.upper,Math.max(j.limit.lower,expected)))<1e-5;
            }) && !v.invalidGroups.has(`hands.${side}`);
        });
      },{positions,sides});
    }
    await handPose([.1,.2,.3,.4,.5,.6]);
    for (const position of [0,1,.5]) {
      fs.writeFileSync(controlPath,JSON.stringify({hand_positions:Array(6).fill(position)}));
      await handPose(Array(6).fill(position));
    }
    fs.writeFileSync(controlPath,JSON.stringify({hand_positions:Array(6).fill(0),right_hand_invalid:true}));
    await handPose(Array(6).fill(0),['left']);
    assert(await page.evaluate(()=>{
      const v=window.testViewer;
      return v.data.hands.right.current===null &&
        Math.abs(v.robot.joints.right_index_1_joint.angle-v.robot.joints.right_index_1_joint.limit.upper*.5)<1e-5 &&
        v.materials.filter(m=>m.kind==='hands.right').every(m=>m.material.color.getHexString()==='999999');
    }));
    fs.writeFileSync(controlPath,'{}');
    await handPose([.1,.2,.3,.4,.5,.6]);
    // A running older backend has valid feedback but no pose_mapping marker.
    // Refreshing the frontend must still animate it using the loaded URDF.
    await page.route('**/api/overview',async route=>{
      const response=await route.fetch(),overview=await response.json();
      for (const side of ['left','right']) {
        const current=overview.hands[side].current;
        if (current) { delete current.pose_mapping; current.pose_state='UNCALIBRATED'; }
      }
      await route.fulfill({response,json:overview});
    });
    fs.writeFileSync(controlPath,JSON.stringify({hand_positions:Array(6).fill(.75)}));
    await handPose(Array(6).fill(.75));
    assert(await page.evaluate(()=>window.testViewer.data.hands.left.current.pose_mapping===undefined));
    await page.unroute('**/api/overview');
    fs.writeFileSync(controlPath,'{}');
    await handPose([.1,.2,.3,.4,.5,.6]);
    assert(await page.evaluate(()=>{
      const v=window.testViewer;
      return v.linkAxes.length===Object.keys(v.robot.links).length &&
        v.jointAxes.length===Object.values(v.robot.joints).filter(j=>j.jointType!=='fixed').length &&
        v.linkAxes.every(({helper})=>!helper.visible) &&
        v.jointAxes.every(({helper})=>!helper.visible) && v.grid.visible &&
        v.materials.every(({material})=>material.opacity===1 && !material.transparent && material.depthWrite);
    }));
    assert(await page.locator('#viewport-gizmo').isVisible());
    assert.strictEqual(await page.locator('#grid-toggle').getAttribute('aria-pressed'),'true');
    assert.strictEqual(await page.locator('#axes-toggle').getAttribute('aria-pressed'),'false');
    assert.strictEqual(await page.locator('#joint-axes-toggle').getAttribute('aria-pressed'),'false');
    assert.strictEqual(await page.locator('#cockpit-opacity').inputValue(),'1');
    assert.strictEqual(await page.locator('#robot-opacity').inputValue(),'100');
    await page.locator('#grid-toggle').click();
    assert.strictEqual(await page.locator('#grid-toggle').getAttribute('aria-pressed'),'false');
    await page.locator('#grid-toggle').click();
    await page.locator('#joint-axes-toggle').click();
    assert.strictEqual(await page.locator('#joint-axes-toggle').getAttribute('aria-pressed'),'true');
    await page.locator('#axes-toggle').click();
    assert.strictEqual(await page.locator('#axes-toggle').getAttribute('aria-pressed'),'true');
    await page.locator('#robot-opacity').fill('50');
    assert(await page.evaluate(()=>{
      const v=window.testViewer;
      return v.linkAxes.every(({helper})=>helper.visible) && v.jointAxes.every(({helper})=>helper.visible) &&
        v.linkAxes.every(({helper,target})=>helper.matrix.equals(target.matrixWorld)) &&
        v.materials.filter(m=>m.kind.startsWith('arms.') || m.kind.startsWith('hands.')).every(m=>m.material.opacity===.5) &&
        v.materials.filter(m=>m.kind==='aircraft').every(m=>m.material.opacity===1);
    }));
    for (const opacity of [0,.5,1]) {
      await page.locator('#cockpit-opacity').fill(String(opacity));
      assert(await page.evaluate(opacity=>{
        const materials=window.testViewer.materials;
        return ['aircraft','yoke_observation'].every(kind=>{
          const group=materials.filter(m=>m.kind===kind);
          return group.length>0 && group.every(m=>m.material.opacity===opacity && m.material.transparent===(opacity<1));
        }) && materials.filter(m=>m.kind.startsWith('arms.') || m.kind.startsWith('hands.')).every(m=>m.material.opacity===.5);
      },opacity));
    }
    await page.locator('#cockpit-opacity').fill('1');
    await page.locator('#viewport-gizmo').click({position:{x:70,y:50}});
    await page.locator('#reset-view').click();
    await page.locator('#robot-opacity').fill('100');
    assert(await page.evaluate(()=>window.testViewer.materials
      .filter(m=>m.kind.startsWith('arms.') || m.kind.startsWith('hands.'))
      .every(m=>m.material.opacity===1 && !m.material.transparent && m.material.depthWrite)));
    await page.locator('#robot-opacity').fill('0');
    assert(await page.evaluate(()=>window.testViewer.materials
      .filter(m=>m.kind.startsWith('arms.') || m.kind.startsWith('hands.'))
      .every(m=>m.material.opacity===0)));
    await page.locator('#robot-opacity').fill('100');
    await page.locator('#joint-axes-toggle').click();
    await page.locator('#axes-toggle').click();
    await page.locator('#reload-model').click();
    await page.waitForFunction(()=>document.getElementById('model-loading').hidden);
    assert(await page.evaluate(()=>{
      const v=window.testViewer;
      return v.linkAxes.every(({helper})=>!helper.visible) && v.jointAxes.every(({helper})=>!helper.visible) &&
        v.grid.visible && v.scene.children.filter(o=>o.type==='AxesHelper').length===v.linkAxes.length;
    }));
    await page.screenshot({path:path.join(directory,'overview.png')});
    await page.locator('#publisher-lights button').filter({hasText:'Camera'}).click();
    await page.waitForFunction(()=>document.querySelectorAll('#rows tr').length===1 && document.getElementById('rows').textContent.includes('camera.detection'));
    await page.locator('#message-filter').fill('');
    await page.waitForFunction(()=>document.querySelectorAll('#rows tr').length>=5);
    const rows=page.locator('#rows tr'); await rows.filter({hasText:'arm.state'}).first().click();
    await page.waitForFunction(()=>document.getElementById('detail').textContent.includes('joint_position'));
    assert(await page.locator('#service-rows').textContent().then(s=>s.includes('ACCEPTED')));
    await page.locator('#pause-display').click();
    const frozen=await page.locator('#rows').textContent(); await page.waitForTimeout(300);
    assert.strictEqual(await page.locator('#rows').textContent(),frozen);
    await page.screenshot({path:path.join(directory,'messages.png')});
    await page.locator('#pause-display').click(); await page.locator('#tab-overview').click();
    fs.writeFileSync(controlPath,JSON.stringify({camera_stale:true}));
    await page.waitForFunction(()=>document.getElementById('camera-overlay').textContent.includes('过期') && document.getElementById('actual-pitch').textContent==='—' && document.getElementById('command-roll').textContent==='+35.0%');
    assert.strictEqual(await page.locator('#actual-pitch').textContent(),'—');
    assert.strictEqual(await page.locator('#command-roll').textContent(),'+35.0%');
    assert(await page.locator('#measured-marker').evaluate(e=>e.hasAttribute('hidden')));
    await page.screenshot({path:path.join(directory,'camera-stale.png')});
    await page.setViewportSize({width:1600,height:900});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth));
    fs.writeFileSync(controlPath,'{}');
    await page.waitForFunction(()=>document.getElementById('actual-pitch').textContent==='73.1 mm');
    // An HTTP stall must expire values locally before the next successful request.
    await page.route('**/api/overview',route=>new Promise(resolve=>setTimeout(()=>{route.abort();resolve();},700)));
    await page.waitForFunction(()=>document.getElementById('command-roll').textContent==='—');
    assert.deepStrictEqual(errors,[]);
    assert.strictEqual(serverErrors,'');
    console.log('Browser checks passed: URDF, two tabs, RGB, camera-only timeout, recovery, HTTP stall, responsive layout.');
    console.log('Screenshots:',directory);
  } finally {
    if(browser) await browser.close();
    for(const child of children) child.kill('SIGTERM');
    await Promise.all(children.map(child=>new Promise(resolve=>{if(child.exitCode!==null)resolve();else child.once('exit',resolve);}))); 
  }
}
main().catch(error=>{console.error(error);for(const child of children)child.kill('SIGKILL');process.exitCode=1;});

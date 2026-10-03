import {THREE, URDFLoader, OrbitControls, ViewportGizmo} from './vendor.js';
const $ = id => document.getElementById(id);
function kindFor(name) {
  if (name.startsWith('AR5-5_07L')) return 'arms.left';
  if (name.startsWith('AR5-5_07R')) return 'arms.right';
  if (name.startsWith('left_') || name === 'l_base_link') return 'hands.left';
  if (name.startsWith('right_') || name === 'r_base_link') return 'hands.right';
  if (name === 'steering_wheel') return 'yoke_observation';
  return 'aircraft';
}
const groupLabels={'arms.left':'左机械臂','arms.right':'右机械臂','hands.left':'左灵巧手','hands.right':'右灵巧手','yoke_observation':'驾驶盘'};
const stateLabels={STALE:'数据过期',UNAVAILABLE:'等待反馈',INVALID:'反馈无效',UNCALIBRATED:'未标定',SOURCE_CONFLICT:'来源冲突',CLOCK_UNKNOWN:'时钟域未知',FUTURE:'采样时间异常'};
export class Viewer {
  constructor(container) {
    this.container=container; this.data=null; this.isLive=()=>false; this.robot=null;
    this.generation=0; this.invalidGroups=new Set(); this.visible=false; this.frame=0; this.lastFrame=0; this.materials=[]; this.sources=new Map();
    this.scene=new THREE.Scene(); this.scene.background=new THREE.Color('#505050');
    this.camera=new THREE.PerspectiveCamera(42,1,.01,100);
    this.camera.up.set(0,0,1); this.camera.position.set(1.7,-2.2,1.3);
    this.renderer=new THREE.WebGLRenderer({antialias:true});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));
    this.renderer.outputColorSpace=THREE.SRGBColorSpace;
    container.append(this.renderer.domElement);
    this.controls=new OrbitControls(this.camera,this.renderer.domElement);
    this.controls.enableDamping=true; this.controls.dampingFactor=.6; this.controls.target.set(0,0,.5);
    this.scene.add(new THREE.HemisphereLight(0xffffff,0x777777,2));
    const key=new THREE.DirectionalLight(0xffffff,3); key.position.set(2,-3,5); this.scene.add(key);
    this.grid=new THREE.GridHelper(6,30,0xaaaaaa,0x777777); this.grid.rotation.x=Math.PI/2; this.scene.add(this.grid);
    this.linkAxes=[]; this.jointAxes=[];
    this.gizmo=new ViewportGizmo(this.camera,this.renderer,{container,size:100,placement:'bottom-left',id:'viewport-gizmo',offset:{left:8,bottom:8}});
    this.gizmo.attachControls(this.controls);
    this.resizeObserver=new ResizeObserver(() => this.resize()); this.resizeObserver.observe(container);
    $('fit-model').onclick=()=>this.fit(); $('reset-view').onclick=()=>this.fit();
    $('grid-toggle').onclick=()=>{this.grid.visible=!this.grid.visible; $('grid-toggle').setAttribute('aria-pressed',String(this.grid.visible));};
    for (const id of ['axes-toggle','joint-axes-toggle']) $(id).onclick=()=>{
      $(id).setAttribute('aria-pressed',String($(id).getAttribute('aria-pressed')!=='true')); this.updateHelpers();
    };
    $('robot-opacity').oninput=()=>this.style(); $('cockpit-opacity').oninput=()=>this.style(); $('reload-model').onclick=()=>this.load();
    this.load();
  }
  resize() {
    const width=this.container.clientWidth,height=this.container.clientHeight;
    if (!width || !height) return;
    this.camera.aspect=width/height; this.camera.updateProjectionMatrix(); this.renderer.setSize(width,height,false); this.gizmo.update();
  }
  note(message,failed=false) {
    $('model-loading').textContent=message; $('model-loading').classList.toggle('failed',failed);
    $('model-loading').hidden=!message;
  }
  disposeRobot(robot) {
    robot?.traverse(object=>{if(object.isMesh){object.geometry?.dispose(); const materials=Array.isArray(object.material)?object.material:[object.material]; materials.forEach(m=>m.dispose());}});
  }
  async load() {
    const generation=++this.generation; this.note('正在加载模型结构…'); this.loaded=false;
    if (this.robot) { this.scene.remove(this.robot); this.disposeRobot(this.robot); this.robot=null; }
    this.clearHelpers(); this.sources.clear(); this.materials=[];
    try {
      const r=await fetch('/api/model-manifest'); if (!r.ok) throw new Error(`HTTP ${r.status}`);
      const manifest=await r.json(); if (generation !== this.generation) return;
      if (manifest.resource_errors.length) throw new Error(`缺失资源：${manifest.resource_errors.join(', ')}`);
      const manager=new THREE.LoadingManager(); const failures=[];
      const resourceURL = url => {
        const path = new URL(url, location.href).pathname;
        return new URL(manifest.aliases[path] ?? path, location.origin).href;
      };
      manager.setURLModifier(resourceURL);
      let robot=null;
      manager.onProgress=(url,done,total)=>{if(generation===this.generation) this.note(`网格加载 ${done} / ${total}`);};
      manager.onError=url=>failures.push(url);
      manager.onLoad=()=>{
        if (generation !== this.generation) { this.disposeRobot(robot); return; }
        if (failures.length) { this.disposeRobot(robot); this.note(`模型资源失败：${failures.join(', ')}`,true); return; }
        this.robot=robot;
        if (!robot) { this.note('模型结构未完成',true); return; }
        for (const [name,limits] of Object.entries(manifest.yoke_display_limits ?? {})) {
          if (robot.joints[name]) Object.assign(robot.joints[name].limit,limits);
        }
        robot.traverse(object=>{
          if (!object.isMesh) return;
          let link=object; while(link.parent && !link.isURDFLink) link=link.parent;
          const kind=kindFor(link.name);
          const existing=Array.isArray(object.material)?object.material:[object.material];
          const cloned=existing.map(original=>{
            const material=original.clone(); material.side=THREE.DoubleSide;
            this.materials.push({material,kind,color:material.color.clone()}); return material;
          });
          object.material=Array.isArray(object.material)?cloned:cloned[0];
        });
        this.scene.add(robot); this.createHelpers(); this.loaded=true; this.apply(); this.fit(); this.note(''); this.style();
      };
      const loader=new URDFLoader(manager); loader.parseVisual=true; loader.parseCollision=false;
      // Reserve browser connections for live state/preview requests during remote downloads.
      // Count queued meshes immediately so LoadingManager cannot finish before the queue drains.
      const queue = []; let active = 0;
      const pump = () => {
        while (active < 3 && queue.length) {
          const {url, material, done} = queue.shift();
          if (generation !== this.generation) { manager.itemEnd(url); continue; }
          active++;
          const childManager = new THREE.LoadingManager();
          childManager.setURLModifier(resourceURL);
          let finished = false;
          const finish = (mesh, error) => {
            if (finished) return;
            finished = true;
            try {
              if (generation === this.generation) done(mesh, error);
              else this.disposeRobot(mesh);
            } finally {
              if (error) manager.itemError(url);
              active--; manager.itemEnd(url); pump();
            }
          };
          try { loader.defaultMeshLoader(resourceURL(url), childManager, material, finish); }
          catch (error) { finish(null, error); }
        }
      };
      loader.loadMeshCb = (url, unusedManager, material, done) => {
        manager.itemStart(url); queue.push({url, material, done}); pump();
      };
      loader.load(resourceURL(manifest.model_url), result=>{robot=result; this.note('结构已解析，正在加载网格…');}, undefined,error=>{if(generation===this.generation) this.note(`模型加载失败：${error.message}`,true);});
    } catch(error) { if (generation===this.generation) this.note(`模型加载失败：${error.message}`,true); }
  }
  clearHelpers() {
    for (const {helper} of [...this.linkAxes,...this.jointAxes]) {
      this.scene.remove(helper);
      helper.traverse(object=>{object.geometry?.dispose();
        const materials=Array.isArray(object.material)?object.material:[object.material];
        materials.forEach(material=>material?.dispose());});
    }
    this.linkAxes=[]; this.jointAxes=[];
  }
  createHelpers() {
    const add=(target,helper,list)=>{
      helper.matrixAutoUpdate=false;
      helper.traverse(object=>{if(object.material) {
        object.material.depthTest=false; object.material.depthWrite=false; object.renderOrder=10;
      }});
      this.scene.add(helper); list.push({target,helper});
    };
    for (const link of Object.values(this.robot.links)) add(link,new THREE.AxesHelper(.055),this.linkAxes);
    for (const joint of Object.values(this.robot.joints)) {
      if (joint.jointType==='fixed') continue;
      const helper=new THREE.Group(),axis=joint.axis.clone().normalize();
      helper.add(new THREE.ArrowHelper(axis,new THREE.Vector3(),.10,0xffcc55,.022,.012));
      if (joint.jointType==='revolute' || joint.jointType==='continuous') {
        // Positive rotation follows the right-hand rule about the URDF axis.
        const arc=new THREE.Group(),points=[],radius=.035,end=Math.PI*1.5;
        for(let i=0;i<=32;i++) {const a=end*i/32;points.push(new THREE.Vector3(radius*Math.cos(a),radius*Math.sin(a),0));}
        arc.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:0xff9955})));
        arc.add(new THREE.ArrowHelper(new THREE.Vector3(-Math.sin(end),Math.cos(end),0),points.at(-1),.016,0xff9955,.012,.009));
        arc.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),axis); helper.add(arc);
      }
      add(joint,helper,this.jointAxes);
    }
    this.updateHelpers();
  }
  updateHelpers() {
    this.robot?.updateMatrixWorld(true);
    for (const [list,id] of [[this.linkAxes,'axes-toggle'],[this.jointAxes,'joint-axes-toggle']])
      for (const {target,helper} of list) {
        helper.visible=$(id).getAttribute('aria-pressed')==='true';
        if(helper.visible) { helper.matrix.copy(target.matrixWorld); helper.matrixWorldNeedsUpdate=true; }
      }
  }
  fit() {
    if (!this.robot) return;
    this.robot.updateMatrixWorld(true);
    const box=new THREE.Box3();
    for (const name of ['AR5-5_07L-W4C4A2_base','AR5-5_07R-W4C4A2_base','steering_wheel']) {
      const link=this.robot.links[name]; if(link) box.union(new THREE.Box3().setFromObject(link));
    }
    if (box.isEmpty()) box.setFromObject(this.robot);
    const center=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3());
    const distance=Math.max(size.x,size.y,size.z,.4)/(2*Math.tan(THREE.MathUtils.degToRad(this.camera.fov/2)))*1.35/Math.min(this.camera.aspect,1);
    this.controls.target.copy(center); this.camera.position.copy(center).add(new THREE.Vector3(1,-1.6,.9).normalize().multiplyScalar(distance));
    this.camera.far=Math.max(100,distance*10); this.camera.updateProjectionMatrix(); this.controls.update();
  }
  setVisible(visible) {
    if (this.visible===visible) return;
    this.visible=visible;
    if (visible) { this.resize(); this.animate(); }
    else { cancelAnimationFrame(this.frame); this.frame=0; }
  }
  setData(data,isLive) { this.data=data; this.isLive=isLive; }
  clearSamples() { this.sources.clear(); if(this.robot) for(const joint of Object.values(this.robot.joints)) if(!joint.mimicJoint) joint.setJointValue(0); }
  groups() {
    return {'arms.left':this.data?.arms.left,'arms.right':this.data?.arms.right,
      'hands.left':this.data?.hands.left,'hands.right':this.data?.hands.right,'yoke_observation':this.data?.yoke_observation};
  }
  modelJoints(kind,g) {
    const current=g?.current;
    if (!kind.startsWith('hands.') || !current || Object.keys(current.model_joints ?? {}).length) return current?.model_joints;
    // Older monitor processes expose the same measured drive feedback but no
    // pose_mapping field. An empty model_joints still needs the URDF mapping.
    const positions=current.drive_position_normalized;
    if (!Array.isArray(positions) || positions.length!==6 ||
        positions.some(p=>!Number.isFinite(p) || p<0 || p>1)) return null;
    const side=kind.split('.')[1],values={};
    const channels=['thumb_1','thumb_2','index_1','middle_1','ring_1','little_1'];
    for (const [i,channel] of channels.entries()) {
      const name=`${side}_${channel}_joint`,joint=this.robot?.joints[name];
      if (!joint || joint.mimicJoint || joint.jointType!=='revolute' ||
          !Number.isFinite(joint.limit.lower) || !Number.isFinite(joint.limit.upper) ||
          joint.limit.upper<=joint.limit.lower) return null;
      // ANGLE_ACT is an opening fraction: 0 closed, 1 open. The URDF's
      // zero/lower limit is open; increasing angles close the fingers.
      values[name]=joint.limit.lower+(1-positions[i])*(joint.limit.upper-joint.limit.lower);
    }
    return values;
  }
  apply() {
    if (!this.robot || !this.data) return;
    const notes=[]; this.invalidGroups.clear();
    for (const [kind,g] of Object.entries(this.groups())) {
      const source=g?.source ? `${g.source.publisher_id}/${g.source.session_id}` : null;
      const old=this.sources.get(kind);
      if (old && source && old.key!==source) for (const name of old.joints) this.robot.joints[name]?.setJointValue(0);
      if (!this.isLive(g) || !g.current) { const state=g?.measurement_state === 'VALID' ? 'STALE' : g?.measurement_state ?? 'UNAVAILABLE'; notes.push(`${groupLabels[kind]}：${stateLabels[state] ?? state} · ${old ? '旧姿态' : '参考姿态'}`); continue; }
      if (kind==='yoke_observation') {
        // Display valid camera measurements even outside the model's nominal
        // travel. Restore limit checking when returning to legacy calibration.
        for (const name of ['roll_input_joint','pitch_input_joint']) {
          if (this.robot.joints[name]) this.robot.joints[name].ignoreLimits=
            g.current.pose_mapping==='CAMERA_STEERING_WHEEL';
        }
      }
      const values=this.modelJoints(kind,g);
      if (!values && kind.startsWith('hands.')) {
        this.invalidGroups.add(kind); notes.push(`${groupLabels[kind]}：关节映射/限位不匹配`); continue;
      }
      if (!values || !Object.keys(values).length) { notes.push(`${groupLabels[kind]}：姿态未标定`); continue; }
      let valid=true;
      for (const [name,q] of Object.entries(values)) {
        const joint=this.robot.joints[name];
        if (!joint || joint.mimicJoint || !Number.isFinite(q) ||
          (!joint.ignoreLimits && (Math.abs(q)>20 ||
            (joint.jointType!=='continuous' && (q<joint.limit.lower-1e-5 || q>joint.limit.upper+1e-5))))) valid=false;
      }
      if (!valid) { this.invalidGroups.add(kind); notes.push(`${groupLabels[kind]}：关节映射/限位不匹配`); continue; }
      this.sources.set(kind,{key:source,joints:Object.keys(values)});
      // Only interpolate between received targets; no extrapolation.
      for (const [name,q] of Object.entries(values)) {
        const joint=this.robot.joints[name]; const current=joint.angle;
        joint.setJointValue(old?.key===source ? current+(q-current)*.45 : q);
      }
    }
    const labels=$('model-state'); labels.replaceChildren(...notes.map(note=>{const span=document.createElement('span');span.textContent=note;return span;}));
    this.style();
  }
  style() {
    const groups=this.groups();
    const validGroups=new Set(Object.entries(groups).filter(([kind,group])=>
      !this.invalidGroups.has(kind) && this.isLive(group) && group?.current &&
      Object.keys(this.modelJoints(kind,group) ?? {}).length>0).map(([kind])=>kind));
    for(const {material,kind,color} of this.materials) {
      if (kind==='aircraft') { material.opacity=Number($('cockpit-opacity').value); material.transparent=material.opacity<1; material.depthWrite=!material.transparent; continue; }
      material.color.copy(validGroups.has(kind) ? color : new THREE.Color('#999999'));
      const opacity=kind==='yoke_observation' ? Number($('cockpit-opacity').value) : Number($('robot-opacity').value)/100;
      material.opacity=opacity; material.transparent=material.opacity<1; material.depthWrite=!material.transparent;
    }
  }
  animate() {
    if (!this.visible) return;
    this.frame=requestAnimationFrame(()=>this.animate());
    const now=performance.now(); if(now-this.lastFrame<30) return;
    this.lastFrame=now; this.apply(); this.controls.update(); this.updateHelpers(); this.renderer.render(this.scene,this.camera); this.gizmo.render();
  }
}

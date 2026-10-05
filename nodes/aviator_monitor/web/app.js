import './settings.js';
import './system.js';
import {setLogsVisible} from './logs.js';
const $ = id => document.getElementById(id);
const text = (id, value) => { $(id).textContent = value ?? '—'; };
const fmt = (value, digits = 1) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const signed = value => Number.isFinite(value) ? `${value > 0 ? '+' : ''}${value.toFixed(1)}` : '—';
const labels = {VALID:'反馈有效',FRESH:'发布新鲜',STALE:'数据过期',INVALID:'反馈无效',UNCALIBRATED:'未标定',UNAVAILABLE:'等待反馈',UNOBSERVED:'未观测',UNCONFIGURED:'预览未配置',WAITING:'等待首帧',CLOCK_UNKNOWN:'时钟域未知',FUTURE:'采样时间异常',SOURCE_CONFLICT:'来源冲突',ESTIMATED:'驱动反馈估算姿态'};
const reasons = {camera_identity_mismatch:'相机身份与观测配置不匹配',no_sample:'尚无样本',multiple_recent_sources:'近期存在多个来源',clock_domain_mismatch:'采样时钟不可比较',future_sample:'采样时间异常',sample_or_receive_timeout:'采样或接收已超时',producer_invalid:'源端标记反馈无效',side_sample_time_missing:'缺少侧级采样时间',future_side_sample:'侧级采样时间异常',negative_feedback_age:'反馈年龄异常',side_feedback_timeout:'设备反馈超时',side_feedback_invalid:'设备反馈无效',invalid_drive_feedback:'驱动反馈格式异常',invalid_joint_positions:'关节反馈格式异常',yoke_calibration_missing:'请配置驾驶盘几何标定',invalid_steering_wheel:'相机方向盘观测无效',steering_wheel_axis_mismatch:'方向盘转轴与标定不匹配',target_not_tracking:'未检测到目标',detection_confidence_low:'检测置信指标不足',invalid_target_pose:'目标位姿无效',ambiguous_rotation:'无法确定滚转角',kinematic_residual_exceeded:'观测偏离机构运动约束',physical_feedback_out_of_range:'观测超出允许反馈范围',command_out_of_range:'指令量超出范围',system_fields_missing:'整机状态字段缺失'};
const reasonText = g => reasons[g?.reason] ?? '反馈不可用，请查看系统消息';
let overview = null, overviewAt = 0, overviewCost = 0, online = false;
let tab = 'overview', latestState = null, displayedState = null, pausedAt = 0;
let selected = null, selectedKind = '', detailBusy = false, detailGeneration = 0, detailKey = '';
let busy = false, lastStateFetch = 0, viewer = null, preview = null, previewAt = 0, previewCost = 0;
let imageBusy = false, imageToken = '', imageURL = '', displayedImage = null, imageGeneration = 0, imageError = '';
let previewReceived = [], filterTopic = '';
async function get(path, binary = false) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 2500);
  try {
    const r = await fetch(path, {cache:'no-store',signal:controller.signal});
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    return binary ? await r.blob() : await r.json();
  } finally { clearTimeout(timeout); }
}
function live(g) {
  return online && g?.measurement_state === 'VALID' && Number.isFinite(g.fresh_for_ms) &&
    g.fresh_for_ms > performance.now() - overviewAt + overviewCost;
}
function stateOf(g) {
  return g?.measurement_state === 'VALID' && !live(g) ? 'STALE' : (g?.measurement_state ?? 'UNAVAILABLE');
}
function badge(element, state, custom) {
  element.className = `badge ${state}`;
  element.textContent = custom ?? labels[state] ?? state;
}
function sourceKey(g) {
  const s = g?.source;
  return s ? `${s.topic}/${s.publisher_id}/${s.session_id}` : '';
}
function setTab(next) {
  tab = next;
  setLogsVisible(next === 'logs');
  for (const name of ['overview','messages','resources','logs','settings']) {
    $(name).hidden = name !== next;
    $(`tab-${name}`).setAttribute('aria-selected', String(name === next));
    $(`tab-${name}`).tabIndex = name === next ? 0 : -1;
  }
  viewer?.setVisible(next === 'overview' && !document.hidden);
  if (next === 'messages' && latestState && !pausedAt) renderMessages(latestState);
  if (next === 'overview') cameraRefresh();
}
for (const name of ['overview','messages','resources','logs','settings']) {
  $(`tab-${name}`).onclick = () => setTab(name);
  $(`tab-${name}`).onkeydown = event => {
    if (['ArrowLeft','ArrowRight','Home','End'].includes(event.key)) {
      event.preventDefault(); const tabs = ['overview','messages','resources','logs','settings'];
      const index = event.key === 'Home' ? 0 : event.key === 'End' ? tabs.length-1 :
        (tabs.indexOf(name) + (event.key === 'ArrowRight' ? 1 : -1) + tabs.length) % tabs.length;
      const next = tabs[index];
      setTab(next); $(`tab-${next}`).focus();
    }
  };
}
const lights = new Map();
function goMessages(topic, publisher) {
  filterTopic = topic ?? '';
  $('message-filter').value = publisher ?? '';
  $('status-filter').value = '';
  setTab('messages');
}
function renderLights() {
  const roles = overview?.publishers ?? ['Flight Gateway','Core','Manipulator','Inspire Hand','Camera','Logger','Bus'].map(name => ({name,state:'UNOBSERVED'}));
  for (const role of roles) {
    let row = lights.get(role.name);
    if (!row) {
      row = document.createElement('button'); row.className = 'light-row';
      const dot = document.createElement('span'); dot.className = 'dot';
      const name = document.createElement('span'); name.textContent = role.name;
      const status = document.createElement('span'); row.append(dot,name,status);
      row.onclick = () => goMessages(role.topic, role.source?.publisher_id);
      lights.set(role.name,row); $('publisher-lights').append(row);
    }
    row.onclick = () => goMessages(role.topic, role.source?.publisher_id);
    let state = role.state;
    if (state === 'FRESH' && (!online || role.receive_age_ms + performance.now() - overviewAt + overviewCost >= role.timeout_ms)) state = 'STALE';
    row.firstChild.className = `dot ${state}`;
    row.lastChild.className = state; row.lastChild.textContent = labels[state] ?? state;
    row.title = `${role.name} · 最近接收 ${fmt(role.receive_age_ms)} ms · 阈值 ${fmt(role.timeout_ms)} ms`;
    row.disabled = !role.topic;
  }
}
const deviceCards = new Map();
const driveLabels = ['拇指旋转','拇指','食指','中指','无名指','小指'];
function createDevice(kind, side) {
  const card = document.createElement('div'); card.className = 'device-card';
  const header = document.createElement('div'); header.className = 'device-title';
  const name = document.createElement('strong'); name.textContent = `${side === 'left' ? '左' : '右'}${kind === 'arms' ? '机械臂' : '灵巧手'}`;
  const status = document.createElement('span'); header.append(name,status);
  const note = document.createElement('p');
  const details = document.createElement('details'); const summary = document.createElement('summary');
  summary.textContent = kind === 'arms' ? '查看实际关节反馈' : '查看实际驱动位置'; details.append(summary);
  const values = [];
  for (let i = 0; i < (kind === 'arms' ? 7 : 6); i++) {
    const row = document.createElement('div'); row.className = kind === 'arms' ? 'joint-row' : 'drive-row';
    const label = document.createElement('span'); label.textContent = kind === 'arms' ? `J${i+1}` : driveLabels[i];
    const value = document.createElement('span');
    let progress = null;
    if (kind === 'hands') { progress = document.createElement('progress'); progress.max = 1; row.append(label,progress,value); }
    else row.append(label,value);
    details.append(row); values.push({value,progress});
  }
  card.append(header,note,details); $('devices').append(card);
  return {card,status,note,values};
}
function renderDevices() {
  for (const kind of ['arms','hands']) for (const side of ['left','right']) {
    const key = `${kind}.${side}`;
    if (!deviceCards.has(key)) deviceCards.set(key, createDevice(kind,side));
    const {status,note,values} = deviceCards.get(key), g = overview?.[kind]?.[side];
    const current = live(g) ? g.current : null, state = stateOf(g);
    status.className = state; status.textContent = labels[state] ?? state;
    note.textContent = current ? (kind === 'hands' ?
      `${labels[current.pose_state]} · 握持${current.grasp_verified ? '已验证' : '未验证'}` :
      `采样 ${fmt(g.sample_age_ms + performance.now() - overviewAt)} ms · ${current.status ?? '—'}`) :
      `${reasonText(g)}${g?.source ? ' · 旧姿态仅供参考' : ''}`;
    if (Number.isFinite(current?.error_code) && current.error_code !== 0) { note.textContent += ` · 错误 ${current.error_code}`; status.className='INVALID'; }
    for (let i = 0; i < values.length; i++) {
      const value = kind === 'arms' ? current?.joint_position_rad?.[i] : current?.drive_position_normalized?.[i];
      values[i].value.textContent = Number.isFinite(value) ? (kind === 'arms' ? `${fmt(value*180/Math.PI)}°` : `${fmt(value*100,0)}%`) : '—';
      if (values[i].progress) { values[i].progress.value = value ?? 0; values[i].progress.hidden = !Number.isFinite(value); }
    }
  }
}
function renderOverview() {
  const sys = live(overview?.system) ? overview.system.current : null;
  text('system', sys?.state ?? (online ? labels[stateOf(overview?.system)] : '状态未知'));
  text('source', sys?.control_source); text('errors', sys ? `${sys.current_error_code ?? '—'} / ${sys.last_error_code ?? '—'}` : '—');
  for (const id of ['system','source','errors']) $(id).title = $(id).textContent;
  text('connection', online ? '监控服务在线' : '监控服务不可用');
  $('connection').className = `badge ${online ? 'FRESH' : 'STALE'}`;
  text('notice', !online ? '连接中断 · 保留画面为旧快照' :
    stateOf(overview?.system) === 'SOURCE_CONFLICT' ? '整机来源冲突，请核对部署' : '消息新鲜度与硬件状态分别判定');
  renderLights(); renderDevices();
  const yg = overview?.yoke_observation, y = live(yg) ? yg.current : null;
  badge($('yoke-state'),stateOf(yg));
  text('actual-roll',y ? `${signed(y.roll_deg)}°` : '—');
  text('actual-pitch',y ? `${fmt(y.pitch_mm)} mm` : '—');
  text('actual-roll-percent',y ? `${signed(y.roll_percent)}%` : '—');
  text('actual-pitch-percent',y ? `${signed(y.pitch_percent)}%` : '—');
  for (const [id,value] of [['actual-roll-percent',y?.roll_percent],['actual-pitch-percent',y?.pitch_percent]])
    $(id).className = Number.isFinite(value) && Math.abs(value)>100 ? 'STALE' : 'measured';
  text('yoke-meta',y ? `视觉实测 · 帧 ${y.frame_id} · 标定 ${yg.calibration_id} · 样本 ${fmt(yg.sample_age_ms)} ms` : `${labels[stateOf(yg)]} · ${reasonText(yg)}`);
  const cg = overview?.flight_command, c = live(cg) ? cg.current : null;
  text('command-source',c?.control_source); text('command-roll',c ? `${signed(c.roll_percent)}%` : '—'); text('command-pitch',c ? `${signed(c.pitch_percent)}%` : '—');
  text('command-roll-physical',c ? `${signed(c.roll_deg)}°` : '—'); text('command-pitch-physical',c ? `${fmt(c.pitch_mm)} mm` : '—');
  for (const [id,value] of [['roll-meter',c?.roll_percent],['pitch-meter',c?.pitch_percent]]) {
    $(id).value = value ?? 0; $(id).hidden = !Number.isFinite(value);
  }
  $('command-marker').toggleAttribute('hidden',!c); $('measured-marker').toggleAttribute('hidden',!y);
  if (c) $('command-marker').setAttribute('transform',`translate(${140+c.roll_percent},${125-c.pitch_percent})`);
  if (y) { $('measured-marker').setAttribute('cx',140+y.roll_percent); $('measured-marker').setAttribute('cy',125-y.pitch_percent); }
  text('command-meta',c ? `观测指令 · 样本 ${fmt(cg.sample_age_ms)} ms · 消息有效不代表执行到位` : `${labels[stateOf(cg)]} · 无当前有效指令`);
}
const streamRows = new Map(), serviceRows = new Map();
function matched(row) {
  const query = $('message-filter').value.trim().toLowerCase(), status = $('status-filter').value;
  const haystack = [row.topic,row.publisher,row.session,row.operation,row.client,row.request_id,row.target].join(' ').toLowerCase();
  return (!query || haystack.includes(query)) && (!status || row.status === status) && (!filterTopic || row.topic === filterTopic);
}
function renderTable(id, rows, kind, values) {
  const map = kind === 'stream' ? streamRows : serviceRows, body = $(id), ids = new Set(rows.map(r => r.id));
  for (const [key,tr] of map) if (!ids.has(key)) { tr.remove(); map.delete(key); }
  body.querySelector('.empty-row')?.remove();
  rows.forEach((row,index) => {
    let tr = map.get(row.id);
    if (!tr) {
      tr = document.createElement('tr'); tr.tabIndex = 0;
      tr.onclick = () => selectRow(row.id,kind);
      tr.onkeydown = e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); selectRow(row.id,kind); } };
      map.set(row.id,tr);
    }
    tr.className = selected === row.id ? 'selected' : '';
    tr.setAttribute('aria-selected',String(selected === row.id));
    tr.replaceChildren(...values(row).map(v => {
      const td = document.createElement('td');
      if (Array.isArray(v)) { td.textContent = v[0] ?? '—'; const small = document.createElement('small'); small.textContent = v[1] ?? ''; td.append(small); }
      else if (v?.badge) { const span = document.createElement('span'); badge(span,v.badge,v.badge); td.append(span); }
      else td.textContent = v ?? '—';
      return td;
    }));
    if (body.children[index] !== tr) body.insertBefore(tr,body.children[index] ?? null);
  });
  if (!rows.length) { const tr = document.createElement('tr'); const td = document.createElement('td'); td.colSpan = kind === 'service' ? 9 : 8; td.textContent = '暂无符合筛选条件的观测记录'; td.className = 'empty-row'; tr.append(td); body.append(tr); }
}
function renderMessages(data) {
  if (!data) return;
  displayedState = data;
  renderTable('rows',data.streams.filter(matched),'stream',r => [[r.topic,r.publisher],[r.session.slice(0,12),r.session],{badge:r.status},fmt(r.hz),fmt(r.age_ms),fmt(r.receive_age_ms),String(r.sequence),`${r.gaps} / ${r.duplicates}`]);
  renderTable('service-rows',data.services.filter(matched),'service',r => [[r.operation ?? '未知操作',r.target],[r.client,r.session],r.request_id,{badge:r.observation},r.reply_status ? {badge:r.reply_status} : '—',r.reply_status ? (r.observation==='REPLY_ONLY' ? '请求未观测' : '已关联') : {badge:r.status},r.error_code,`${r.request_count} / ${r.reply_count}`,fmt(r.observed_reply_ms)]);
  text('counts',`缓存 ${data.streams.length} / 64 · 淘汰 ${data.evicted}`);
  text('service-counts',`缓存 ${data.services.length} / 64 · 淘汰 ${data.service_evicted}`);
  text('diagnostics',`拒绝 ${data.rejected} · 最近异常 ${data.error || '无'} · 仅内存缓存`);
  if (selected !== null && !pausedAt) updateDetail();
}
function selectRow(id,kind) {
  selected=id; selectedKind=kind; detailKey=''; ++detailGeneration;
  renderMessages(displayedState);
  if (pausedAt) { text('detail-title','消息详情 · 暂停快照'); text('detail','暂停时未缓存该行详情，恢复显示后可查看。'); }
  else updateDetail();
}
async function updateDetail() {
  if (detailBusy || selected === null || pausedAt || !online) return;
  const rows = selectedKind === 'stream' ? displayedState?.streams : displayedState?.services;
  const row = rows?.find(r => r.id === selected);
  if (!row) { ++detailGeneration; text('detail-title','消息详情 · 已移出缓存'); text('detail','选中记录已被缓存淘汰。'); selected=null; return; }
  const key = `${selected}/${row.sequence ?? `${row.request_count}/${row.reply_count}`}`;
  if (key === detailKey) return;
  detailBusy=true; const generation=detailGeneration, id=selected;
  try {
    const detail=await get(`/api/message?id=${id}`);
    if (generation !== detailGeneration || id !== selected || pausedAt) return;
    text('detail-title',`${selectedKind === 'stream' ? row.topic : row.operation ?? '未知操作'} · 只读`);
    text('detail',JSON.stringify(detail,null,2)); detailKey=key;
  } catch(error) { if (generation === detailGeneration) text('detail',`详情不可用：${error.message}`); }
  finally { detailBusy=false; }
}
$('message-filter').oninput = () => { filterTopic=''; renderMessages(displayedState); };
$('status-filter').onchange = () => renderMessages(displayedState);
$('pause-display').onclick = () => {
  pausedAt = pausedAt ? 0 : performance.now(); ++detailGeneration;
  $('pause-display').setAttribute('aria-pressed',String(!!pausedAt));
  text('pause-display',pausedAt ? '恢复显示' : '暂停显示');
  if (pausedAt) text('detail-title','消息详情 · 暂停快照');
  if (!pausedAt) { detailKey=''; renderMessages(latestState); }
};
$('copy-detail').onclick = async () => {
  try { await navigator.clipboard.writeText($('detail').textContent); text('copy-detail','已复制'); }
  catch { text('copy-detail','复制失败'); }
  setTimeout(() => text('copy-detail','复制'),1200);
};
function previewState() {
  let state = preview?.state ?? 'WAITING';
  if (state === 'FRESH' && (!online || !Number.isFinite(preview.fresh_for_ms) || preview.fresh_for_ms <= performance.now()-previewAt+previewCost)) state='STALE';
  return state;
}
function renderCamera() {
  let state = previewState();
  const shown = displayedImage;
  if (imageURL && state === 'FRESH') {
    const identity = m => m ? `${m.publisher_id}/${m.session_id}/${m.camera_id}` : '';
    if (identity(shown?.current) !== identity(preview?.current)) state='WAITING';
    else if (shown.state !== 'FRESH') state=shown.state;
    else if (!Number.isFinite(shown.fresh_for_ms) || shown.fresh_for_ms <= performance.now()-shown.at+shown.cost) state='STALE';
  }
  const current=shown?.current ?? preview?.current, stale=state !== 'FRESH';
  $('camera-frame').classList.toggle('stale',stale || !!imageError);
  $('camera-overlay').hidden = !stale && !!imageURL && !imageError;
  text('camera-overlay',imageError || (state === 'STALE' ? 'RGB 预览已过期' : labels[state] ?? state));
  text('camera-name',preview?.camera_id ?? 'cockpit');
  const age = current ? fmt(shown ? shown.receive_age_ms+performance.now()-shown.at+shown.cost : (preview.receive_age_ms ?? 0)+performance.now()-previewAt+previewCost) : '—';
  previewReceived = previewReceived.filter(t => performance.now()-t < 2000);
  const meta = `${labels[state] ?? state} · 接收 ${fmt(previewReceived.length/2)} fps · 最近接收 ${age} ms${current ? ` · 帧 ${current.frame_id}` : ''}`;
  text('camera-meta',`${meta}${preview?.rejected ? ` · 拒绝 ${preview.rejected}：${preview.error}` : ''}`); text('camera-large-state',meta);
  $('camera-large').classList.toggle('stale',stale || !!imageError);
  $('camera-expand').disabled=!imageURL;
}
async function cameraRefresh() {
  if (imageBusy || tab !== 'overview' || document.hidden) return;
  imageBusy=true; const started=performance.now(), generation=imageGeneration;
  try {
    const data=await get('/api/camera/latest');
    if (generation !== imageGeneration) return;
    preview=data; previewAt=performance.now(); previewCost=previewAt-started;
    if (data.current?.token === imageToken && imageURL) imageError='';
    if (data.current && data.current.token !== imageToken && ['FRESH','STALE','CLOCK_UNKNOWN'].includes(data.state)) {
      const blob=await get(data.current.url,true), url=URL.createObjectURL(blob);
      const image=new Image(); image.src=url;
      try { await image.decode(); }
      catch { URL.revokeObjectURL(url); throw new Error('图像解码失败'); }
      if (generation !== imageGeneration || document.hidden || tab !== 'overview') { URL.revokeObjectURL(url); return; }
      const old=imageURL; imageURL=url; imageToken=data.current.token;
      displayedImage={current:data.current,state:data.state,at:previewAt,cost:previewCost,receive_age_ms:data.receive_age_ms,fresh_for_ms:data.fresh_for_ms};
      $('camera-image').src=url; $('camera-image').hidden=false; $('camera-large').src=url;
      if (old) URL.revokeObjectURL(old);
      previewReceived.push(performance.now()); imageError='';
    }
  } catch(error) { imageError=error.message; }
  finally { imageBusy=false; renderCamera(); }
}
$('camera-expand').onclick = () => $('camera-dialog').showModal();
$('camera-close').onclick = () => $('camera-dialog').close();
async function refresh() {
  if (busy) return;
  busy=true; const started=performance.now();
  try {
    const stateDue = !latestState || performance.now()-lastStateFetch >= 100;
    if (stateDue) lastStateFetch=performance.now();
    const [ov,state]=await Promise.all([get('/api/overview'),stateDue ? get('/api/state') : Promise.resolve(latestState)]);
    if (overview && overview.monitor_session_id !== ov.monitor_session_id) {
      selected=null; ++detailGeneration; detailKey=''; ++imageGeneration; imageToken=''; preview=null; previewReceived=[];
      if (imageURL) URL.revokeObjectURL(imageURL); imageURL=''; displayedImage=null; $('camera-image').hidden=true; $('camera-large').removeAttribute('src');
      viewer?.clearSamples(); displayedState=null; pausedAt=0;
      text('detail','监控服务会话已变化，请重新选择记录。'); text('pause-display','暂停显示'); $('pause-display').setAttribute('aria-pressed','false');
    }
    if (overview && (overview.config_revision !== ov.config_revision || overview.monitor_session_id !== ov.monitor_session_id)) {
      viewer?.clearSamples(); viewer?.load();
      ++imageGeneration; imageToken=''; imageError=''; preview=null; displayedImage=null; previewReceived=[];
      if (imageURL) URL.revokeObjectURL(imageURL);
      imageURL=''; $('camera-image').hidden=true; $('camera-large').removeAttribute('src');
    }
    overview=ov; overviewAt=performance.now(); overviewCost=overviewAt-started; latestState=state; online=true;
    viewer?.setData(overview,live);
    if (tab === 'messages' && !pausedAt && displayedState !== state) renderMessages(state);
  } catch(error) {
    online=false; ++detailGeneration;
    text('detail-title','消息详情 · 旧快照'); text('notice',error.message);
  } finally { busy=false; renderOverview(); renderCamera(); }
}
import('./viewer.js').then(({Viewer}) => {
  viewer=new Viewer($('viewport'));
  viewer.setVisible(tab === 'overview' && !document.hidden);
  if (overview) viewer.setData(overview,live);
}).catch(error => { text('model-loading',`三维显示不可用：${error.message}`); $('model-loading').classList.add('failed'); });
document.addEventListener('visibilitychange',() => {
  viewer?.setVisible(tab === 'overview' && !document.hidden);
  if (!document.hidden) { refresh(); cameraRefresh(); }
});
setInterval(() => { if (!document.hidden || performance.now()-overviewAt > 2000) refresh(); },20);
setInterval(cameraRefresh,67);
setInterval(() => {
  renderOverview(); renderCamera();
  text('pause-note',pausedAt ? `已暂停 ${(performance.now()-pausedAt)/1000|0} s · 表格为暂停快照，后台仍在接收` : '');
},100);
window.addEventListener('beforeunload',() => { if (imageURL) URL.revokeObjectURL(imageURL); });
refresh(); renderOverview();

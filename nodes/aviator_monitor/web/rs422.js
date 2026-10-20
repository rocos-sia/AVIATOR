const $ = id => document.getElementById(id);
const hex = bytes => bytes.map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
const operations = {0:'不发起新操作',16:'进入待机 · enter_standby',32:'握盘 · grasp_wheel',48:'开始控制 · start_control',64:'退出控制 · exit_control',80:'松盘 · leave_wheel',96:'复位错误 · reset_error'};
const states = {FRESH:'报文新鲜',STALE:'报文已过期',WAITING:'等待报文',CLOCK_UNKNOWN:'时钟域未知',FUTURE:'接收时间异常'};
// HTTP snapshots arrive every ~100 ms; the serial lease can be only 25–50 ms.
// Keep the backend's observation separate from the health of the web updates.
const snapshotTimeoutMs = 500;
let latest = null, latestAt = 0, cost = 0, frozen = null, frozenCost = 0, visible = false;
const element = (tag, content, className) => {
  const node = document.createElement(tag);
  if (content != null) node.textContent = content;
  if (className) node.className = className;
  return node;
};
const cards = new Map();
function cardFor(name, saved = false) {
  const key = `${saved ? "saved" : "live"}-${name}`;
  if (cards.has(key)) return cards.get(key);
  const panel=element('section',null,`panel rs422-frame${saved ? ' rs422-operation' : ''}`), heading=element('div',null,'panel-head');
  const title=element('h2'), badge=element('span'), meta=element('p',null,'footnote');
  heading.append(title,badge);
  const raw=element('pre',null,'rs422-hex'), copy=element('button','复制 HEX');
  raw.setAttribute('aria-label',`通道 ${name} ${saved ? "保留操作" : "实时"}原始十六进制报文`);
  const empty=element('p','尚未接收到可用报文','empty-row');
  const table=element('table'), head=element('thead'), header=element('tr'), body=element('tbody');
  ['字节偏移','字段','原始 HEX','解析值 / 含义'].forEach(label => header.append(element('th',label)));
  head.append(header); table.append(head,body);
  const scroll=element('div',null,'rs422-table'); scroll.append(table);
  const cells=Array.from({length:7},()=>{
    const row=element('tr'), values=Array.from({length:4},()=>element('td'));
    row.append(...values); body.append(row); return values;
  });
  const note=element('p','位置字段：int16 小端，1 = 0.1%，有效范围 −1000～1000。操作码仅表示收到的请求，执行结果查看 REQ / REP。','footnote');
  panel.append(heading,meta,empty,raw,copy,scroll,note);
  const card={panel,title,badge,meta,empty,raw,copy,scroll,note,cells,bytes:[]};
  copy.onclick=async()=>{
    try { await navigator.clipboard.writeText(hex(card.bytes)); copy.textContent='已复制'; }
    catch { copy.textContent='复制失败'; }
    setTimeout(()=>{copy.textContent='复制 HEX';},1200);
  };
  cards.set(key,card); return card;
}
function drawFrame(card, bytes) {
  card.bytes = bytes;
  card.raw.textContent=`00  ${hex(bytes.slice(0,16))}\n10  ${hex(bytes.slice(16))}`;
  const i16 = offset => {
    const value = bytes[offset] | bytes[offset+1] << 8;
    return value > 32767 ? value-65536 : value;
  };
  const checksum = bytes.slice(2,31).reduce((sum,b) => sum+b,0) & 255;
  const axis = offset => `${i16(offset)} → ${(i16(offset)/10).toFixed(1)}% · 归一化 ${(i16(offset)/1000).toFixed(3)}`;
  const fields = [
    ['0–1','同步字',hex(bytes.slice(0,2)),bytes[0] === 235 && bytes[1] === 144 ? 'EB 90 · 帧头匹配' : '帧头异常'],
    ['2','帧计数器',hex(bytes.slice(2,3)),`${bytes[2]} · 0–255 循环`],
    ['3','状态操作码',hex(bytes.slice(3,4)),operations[bytes[3]] ?? '未知操作码'],
    ['4–5','滚转指令',hex(bytes.slice(4,6)),axis(4)],
    ['6–7','俯仰指令',hex(bytes.slice(6,8)),axis(6)],
    ['8–30','保留位',hex(bytes.slice(8,31)),bytes.slice(8,31).some(Boolean) ? '含非零字节 · 保留原样显示' : '全零'],
    ['31','累加校验',hex(bytes.slice(31)),`${checksum === bytes[31] ? '通过' : '失败'} · 字节 2–30 之和低 8 位 = ${hex([checksum])}`]
  ];
  fields.forEach((values,i)=>values.forEach((value,j)=>{card.cells[i][j].textContent=value;}));
}
const outcomes = {
  RECEIVED: '已收到 · 尚无关联的 Core 请求',
  CONFIRMED: '已通过操作确认 · 请求结果未观测到或已移出缓存',
  WAITING: '已提交 Core · 等待应答',
  NOT_SENT: '已通过操作确认 · 未提交 Core',
  ACCEPTED: 'Core 已受理 · 动作完成情况请查看机器人状态',
  RUNNING: 'Core 正在执行', COMPLETED: 'Core 返回完成', REJECTED: 'Core 已拒绝',
  FAILED: 'Core 执行失败', CANCELLED: 'Core 已取消',
  TIMEOUT_UNKNOWN: '应答超时 · 执行结果未知', EXPIRED: 'Core 拒绝：请求已过期',
  UNKNOWN: 'Core 返回结果未知', CLOCK_UNKNOWN: '请求时钟域未知', FUTURE: '请求时间异常'
};
function drawOperations(observation, elapsed) {
  const panels = observation.channels.map(channel => {
    const card = cardFor(channel.channel, true), saved = channel.last_operation;
    card.title.textContent = `通道 ${channel.channel} · 最近操作`;
    card.badge.textContent = '历史保留'; card.badge.className = 'badge EVENT';
    card.empty.hidden = !!saved && !saved.error;
    card.empty.textContent = saved?.error ?? (saved === undefined
      ? '当前网关未提供保留数据，请更新并重启网关' : '尚未收到有效操作帧');
    for (const node of [card.raw,card.copy,card.meta,card.scroll,card.note]) node.hidden = !saved || !!saved.error;
    if (!saved || saved.error) return card.panel;
    const bytes = saved.raw_bytes;
    card.title.textContent = `通道 ${channel.channel} · ${operations[bytes[3]]}`;
    drawFrame(card, bytes);
    const time = new Date(saved.received_utc_us / 1000).toLocaleString('zh-CN', {
      timeZone:'Asia/Shanghai', year:'numeric', month:'2-digit', day:'2-digit',
      hour:'2-digit', minute:'2-digit', second:'2-digit', hour12:false, fractionalSecondDigits:3
    });
    const age = Number.isFinite(saved.age_ms) ? `${((saved.age_ms + elapsed) / 1000).toFixed(1)} 秒前` : '时间差未知';
    const axis = offset => { const value = bytes[offset] | bytes[offset+1] << 8; return (value > 32767 ? value - 65536 : value) / 10; };
    card.meta.textContent = `接收时间 ${time}（北京时间） · ${age} · 帧计数 ${bytes[2]} · roll ${axis(4)}% · pitch ${axis(6)}% · 帧校验通过`;
    const execution = saved.execution ?? {status:'RECEIVED'};
    card.note.textContent = outcomes[execution.status] ?? execution.status;
    if (execution.state) card.note.textContent += ` · 应答时状态 ${execution.state}`;
    if (execution.reason) card.note.textContent += ` · 原因 ${execution.reason}`;
    if (execution.error_code != null) card.note.textContent += ` · 错误码 ${execution.error_code}`;
    if (saved.request_id) card.note.textContent += ` · request_id ${saved.request_id}`;
    card.note.dataset.status = execution.status;
    return card.panel;
  });
  const container = $('rs422-operations');
  if (panels.some((panel,i) => container.children[i] !== panel) || container.children.length !== panels.length)
    container.replaceChildren(...panels);
}
function draw() {
  if (!visible) return;
  const data = frozen ?? latest;
  const elapsed = frozen ? frozenCost : Math.max(0, performance.now()-latestAt) + cost;
  const streams = (data?.streams ?? []).filter(s => s.topic === 'flight.command' && s.rs422)
    .sort((a,b) => a.receive_age_ms-b.receive_age_ms);
  const select = $('rs422-source'), selected = select.value;
  const ids = streams.map(s => String(s.id));
  if (Array.from(select.options, o => o.value).join(',') !== ids.join(',')) {
    select.replaceChildren(...streams.map(s => {
      const option = element('option', `${s.publisher} · ${s.session}`);
      option.value = String(s.id); return option;
    }));
    if (ids.includes(selected)) select.value = selected;
  }
  select.disabled = !streams.length;
  const stream = streams.find(s => String(s.id) === select.value) ?? streams[0];
  $('rs422-pause').disabled = !stream;
  if (!stream) {
    $('rs422-note').textContent = '暂无 RS422 报文 · 未启用或发布者未提供附加数据';
    $('rs422-channels').replaceChildren(); $('rs422-operations').replaceChildren(); return;
  }
  const observation = stream.rs422;
  if (observation.error) {
    $('rs422-note').textContent = observation.error;
    $('rs422-channels').replaceChildren(); $('rs422-operations').replaceChildren(); return;
  }
  drawOperations(observation, elapsed);
  const snapshotStale = !frozen && elapsed >= snapshotTimeoutMs;
  const chosen = observation.channels.find(c => c.channel === observation.selected_channel);
  const controlValid = stream.valid && stream.status === 'FRESH' && chosen?.status === 'FRESH';
  const context = frozen ? '已暂停 · 以下为历史快照 · ' : snapshotStale
    ? '网页数据未更新 · 当前控制状态未知 · 最后观测：' : '最近观测：';
  $('rs422-note').textContent = `${context}选用通道 ${observation.selected_channel ?? '无'} · ${controlValid ? '控制有效' : '无有效控制指令'}${observation.blocked ? ' · 切换／冲突保护中' : ''}`;
  const panels = observation.channels.map(channel => {
    const card=cardFor(channel.channel), {panel}=card;
    card.title.textContent=`通道 ${channel.channel}${channel.channel === observation.selected_channel ? ' · 当前选用' : ''}`;
    const status = channel.status;
    card.badge.textContent=frozen ? '暂停快照' : snapshotStale ? '网页数据未更新' : states[status] ?? status;
    card.badge.className=`badge ${frozen || snapshotStale ? 'WAITING' : status}`;
    const age=Number.isFinite(channel.age_ms) ? `${channel.age_ms.toFixed(0)} ms` : '未知';
    card.meta.textContent=`观测时报文年龄 ${age} · 网页快照 ${elapsed.toFixed(0)} ms 前 · 累计接纳 ${channel.accepted_frames} 帧 · 解析器丢弃 ${channel.rejected_bytes} 字节${snapshotStale ? ` · 最后串口状态：${states[status] ?? status}` : ''}`;
    const bytes=channel.raw_bytes; card.bytes=bytes;
    card.empty.hidden=!!bytes.length;
    for (const node of [card.raw,card.copy,card.scroll,card.note]) node.hidden=!bytes.length;
    if (!bytes.length) return panel;
    drawFrame(card, bytes);
    return panel;
  });
  const container=$('rs422-channels');
  if (panels.some((panel,i)=>container.children[i] !== panel) || container.children.length !== panels.length)
    container.replaceChildren(...panels);
}
$('rs422-source').onchange = draw;
$('rs422-pause').onclick = () => {
  if (frozen) frozen = null;
  else { frozen = latest; frozenCost = cost + Math.max(0, performance.now()-latestAt); }
  $('rs422-pause').textContent = frozen ? '恢复报文' : '暂停报文';
  $('rs422-pause').setAttribute('aria-pressed',String(!!frozen));
  draw();
};
export function updateRs422(data, requestCost) { latest=data; latestAt=performance.now(); cost=requestCost; draw(); }
export function setRs422Visible(value) { visible=value; draw(); }
export function clearRs422() {
  latest=null; frozen=null;
  $('rs422-pause').textContent='暂停报文'; $('rs422-pause').setAttribute('aria-pressed','false'); draw();
}
setInterval(draw,100);

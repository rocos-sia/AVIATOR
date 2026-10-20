const $ = id => document.getElementById(id);
const hex = bytes => bytes.map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
const operations = {0:'不发起新操作',16:'进入待机 · enter_standby',32:'握盘 · grasp_wheel',48:'开始控制 · start_control',64:'退出控制 · exit_control',80:'松盘 · leave_wheel',96:'复位错误 · reset_error'};
const states = {FRESH:'报文新鲜',STALE:'报文已过期',WAITING:'等待报文',CLOCK_UNKNOWN:'时钟域未知',FUTURE:'接收时间异常'};
let latest = null, latestAt = 0, cost = 0, frozen = null, frozenAt = 0, frozenCost = 0, visible = false;
const element = (tag, content, className) => {
  const node = document.createElement(tag);
  if (content != null) node.textContent = content;
  if (className) node.className = className;
  return node;
};
const cards = new Map();
function cardFor(name) {
  if (cards.has(name)) return cards.get(name);
  const panel=element('section',null,'panel rs422-frame'), heading=element('div',null,'panel-head');
  const title=element('h2'), badge=element('span'), meta=element('p',null,'footnote');
  heading.append(title,badge);
  const raw=element('pre',null,'rs422-hex'), copy=element('button','复制 HEX');
  raw.setAttribute('aria-label',`通道 ${name} 原始十六进制报文`);
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
  cards.set(name,card); return card;
}
function draw() {
  if (!visible) return;
  const data = frozen ?? latest;
  const elapsed = Math.max(0, performance.now()-(frozen ? frozenAt : latestAt)) + (frozen ? frozenCost : cost);
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
    $('rs422-channels').replaceChildren(); return;
  }
  const observation = stream.rs422;
  if (observation.error) {
    $('rs422-note').textContent = observation.error;
    $('rs422-channels').replaceChildren(); return;
  }
  const live = c => !frozen && c.status === 'FRESH' && c.fresh_for_ms > elapsed;
  const chosen = observation.channels.find(c => c.channel === observation.selected_channel);
  const controlValid = stream.valid && stream.status === 'FRESH' && chosen && live(chosen);
  $('rs422-note').textContent = `${frozen ? '已暂停 · 以下为历史快照 · ' : ''}选用通道 ${observation.selected_channel ?? '无'} · ${controlValid ? '控制有效' : '无当前有效控制指令'}${observation.blocked ? ' · 切换／冲突保护中' : ''}`;
  const panels = observation.channels.map(channel => {
    const card=cardFor(channel.channel), {panel}=card;
    card.title.textContent=`通道 ${channel.channel}${channel.channel === observation.selected_channel ? ' · 当前选用' : ''}`;
    const status = channel.status === 'FRESH' && !live(channel) ? 'STALE' : channel.status;
    card.badge.textContent=frozen ? '暂停快照' : states[status] ?? status;
    card.badge.className=`badge ${status}`;
    const age=Number.isFinite(channel.age_ms) ? `${(channel.age_ms+elapsed).toFixed(0)} ms` : '未知';
    card.meta.textContent=`报文年龄 ${age} · 累计接纳 ${channel.accepted_frames} 帧 · 解析器丢弃 ${channel.rejected_bytes} 字节`;
    const bytes=channel.raw_bytes; card.bytes=bytes;
    card.empty.hidden=!!bytes.length;
    for (const node of [card.raw,card.copy,card.scroll,card.note]) node.hidden=!bytes.length;
    if (!bytes.length) return panel;
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
    return panel;
  });
  const container=$('rs422-channels');
  if (panels.some((panel,i)=>container.children[i] !== panel) || container.children.length !== panels.length)
    container.replaceChildren(...panels);
}
$('rs422-source').onchange = draw;
$('rs422-pause').onclick = () => {
  if (frozen) frozen = null;
  else { frozen = latest; frozenAt = latestAt; frozenCost = cost; }
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

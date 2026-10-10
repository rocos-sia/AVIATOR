const $ = id => document.getElementById(id);
const percent = value => Number.isFinite(value) ? `${value.toFixed(1)}%` : '—';
const bytes = value => {
  if (!Number.isFinite(value)) return '—';
  const units = ['B','KiB','MiB','GiB','TiB'];
  let unit = 0;
  while (value >= 1024 && unit < units.length - 1) { value /= 1024; ++unit; }
  return `${value.toFixed(unit ? 1 : 0)} ${units[unit]}`;
};
const rate = value => Number.isFinite(value) ? `${bytes(value)}/s` : '—';
const icons = {
  cpu:'M6 6h12v12H6z M9 9h6v6H9z M9 2v4m6-4v4M9 18v4m6-4v4M2 9h4m-4 6h4m12-6h4m-4 6h4',
  gpu:'M2 5h19v13H2z M5 18v3m4-3v3m4-3v3M21 8h2m-2 5h2 M15 11.5a4 4 0 1 0-8 0 4 4 0 0 0 8 0 M11 7.5v8m-4-4h8',
  memory:'M3 6h18v12H3z M7 9v6m5-6v6m5-6v6M6 18v3m4-3v3m4-3v3m4-3v3',
  disk:'M5 3h14l3 12v6H2v-6z M2 15h20 M6 18h1m3 0h1',
  network:'M7 3v17m-5-5 5 5 5-5M17 21V4m-5 5 5-5 5 5'
};
function svgElement(name, attributes = {}) {
  const element = document.createElementNS('http://www.w3.org/2000/svg',name);
  for (const [key,value] of Object.entries(attributes)) element.setAttribute(key,value);
  return element;
}
const cards = new Map(), charts = new Map();
for (const [key,label] of [['cpu','CPU'],['gpu','GPU'],['memory','内存'],['disk','硬盘'],['network','网络']]) {
  const card = document.createElement('div'); card.className = 'resource-summary';
  const icon = svgElement('svg',{viewBox:'0 0 24 24','aria-hidden':'true'});
  icon.append(svgElement('path',{d:icons[key]}));
  const title = document.createElement('span'); title.textContent = label;
  const value = document.createElement('strong'); value.textContent = '—';
  card.append(icon,title,value); $('system-summary').append(card); cards.set(key,value);
  if (key === 'gpu') { card.title = 'GPU 使用率（可读取设备的平均值；Intel 为可读进程中最繁忙引擎的估算值）'; continue; }
  const panel = document.createElement('section'); panel.className = 'panel resource-panel';
  const heading = document.createElement('h2'); heading.textContent = key === 'memory' ? '内存与交换空间' : label;
  const head = document.createElement('div'); head.className = 'panel-head'; head.append(heading);
  const plot = document.createElement('div'); plot.className = 'resource-plot';
  const svg = svgElement('svg',{viewBox:'0 0 1000 150',preserveAspectRatio:'none',role:'img','aria-label':`${label}最近 60 秒趋势`});
  const maximum = document.createElement('span'); maximum.className = 'resource-maximum';
  const zero = document.createElement('span'); zero.className = 'resource-zero'; zero.textContent = '0';
  plot.append(svg,maximum,zero);
  const times = document.createElement('div'); times.className = 'resource-times';
  for (const text of ['60 秒前','30 秒前','现在']) { const span = document.createElement('span'); span.textContent = text; times.append(span); }
  const legend = document.createElement('div'); legend.className = 'resource-legend';
  panel.append(head,plot,times,legend); $('resource-charts').append(panel);
  charts.set(key,{svg,maximum,legend});
}
let history = [], current = null, busy = false;
function series(key, data) {
  if (key === 'cpu') return [
    {label:'总计',color:'#79bbff',value:data?.cpu?.percent,get:d=>d.cpu?.percent},
    ...(data?.cpu?.cores ?? []).map((core,i) => ({label:core.name.toUpperCase(),color:`hsl(${i*137.5%360} 75% 68%)`,
      value:core.percent,get:d=>d.cpu?.cores?.find(c=>c.name === core.name)?.percent}))];
  if (key === 'memory') return ['memory','swap'].map((name,i) => ({label:i ? '交换空间' : '内存',
    color:i ? '#67d6a0' : '#f58bb8',value:data?.[name]?.percent,get:d=>d[name]?.percent,
    detail:data?.[name] ? `${bytes(data[name].used_bytes)} / ${bytes(data[name].total_bytes)}` : ''}));
  return (key === 'disk' ? [['read_bytes_per_sec','读取'],['write_bytes_per_sec','写入']] :
    [['receive_bytes_per_sec','接收'],['send_bytes_per_sec','发送']]).map(([name,label],i) =>
    ({label,color:i ? '#ff9b70' : '#79bbff',value:data?.[key]?.[name],get:d=>d[key]?.[name]}));
}
function render(data) {
  for (const [id,seconds] of [['system-uptime',data?.uptime_seconds],['program-uptime',data?.program_uptime_seconds]]) {
    const duration = Number.isFinite(seconds) ? Math.floor(seconds) : null;
    $(id).textContent = duration === null ? '—' :
      `${Math.floor(duration/86400)} 天 ${[duration/3600%24,duration/60%60,duration%60].map(v=>String(Math.floor(v)).padStart(2,'0')).join(':')}`;
  }
  cards.get('cpu').textContent = percent(data?.cpu?.percent);
  cards.get('gpu').textContent = percent(data?.gpu?.percent);
  cards.get('memory').textContent = percent(data?.memory?.percent);
  cards.get('disk').textContent = `读 ${rate(data?.disk?.read_bytes_per_sec)} · 写 ${rate(data?.disk?.write_bytes_per_sec)}`;
  cards.get('network').textContent = `↓ ${rate(data?.network?.receive_bytes_per_sec)} · ↑ ${rate(data?.network?.send_bytes_per_sec)}`;
  for (const value of cards.values()) value.title = value.textContent;
  const end = current?.sample_mono_us / 1e6;
  for (const [key,chart] of charts) {
    const lines = series(key,current), format = ['cpu','memory'].includes(key) ? percent : rate;
    let maximum = format === percent ? 100 : 1024;
    if (format !== percent) for (const sample of history) for (const line of lines)
      maximum = Math.max(maximum,(line.get(sample) ?? 0)*1.1);
    chart.maximum.textContent = format(maximum);
    chart.svg.replaceChildren(); chart.legend.replaceChildren();
    for (const line of lines) {
      let path = '', previous = null;
      for (const sample of history) {
        const value = line.get(sample), time = sample.sample_mono_us/1e6;
        if (!Number.isFinite(value)) { previous = null; continue; }
        const x = (time-end+60)/60*1000, y = 150-Math.min(value/maximum,1)*150;
        path += `${previous === null || time-previous > 2.5 ? 'M' : 'L'}${x.toFixed(1)},${y.toFixed(1)} `;
        previous = time;
      }
      chart.svg.append(svgElement('path',{d:path,stroke:line.color,fill:'none','stroke-width':1.5,'vector-effect':'non-scaling-stroke'}));
      const item = document.createElement('span'), dot = document.createElement('i'); dot.style.background = line.color;
      item.append(dot,`${line.label}  ${format(data ? line.value : null)}${data && line.detail ? ` · ${line.detail}` : ''}`);
      chart.legend.append(item);
    }
  }
}
async function refresh() {
  if (busy || document.hidden) return;
  busy = true;
  const controller = new AbortController(), timeout = setTimeout(()=>controller.abort(),2500);
  try {
    const response = await fetch('/api/system',{cache:'no-store',signal:controller.signal});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    if (current && data.sample_mono_us < current.sample_mono_us) history = [];
    if (!current || data.sample_mono_us !== current.sample_mono_us) history.push(data);
    current = data;
    history = history.filter(sample=>data.sample_mono_us-sample.sample_mono_us <= 60e6).slice(-61);
    render(data);
    $('resource-status').textContent = 'Monitor 所在主机 · 每秒刷新 · 最近 60 秒 · 硬盘为整盘硬件设备合计，网络为非回环接口合计 · — 表示等待采样或不可用';
    $('system-summary').classList.remove('resource-stale');
  } catch {
    render(null);
    $('resource-status').textContent = '主机资源读取失败，历史曲线为旧快照，正在重试…';
    $('system-summary').classList.add('resource-stale');
  } finally { clearTimeout(timeout); busy = false; }
}
document.addEventListener('visibilitychange',()=>{ if (!document.hidden) refresh(); });
setInterval(refresh,1000);
refresh();

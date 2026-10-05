const $ = id => document.getElementById(id);
let visible = false, selected = '', entries = [], truncated = false;
let request = null, generation = 0, filesKey = '', contentKey = '';

function cancel() {
  ++generation;
  request?.abort();
  request = null;
}
function render() {
  const level = $('log-level').value;
  const lines = entries.filter(entry => !level || entry.level === level);
  const key = JSON.stringify(lines);
  if (key !== contentKey) {
    const fragment = document.createDocumentFragment();
    for (const entry of lines) {
      const line = document.createElement('span');
      line.className = `log-line log-${entry.level}`;
      line.textContent = entry.text + '\n';
      fragment.append(line);
    }
    $('log-content').replaceChildren(fragment);
    contentKey = key;
    if ($('log-follow').checked) $('log-content').scrollTop = $('log-content').scrollHeight;
  }
  if (selected) {
    $('log-status').textContent = `${lines.length} / ${entries.length} 行 · 每 1 秒刷新` +
      (truncated ? ' · 仅显示文件末尾 512 KiB 内的最新 2000 行，筛选仅作用于此范围' : '') +
      (!entries.length ? ' · 文件暂无日志' : !lines.length ? ' · 没有匹配级别的日志' : '');
  }
}
function choose(name) {
  cancel();
  selected = name;
  entries = []; truncated = false;
  $('log-title').textContent = name || '选择节点日志';
  render();
  $('log-status').textContent = '正在读取日志…';
  for (const button of $('log-files').children)
    button.setAttribute('aria-pressed', String(button.dataset.file === selected));
  refresh();
}
async function refresh() {
  if (!visible || document.hidden || request) return;
  const controller = new AbortController(), current = generation;
  request = controller;
  const timeout = setTimeout(() => controller.abort(), 2500);
  const get = async path => {
    const response = await fetch(path, {cache:'no-store', signal:controller.signal});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
    return data;
  };
  try {
    const listing = await get('/api/logs');
    if (current !== generation) return;
    $('log-directory').textContent = listing.directory || '未指定（启动 Monitor 时使用 --log-dir）';
    if (!listing.files.some(file => file.name === selected)) {
      selected = listing.files[0]?.name || '';
      entries = []; truncated = false;
      $('log-title').textContent = selected || '选择节点日志';
      render();
    }
    const key = JSON.stringify([listing.files, selected]);
    if (key !== filesKey) {
      const buttons = listing.files.map(file => {
        const button = document.createElement('button');
        button.type = 'button'; button.dataset.file = file.name;
        button.textContent = file.node; button.title = file.name;
        button.setAttribute('aria-pressed', String(file.name === selected));
        button.onclick = () => choose(file.name);
        return button;
      });
      $('log-files').replaceChildren(...buttons);
      filesKey = key;
    }
    if (!selected) {
      $('log-status').textContent = listing.configured ? '目录中暂无 .log 文件，等待节点输出…' :
        '未配置日志目录。通过 start_aviator.sh 启动时自动传入，也可使用 --log-dir 指定。';
      return;
    }
    const data = await get(`/api/logs?file=${encodeURIComponent(selected)}`);
    if (current !== generation) return;
    entries = data.entries; truncated = data.truncated;
    render();
  } catch (error) {
    if (current === generation)
      $('log-status').textContent = `日志刷新失败：${error.message}。显示内容可能已过期，1 秒后重试。`;
  } finally {
    clearTimeout(timeout);
    if (current === generation) request = null;
  }
}
export function setLogsVisible(value) {
  visible = value;
  cancel();
  if (value) refresh();
}
$('log-level').onchange = render;
$('log-follow').onchange = () => {
  if ($('log-follow').checked) $('log-content').scrollTop = $('log-content').scrollHeight;
};
document.addEventListener('visibilitychange', () => {
  cancel();
  if (!document.hidden) refresh();
});
setInterval(refresh, 1000);

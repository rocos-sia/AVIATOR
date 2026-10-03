const $ = id => document.getElementById(id);
let current = null, dirty = false, busy = false, mode = 'form';
const fields = new Map();
function message(value, error = false) {
  $('config-status').textContent = value;
  $('config-status').classList.toggle('INVALID', error);
}
function field(container, group, key, value, numeric = false) {
  const label = document.createElement('label');
  const title = document.createElement('span'); title.textContent = key;
  const input = document.createElement('input'); input.name = `${group}.${key}`;
  input.type = numeric ? 'number' : 'text'; input.value = value;
  if (numeric) { input.min = '1'; input.max = '10000'; input.step = 'any'; input.required = true; }
  else input.maxLength = group === 'preview' && key === 'camera_id' ? 64 : 512;
  if (group === 'preview' && ['publisher_id','camera_id'].includes(key)) input.required = true;
  label.append(title, input); $(container).append(label); fields.set(`${group}.${key}`, input);
}
function display(data) {
  current = data; fields.clear();
  for (const id of ['config-sources','config-timeouts','config-preview']) $(id).replaceChildren();
  for (const [key,value] of Object.entries(data.config.sources))
    field('config-sources','sources',key,typeof value === 'string' ? value : value.publisher_id);
  for (const [key,value] of Object.entries(data.config.timeouts_ms))
    field('config-timeouts','timeouts_ms',key,value,true);
  for (const [key,value] of Object.entries(data.config.preview))
    field('config-preview','preview',key,value,key === 'timeout_ms');
  $('config-yaml').value = data.yaml;
  $('config-path').textContent = data.path || '未指定配置文件，无法保存';
  $('config-revision').textContent = data.revision;
  dirty = false;
}
async function request(options) {
  const response = await fetch('/api/config', {cache:'no-store', ...options});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}
async function reload() {
  if (busy) return;
  busy = true; $('config-fields').disabled = true;
  try { display(await request()); message('已读取当前生效配置。'); }
  catch (error) { message(`读取失败：${error.message}`, true); }
  finally { busy = false; $('config-fields').disabled = !current; }
}
$('config-reload').onclick = reload;
$('config-mode').onchange = event => {
  if (dirty) {
    event.target.value = mode;
    message('有未保存修改，请先保存或重新读取，再切换编辑方式。', true);
    return;
  }
  mode = event.target.value;
  $('config-form-fields').hidden = mode !== 'form';
  $('config-yaml-label').hidden = mode !== 'yaml';
};
$('config-form').addEventListener('input', event => {
  if (event.target.id === 'config-mode') return;
  dirty = true; message('有未保存修改。');
});
$('config-form').onsubmit = async event => {
  event.preventDefault();
  if (busy || !current) return;
  const payload = {revision:current.revision};
  if (mode === 'yaml') payload.yaml = $('config-yaml').value;
  else {
    payload.config = structuredClone(current.config);
    for (const [path,input] of fields) {
      const dot = path.indexOf('.'), group = path.slice(0,dot), key = path.slice(dot+1);
      const value = input.type === 'number' ? Number(input.value) : input.value.trim();
      // Preserve legacy selectors when the publisher was not edited.
      const old = payload.config[group][key];
      if (group === 'sources' && typeof old === 'object' && old.publisher_id === value) continue;
      payload.config[group][key] = value;
    }
  }
  busy = true; $('config-fields').disabled = true;
  message('正在保存并应用…');
  try {
    display(await request({method:'PUT', headers:{'Content-Type':'application/json','X-Monitor-Config':'1'},body:JSON.stringify(payload)}));
    message('已保存到 YAML 文件并立即应用。');
  } catch (error) { message(`保存失败：${error.message}`, true); }
  finally { busy = false; $('config-fields').disabled = false; }
};
window.addEventListener('beforeunload', event => { if (dirty) { event.preventDefault(); event.returnValue = ''; } });
reload();

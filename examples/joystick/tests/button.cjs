// 使用最小 DOM 替身检查按住/释放与异步指令顺序：node tests/button.cjs
const fs = require('fs'), vm = require('vm'), assert = require('assert').strict;
const elements = {};
function element() {
  return {disabled:false, textContent:'', handlers:{}, classList:{add(){},remove(){}},
    addEventListener(name, fn) { this.handlers[name] = fn; },
    setPointerCapture(id) { this.pointer = id; },
    hasPointerCapture(id) { return this.pointer === id; },
    releasePointerCapture() { this.pointer = null; },
    getBoundingClientRect() { return {left:0,right:150,top:0,bottom:50}; }};
}
const window = element(), document = element();
document.getElementById = id => (elements[id] || (elements[id] = element()));
const pending = [], calls = [];
const context = vm.createContext({window, document, AbortSignal:{timeout(){}},
  setTimeout(fn) { pending.push(fn); return fn; },
  clearTimeout(fn) { const i=pending.indexOf(fn); if(i>=0) pending.splice(i,1); },
  fetch: async path => { calls.push(path); return {ok:true}; }
});
const source = fs.readFileSync(__dirname + '/../index.html','utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
vm.runInContext(source.replace(/update\(\);\s*$/, ''), context);
const button = elements.rumble;
const down = () => button.handlers.pointerdown({button:0,pointerId:1,preventDefault(){}});
const up = () => window.handlers.pointerup({pointerId:1});
const flush = () => new Promise(resolve => setImmediate(resolve));
(async () => {
  down(); await flush();
  assert.equal(calls[calls.length - 1], '/api/rumble/start');
  assert.equal(button.textContent, '震动中…');
  assert.equal(pending.length,1);
  pending.shift()(); await flush();
  assert.equal(calls.filter(x=>x.endsWith('start')).length,2);
  up(); await flush();
  assert.equal(calls[calls.length - 1], '/api/rumble/stop');
  assert.equal(pending.length,0);
  // 快速释放后重新按住，不得遗留两个续期循环。
  down(); up(); down(); await flush();
  assert.equal(pending.length,1);
  button.handlers.pointermove({pointerId:1,clientX:151,clientY:20}); await flush();
  assert.equal(calls[calls.length - 1], '/api/rumble/stop');
  assert.equal(pending.length,0);
  down(); await flush(); window.handlers.blur(); await flush();
  assert.equal(calls[calls.length - 1], '/api/rumble/stop');
  button.disabled = true; const count = calls.length;
  down(); await flush(); assert.equal(calls.length,count);
  console.log('PASS: hold, renewal, release, rapid repress, move out, blur, disabled');
})().catch(e => { console.error(e); process.exitCode=1; });

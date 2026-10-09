"use strict";

(() => {
  const token = document.querySelector('meta[name="calibration-token"]').content;
  const $ = (id) => document.getElementById(id);
  const phases = {idle: "待准备", preparing: "准备中", ready: "就绪", dragging: "拖动中", sampling: "采样中", preview: "已预览", saving: "写回中", stopping: "结束中", error: "需处理"};
  const sideNames = {left: "左侧", right: "右侧"};
  const configNames = {robot: "机械臂配置", system: "系统配置", camera: "相机配置", hand: "机械手配置", grasp: "抓取配置", left_ip: "左臂地址", right_ip: "右臂地址", camera_python: "相机 Python"};
  let state = null;
  let online = false;
  let submitting = false;
  let localError = "";
  let networkError = "";
  let manualChanged = false;
  let lastResult = "";
  let lastLogs = "";

  function text(id, value) { $(id).textContent = value; }
  function number(value, digits = 3) { return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "—"; }
  function scaled(value, scale, digits = 3) { return number(typeof value === "number" ? value * scale : value, digits); }
  function vector(value, digits = 6) { return Array.isArray(value) ? `[${value.map((v) => number(v, digits)).join(", ")}]` : "—"; }
  function badge(id, value, tone = "") { text(id, value); $(id).className = `badge ${tone}`; }
  function showError() { const message = networkError || localError || state?.error || ""; $("error").hidden = !message; text("error", message); }
  function manualValid() {
    return $("angle-deg").value.trim() !== "" && $("displacement-m").value.trim() !== "" && $("angle-deg").checkValidity() && $("displacement-m").checkValidity();
  }

  function renderButtons() {
    const s = state || {};
    const blocked = !online || submitting || s.busy;
    $("prepare").disabled = blocked || s.prepared;
    $("stop").disabled = submitting || !(s.prepared || s.busy || s.phase === "error");
    $("wheel-source").disabled = blocked || s.prepared;
    const source = s.prepared ? s.wheel_source : $("wheel-source").value;
    $("camera-input").hidden = source !== "camera";
    $("manual-input").hidden = source !== "manual";
    $("angle-deg").disabled = blocked;
    $("displacement-m").disabled = blocked;
    $("sample").disabled = blocked || !s.can_sample || (source === "manual" && !manualValid());
    $("save").disabled = blocked || !s.can_save || s.dry_run || manualChanged;
    for (const button of document.querySelectorAll("[data-action]")) {
      const side = button.dataset.side;
      const arms = side === "both" ? [s.arms?.left, s.arms?.right] : [s.arms?.[side]];
      if (button.dataset.action === "hand") button.disabled = blocked || !s.prepared || !s.hand?.ready || Boolean(s.hand?.conflict);
      else if (button.dataset.action === "drag_start") button.disabled = blocked || !s.prepared || !arms.every((arm) => arm?.connected && !arm.dragging);
      else button.disabled = blocked || !s.prepared || !arms.some((arm) => arm?.connected && arm.dragging);
    }
    if (manualChanged) text("save-hint", "手动轮盘值已更改，请重新采样后再写回。下方仍显示上一次采样的结果。");
    else if (s.dry_run) text("save-hint", "模拟模式仅供检查操作流程，不能写回 grasp.json。");
    else text("save-hint", "写回仅更新 tool.left / tool.right，保留原文件备份。后续控制程序重新启动后使用新参数。");
  }

  function renderResult(report) {
    const serialized = JSON.stringify(report);
    if (serialized === lastResult) return;
    lastResult = serialized;
    manualChanged = false;
    $("result-empty").hidden = Boolean(report);
    $("result-content").hidden = !report;
    $("result-tables").replaceChildren();
    if (!report) return;
    text("result-summary", `${report.sample_count} 组样本 · 轮盘平均角度 ${scaled(report.wheel?.mean_angle_rad, 180 / Math.PI)}° · 平均位移 ${number(report.wheel?.mean_displacement_m, 5)} m`);
    for (const side of ["left", "right"]) {
      const arm = report.arms?.[side];
      if (!arm) continue;
      const article = document.createElement("article"); article.className = "result-arm";
      const title = document.createElement("h3"); title.textContent = `${sideNames[side]}工具`;
      const table = document.createElement("table");
      for (const [label, value] of [
        ["原位置", vector(arm.old?.position)], ["新位置", vector(arm.new?.position)],
        ["原四元数", vector(arm.old?.quaternion)], ["新四元数", vector(arm.new?.quaternion)],
        ["位置变化 / mm", vector(arm.delta_position_m?.map((v) => v * 1000), 3)],
        ["关节最大波动", `${scaled(arm.max_joint_span_rad, 180 / Math.PI)}°`],
        ["法兰波动", `${scaled(arm.position_span_m, 1000)} mm / ${scaled(arm.rotation_span_rad, 180 / Math.PI)}°`],
        ["重建最大残差", `${scaled(arm.reconstruction_max_position_error_m, 1000, 5)} mm / ${scaled(arm.reconstruction_max_rotation_error_rad, 180 / Math.PI, 5)}°`],
      ]) {
        const row = document.createElement("tr"), heading = document.createElement("th"), cell = document.createElement("td");
        heading.scope = "row"; heading.textContent = label; cell.textContent = value; row.append(heading, cell); table.append(row);
      }
      const deltas = document.createElement("div"); deltas.className = "result-deltas";
      for (const value of [`平移变化 ${scaled(arm.delta_position_norm_m, 1000)} mm`, `旋转变化 ${scaled(arm.delta_rotation_rad, 180 / Math.PI)}°`]) { const span = document.createElement("span"); span.textContent = value; deltas.append(span); }
      article.append(title, table, deltas); $("result-tables").append(article);
    }
  }

  function render(s) {
    state = s;
    text("phase", phases[s.phase] || s.phase || "待准备");
    $("dry-run").hidden = !s.dry_run;
    $("status-message").hidden = !s.message; text("status-message", s.message || "");
    text("session-state", s.prepared ? "会话已连接" : "尚未准备");
    if (s.prepared) $("wheel-source").value = s.wheel_source || "camera";
    const configs = $("config-list"); configs.replaceChildren();
    for (const [key, label] of Object.entries(configNames)) {
      if (!s.config?.[key]) continue;
      const term = document.createElement("dt"), value = document.createElement("dd"); term.textContent = label; value.textContent = s.config[key]; configs.append(term, value);
    }
    $("nodes").replaceChildren();
    for (const node of s.nodes || []) {
      const pill = document.createElement("span"); pill.className = `node-pill ${node.running ? "running" : node.returncode ? "failed" : ""}`;
      pill.textContent = `${node.name} · ${node.running ? s.dry_run ? "模拟" : "运行中" : "已停止"}${node.pid ? ` / ${node.pid}` : ""}`;
      pill.title = node.log_path || ""; $("nodes").append(pill);
    }
    const hand = s.hand || {};
    badge("hand-ready", hand.conflict ? "命令来源冲突" : hand.ready ? "指令已确认" : hand.fresh ? "等待指令确认" : "等待有效反馈", hand.conflict || hand.error ? "bad" : hand.ready ? "good" : "");
    $("hand-ready").title = hand.error || "指令确认不等于实际抓握成功";
    for (const side of ["left", "right"]) {
      const arm = s.arms?.[side] || {};
      badge(`${side}-arm-state`, !arm.connected ? "未连接" : arm.dragging ? "拖动已开启" : "已连接", arm.dragging ? "active" : arm.connected ? "good" : "");
      text(`${side}-arm-detail`, arm.connected ? `状态：${arm.operation_state || "未知"} · ${arm.powered === true ? "已上电" : arm.powered === false ? "已下电" : "电源状态未知"}` : "等待连接机械臂");
      const pose = hand.poses?.[side];
      text(`${side}-hand-state`, `机械手目标：${pose === "open" ? "张开" : pose === "close" ? "握紧" : "—"}${hand.fresh ? " · 反馈有效" : " · 等待反馈"}`);
      text(`${side}-hand-detail`, JSON.stringify({target: hand.targets?.[side] || null, feedback: hand.hands?.[side] || null}, null, 2));
    }
    const camera = s.camera || {};
    const manual = (s.prepared ? s.wheel_source : $("wheel-source").value) === "manual";
    badge("camera-state", manual ? "手动输入" : camera.valid ? "测量有效" : "等待有效测量", camera.valid && !manual ? "good" : "");
    text("wheel-angle", scaled(camera.angle_rad, 180 / Math.PI, 2)); text("wheel-displacement", number(camera.displacement_m, 4));
    text("camera-reason", camera.reason || (camera.valid ? "已按 Core 坐标约定转换，超出范围的测量将拒绝。" : "准备环境后接收相机测量。"));
    text("raw-theta", `${number(camera.theta_rad, 5)} rad`); text("raw-travel", `${number(camera.translation_along_axis_m, 5)} m`); text("camera-age", `${number(camera.age_ms, 0)} ms`);
    const progress = s.progress || {}; $("sample-progress").max = progress.total || 20; $("sample-progress").value = progress.current || 0;
    text("sample-count", progress.current ? `${progress.current} / ${progress.total || 20}` : "");
    text("sample-status", s.phase === "sampling" ? "正在采样，请保持静止" : s.can_sample ? "可以采样，请确认双臂和轮盘静止" : "等待连接、结束拖动及有效测量");
    renderResult(s.result || null);
    $("backup").hidden = !s.backup; text("backup", s.backup ? `已写回。原文件备份：${s.backup}` : "");
    const logs = (s.logs || []).map((entry) => typeof entry === "string" ? entry : `${entry.time || ""}  ${entry.text || ""}`).join("\n");
    if (logs !== lastLogs) { const nearBottom = $("logs").scrollHeight - $("logs").scrollTop - $("logs").clientHeight < 35; text("logs", logs || "尚无操作记录。"); if (nearBottom) $("logs").scrollTop = $("logs").scrollHeight; lastLogs = logs; }
    showError(); renderButtons();
  }

  async function requestAction(payload) {
    if (submitting) return;
    submitting = true; localError = ""; showError(); renderButtons();
    try {
      const response = await fetch("/api/action", {method: "POST", headers: {"Content-Type": "application/json", "X-Calibration-Token": token}, body: JSON.stringify(payload), signal: AbortSignal.timeout(10000)});
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || `操作未接受（${response.status}）`);
      if (payload.action === "sample") manualChanged = false;
    } catch (error) { localError = error.message || String(error); }
    finally { submitting = false; showError(); renderButtons(); }
  }

  async function poll() {
    try {
      const response = await fetch("/api/state", {headers: {"X-Calibration-Token": token}, cache: "no-store", signal: AbortSignal.timeout(4000)});
      if (!response.ok) throw new Error(`界面服务返回 ${response.status}`);
      const data = await response.json(); online = true; networkError = "";
      $("connection-dot").className = "dot live"; text("connection-label", "界面服务已连接"); render(data);
    } catch (error) {
      online = false; $("connection-dot").className = "dot offline"; text("connection-label", "界面服务连接中断");
      networkError = `无法读取状态：${error.message}。服务仍运行时，心跳超时会结束拖动并张开双手。`; showError(); renderButtons();
    } finally { setTimeout(poll, 500); }
  }

  $("prepare").addEventListener("click", () => requestAction({action: "prepare", wheel_source: $("wheel-source").value}));
  $("stop").addEventListener("click", () => requestAction({action: "stop"}));
  $("save").addEventListener("click", () => requestAction({action: "save"}));
  $("sample").addEventListener("click", () => {
    if (($("wheel-source").value === "manual") && !manualValid()) return;
    requestAction({action: "sample", angle_deg: Number($("angle-deg").value), displacement_m: Number($("displacement-m").value)});
  });
  for (const button of document.querySelectorAll("[data-action]")) button.addEventListener("click", () => {
    const payload = {action: button.dataset.action, side: button.dataset.side};
    if (button.dataset.pose) payload.pose = button.dataset.pose;
    requestAction(payload);
  });
  $("wheel-source").addEventListener("change", () => { renderButtons(); badge("camera-state", $("wheel-source").value === "manual" ? "手动输入" : "等待有效测量"); });
  for (const id of ["angle-deg", "displacement-m"]) $(id).addEventListener("input", () => { if (state?.result) manualChanged = true; renderButtons(); });
  window.addEventListener("pagehide", () => {
    if (state?.prepared || state?.busy || submitting) navigator.sendBeacon("/api/action", new Blob([JSON.stringify({action: "stop", token})], {type: "application/json"}));
  });
  poll();
})();

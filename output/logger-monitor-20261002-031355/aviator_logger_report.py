import base64, html, json, math, sys
from pathlib import Path
from datetime import datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

out=Path(sys.argv[1])
summary=json.loads((out/'summary.json').read_text())
samples=[json.loads(line) for line in (out/'samples.jsonl').read_text().splitlines()]
font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if font.exists():
    font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
plt.rcParams['axes.unicode_minus']=False
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
colors={'data':'#2563eb','images':'#0d9488','total':'#475569'}
x=[s['elapsed_s']/60 for s in samples]
fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True,constrained_layout=True)
fig.suptitle('AVIATOR Logger · 10 分钟实测',fontsize=19,fontweight='bold')
axes[0].plot(x[1:],[s.get('process_cpu_percent',math.nan) for s in samples[1:]],color='#2563eb',linewidth=2,label='Logger CPU')
axes[0].axhline(summary['cpu']['mean_percent'],color='#2563eb',linestyle='--',alpha=.55,label=f"均值 {summary['cpu']['mean_percent']:.1f}%")
axes[0].set_ylabel('进程 CPU（100% = 1 核）')
axes[0].set_title('CPU 为相邻采样点之间的平均占用，采样间隔 10 秒',loc='left',fontsize=11)
axes[0].set_ylim(bottom=0)
axes[0].legend(loc='upper right')
axes[1].plot(x[1:],[s.get('host_cpu_percent',math.nan) for s in samples[1:]],color='#7c3aed',linewidth=2,label='整机 CPU 忙碌率')
axes[1].plot(x[1:],[s.get('host_iowait_percent',math.nan) for s in samples[1:]],color='#f59e0b',linewidth=1.5,label='整机 I/O 等待')
axes[1].set_ylabel(f"整机 CPU（{summary['logical_cpus']} 逻辑核，%）")
axes[1].set_ylim(0,100)
axes[1].legend(loc='upper right')
for f in summary['files']:
    values=[]
    for s in samples:
        found=next((item for item in s['files'] if item['device']==f['device'] and item['inode']==f['inode']),None)
        values.append(((found['bytes'] if found else f['initial_bytes'])-f['initial_bytes'])/2**20)
    image='.images.mcap' in f['path']
    axes[2].plot(x,values,linewidth=2,color=colors['images' if image else 'data'],label='图像 MCAP' if image else '业务数据 MCAP')
axes[2].plot(x,[(s['total_file_bytes']-samples[0]['total_file_bytes'])/2**20 for s in samples],color=colors['total'],linestyle='--',linewidth=1.7,label='合计')
axes[2].set_ylabel('自采样开始的文件增量（MiB）')
axes[2].set_xlabel('采样经过时间（分钟）')
axes[2].legend(loc='upper left')
for ax in axes:
    ax.grid(alpha=.18)
    ax.set_xlim(0,10)
    ax.set_xticks(range(11))
fig.savefig(out/'load-and-growth.png',dpi=160,facecolor='white')
plt.close(fig)

minute_rates=[]
for a,b in zip(samples[::6],samples[6::6]):
    dt=b['elapsed_s']-a['elapsed_s']
    minute_rates.append((b['total_file_bytes']-a['total_file_bytes'])/dt)
summary['minute_rate_min_mib_s']=min(minute_rates)/2**20
summary['minute_rate_max_mib_s']=max(minute_rates)/2**20
summary['minute_rate_hourly_range_gib']=[min(minute_rates)*3600/2**30,max(minute_rates)*3600/2**30]
summary['suggested_gib_per_hour_with_20_percent_margin']=math.ceil(summary['projected_gib_per_hour']*1.2)
summary['cpu_mean_machine_capacity_percent']=summary['cpu']['mean_percent']/summary['logical_cpus']
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))

rows=[]
for f in summary['files']:
    label='图像' if '.images.mcap' in f['path'] else '业务数据'
    rows.append(f'<tr><td>{label}<small>{html.escape(Path(f["path"]).name)}</small></td><td>{f["initial_bytes"]/2**20:,.1f}</td><td>{f["bytes"]/2**20:,.1f}</td><td>{f["growth_bytes"]/2**20:,.1f}</td><td>{f["projected_gib_per_hour"]:.2f}</td></tr>')
rows.append(f'<tr class="total"><td>合计</td><td>{summary["initial_total_bytes"]/2**20:,.1f}</td><td>{summary["final_total_bytes"]/2**20:,.1f}</td><td>{summary["growth_total_bytes"]/2**20:,.1f}</td><td>{summary["projected_gib_per_hour"]:.2f}</td></tr>')
start=datetime.fromisoformat(summary['start']).strftime('%Y-%m-%d %H:%M:%S')
end=datetime.fromisoformat(summary['end']).strftime('%H:%M:%S')
image=base64.b64encode((out/'load-and-growth.png').read_bytes()).decode()
cpu=summary['cpu']
report=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AVIATOR Logger 10 分钟监测报告</title>
<style>body{{font-family:system-ui,"Noto Sans CJK SC",sans-serif;background:#f3f5f8;color:#172033;margin:0;line-height:1.7}}main{{max-width:1000px;margin:32px auto;background:white;padding:40px;border-radius:16px}}h1{{font-size:28px;margin:0 0 6px}}h2{{font-size:19px;margin-top:30px}}.sub,small{{color:#64748b}}small{{display:block;font-size:12px}}.cards{{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin:24px 0}}.card{{background:#eff6ff;border-radius:10px;padding:18px}}.card b{{display:block;font-size:27px;color:#1d4ed8}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{padding:12px;border-bottom:1px solid #e2e8f0;text-align:right}}th:first-child,td:first-child{{text-align:left}}th{{color:#475569;background:#f8fafc}}.total{{font-weight:700}}code{{overflow-wrap:anywhere}}img{{width:100%;height:auto}}.note{{padding:16px;background:#f8fafc;border-left:4px solid #94a3b8}}@media(max-width:650px){{main{{margin:0;padding:20px}}.cards{{grid-template-columns:1fr}}table{{font-size:12px}}th,td{{padding:7px}}}}@media print{{body{{background:white}}main{{margin:0;padding:0}}h2{{break-after:avoid}}table,.cards{{break-inside:avoid}}}}</style>
<main><h1>AVIATOR Logger · 10 分钟监测报告</h1><div class="sub">{start}–{end} · 北京时间（UTC+8） · PID {summary['pid']}</div>
<div class="cards"><div class="card">一小时预计新增<b>{summary['projected_gib_per_hour']:.2f} GiB</b><small>基于本次 10 分钟实测平均增长速度</small></div><div class="card">Logger 平均 CPU<b>{cpu['mean_percent']:.1f}%</b><small>100% 表示占满一个逻辑核心</small></div><div class="card">10 分钟文件增量<b>{summary['growth_total_bytes']/2**20:,.1f} MiB</b><small>业务数据 + 压缩图像</small></div></div>
<h2>磁盘占用</h2><table><thead><tr><th>文件类别</th><th>开始 MiB</th><th>结束 MiB</th><th>10 分钟增量 MiB</th><th>一小时增量 GiB</th></tr></thead><tbody>{''.join(rows)}</tbody></table>
<p>平均写入文件增长速度为 <b>{summary['average_bytes_per_s']/2**20:.2f} MiB/s</b>。各分钟平均速度为 {summary['minute_rate_min_mib_s']:.2f}–{summary['minute_rate_max_mib_s']:.2f} MiB/s，对应一小时约 {summary['minute_rate_hourly_range_gib'][0]:.2f}–{summary['minute_rate_hourly_range_gib'][1]:.2f} GiB（观测区间，不是统计置信区间）。按平均值增加 20% 容量余量并向上取整，可预留 <b>{summary['suggested_gib_per_hour_with_20_percent_margin']} GiB/小时</b>。</p>
<p>监测结束时所在磁盘可用空间 {summary['disk_available_bytes']/2**30:,.1f} GiB。仅按本次 Logger 增长速度计算约可再录 {summary['estimated_hours_until_full']:.1f} 小时；其他进程用盘和预留空间未计入。</p>
<h2>CPU 与内存</h2><table><tbody><tr><td>Logger CPU 平均 / P95 / 最大采样区间均值</td><td>{cpu['mean_percent']:.1f}% / {cpu['p95_percent']:.1f}% / {cpu['max_percent']:.1f}%</td></tr><tr><td>Logger 平均使用的逻辑核心数</td><td>{cpu['mean_percent']/100:.2f} 核（整机 {summary['logical_cpus']} 核容量的 {summary['cpu_mean_machine_capacity_percent']:.2f}%）</td></tr><tr><td>整机 CPU 平均 / 最大采样区间均值</td><td>{summary['host_cpu_mean_percent']:.1f}% / {summary['host_cpu_max_percent']:.1f}%</td></tr><tr><td>整机平均 I/O 等待</td><td>{summary['host_iowait_mean_percent']:.2f}%</td></tr><tr><td>Logger RSS 内存峰值 / 结束值</td><td>{summary['rss_peak_bytes']/2**20:.1f} / {summary['rss_final_bytes']/2**20:.1f} MiB</td></tr><tr><td>系统 1 分钟负载：开始 / 结束 / 采样峰值</td><td>{summary['load1_initial']:.2f} / {summary['load1_final']:.2f} / {summary['load1_peak']:.2f}</td></tr></tbody></table>
<h2>趋势</h2><img src="data:image/png;base64,{image}" alt="Logger CPU、整机 CPU 和 MCAP 增长趋势">
<h2>采样与估算方法</h2><p>实际观测 {summary['elapsed_s']:.3f} 秒，共 {summary['sample_count']} 个时间点，间隔约 10 秒。读取 Linux /proc 中进程 CPU 时间、RSS、整机 CPU 计数和文件 stat 大小；进程 CPU 按 CPU 时间增量除以实际间隔计算。一小时容量 = 文件总增量 ÷ 实际观测秒数 × 3600。MiB = 2²⁰ 字节，GiB = 2³⁰ 字节。</p>
<p>启动命令：<code>{html.escape(summary['command'])}</code>。采样时配置文件为业务目标 compact 模式、彩色 H.264 软件压缩、目标码率 8 Mbps、未录制深度。配置仅作为背景，容量估算使用文件实测增量。</p>
<div class="note">文件在采样开始前已有数据，表格明确区分已有大小与新增大小。观测期间 Logger {'始终存活' if summary['process_alive_all_samples'] else '出现停止或不可读取情况'}；监测未停止录制。运行中的 .partial 文件包含已写入数据，正常结束后的最终索引等开销尚未计入。CPU 最大值为 10 秒区间均值，不能代表更短瞬时峰值。容量预测假设消息频率、分辨率、编码配置和画面复杂度保持近似；本次只监测资源用量，不据此判断是否丢帧。</div>
<p class="sub">原始数据：samples.jsonl · 统计结果：summary.json · 图表：load-and-growth.png</p></main></html>'''
(out/'report.html').write_text(report)
print(json.dumps({'report':str(out/'report.html'),'chart':str(out/'load-and-growth.png'),'summary':summary},ensure_ascii=False,indent=2))

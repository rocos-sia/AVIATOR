from pathlib import Path
from html import escape

W,H=2600,2390
out=Path('docs/assets/aviator_state_machine')
out.mkdir(parents=True,exist_ok=True)
s=[]
s.append(f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<defs><marker id="arrow" viewBox="0 0 12 12" refX="10" refY="6" markerWidth="10" markerHeight="10" orient="auto-start-reverse"><path d="M2 2 L10 6 L2 10" fill="none" stroke="#282b30" stroke-width="1.5"/></marker></defs>
<style>text{{font-family:'Microsoft YaHei','Arial',sans-serif;fill:#282b30}} .event{{fill:#e35c5c;font-weight:700;paint-order:stroke;stroke:white;stroke-width:12;stroke-linejoin:round}} .note{{fill:#687080}} .guard{{fill:#687080;paint-order:stroke;stroke:white;stroke-width:10}} .edge{{fill:none;stroke:#282b30;stroke-width:2.6;marker-end:url(#arrow)}} .action{{font-style:italic;font-weight:600;paint-order:stroke;stroke:white;stroke-width:9}}</style>
<rect width="100%" height="100%" fill="white"/>
<title>AVIATOR 整机状态机流转图</title><desc>依据 RobotStateMachine.hpp 和 Aviator::Managed，含十二状态、六操作、异步任务、保护及恢复路径。下方 HOMING 与 RELEASING 是上方同名状态的引用。</desc>''')
def text(x,y,t,size=25,anchor='middle',cls='',weight=None):
    s.append(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" class="{cls}"'+(f' font-weight="{weight}"' if weight else '')+f'>{escape(t)}</text>')
def edge(d,dash=False):
    s.append(f'<path d="{d}" class="edge"'+(' stroke-dasharray="5 6"' if dash else '')+'/>')
def node(x,y,name,kind='normal',sub='',rx=64):
    fills={'normal':'#527bc5','initial':'#d7b958','transition':'#fde4e5'}
    s.append(f'<ellipse cx="{x}" cy="{y}" rx="{rx}" ry="48" fill="{fills[kind]}" stroke="#6288d5" stroke-width="2.3"/>')
    text(x,y+83,name,29,weight='700')
    if sub:text(x,y+115,sub,22,cls='note')
def task(x,y,name,sub,width=280):
    s.append(f'<rect x="{x-width/2}" y="{y-40}" width="{width}" height="80" rx="39" fill="#eae3fb" stroke="#282b30" stroke-width="2.3" stroke-dasharray="4 5"/>')
    text(x,y+1,name,26,weight='600');text(x,y+29,sub,20,cls='note')
def ev(x,y,t,guard=None,size=24):
    text(x,y,t,size,cls='event')
    if guard:text(x,y+31,guard,21,cls='guard')
def line(y):s.append(f'<path d="M85 {y} H2515" stroke="#e3e6ec" stroke-width="2"/>')

text(85,82,'AVIATOR 整机状态机',44,'start',weight='700')
text(85,127,'Managed 模式 · 12 个状态 · 6 个外部操作 · 单任务异步执行',25,'start',cls='note')
text(2515,83,'与当前代码一致',22,'end',cls='note')

# Normal flow, left to right then back to the left.
edge('M244 285 C335 285 404 285 496 285');ev(365,260,'Boot','[G0]')
edge('M624 265 C676 238 714 225 750 225',True);text(692,207,'initialize',22,cls='action')
edge('M1030 225 C1090 225 1118 264 1166 278');ev(1100,210,'Done [D1]',size=22)
edge('M1294 285 C1400 285 1455 285 1556 285');ev(1425,254,'ENTER_STANDBY','[G1]',22)
edge('M1684 264 C1740 235 1767 225 1795 225',True);text(1738,207,'home',22,cls='action')
edge('M2145 225 C2180 225 2215 264 2286 278');ev(2200,204,'Done [D1]',size=22)
edge('M2377 241 C2485 105 2185 105 2305 243');ev(2330,176,'ENTER_STANDBY',size=21)
text(2467,217,'幂等',20,cls='note')

edge('M2350 401 C2350 478 2350 556 2350 662');ev(2350,511,'GRASP_WHEEL','[G2]')
edge('M2286 710 C2210 710 2160 710 2110 710',True);text(2195,682,'grasp',22,cls='action')
edge('M1750 710 C1570 710 1400 710 1234 710');ev(1500,684,'Done [D2]')
edge('M1106 679 C961 522 760 523 600 677');ev(846,565,'START_CONTROL','[G3]')
edge('M615 732 C790 845 935 845 1105 739');ev(850,819,'EXIT_CONTROL','[following_authorized]')
edge('M1205 666 C1375 448 1420 755 1234 720');ev(1410,567,'EXIT_CONTROL',size=22);text(1410,598,'幂等',21,cls='note')
edge('M1170 831 C1170 905 1170 980 1170 1052');ev(1350,924,'LEAVE_WHEEL',size=23);ev(1350,958,'或 ENTER_STANDBY',size=23);text(1350,991,'[G4]',22,cls='note')
edge('M1234 1100 C1325 1100 1370 1100 1450 1100',True);text(1345,1068,'release',23,cls='action')
edge('M1790 1100 C2100 1100 2500 1110 2500 820 L2500 456 C2500 345 2470 285 2414 285');ev(2155,1065,'Done [D3]')
text(2180,1140,'释放、撤离并回 home 后到达 STANDBY',23,cls='note')

node(180,285,'INIT','initial','初始状态')
node(560,285,'INITIALIZING','transition','初始化 / 使能')
task(890,225,'initialize task','init → enable')
node(1230,285,'READY','normal','准备完成，保持当前位置')
node(1620,285,'HOMING','transition','回 home / 到位确认')
task(1970,225,'home task','enable if needed → moveHome',350)
node(2350,285,'STANDBY','normal','已到 home 且停稳')
node(2350,710,'GRASPING','transition','接近 / 抓握')
task(1930,710,'grasp task','prepareGrasp → approach → lock',360)
node(1170,710,'FOLLOWING','normal','保持参考 / 当前未实现柔顺控制')
node(550,710,'CONTROL','normal','唯一接纳主动操控目标的状态')
node(1170,1100,'RELEASING','transition','松手 / 撤离 / 回 home')
task(1620,1100,'release task','releaseHandles → moveHome',340)

# Guard key in the intentionally open lower left of the main flow.
text(100,958,'转换守卫',26,'start',weight='700')
guards=[
'G0  任务空闲，且无急停锁存',
'G1  ready + settled + clear_of_wheel + 任务空闲',
'G2  ready + settled + following_authorized + 任务空闲',
'G3  G2 + source_authorized + input_ready',
'G4  ready + settled + release_authorized + 任务空闲',
'G5  fault_cleared + settled + executor_idle + 无急停锁存']
for i,t in enumerate(guards):text(100,1001+39*i,t,21,'start')
text(100,1263,'任务空闲 = job == none 且 executor_idle',21,'start',cls='note')

line(1310)
text(85,1366,'保护与恢复',31,'start',weight='700')
text(2515,1366,'保护先于完成事件和普通请求；下方状态集合用于合并同类连线',22,'end',cls='note')

# Aggregated global edges, precise source sets described under the glyphs.
def group(x,y,title,lines):
    s.append(f'<ellipse cx="{x-21}" cy="{y}" rx="34" ry="26" fill="#527bc5"/><ellipse cx="{x+21}" cy="{y}" rx="34" ry="26" fill="#fde4e5" stroke="#6288d5" stroke-width="2"/>')
    text(x,y+58,title,25,weight='700')
    for i,t in enumerate(lines):text(x,y+89+i*29,t,20,cls='note')
group(480,1440,'运行状态集合',['READY / HOMING / STANDBY / GRASPING', 'FOLLOWING / CONTROL / RELEASING'])
group(1270,1440,'可进入故障的状态集合',['除 ERROR、EMERGENCY_STOP 外的全部状态', 'ERROR 再次 Fault：仅更新诊断'])
group(2200,1440,'任意非急停状态',['包括 SAFE、ERROR 和所有过渡状态'])
edge('M480 1594 L480 1652');ev(480,1630,'SafetyLost',size=23)
edge('M1270 1594 L1270 1652');ev(1270,1630,'Fault',size=23)
edge('M2200 1565 L2200 1652');ev(2200,1628,'Emergency',size=23)
node(480,1700,'SAFE','normal','等待显式恢复')
node(1270,1700,'ERROR','normal','记录错误 / 取消任务 / 请求停止')
node(2200,1700,'EMERGENCY_STOP','normal','当前实例锁存，无退出路径')
edge('M1206 1700 C1000 1700 780 1700 544 1700');ev(875,1670,'RESET_ERROR','[G5]')
edge('M445 1740 C395 1818 310 1865 280 1924');ev(248,1878,'ENTER_STANDBY [G1]',size=21)
edge('M517 1740 C571 1818 652 1865 680 1924');ev(718,1878,'LEAVE_WHEEL [G4]',size=21)
node(280,1970,'HOMING','transition','上方同名状态')
node(680,1970,'RELEASING','transition','上方同名状态')
text(1250,1898,'Fault：执行失败、任务超时、阻断故障或后置条件不满足',22)
text(1250,1936,'SafetyLost：当前模式所需资源、授权或输入失效',22)
text(1250,1974,'CONTROL 输入期限 100 ms；输入恢复不会自动恢复操控',22)
text(2200,1898,'cancel + stop + request_brake',24,cls='action')
text(2200,1937,'物理手刹尚未接入',22,cls='note')
text(2200,1975,'软件锁存不跨进程重启保存',22,cls='note')

# Completion contract and compact legend.
text(85,2324,'Done 均校验 generation、任务类型及 executor_idle；D1: ready + settled + clear_of_wheel；D2: ready + following_authorized；D3: settled + clear_of_wheel。',20,'start',cls='note')
text(85,2360,'依据 nodes/aviator_core/include/aviator/RobotStateMachine.hpp 与 Aviator::Managed；紫色框表示异步动作，完成箭头属于其前置过渡状态。',20,'start',cls='note')
line(2270)
legend=[(100,'#d7b958','初始状态'),(370,'#fde4e5','过渡状态'),(640,'#527bc5','稳定 / 保护状态')]
for x,c,t in legend:
    s.append(f'<ellipse cx="{x}" cy="{2222}" rx="30" ry="22" fill="{c}" stroke="#6288d5" stroke-width="2"/>');text(x+45,2230,t,22,'start')
s.append('<rect x="920" y="2200" width="70" height="44" rx="22" fill="#eae3fb" stroke="#282b30" stroke-width="2" stroke-dasharray="4 5"/>');text(1005,2230,'异步执行',22,'start')
edge('M1200 2222 H1300');text(1320,2230,'事件 / 状态流转',22,'start')
edge('M1590 2222 H1690',True);text(1710,2230,'动作提交',22,'start')
text(1995,2230,'[G] 转换条件    [D] 完成条件',22,'start',cls='note')
s.append('</svg>')
(out/'aviator_state_machine.svg').write_text('\n'.join(s),encoding='utf-8')
print(out/'aviator_state_machine.svg')

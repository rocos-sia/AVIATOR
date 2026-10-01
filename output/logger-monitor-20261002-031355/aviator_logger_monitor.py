import os, sys, time, json, math, statistics
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo

PID = int(sys.argv[1])
DURATION = 600
INTERVAL = 10
TZ = ZoneInfo('Asia/Shanghai')
ROOT = Path('/home/rocos/Documents/GitHub/AVIATOR')
OUT = ROOT / 'output' / ('logger-monitor-' + datetime.now(TZ).strftime('%Y%m%d-%H%M%S'))
OUT.mkdir(parents=True)
PROC = Path('/proc') / str(PID)
HZ = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')

def process_stat():
    fields = (PROC / 'stat').read_text().rsplit(')', 1)[1].split()
    return {'cpu_s': (int(fields[11]) + int(fields[12])) / HZ,
            'start_ticks': int(fields[19]), 'rss_bytes': int(fields[21]) * PAGE,
            'threads': int(fields[17])}

def host_stat():
    lines = Path('/proc/stat').read_text().splitlines()
    values = list(map(int, lines[0].split()[1:9]))
    return {'total': sum(values), 'idle': values[3], 'iowait': values[4],
            'cpus': sum(line.startswith('cpu') and len(line)>3 and line[3].isdigit() for line in lines)}

initial_proc = process_stat()
initial_host = host_stat()
meta = {'pid': PID, 'command': (PROC/'cmdline').read_bytes().replace(b'\0',b' ').decode().strip(),
        'cwd': os.readlink(PROC/'cwd'), 'timezone': str(TZ), 'duration_target_s': DURATION,
        'interval_target_s': INTERVAL, 'logical_cpus': initial_host['cpus'],
        'affinity_cpus': len(os.sched_getaffinity(PID)), 'clock_ticks_per_s': HZ,
        'process_start_ticks': initial_proc['start_ticks'], 'output_dir': str(OUT)}
(OUT/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2))
known = {}
samples = []
start = time.monotonic()
previous = None

with (OUT/'samples.jsonl').open('w') as raw:
    for index in range(DURATION // INTERVAL + 1):
        target = start + min(index * INTERVAL, DURATION)
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        now = time.monotonic()
        sample = {'timestamp':datetime.now(TZ).isoformat(timespec='milliseconds'),
                  'elapsed_s':now-start,'process_alive':False,'files':[],'events':[]}
        try:
            p = process_stat()
            if p['start_ticks'] != initial_proc['start_ticks']:
                raise RuntimeError('PID was reused')
            sample.update(p)
            sample['process_alive'] = True
            for fd in (PROC/'fd').iterdir():
                try:
                    path = os.readlink(fd)
                    if not (path.endswith('.mcap') or path.endswith('.mcap.partial')):
                        continue
                    st = fd.stat()
                    key = f'{st.st_dev}:{st.st_ino}'
                    known[key] = {'path':path,'fd':str(fd),'device':st.st_dev,'inode':st.st_ino}
                except OSError:
                    continue
            io = dict(line.split(':',1) for line in (PROC/'io').read_text().splitlines())
            sample['write_bytes'] = int(io['write_bytes'])
            sample['read_bytes'] = int(io['read_bytes'])
        except (OSError,RuntimeError) as error:
            sample['events'].append(str(error))
        for key,info in known.items():
            candidates=[info['fd'],info['path']]
            if info['path'].endswith('.partial'):
                candidates.append(info['path'][:-8])
            file = dict(info)
            file.pop('fd')
            for candidate in candidates:
                try:
                    st = os.stat(candidate)
                    if st.st_dev != info['device'] or st.st_ino != info['inode']:
                        continue
                    file.update(bytes=st.st_size, allocated_bytes=st.st_blocks*512)
                    break
                except OSError:
                    pass
            else:
                file.update(bytes=None,allocated_bytes=None)
                sample['events'].append('file unavailable: '+info['path'])
            sample['files'].append(file)
        h = host_stat()
        sample['host_counters'] = h
        sample['load_average'] = list(os.getloadavg())
        vfs=os.statvfs(meta['cwd'])
        sample['disk_available_bytes']=vfs.f_bavail*vfs.f_frsize
        sample['disk_total_bytes']=vfs.f_blocks*vfs.f_frsize
        mem=dict(line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())
        sample['host_memory_available_bytes']=int(mem['MemAvailable'].split()[0])*1024
        sample['total_file_bytes']=sum(f['bytes'] or 0 for f in sample['files'])
        if previous is not None:
            dt=sample['elapsed_s']-previous['elapsed_s']
            sample['interval_s']=dt
            if sample['process_alive'] and previous['process_alive']:
                sample['process_cpu_percent']=100*(sample['cpu_s']-previous['cpu_s'])/dt
                sample['write_bytes_per_s']=(sample['write_bytes']-previous['write_bytes'])/dt
            dh=h['total']-previous['host_counters']['total']
            if dh>0:
                sample['host_cpu_percent']=100*(dh-(h['idle']-previous['host_counters']['idle'])-(h['iowait']-previous['host_counters']['iowait']))/dh
                sample['host_iowait_percent']=100*(h['iowait']-previous['host_counters']['iowait'])/dh
            sample['file_growth_bytes_per_s']=(sample['total_file_bytes']-previous['total_file_bytes'])/dt
        raw.write(json.dumps(sample,ensure_ascii=False)+'\n')
        raw.flush()
        samples.append(sample)
        previous=sample
        (OUT/'latest.json').write_text(json.dumps(sample,ensure_ascii=False,indent=2))
        if index % 6 == 0 or index == DURATION // INTERVAL:
            print(json.dumps({'output_dir':str(OUT),'elapsed_s':round(sample['elapsed_s'],1),
                'cpu_percent':round(sample.get('process_cpu_percent',0),1),
                'host_cpu_percent':round(sample.get('host_cpu_percent',0),1),
                'total_mib':round(sample['total_file_bytes']/2**20,1),
                'files':{Path(f['path']).name:round((f['bytes'] or 0)/2**20,1) for f in sample['files']},
                'events':sample['events']},ensure_ascii=False),flush=True)

first,last=samples[0],samples[-1]
elapsed=last['elapsed_s']-first['elapsed_s']
intervals=[s for s in samples[1:] if 'process_cpu_percent' in s]
valid_time=sum(s['interval_s'] for s in intervals)
cpus=sorted(s['process_cpu_percent'] for s in intervals)
files=[]
for f in last['files']:
    initial=next((v for v in first['files'] if v['inode']==f['inode'] and v['device']==f['device']),None)
    before=initial['bytes'] if initial else 0
    growth=None if f['bytes'] is None or before is None else f['bytes']-before
    files.append({**f,'initial_bytes':before,'growth_bytes':growth,
                  'bytes_per_s':None if growth is None else growth/elapsed,
                  'projected_gib_per_hour':None if growth is None else growth/elapsed*3600/2**30})
total_growth=last['total_file_bytes']-first['total_file_bytes']
summary={**meta,'start':first['timestamp'],'end':last['timestamp'],'elapsed_s':elapsed,'sample_count':len(samples),
    'cpu':{'mean_percent':sum(s['process_cpu_percent']*s['interval_s'] for s in intervals)/valid_time if valid_time else None,
           'min_percent':min(cpus) if cpus else None,'max_percent':max(cpus) if cpus else None,
           'p95_percent':cpus[math.ceil(.95*len(cpus))-1] if cpus else None},
    'host_cpu_mean_percent':sum(s.get('host_cpu_percent',0)*s['interval_s'] for s in samples[1:])/elapsed,
    'host_cpu_max_percent':max(s.get('host_cpu_percent',0) for s in samples),
    'host_iowait_mean_percent':sum(s.get('host_iowait_percent',0)*s['interval_s'] for s in samples[1:])/elapsed,
    'rss_peak_bytes':max(s.get('rss_bytes',0) for s in samples),
    'rss_final_bytes':last.get('rss_bytes'),'load1_initial':first['load_average'][0],
    'load1_final':last['load_average'][0],'load1_peak':max(s['load_average'][0] for s in samples),
    'files':files,'initial_total_bytes':first['total_file_bytes'],'final_total_bytes':last['total_file_bytes'],
    'growth_total_bytes':total_growth,'average_bytes_per_s':total_growth/elapsed,
    'projected_gib_per_hour':total_growth/elapsed*3600/2**30,
    'disk_available_bytes':last['disk_available_bytes'],'disk_total_bytes':last['disk_total_bytes'],
    'estimated_hours_until_full':last['disk_available_bytes']/(total_growth/elapsed*3600) if total_growth>0 else None,
    'process_alive_all_samples':all(s['process_alive'] for s in samples),
    'events':[{'elapsed_s':s['elapsed_s'],'events':s['events']} for s in samples if s['events']]}
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print('COMPLETE '+str(OUT/'summary.json'),flush=True)

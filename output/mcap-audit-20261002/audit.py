import sys,json,collections,statistics,datetime,struct,zlib
from pathlib import Path
sys.path.insert(0,'/tmp/aviator-mcap-audit-deps')
from mcap.stream_reader import StreamReader
from mcap.reader import make_reader
from mcap.records import Schema,Channel,Message,Metadata,Footer,DataEnd
from google.protobuf import descriptor_pb2,descriptor_pool,message_factory
OUT=Path(__file__).parent
results=[]; examples={}; frame_sets={}; service=[]
def decode_metadata(d):
 out={}
 for k,v in d.items():
  try: out[k]=json.loads(v)
  except: out[k]=v
 return out
for path in [Path('recording.mcap'),Path('recording.images.mcap')]:
 print('Scanning',path,flush=True)
 schemas={};channels={};classes={};meta={};stats={};footer=False;dataend=False
 with path.open('rb') as f:
  for r in StreamReader(f,validate_crcs=True).records:
   if isinstance(r,Schema):
    schemas[r.id]=r
    if r.encoding=='protobuf' and r.id not in classes:
     pool=descriptor_pool.DescriptorPool()
     for fd in descriptor_pb2.FileDescriptorSet.FromString(r.data).file:pool.Add(fd)
     classes[r.id]=message_factory.GetMessageClass(pool.FindMessageTypeByName(r.name))
   elif isinstance(r,Channel): channels[r.id]=r
   elif isinstance(r,Metadata): meta[r.name]=decode_metadata(r.metadata)
   elif isinstance(r,Footer): footer=True
   elif isinstance(r,DataEnd): dataend=True
   elif isinstance(r,Message):
    c=channels[r.channel_id];topic=c.topic
    if c.message_encoding=='json':d=json.loads(r.data)
    else:d=json.loads(classes[c.schema_id].FromString(r.data).metadata_json)
    k=(topic,d.get('publisher_id',c.metadata.get('publisher_id')),d.get('session_id',c.metadata.get('session_id')))
    if k not in stats:stats[k]={'topic':topic,'publisher':k[1],'session':k[2],'count':0,'first_ns':r.log_time,'last_ns':r.log_time,'payload_bytes':0,'gaps':0,'gap_events':[],'duplicates_or_reordered':0,'valid':collections.Counter(),'reason':collections.Counter(),'statuses':collections.Counter(),'dt_ms':[],'sample_dt_ms':[],'last_seq':None,'first_seq':d.get('sequence'),'last_sample':None,'field_nonnull':collections.Counter()}
    s=stats[k];seq=d.get('sequence');sample=d.get('sample_mono_us')
    if s['count']:
     s['dt_ms'].append((r.log_time-s['last_ns'])/1e6)
     if isinstance(sample,int) and isinstance(s['last_sample'],int):s['sample_dt_ms'].append((sample-s['last_sample'])/1000)
    if seq is not None and s['last_seq'] is not None:
     gap=seq-s['last_seq']-1
     if gap>0:
      s['gaps']+=gap
      if len(s['gap_events'])<20:s['gap_events'].append({'previous':s['last_seq'],'next':seq,'missing':gap,'log_ns':r.log_time})
     elif gap<0:s['duplicates_or_reordered']+=1
    if seq is not None:s['last_seq']=max(s['last_seq'] or 0,seq)
    s['last_sample']=sample;s['last_ns']=r.log_time;s['count']+=1;s['payload_bytes']+=len(r.data)
    s['valid'][str(d.get('valid','not_present'))]+=1
    if 'reason' in d:s['reason'][str(d['reason'])]+=1
    for field in ['status','state','mode','source']:
     if field in d and isinstance(d[field],(str,int)):s['statuses'][field+':'+str(d[field])]+=1
    if isinstance(d.get('system'),dict):s['statuses']['system.state:'+str(d['system'].get('state'))]+=1
    for field,v in d.items():
     if v is not None:s['field_nonnull'][field]+=1
    if topic not in examples:examples[topic]=d
    if 'frame_id' in d:frame_sets.setdefault(topic,set()).add(d['frame_id'])
    if topic.startswith('record.service.'):service.append({'topic':topic,'log_ns':r.log_time,'data':d})
    if s['count']%100000==0: print(topic,s['count'],flush=True)
 for s in stats.values():
  duration=(s['last_ns']-s['first_ns'])/1e9;s['duration_s']=duration;s['hz']=(s['count']-1)/duration if duration else 0
  for field in ['dt_ms','sample_dt_ms']:
   a=sorted(s.pop(field));s[field]={'min':min(a) if a else None,'median':statistics.median(a) if a else None,'p99':a[int(.99*(len(a)-1))] if a else None,'max':max(a) if a else None,'over_100ms':sum(v>100 for v in a),'over_1s':sum(v>1000 for v in a),'negative':sum(v<0 for v in a)}
 with path.open('rb') as f:
  summ=make_reader(f).get_summary();index_counts=dict(summ.statistics.channel_message_counts)
 result={'path':str(path.resolve()),'bytes':path.stat().st_size,'crc_chunk_and_data_passed':True,'footer':footer,'data_end':dataend,'metadata':meta,'channels':[{ 'id':c.id,'topic':c.topic,'encoding':c.message_encoding,'metadata':c.metadata,'indexed_count':index_counts.get(c.id)} for c in channels.values()],'stats':list(stats.values()),'count':sum(s['count'] for s in stats.values()),'indexed_count':summ.statistics.message_count}
 results.append(result)
 print(json.dumps({'file':str(path),'messages':result['count'],'metadata':meta,'topics':[{k:s[k] for k in ['topic','publisher','count','hz','gaps','duplicates_or_reordered','valid','reason','statuses','dt_ms']} for s in stats.values()]},ensure_ascii=False,indent=2),flush=True)
(OUT/'audit.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
(OUT/'examples.json').write_text(json.dumps(examples,ensure_ascii=False,indent=2))
(OUT/'services.json').write_text(json.dumps(service,ensure_ascii=False,indent=2))
a=frame_sets.get('camera.detection',set());b=frame_sets.get('record.camera.cockpit.rgb',set())
correlation={'detection_frames':len(a),'image_frames':len(b),'detection_without_image':len(a-b),'image_without_detection':len(b-a),'detection_without_image_examples':sorted(a-b)[:30],'image_without_detection_examples':sorted(b-a)[:30]}
(OUT/'frame-correlation.json').write_text(json.dumps(correlation,indent=2));print(correlation)

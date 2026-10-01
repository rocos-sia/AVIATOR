import sys,json,collections,subprocess,math
from pathlib import Path
sys.path.insert(0,'/tmp/aviator-mcap-audit-deps')
from mcap.reader import make_reader
from google.protobuf import descriptor_pb2,descriptor_pool,message_factory
p=Path(__file__).parent
stats=collections.Counter();bad_by_phase=collections.Counter();ranges=[];phase='unknown';prev=None;first_bad=None
with open('recording.mcap','rb') as f:
 for sch,c,m in make_reader(f).iter_messages(log_time_order=False):
  if c.topic not in ['arm.state','record.arm.target','flight.state','hand.state','camera.detection']:continue
  d=json.loads(m.data)
  if c.topic=='flight.state':
   phase=d.get('system',{}).get('state','unknown')
   stats['flight.camera_freshness.'+str(d.get('freshness',{}).get('camera',{}).get('valid'))]+=1
   stats['flight.vision.'+str(d.get('vision',{}).get('status'))]+=1
  elif c.topic=='record.arm.target':
   stats['target.streaming.'+str(d.get('streaming'))]+=1
   stats['target.derivatives.'+str(d.get('derivatives_available'))]+=1
   if d['reason']=='invalid_window':
    bad_by_phase[phase]+=1
    if first_bad is None:first_bad=d
   reason=d['reason']
   if reason!=prev:
    ranges.append({'reason':reason,'first_ns':m.log_time,'last_ns':m.log_time,'count':1});prev=reason
   else:ranges[-1]['count']+=1;ranges[-1]['last_ns']=m.log_time
  elif c.topic=='arm.state':
   t=d.get('execution',{}).get('target');stats['arm.execution_target.valid14']+=isinstance(t,list) and len(t)==14 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in t)
   for side in ['left','right']:
    a=d.get('arms',{}).get(side,{})
    for k in ['joint_position','joint_velocity','tcp_pose']:
     stats['arm.'+side+'.'+k+'.nonnull']+=a.get(k) is not None
  elif c.topic=='hand.state':
   for side in ['left','right']:
    for k in ['joint_position','joint_velocity','drive_position_normalized','force','current','temperature']:
     stats['hand.'+side+'.'+k+'.nonnull']+=d.get('hands',{}).get(side,{}).get(k) is not None
  else:
   stats['camera.steering_wheel.valid.'+str(d.get('steering_wheel',{}).get('valid'))]+=1
(p/'detail.json').write_text(json.dumps({'counts':stats,'invalid_window_by_phase':bad_by_phase,'target_reason_ranges':ranges,'first_invalid_window':first_bad},ensure_ascii=False,indent=2))
print(json.dumps({'counts':stats,'invalid_window_by_phase':bad_by_phase,'reason_range_count':len(ranges)},ensure_ascii=False,indent=2),flush=True)
with open('recording.images.mcap','rb') as f, (p/'ffmpeg-errors.log').open('w') as err:
 r=make_reader(f);s=r.get_summary();sc=next(iter(s.schemas.values()));pool=descriptor_pool.DescriptorPool()
 for fd in descriptor_pb2.FileDescriptorSet.FromString(sc.data).file:pool.Add(fd)
 Packet=message_factory.GetMessageClass(pool.FindMessageTypeByName(sc.name))
 process=subprocess.Popen(['ffmpeg','-nostdin','-hide_banner','-v','error','-xerror','-threads','2','-f','h264','-i','pipe:0','-an','-f','null','-','-progress',str(p/'decode-progress.txt')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=err)
 count=0;seqprev=None;frameprev=None;gaps=[]
 for _,c,m in r.iter_messages(log_time_order=False):
  packet=Packet.FromString(m.data);d=json.loads(packet.metadata_json)
  if seqprev is not None and d['sequence']-seqprev>1:gaps.append({'previous_sensor_sequence':seqprev,'next_sensor_sequence':d['sequence'],'missing':d['sequence']-seqprev-1,'application_frame_id_delta':d['frame_id']-frameprev,'log_ns':m.log_time})
  seqprev=d['sequence'];frameprev=d['frame_id'];process.stdin.write(packet.data);count+=1
 process.stdin.close();code=process.wait()
res={'input_packets':count,'ffmpeg_exit_code':code,'sensor_gap_events':gaps}
(p/'decode.json').write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2),flush=True)

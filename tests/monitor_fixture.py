"""Isolated browser-test publisher. Never connects to the production bus."""
import json
import math
from pathlib import Path
import signal
import socket
import sys
import time
import uuid

import cv2
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nodes/camera'))
from preview_client import PreviewClient

bus, image_endpoint, control_path = sys.argv[1:4]
context = zmq.Context()
pub = context.socket(zmq.PUB)
pub.setsockopt(zmq.LINGER, 0)
pub.bind(bus)
preview = PreviewClient(dict(enabled=True, endpoint=image_endpoint, fps=15), 'cockpit')
session = str(uuid.uuid4())
clock = socket.gethostname()[:80] + '-' + Path('/proc/sys/kernel/random/boot_id').read_text().strip()
request = dict(msg_type='ServiceRequest', version='1.0', request_id=str(uuid.uuid4()),
               client_id='flight_gateway', client_session_id=session, target='aviator_core',
               operation='grasp_wheel', issued_mono_us=time.monotonic_ns() // 1000,
               clock_id=clock, timestamp=time.time_ns() // 1000, deadline_ms=100,
               parameters={}, gateway_observation='QUEUED')
reply = dict(msg_type='ServiceReply', version='1.0', request_id=request['request_id'],
             client_id='flight_gateway', client_session_id=session, server_id='aviator_core',
             server_session_id=session, status='ACCEPTED', error_code=0, result={},
             timestamp=time.time_ns() // 1000)
image = np.full((180,320,3),65,np.uint8)
cv2.putText(image,'MONITOR TEST STREAM',(15,80),cv2.FONT_HERSHEY_SIMPLEX,.55,(220,220,220),1)
cv2.putText(image,'RGB preview',(70,110),cv2.FONT_HERSHEY_SIMPLEX,.55,(150,210,230),1)
sequence = 0
running = True

def stop(*_):
    global running
    running = False

signal.signal(signal.SIGTERM,stop)

def send(topic, kind, publisher, fields, sample, valid=True):
    message = dict(msg_type=kind, version='1.0', sequence=sequence//5 if topic=='hand.state' else sequence, timestamp=time.time_ns()//1000,
                   sample_mono_us=sample, clock_id=clock, publisher_id=publisher, session_id=session, valid=valid)
    message.update(fields)
    pub.send_multipart([topic.encode(),json.dumps(message).encode()])

try:
    while running:
        sequence += 1
        sample = time.monotonic_ns() // 1000
        try:
            control = json.loads(Path(control_path).read_text())
        except (FileNotFoundError,json.JSONDecodeError):
            control = {}
        send('flight.state','FlightState','aviator_core',dict(system=dict(state='CONTROL',control_source='JOYSTICK',current_error_code=0,last_error_code=0)),sample)
        send('flight.command','FlightCommand','flight_gateway',dict(source='JOYSTICK',control=dict(roll=.35,pitch=-.12)),sample)
        side = dict(valid=True,status='ACTIVE',enabled=True,error_code=0,sample_mono_us=sample,
                    joint_position=[0,.2,-.4,0,.2,0,0])
        right = dict(side, joint_position=[-.5,-.8,.6,1,-.3,-.4,-.2])
        side['joint_position'] = [.5,.8,-.6,1,.3,.4,.2]
        send('arm.state','ArmState','manipulator',dict(arms=dict(left=side,right=right)),sample)
        positions = control.get('hand_positions', [.1,.2,.3,.4,.5,.6])
        hand = dict(valid=True,status='READY',feedback_available=True,enabled=False,error_code=0,
                    sample_mono_us=sample,feedback_age_ms=0,drive_position_raw=[round(p*1000) for p in positions],
                    drive_position_normalized=positions,commanded_drive_position_normalized=[.8]*6,
                    joint_position=None,joint_velocity=None,
                    grasp_verified=False,sample_time_basis='host_read_request')
        if sequence % 5 == 0:
            right_hand = dict(hand, valid=not control.get('right_hand_invalid', False))
            send('hand.state','HandState','inspire_hand',dict(hands=dict(left=hand,right=right_hand)),sample,
                 valid=right_hand['valid'])
        if not control.get('camera_stale'):
            angle = math.radians(17)
            pose = dict(position=dict(x=0,y=0,z=(73.1-85)/1000),
                        orientation=dict(qx=0,qy=0,qz=math.sin(angle/2),qw=math.cos(angle/2)))
            send('camera.detection','CameraDetection','camera',dict(camera_id='cockpit',frame_id=sequence,
                 status='TRACKING',confidence=.9,pose=pose),sample)
            preview.submit(image,session_id=session,clock_id=clock,frame_id=sequence,sample_mono_us=sample)
        pub.send_multipart([b'record.service.request',json.dumps(request).encode()])
        pub.send_multipart([b'record.service.reply',json.dumps(reply).encode()])
        time.sleep(.02)
finally:
    preview.close()
    pub.close()
    context.term()

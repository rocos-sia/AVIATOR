"""Hardware-free integration of latest-only preview worker and multipart wire format."""
import json
import socket
import time
import unittest

import cv2
import numpy as np
import zmq
from preview_client import PreviewClient


class PreviewTest(unittest.TestCase):
    def test_independent_jpeg_channel(self):
        with socket.socket() as reserve:
            reserve.bind(('127.0.0.1',0))
            endpoint = f'tcp://127.0.0.1:{reserve.getsockname()[1]}'
        client = PreviewClient(dict(enabled=True,endpoint=endpoint,fps=15), 'cockpit')
        context = zmq.Context()
        sub = context.socket(zmq.SUB)
        sub.setsockopt(zmq.SUBSCRIBE,b'camera.rgb.cockpit')
        sub.setsockopt(zmq.LINGER,0)
        sub.connect(endpoint)
        image = np.full((90,160,3),100,np.uint8)
        try:
            received = None
            for i in range(1,50):
                client.submit(image,session_id='session',clock_id='clock',frame_id=i,sample_mono_us=12345)
                if sub.poll(30):
                    received = sub.recv_multipart()
                    break
            self.assertIsNone(client.error)
            self.assertIsNotNone(received)
            self.assertEqual(len(received),3)
            meta = json.loads(received[1])
            self.assertEqual((meta['camera_id'],meta['width'],meta['height']),('cockpit',160,90))
            self.assertEqual(meta['sequence'],meta['frame_id'])
            decoded = cv2.imdecode(np.frombuffer(received[2],dtype=np.uint8),cv2.IMREAD_COLOR)
            self.assertEqual(decoded.shape,(90,160,3))
            # Burst input remains one replaceable pending snapshot.
            for i in range(100,300):
                client.submit(image,session_id='session',clock_id='clock',frame_id=i,sample_mono_us=12345)
            deadline=time.monotonic()+2
            latest=0
            while time.monotonic()<deadline and latest<299:
                if sub.poll(100): latest=json.loads(sub.recv_multipart()[1])['frame_id']
            self.assertEqual(latest,299)
        finally:
            client.close()
            sub.close()
            context.term()
        self.assertFalse(client._thread.is_alive())

    def test_disabled_and_validation(self):
        client=PreviewClient(dict(enabled=False),'cockpit')
        self.assertIsNone(client._thread)
        client.close()
        for settings in [dict(enabled='yes'),dict(fps=0),dict(endpoint='tcp://0.0.0.0:5561')]:
            with self.assertRaises(ValueError): PreviewClient(settings,'cockpit')


if __name__ == '__main__':
    unittest.main()

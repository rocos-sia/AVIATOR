"""Exercise the real HTTP handlers without opening TCP ports or devices."""
from email.message import Message
from io import BytesIO
import json
from types import SimpleNamespace
import unittest

from ui import Handler


class FakeController:
    def __init__(self):
        self.beats = 0
        self.actions = []

    def heartbeat(self):
        self.beats += 1

    def snapshot(self):
        return {"prepared": False, "dry_run": True}

    def submit(self, body):
        self.actions.append(body)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.controller = FakeController()

    def request(self, method, path, body=None, headers=None):
        handler = Handler.__new__(Handler)
        handler.server = SimpleNamespace(server_port=8766, token="test-token", controller=self.controller)
        handler.headers = Message()
        values = {"Host": "127.0.0.1:8766", **(headers or {})}
        data = json.dumps(body).encode() if body is not None else b""
        if data:
            values["Content-Length"] = str(len(data))
        for key, value in values.items():
            handler.headers[key] = value
        handler.rfile, handler.wfile = BytesIO(data), BytesIO()
        handler.path, handler.command = path, method
        handler.request_version = "HTTP/1.1"
        handler.requestline = method + " " + path + " HTTP/1.1"
        handler.connection = SimpleNamespace(settimeout=lambda value: None)
        getattr(handler, "do_" + method)()
        status, content = handler.wfile.getvalue().split(b"\r\n\r\n", 1)
        return int(status.split(b" ")[1]), content, status

    def test_idle_page_injects_token_but_does_not_connect(self):
        code, body, headers = self.request("GET", "/")
        self.assertEqual(code, 200)
        self.assertIn(b'test-token', body)
        self.assertNotIn(b'__CALIBRATION_TOKEN__', body)
        self.assertIn(b"frame-ancestors 'none'", headers)
        self.assertEqual(self.controller.actions, [])

    def test_authenticated_state_is_the_only_heartbeat(self):
        self.assertEqual(self.request("GET", "/api/state")[0], 403)
        self.assertEqual(self.controller.beats, 0)
        code, body, _ = self.request("GET", "/api/state", headers={"X-Calibration-Token": "test-token"})
        self.assertEqual(code, 200)
        self.assertTrue(json.loads(body)["dry_run"])
        self.assertEqual(self.controller.beats, 1)

    def test_cross_origin_and_rebinding_rejected_even_with_token(self):
        for extra in ({"Origin": "https://other.example"}, {"Host": "other.example:8766"},
                      {"Sec-Fetch-Site": "cross-site"}):
            self.assertEqual(self.request("POST", "/api/action", {"action": "prepare"},
                                          {"X-Calibration-Token": "test-token", **extra})[0], 403)
        self.assertEqual(self.controller.actions, [])

    def test_beacon_body_token_only_allows_stop(self):
        self.assertEqual(self.request("POST", "/api/action", {"action": "stop", "token": "test-token"})[0], 202)
        self.assertEqual(self.request("POST", "/api/action", {"action": "prepare", "token": "test-token"})[0], 403)
        self.assertEqual(len(self.controller.actions), 1)

    def test_bad_json_shape_and_arbitrary_file_paths_rejected(self):
        self.assertEqual(self.request("POST", "/api/action", [], {"X-Calibration-Token": "test-token"})[0], 400)
        for path in ("/../config/grasp.json", "/config/grasp.json", "/web/../ui.py"):
            self.assertEqual(self.request("GET", path)[0], 404)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Read-only collector semantics and real subprocess lifecycle tests, without hardware."""
import csv
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('evaluate', ROOT / 'scripts/evaluate_aviator.py')
evaluate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluate)


class MetricsTest(unittest.TestCase):
    def test_natural_perf_deadline_waits_before_signalling_sudo(self):
        with tempfile.TemporaryDirectory() as temp, \
                mock.patch.object(evaluate.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'perf fixture', '')), \
                mock.patch.object(evaluate.subprocess, 'Popen') as popen:
            recorder = evaluate.PerfRecorder(Path(temp), [101], 60, 19, False, lambda *_: None)
            (Path(temp) / 'perf.data').write_bytes(b'fixture')
            recorder.started = time.monotonic() - 59.97
            popen.return_value.poll.side_effect = [None, 0, 0]
            recorder.stop()
            popen.return_value.send_signal.assert_not_called()
            popen.return_value.wait.assert_called_once()
            self.assertGreater(popen.return_value.wait.call_args.kwargs['timeout'], 30)
            self.assertEqual(recorder.status, 'recorded')

    def test_function_percentages_separate_pmus_and_keep_unknown_cost(self):
        raw = """# Total Lost Samples: 2
# Samples: 1K of event 'cpu_atom/cycles/u'
# Overhead; Samples; Period;Shared Object;Symbol
 1.00%; 12; 60;aviator_core_managed;[.] work<int>()
 0.50%;  3; 40;libx.so;[.] 0x00000123
# Samples: 2K of event 'cpu_core/cycles/u'
 0.01%;  2; 10;aviator_core_managed;[.] work<int>()
 1.00%; 50; 90;libx.so;[.] allocate()
"""
        parsed = evaluate.parse_function_report(raw)
        self.assertEqual(parsed['lost_samples'], 2)
        self.assertEqual(parsed['events']['cpu_atom/cycles/u']['unresolved_pct'], 40)
        functions = {(r['event'], r['symbol']): r for r in parsed['rows']}
        self.assertEqual(functions['cpu_atom/cycles/u', 'work<int>()']['self_pct'], 60)
        self.assertEqual(functions['cpu_core/cycles/u', 'work<int>()']['self_pct'], 10)
        self.assertEqual(parsed['events']['cpu_atom/cycles/u']['samples'], 15)
        with self.assertRaisesRegex(ValueError, '事件名称'):
            evaluate.parse_function_report('1.0%;1;10;app;[.] work()')

    def test_symbol_buildid_mismatch_invalidates_previous_analysis(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'perf.data').write_bytes(b'fixture')
            (root / 'functions.csv').write_text('stale ranking')
            args = evaluate.arguments(['--report', temp, '--functions', '--symbol-file', '/tmp/symbol'])
            responses = [subprocess.CompletedProcess([], 0, 'perf fixture', ''),
                         subprocess.CompletedProcess([], 0, 'aabbccdd /opt/app\n', ''),
                         subprocess.CompletedProcess([], 0, 'Build ID: 11223344\n', '')]
            with mock.patch.object(evaluate.subprocess, 'run', side_effect=responses) as run:
                result = evaluate.analyze_functions(root, args, {'perf': {'status': 'recorded'}})
            self.assertEqual(result['status'], 'failed')
            self.assertIn('不匹配', result['error'])
            self.assertFalse((root / 'functions.csv').exists())
            self.assertEqual(result['rows'], [])
            self.assertEqual(run.call_count, 3)  # no caching or report with the wrong binary

    def test_light_defaults_and_explicit_stack_modes(self):
        args = evaluate.arguments([])
        self.assertFalse(args.cpu)
        self.assertFalse(args.pss)
        self.assertEqual((args.interval, args.perf_duration, args.frequency), (2, 30, 19))
        self.assertEqual(args.perf_selected, {'core'})
        self.assertEqual(args.call_graph, 'none')
        with tempfile.TemporaryDirectory() as temp, \
                mock.patch.object(evaluate.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'perf fixture', '')), \
                mock.patch.object(evaluate.subprocess, 'Popen') as popen:
            popen.return_value.poll.return_value = 0
            for mode in ('none', 'dwarf', 'fp'):
                recorder = evaluate.PerfRecorder(Path(temp), [101], 30, 19, False, lambda *_: None,
                                                  call_graph=mode)
                if mode == 'none':
                    self.assertNotIn('--call-graph', recorder.command)
                else:
                    self.assertIn('dwarf,2048' if mode == 'dwarf' else 'fp', recorder.command)
                recorder.stop()

    def test_stat_comm_parentheses_and_cpu_fields(self):
        fields = ['0'] * 24
        for index, value in {0: 'S', 1: '7', 2: '8', 7: '9', 9: '10', 11: '110', 12: '90', 19: '300'}.items():
            fields[index] = value
        value = evaluate.parse_stat('42 (a ) name (with space)) ' + ' '.join(fields))
        self.assertEqual(value['pid'], 42)
        self.assertEqual(value['comm'], 'a ) name (with space)')
        self.assertEqual(value['ticks'], 200)
        self.assertEqual(value['start_ticks'], 300)

    def test_cpu_uses_actual_dt_not_nominal_interval(self):
        item = dict(pid=1, start_ticks=10, node='core')
        previous = dict(start_ticks=10, sampled_at=1, ticks=100, read_bytes=100, write_bytes=80)
        current = dict(start_ticks=10, sampled_at=3, ticks=500, read_bytes=1100, write_bytes=40,
                       rss_bytes=2048, thread_count=4, errors=[])
        row = evaluate.metric_row(item, current, previous, 3, 100)
        self.assertEqual(row['cpu_pct'], 200)
        self.assertEqual(row['read_bytes_per_s'], 500)
        self.assertIsNone(row['write_bytes_per_s'])
        self.assertIn('counter_reset', row['missing_reason'])
        self.assertIsNone(evaluate.metric_row(item, current, None, 3, 100)['cpu_pct'])
        previous['start_ticks'] = 9
        self.assertIsNone(evaluate.metric_row(item, current, previous, 3, 100)['cpu_pct'])

    def test_timing_scopes_and_nonfinite_values(self):
        line = '[2026-11-11 10:00:00.000] Motion timing side=left window_s=1 rt_max_gap_ms=2.3 max_target_error_rad=0.2 callback_hz=nan'
        rows = evaluate.parse_timing(line, 2, 'manipulator', 42, 'x.log', 30)
        by_name = {row['metric_name']: row for row in rows}
        self.assertEqual(by_name['rt_max_gap_ms']['aggregation_scope'], 'since_enable_max')
        self.assertEqual(by_name['max_target_error_rad']['aggregation_scope'], 'snapshot_joint_max')
        self.assertNotIn('callback_hz', by_name)
        self.assertEqual(rows[0]['source_timestamp'], '2026-11-11 10:00:00.000')

    def test_log_tail_no_history_partial_line_and_rotation(self):
        events = []
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'core.log'
            path.write_text('Servo timing steps=3 plan_mean_us=99\n')
            tail = evaluate.LogTail(path, 'core', lambda *x: events.append(x))
            self.assertEqual(tail.read(1, 2), [])
            with path.open('a') as stream:
                stream.write('Servo timing steps=2 plan_mean_us=')
            self.assertEqual(tail.read(2, 2), [])
            with path.open('a') as stream:
                stream.write('10 plan_max_us=12\n')
            self.assertEqual(len(tail.read(3, 2)), 2)
            path.rename(path.with_suffix('.old'))
            path.write_text('Servo timing steps=1 plan_mean_us=7\n')
            self.assertEqual(tail.read(4, 2)[0]['value'], 7)
            self.assertEqual(events[0][0], 'log_reset')

    def test_plot_preserves_extrema_and_gap(self):
        plot = evaluate.Plot(8000)
        for x, y in [(0, 10), (1, 100), (2, None), (3, 2), (4, 10)]:
            plot.add(x, y)
        self.assertIn([1, 100], plot.points())
        self.assertIn([2, None], plot.points())
        self.assertIn([3, 2], plot.points())

    def test_launcher_does_not_match_editor_or_shell_source(self):
        self.assertTrue(evaluate.is_launcher(dict(argv=['bash', '/tmp/start_aviator.sh'])))
        self.assertFalse(evaluate.is_launcher(dict(argv=['code', '/tmp/start_aviator.sh'])))
        self.assertFalse(evaluate.is_launcher(dict(argv=['bash', '-c', 'start_aviator.sh'])))

    def test_descendant_worker_and_unrelated_process(self):
        table = {1: {'ppid': 0}, 2: {'ppid': 1}, 3: {'ppid': 2}, 4: {'ppid': 0}}
        self.assertEqual(evaluate.descendants(table, 1), {2, 3})

    def test_launcher_families_preserve_independent_instances(self):
        def launcher(pid, parent, pgid, extra=()):
            return dict(pid=pid, ppid=parent, pgid=pgid, uid=os.getuid(),
                        argv=['bash', './scripts/start_aviator.sh', *extra])
        table = {10: launcher(10, 1, 10), 11: launcher(11, 10, 10),
                 12: launcher(12, 11, 10), 20: launcher(20, 1, 20),
                 30: launcher(30, 10, 30), 40: launcher(40, 10, 10, ['--simulation'])}
        self.assertEqual(evaluate.launcher_families(table), {10: 10, 11: 10, 12: 10, 20: 20, 30: 30, 40: 40})

    def test_pid_reuse_and_unreadable_io_are_not_zero_samples(self):
        item = dict(pid=os.getpid(), start_ticks=0, node='core')
        with self.assertRaises(ProcessLookupError):
            evaluate.sample_process(item)
        item['start_ticks'] = evaluate.parse_stat(Path(f'/proc/{os.getpid()}/stat').read_text())['start_ticks']
        original = evaluate.key_values

        def denied(path):
            if path.name == 'io':
                raise PermissionError('restricted')
            return original(path)

        with mock.patch.object(evaluate, 'key_values', side_effect=denied):
            sample = evaluate.sample_process(item)
        self.assertNotIn('read_bytes', sample)
        self.assertIn('io:PermissionError', sample['errors'])
        self.assertGreater(sample['rss_bytes'], 0)

    def test_privileged_perf_preserves_tty_and_has_timeout(self):
        with tempfile.TemporaryDirectory() as temp, \
                mock.patch.object(evaluate.os, 'geteuid', return_value=1000), \
                mock.patch.object(evaluate.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'perf fixture', '')), \
                mock.patch.object(evaluate.subprocess, 'Popen') as popen:
            popen.return_value.poll.return_value = 0
            recorder = evaluate.PerfRecorder(Path(temp), [101, 102], 60, 199, True, lambda *_: None)
            self.assertEqual(recorder.command[:3], ['sudo', '-n', 'timeout'])
            self.assertEqual(recorder.command[5], '90')
            self.assertFalse(popen.call_args.kwargs['start_new_session'])
            self.assertIs(popen.call_args.kwargs['preexec_fn'], os.setpgrp)
            recorder.stop()


@unittest.skipUnless(sys.platform == 'linux', 'Linux /proc required')
class CollectorIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.logs = self.root / 'logs'
        self.logs.mkdir()
        worker = self.root / 'nodes/camera/main.py'
        worker.parent.mkdir(parents=True)
        worker.write_text('''import time
memory = bytearray(16 * 1024 * 1024)
while True:
    end = time.monotonic() + .025
    while time.monotonic() < end:
        pass
    print('[2026-11-11 10:00:00.000] Servo timing trajectory=1 steps=20 plan_mean_us=12 plan_max_us=20 input_age_ms=1', flush=True)
    time.sleep(.025)
''')
        hand = self.root / 'nodes/rh56ftp_hand/launch.py'
        hand.parent.mkdir(parents=True)
        hand.write_text(worker.read_text())
        launcher = self.root / 'start_aviator.sh'
        launcher.write_text('''#!/bin/bash
"$EVAL_TEST_PYTHON" -u "$EVAL_TEST_WORKER" > "$EVAL_TEST_LOGS/camera.log" 2>&1 &
child=$!
echo "$child" > "$EVAL_TEST_LOGS/camera.pid"
"$EVAL_TEST_PYTHON" -u "$EVAL_TEST_HAND" > "$EVAL_TEST_LOGS/hand.log" 2>&1 &
hand=$!
echo "$hand" > "$EVAL_TEST_LOGS/hand.pid"
( sleep 60; : ) &
echo "$!" > "$EVAL_TEST_LOGS/browser-wrapper.pid"
trap 'kill "$child" "$hand" 2>/dev/null; wait "$child" "$hand" 2>/dev/null; exit 0' TERM INT
wait "$child"
''')
        env = dict(os.environ, EVAL_TEST_PYTHON=sys.executable, EVAL_TEST_WORKER=str(worker),
                   EVAL_TEST_HAND=str(hand), EVAL_TEST_LOGS=str(self.logs))
        self.launcher = subprocess.Popen(['bash', launcher], env=env, start_new_session=True,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(self.stop_launcher)
        deadline = time.monotonic() + 5
        while not (self.logs / 'browser-wrapper.pid').exists() or not (self.logs / 'camera.log').exists():
            if time.monotonic() > deadline:
                self.fail('fixture did not start')
            time.sleep(.02)
        self.worker_pid = int((self.logs / 'camera.pid').read_text())
        self.hand_pid = int((self.logs / 'hand.pid').read_text())
        self.browser_pid = int((self.logs / 'browser-wrapper.pid').read_text())

    def stop_launcher(self):
        if self.launcher.poll() is None:
            os.killpg(self.launcher.pid, signal.SIGTERM)
            self.launcher.wait(timeout=5)

    def command(self, name='result', duration='.8', extra=()):
        return [sys.executable, '-B', str(ROOT / 'scripts/evaluate_aviator.py'),
                '--launcher-pid', str(self.launcher.pid), '--log-dir', str(self.logs),
                '--duration', duration, '--interval', '.1', '--output', str(self.root / name), *extra]

    def assert_business_alive(self):
        self.assertIsNone(self.launcher.poll())
        os.kill(self.worker_pid, 0)
        os.kill(self.hand_pid, 0)

    def test_real_background_shell_does_not_create_second_instance(self):
        table = evaluate.process_table()
        owned = evaluate.descendants(table, self.launcher.pid) | {self.launcher.pid}
        table = {pid: item for pid, item in table.items() if pid in owned}
        self.assertTrue(evaluate.is_launcher(table[self.browser_pid]))
        with mock.patch.object(evaluate, 'process_table', return_value=table):
            instance = evaluate.Instance()
            self.assertEqual(instance.pid, self.launcher.pid)
            self.assertEqual({p['node'] for p in instance.discover().values()}, {'camera', 'hand'})
            self.assertEqual(evaluate.Instance(self.browser_pid).pid, self.launcher.pid)

    def test_orphan_browser_is_ignored_but_two_active_launchers_are_ambiguous(self):
        table = evaluate.process_table()
        owned = evaluate.descendants(table, self.launcher.pid) | {self.launcher.pid}
        table = {pid: item for pid, item in table.items() if pid in owned}
        orphan = max(table) + 100000
        table[orphan] = dict(table[self.launcher.pid], pid=orphan, ppid=1, pgid=orphan)
        with mock.patch.object(evaluate, 'process_table', return_value=table):
            self.assertEqual(evaluate.Instance().pid, self.launcher.pid)
        table[orphan + 1] = dict(table[self.worker_pid], pid=orphan + 1, ppid=orphan)
        with mock.patch.object(evaluate, 'process_table', return_value=table):
            with self.assertRaisesRegex(ValueError, '2 个独立启动器'):
                evaluate.Instance()

    def test_real_cpu_memory_logs_offline_report_and_no_overwrite(self):
        label = '</script><img src=x onerror=alert(1)>'
        result = subprocess.run(self.command(extra=('--label', label)), capture_output=True, text=True, timeout=10)
        self.assertIn(result.returncode, (0, 2), result.stderr)
        output = self.root / 'result'
        summary = json.loads((output / 'summary.json').read_text())
        self.assertEqual(summary['perf']['status'], 'not_requested')
        self.assertGreaterEqual(summary['priority']['nice'], 10)
        self.assertFalse((output / 'perf.data').exists())
        self.assertEqual(summary['process_count'], 2)
        processes = json.loads((output / 'processes.json').read_text())
        self.assertEqual({p['node'] for p in processes}, {'camera', 'hand'})
        self.assertGreater(summary['timing_field_count'], 0)
        with (output / 'metrics.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertTrue(any(float(row['cpu_pct'] or 0) > 5 for row in rows))
        self.assertTrue(all(float(row['rss_bytes'] or 0) > 10 * 1024 * 1024 for row in rows))
        report = (output / 'report.html').read_text()
        self.assertNotIn(label, report)
        self.assertIn('\\u003c/script\\u003e', report)
        self.assertNotIn('https://', report)
        self.assert_business_alive()
        before = (output / 'metrics.csv').read_bytes()
        repeat = subprocess.run(self.command(), capture_output=True, timeout=10)
        self.assertEqual(repeat.returncode, 1)
        self.assertEqual((output / 'metrics.csv').read_bytes(), before)

    def test_collect_only_and_report_without_running_instance(self):
        result = subprocess.run(self.command(extra=('--collect-only',)), capture_output=True, text=True, timeout=10)
        self.assertIn(result.returncode, (0, 2), result.stderr)
        output = self.root / 'result'
        self.assertFalse((output / 'report.html').exists())
        before = {p.name: p.read_bytes() for p in output.iterdir()}
        self.stop_launcher()
        # Offline analysis must not look at /proc, contact hardware, authenticate or start perf.
        with mock.patch.object(evaluate, 'Instance', side_effect=AssertionError('discovery')), \
                mock.patch.object(evaluate, 'PerfRecorder', side_effect=AssertionError('perf')), \
                mock.patch.object(evaluate, 'lower_priority', return_value={}):
            self.assertEqual(evaluate.main(['--report', str(output)]), 0)
        self.assertTrue((output / 'report.html').exists())
        self.assertEqual(before, {name: (output / name).read_bytes() for name in before})
        with mock.patch.object(evaluate, 'lower_priority', return_value={}), \
                mock.patch.object(evaluate, 'Instance', side_effect=AssertionError('discovery')), \
                mock.patch.object(evaluate.subprocess, 'run', side_effect=AssertionError('no perf for basic')):
            self.assertEqual(evaluate.main(['--report', str(output), '--functions']), 2)
        self.assertIn('没有 perf.data', json.loads((output / 'functions.json').read_text())['error'])
        self.assertEqual(before, {name: (output / name).read_bytes() for name in before})

    def test_missing_default_perf_node_does_not_expand_to_all_nodes(self):
        result = subprocess.run(self.command(extra=('--cpu',)), capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 1)
        self.assertIn('CPU 采样节点', result.stderr)
        self.assertFalse((self.root / 'result').exists())
        self.assert_business_alive()

    def test_interrupt_keeps_business_running_and_saves_partial(self):
        process = subprocess.Popen(self.command(duration='30'), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 5
            while not (self.root / 'result/metrics.csv').exists():
                if time.monotonic() > deadline:
                    self.fail('collector did not start')
                time.sleep(.03)
            time.sleep(.2)
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=10)
            self.assertEqual(process.returncode, 2, stdout + stderr)
            self.assertEqual(json.loads((self.root / 'result/summary.json').read_text())['stop_reason'], 'interrupted')
            self.assert_business_alive()
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()

    def test_perf_failure_retains_basic_report(self):
        bindir = self.root / 'bin'
        bindir.mkdir()
        perf = bindir / 'perf'
        perf.write_text('#!/bin/sh\necho "perf missing for kernel" >&2\nexit 1\n')
        perf.chmod(0o755)
        env = dict(os.environ, PATH=str(bindir) + os.pathsep + os.environ.get('PATH', ''))
        result = subprocess.run(self.command(extra=('--cpu', '--perf-nodes', 'camera')), env=env, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2, result.stderr)
        summary = json.loads((self.root / 'result/summary.json').read_text())
        self.assertEqual(summary['perf']['status'], 'unavailable')
        self.assertTrue((self.root / 'result/report.html').is_file())
        self.assert_business_alive()

    def fake_perf_environment(self):
        bindir = self.root / 'bin'
        bindir.mkdir()
        perf = bindir / 'perf'
        perf.write_text('''#!/usr/bin/env python3
import os, signal, sys, time
from pathlib import Path
if '--version' in sys.argv:
    print('perf test fixture')
    sys.exit(0)
out = Path(sys.argv[sys.argv.index('-o') + 1])
out.write_bytes(b'fixture, not real perf data')
out.with_suffix('.pid').write_text(str(os.getpid()))
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
if os.environ.get('EVAL_TEST_LARGE_PERF'):
    with out.open('wb') as stream:
        stream.truncate(2 * 1024 * 1024)
time.sleep(float(sys.argv[-1]))
''')
        perf.chmod(0o755)
        env = dict(os.environ, PATH=str(bindir) + os.pathsep + os.environ.get('PATH', ''))
        return env

    def test_perf_recorder_is_reaped_and_does_not_stop_targets(self):
        env = self.fake_perf_environment()
        result = subprocess.run(self.command(extra=('--cpu', '--perf-nodes', 'camera')), env=env, capture_output=True, text=True, timeout=10)
        self.assertIn(result.returncode, (0, 2), result.stderr)
        summary = json.loads((self.root / 'result/summary.json').read_text())
        self.assertEqual(summary['perf']['status'], 'recorded')
        pid = int((self.root / 'result/perf.pid').read_text())
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)
        self.assert_business_alive()

    def test_short_perf_and_size_limit_leave_basic_monitoring_and_targets_running(self):
        env = self.fake_perf_environment()
        before_priority = os.getpriority(os.PRIO_PROCESS, self.worker_pid)
        for name, options in [('short', ('--perf-duration', '.2')),
                              ('limited', ('--perf-max-mb', '1'))]:
            if name == 'limited':
                env['EVAL_TEST_LARGE_PERF'] = '1'
            result = subprocess.run(self.command(name=name, duration='1',
                extra=('--cpu', '--perf-nodes', 'camera', *options)),
                env=env, capture_output=True, text=True, timeout=10)
            self.assertIn(result.returncode, (0, 2), result.stderr)
            output = self.root / name
            summary = json.loads((output / 'summary.json').read_text())
            self.assertEqual(summary['perf']['status'], 'recorded' if name == 'short' else 'limited')
            self.assertEqual(summary['perf']['pids'], [self.worker_pid])
            self.assertEqual(summary['process_count'], 2)
            self.assertNotIn('--call-graph', summary['perf']['command'])
            self.assertGreaterEqual(summary['duration_s'], 1)
            if name == 'short':
                self.assertNotIn('perf_early_exit', {e['kind'] for e in summary['events']})
            else:
                self.assertEqual(summary['perf']['stop_reason'], 'size_limit')
            pid = int((output / 'perf.pid').read_text())
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)
            self.assert_business_alive()
            self.assertEqual(os.getpriority(os.PRIO_PROCESS, self.worker_pid), before_priority)

    def test_wrong_instance_log_dir_and_missing_node_fail_before_output(self):
        other = self.root / 'other'
        other.mkdir()
        with self.assertRaisesRegex(ValueError, '无法'):
            evaluate.Instance(self.launcher.pid, other)
        result = subprocess.run(self.command(extra=('--nodes', 'manipulator')), capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 1)
        self.assertFalse((self.root / 'result').exists())


if __name__ == '__main__':
    unittest.main()

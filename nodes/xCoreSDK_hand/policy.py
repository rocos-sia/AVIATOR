"""Event-driven whole-hand grip policy shared by the SDK node and grip demo.

The inherited -1 target is an internal stop marker only. It never reaches the
angle registers: all six ready channels produce one acknowledged pause write.
"""
from rh56ftp_node import Rh56FtpNode, CommandGuard, ClosingHold, SIDES, monotonic_us


class EventHandNode(Rh56FtpNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.paused = dict.fromkeys(SIDES, False)
        self.action_id = None
        self.retired_action = None

    def _command_gap_expired(self, now, previous):
        return now < previous

    def _apply_command(self, command, now):
        # Validate before changing any motion or hold state.
        candidate = CommandGuard(**self.guard.__dict__)
        candidate.accept(command, now, self.command_timeout_us, self.clock_id)
        action = command.get('action_id')
        if action is not None and (not isinstance(action, str) or not 0 < len(action) <= 128):
            raise ValueError('action_id must be a nonempty string of at most 128 characters')
        identity = (command['session_id'], action)
        if command['valid'] and action is not None and identity == self.retired_action:
            raise ValueError('Action was revoked/protected; send a new action_id to resume')
        new_action = action is not None and identity != self.action_id
        if command['valid']:
            for side in SIDES:
                before = self.last_command
                changed = (before is None or before['mode'] != command['mode'] or
                           before.get('hands', {}).get(side) != command['hands'][side])
                if new_action or not self.command_valid or (changed and self.paused[side]):
                    self.closing_holds[side] = [ClosingHold() for _ in range(6)]
                    # A new request must resume even an identical paused target.
                    self._angle_applied.pop(side, None)
        try:
            super()._apply_command(command, now)
        except Exception:
            # A partially written new request must not replay the old action.
            self.command_valid = False
            self._safe_required, self._safe_applied = True, False
            raise
        self.action_id = identity

    def _safe_pose(self, *, force_open=False, now=None):
        if self.action_id is not None:
            self.retired_action = self.action_id
        super()._safe_pose(force_open=force_open, now=now)

    def _closing_targets(self, requested, now):
        _, pending = super()._closing_targets(requested, now)
        effective = {}
        for side, channels in pending.items():
            ready = all(c.held_raw is not None for c in channels)
            effective[side] = [-1] * 6 if ready else list(requested[side])
            for c in channels:
                c.phase = 'holding' if ready else 'ready' if c.held_raw is not None else 'tracking'
        return effective, pending

    def _hold_settings(self, channel):
        return {}  # No hold speed/force writes in the pause-register policy.

    def _write_settings(self, side, targets):
        # Pause is self-contained; never set speed/force to zero before it.
        if -1 not in targets:
            super()._write_settings(side, targets)

    def _write_angles(self, side, targets):
        link = self.links[side]
        if link is None:
            return
        if -1 in targets:
            if targets != [-1] * 6:
                raise ValueError('Whole-hand pause requires six stop markers')
            if not self.paused[side]:
                self._write_call(side, 'pause', link.write_pause, [1])
                self.paused[side] = True
                self._angle_applied.pop(side, None)
                self.last_ack_write_us = monotonic_us()
                self.diag.emit('hand_paused', side=side, register=1634, value=1)
        else:
            if self.paused[side]:
                self._angle_applied.pop(side, None)
            super()._write_angles(side, targets)
            self.paused[side] = False

    def supervise(self, now=None):
        now = monotonic_us() if now is None else now
        if self.feedback_only:
            return
        self._observe_faults(now)
        unavailable = self.command_valid and any(
            link is not None and (self.snapshots[side].error or
                                  not self.snapshots[side].fresh(now, self.feedback_timeout_us))
            for side, link in self.links.items())
        if unavailable:
            self._safe_required = True
        if self._safe_required:
            if not self._safe_applied:
                try:
                    self._safe_pose(now=now)
                except Exception as exc:
                    self.diag.emit('safe_pose_failed', reason=str(exc))
            return
        if self.command_valid and self.last_command:
            try:
                self._apply_motion(self.last_command, now)
            except Exception as exc:
                # Failed pause writes remain pending; no fake successful hold/ACK.
                self.diag.emit('hold_write_failed', reason=str(exc))

    def make_state(self, now=None):
        state = super().make_state(now)
        state['command_delivery'] = 'event'
        if state['accepted_command'] is not None and self.last_command:
            state['accepted_command']['action_id'] = self.last_command.get('action_id')
        for side, hand in state['hands'].items():
            hand['paused'] = self.paused[side]
            hand['stop_criteria_met'] = [c.held_raw is not None for c in self.closing_holds[side]]
            hand['closing_hold_active'] = [self.paused[side]] * 6
            hand['sample_time_basis'] = 'host_rs485_read'
        return state

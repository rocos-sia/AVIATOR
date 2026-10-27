"""Event-driven whole-hand grip policy shared by the SDK node and grip demo.

The inherited -1 target is an internal stop marker only. It never reaches the
angle registers: all six ready channels produce one acknowledged pause write.
"""
from rh56ftp_node import WholeHandPauseNode, monotonic_us


class EventHandNode(WholeHandPauseNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.retired_action = None

    def _command_gap_expired(self, now, previous):
        return now < previous

    def _apply_command(self, command, now):
        action = command.get('action_id')
        if (command['valid'] and action is not None and
                (command['session_id'], action) == self.retired_action):
            raise ValueError('Action was revoked/protected; send a new action_id to resume')
        super()._apply_command(command, now)

    def _motion_write_failed(self, new_action, motion_changed):
        self.command_valid = False
        self._safe_required, self._safe_applied = True, False

    def _safe_pose(self, *, force_open=False, now=None):
        if self.action_id is not None:
            self.retired_action = self.action_id
        super()._safe_pose(force_open=force_open, now=now)

    def supervise(self, now=None):
        now = monotonic_us() if now is None else now
        if self.feedback_only:
            return
        self._observe_faults(now)
        if self._recovering():
            return
        # A failed poll does not erase the last successful measurement. Allow
        # retries within its existing freshness budget; the scheduler retains
        # each failed read group's timestamp, so other successful reads cannot
        # extend this deadline. Failed feedback remains invalid in make_state
        # and cannot accumulate closing criteria. Device faults above still
        # require immediate protection.
        unavailable = self.command_valid and any(
            link is not None and not self.snapshots[side].fresh(now, self.feedback_timeout_us)
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
        for side, hand in state['hands'].items():
            hand['sample_time_basis'] = 'host_rs485_read'
        return state

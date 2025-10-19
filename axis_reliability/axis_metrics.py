import os
import csv
from typing import Optional

class AxisMetrics:
    """Collects and writes key performance metrics for reliability evaluation."""

    def __init__(self, log_dir: str):
        self.log_path = os.path.join(log_dir, 'pilot_metrics.csv')
        self._ensure_file()
        self.current_episode = None
        self.last_degrade_time = None
        self.last_recovery_time = None

    def _ensure_file(self):
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'event', 'state',
                    'fallback_start_time', 'fallback_end_time',
                    'duration_s', 'avg_heading_error', 'max_heading_error',
                    'drift_rate', 'gps_downtime_s', 'abort_reason',
                    'time_to_degrade_s', 'time_to_recover_s', 'handoff_jump_m'
                ])

    # --- tracking lifecycle ---
    def mark_degrade(self, now):
        """When GPS becomes bad, before entering fallback."""
        self.last_degrade_time = now.nanoseconds / 1e9

    def start_fallback(self, now):
        """Called once when fallback mode begins."""
        start_time = now.nanoseconds / 1e9
        self.current_episode = {
            'fallback_start_time': start_time,
            'heading_errors': [],
            'abort_reason': '',
            'gps_downtime_s': 0.0,
            'time_to_degrade_s': 0.0,
            'time_to_recover_s': 0.0,
            'handoff_jump_m': 0.0,
        }
        if self.last_degrade_time:
            self.current_episode['time_to_degrade_s'] = start_time - self.last_degrade_time

    def record_heading_error(self, error: float):
        if self.current_episode:
            self.current_episode['heading_errors'].append(abs(error))

    def record_gps_downtime(self, downtime_s: float):
        if self.current_episode:
            self.current_episode['gps_downtime_s'] = downtime_s

    def finish(self, now, state: str, abort_reason: str = '', handoff_jump_m: float = 0.0):
        if not self.current_episode:
            return
        end_time = now.nanoseconds / 1e9
        start_time = self.current_episode['fallback_start_time']
        duration = end_time - start_time
        avg_error = sum(self.current_episode['heading_errors']) / len(self.current_episode['heading_errors']) \
            if self.current_episode['heading_errors'] else 0.0
        max_error = max(self.current_episode['heading_errors']) if self.current_episode['heading_errors'] else 0.0
        drift_rate = avg_error / duration if duration > 0 else 0.0

        # if recovery: compute recovery latency
        if state == 'RECOVERY':
            if not self.last_recovery_time:
                self.last_recovery_time = now.nanoseconds / 1e9
            self.current_episode['time_to_recover_s'] = self.last_recovery_time - end_time

        row = [
            now.nanoseconds / 1e9,
            'END',
            state,
            start_time,
            end_time,
            round(duration, 2),
            round(avg_error, 3),
            round(max_error, 3),
            round(drift_rate * 100, 2),
            round(self.current_episode['gps_downtime_s'], 2),
            abort_reason,
            round(self.current_episode['time_to_degrade_s'], 2),
            round(self.current_episode['time_to_recover_s'], 2),
            round(handoff_jump_m, 2)
        ]
        with open(self.log_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)

        self.summarize(row)
        self.current_episode = None

    def start_baseline(self, now):
        """Initialize a baseline-only session (no fallback/recovery)."""
        # timestamps
        self._start_time = now
        self._end_time = None

        # mode/result
        self._mode = 'BASELINE'
        self._result = None
        self._abort_reason = None

        # metrics we want to show in the summary
        self._drift_rate = 0.0
        self._avg_heading_error = 0.0
        self._max_heading_error = 0.0
        self._gps_downtime = 0.0

        # optional helpers other code might expect
        self._marked_degrade_time = None
        self._marked_recover_time = None

    def mark_recovery(self, now, downtime_s: float):
        """For baseline: accumulate GPS downtime when it recovers."""
        try:
            self._gps_downtime += float(downtime_s)
        except Exception:
            # be robust if unset
            if not hasattr(self, "_gps_downtime"):
                self._gps_downtime = 0.0
            self._gps_downtime += float(downtime_s)

    def summarize(self, row):
        print("\n===== AXIS METRIC SUMMARY =====")
        print(f"Drift rate:         {row[8]:.2f}%")
        print(f"Avg heading error:  {row[6]:.3f} rad")
        print(f"Max heading error:  {row[7]:.3f} rad")
        print(f"Fallback duration:  {row[5]:.2f} s")
        print(f"GPS downtime:       {row[9]:.2f} s")
        print(f"Time to degrade:    {row[11]:.2f} s")
        print(f"Time to recover:    {row[12]:.2f} s")
        print(f"Handoff jump:       {row[13]:.2f} m")
        print(f"Abort reason:       {row[10] or '—'}")
        print("================================")
        print("Check /tmp/axis_logs/pilot_metrics.csv for full session logs\n")

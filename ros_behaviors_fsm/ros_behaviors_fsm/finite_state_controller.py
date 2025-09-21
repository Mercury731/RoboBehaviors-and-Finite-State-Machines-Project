"""
Finite-State Supervisor for Behavior Switching (+ periodic 360 spin during FOLLOW)
---------------------------------------------------------------------------------
Runs ONE of two existing behaviors as a subprocess:
  - draw_pentagon (time-based driving)
  - person_follower (LaserScan-based following)

NEW:
  - While in FOLLOW, performs a 360° spin every `spin_interval_s` seconds by
    temporarily stopping the follower, running a one-shot spin node, then
    resuming the follower.

Assumed console entry points:
  'person_follower = ros_behaviors_fsm.person_follower:main'
  'draw_pentagon   = ros_behaviors_fsm.draw_pentagon:main'
  'spin_360  = ros_behaviors_fsm.spin_360:main'   # <-- one-shot spin
"""

import math
import os
import signal
import subprocess
import sys
import time
from enum import Enum
from threading import Thread, Event, Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from gazebo_msgs.msg import ContactsState


class Mode(Enum):
    PENTAGON = 0
    FOLLOW = 1
    IDLE = 2


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm_supervisor')

        # ---- Parameters
        self.declare_parameter('object_present_threshold_m', 1.5)
        self.declare_parameter('lost_object_timeout_s', 2.0)
        self.declare_parameter('poll_rate_hz', 10.0)

        self.declare_parameter('bumper_cooldown_s', 0.5)
        self.declare_parameter('bumper_min_msgs', 1)

        # NEW: spin controls
        self.declare_parameter('spin_interval_s', 15.0)
        self.declare_parameter('spin_timeout_s', 10.0)
        self.declare_parameter('spin_pkg', 'ros_behaviors_fsm')
        self.declare_parameter('spin_exec', 'spin_360')

        self.object_present_threshold_m = float(self.get_parameter('object_present_threshold_m').value)
        self.lost_object_timeout_s = float(self.get_parameter('lost_object_timeout_s').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.poll_dt = 1.0 / max(self.poll_rate_hz, 1.0)

        self._bumper_cooldown_s = float(self.get_parameter('bumper_cooldown_s').value)
        self._bumper_min_msgs = int(self.get_parameter('bumper_min_msgs').value)

        # NEW: spin params
        self._spin_interval_s = float(self.get_parameter('spin_interval_s').value)
        self._spin_timeout_s = float(self.get_parameter('spin_timeout_s').value)
        self._spin_pkg = str(self.get_parameter('spin_pkg').value)
        self._spin_exec = str(self.get_parameter('spin_exec').value)

        # ---- Pub/Sub
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(ContactsState, 'bumper', self._on_bumper_contacts, 10)

        # ---- Internal state
        self.mode = Mode.PENTAGON
        self.child_proc = None
        self.estopped = False

        # LaserScan bookkeeping
        self._last_seen_ts = None
        self._has_target_now = False

        # Bumper debounce/cooldown
        self._bumper_contact_streak = 0
        self._last_bumper_switch_ts = 0.0

        # NEW: spin scheduler state
        self._last_spin_ts = 0.0
        self._spin_thread = Thread(target=self._spin_scheduler_loop, daemon=True)
        self._spin_in_progress = Event()
        self._proc_lock = Lock()  # guard child_proc mutation between threads

        # Start default behavior
        self._start_pentagon()

        # Timer to evaluate FOLLOW → PENTAGON fallback
        self.create_timer(self.poll_dt, self._tick)

        # Kick off spin scheduler
        self._spin_thread.start()

        self.get_logger().info(
            f"BehaviorFSM ready. scan_threshold={self.object_present_threshold_m:.2f} m, "
            f"lost_timeout={self.lost_object_timeout_s:.1f} s, "
            f"bumper_min_msgs={self._bumper_min_msgs}, bumper_cooldown={self._bumper_cooldown_s:.2f}s, "
            f"spin_interval={self._spin_interval_s:.1f}s"
        )

    # -------------------- Subscribers --------------------

    def _on_bumper_contacts(self, msg: ContactsState):
        if self.estopped:
            return

        has_contact = len(msg.states) > 0
        now = time.monotonic()

        if has_contact:
            self._bumper_contact_streak += 1
        else:
            self._bumper_contact_streak = 0

        should_switch = (
            has_contact
            and self._bumper_contact_streak >= self._bumper_min_msgs
            and (now - self._last_bumper_switch_ts) >= self._bumper_cooldown_s
            and self.mode != Mode.FOLLOW
        )

        if should_switch:
            self.get_logger().info("Bumper contact (confirmed) → switching to FOLLOW.")
            self._last_bumper_switch_ts = now
            self._switch_to_follow()

    def _on_estop(self, msg: Bool):
        if msg.data and not self.estopped:
            self.estopped = True
            self.get_logger().warn("ESTOP asserted → entering IDLE and stopping.")
            # Abort any ongoing spin
            self._abort_spin_if_running()
            self._kill_child()
            self._publish_stop()
            self.mode = Mode.IDLE
        elif not msg.data and self.estopped:
            self.estopped = False
            self.get_logger().warn("ESTOP cleared → resuming PENTAGON.")
            self._start_pentagon()

    def _on_scan(self, msg: LaserScan):
        has_target = False
        thr = self.object_present_threshold_m
        for d in msg.ranges:
            if d is None or math.isinf(d) or math.isnan(d) or d <= 0.0:
                continue
            if d <= thr:
                has_target = True
                break

        self._has_target_now = has_target
        if has_target:
            self._last_seen_ts = time.monotonic()

    # -------------------- Timer: FOLLOW fallback --------------------

    def _tick(self):
        if self.estopped:
            return
        if self.mode == Mode.FOLLOW:
            now = time.monotonic()
            no_view_duration = float('inf') if self._last_seen_ts is None else (now - self._last_seen_ts)
            if no_view_duration > self.lost_object_timeout_s and not self._spin_in_progress.is_set():
                self.get_logger().info(
                    f"FOLLOW: lost target for {no_view_duration:.1f}s → switching back to PENTAGON."
                )
                self._start_pentagon()

    # -------------------- NEW: Spin scheduler --------------------

    def _spin_scheduler_loop(self):
        """Background loop that triggers a 360° spin every _spin_interval_s while FOLLOW is active."""
        while rclpy.ok():
            time.sleep(0.1)
            if self.estopped or self.mode != Mode.FOLLOW:
                continue
            now = time.monotonic()
            if (now - self._last_spin_ts) >= self._spin_interval_s and not self._spin_in_progress.is_set():
                self._perform_spin_once()

    def _perform_spin_once(self):
        """Pause follower, run one-shot spin subprocess, then resume follower."""
        if self.estopped or self.mode != Mode.FOLLOW:
            return

        self._spin_in_progress.set()
        self.get_logger().info("FOLLOW active → scheduling 360° spin.")

        # 1) Stop current follower cleanly
        with self._proc_lock:
            self._kill_child()

        if self.estopped:
            self._spin_in_progress.clear()
            return

        # 2) Run the spin node as a blocking subprocess
        spin_cmd = ['ros2', 'run', self._spin_pkg, self._spin_exec]
        self.get_logger().info(f"Running spin: {' '.join(spin_cmd)}")
        try:
            spin_proc = subprocess.Popen(
                spin_cmd,
                stdout=sys.stdout,
                stderr=sys.stderr,
                preexec_fn=os.setsid
            )

            start = time.monotonic()
            # Wait until done or timeout/estop
            while spin_proc.poll() is None:
                if self.estopped:
                    self.get_logger().warn("ESTOP during spin → aborting spin.")
                    self._terminate_proc_group(spin_proc)
                    break
                if (time.monotonic() - start) > self._spin_timeout_s:
                    self.get_logger().warn("Spin exceeded timeout → terminating spin.")
                    self._terminate_proc_group(spin_proc)
                    break
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"Failed to launch spin: {e}")

        # 3) Resume FOLLOW if appropriate
        if not self.estopped and self.mode != Mode.IDLE:
            self.get_logger().info("Resuming FOLLOW after spin.")
            self._spawn_behavior(['ros2', 'run', 'ros_behaviors_fsm', 'person_follower'])
            self.mode = Mode.FOLLOW
            # Give FOLLOW a fair chance to reacquire
            self._last_seen_ts = time.monotonic() if self._has_target_now else None

        self._last_spin_ts = time.monotonic()
        self._spin_in_progress.clear()

    def _terminate_proc_group(self, proc):
        """Helper to gracefully then forcefully terminate a subprocess group."""
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            return
        # brief wait
        waited = 0.0
        while proc.poll() is None and waited < 1.5:
            time.sleep(0.1)
            waited += 0.1
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        waited2 = 0.0
        while proc.poll() is None and waited2 < 1.0:
            time.sleep(0.1)
            waited2 += 0.1
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    # -------------------- Process Management --------------------

    def _start_pentagon(self):
        with self._proc_lock:
            self._spawn_behavior(['ros2', 'run', 'ros_behaviors_fsm', 'draw_pentagon'])
            self.mode = Mode.PENTAGON
            self._last_seen_ts = None
            self._has_target_now = False

    def _switch_to_follow(self):
        with self._proc_lock:
            self._spawn_behavior(['ros2', 'run', 'ros_behaviors_fsm', 'person_follower'])
            self.mode = Mode.FOLLOW
            self._last_seen_ts = time.monotonic() if self._has_target_now else None
            # Reset spin timer so we don't spin immediately on entry
            self._last_spin_ts = time.monotonic()

    def _spawn_behavior(self, cmd):
        self._kill_child()
        self.get_logger().info(f"Launching behavior: {' '.join(cmd)}")
        try:
            self.child_proc = subprocess.Popen(
                cmd,
                stdout=sys.stdout,
                stderr=sys.stderr,
                preexec_fn=os.setsid
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start behavior '{cmd}': {e}")
            self.child_proc = None

    def _kill_child(self):
        if self.child_proc is None:
            return
        if self.child_proc.poll() is not None:
            self.child_proc = None
            return
        try:
            self.get_logger().info("Stopping current behavior...")
            os.killpg(os.getpgid(self.child_proc.pid), signal.SIGINT)
            waited = 0.0
            while self.child_proc.poll() is None and waited < 2.0:
                time.sleep(0.1)
                waited += 0.1
            if self.child_proc.poll() is None:
                self.get_logger().warn("Behavior not exiting; sending SIGTERM.")
                os.killpg(os.getpgid(self.child_proc.pid), signal.SIGTERM)
            waited2 = 0.0
            while self.child_proc.poll() is None and waited2 < 1.0:
                time.sleep(0.1)
                waited2 += 0.1
            if self.child_proc.poll() is None:
                self.get_logger().error("Force-killing behavior (SIGKILL).")
                os.killpg(os.getpgid(self.child_proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        finally:
            self.child_proc = None

    # -------------------- Utilities --------------------

    def _publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

    def _abort_spin_if_running(self):
        """Flag for any spinning loop to bail; actual proc kill is handled in _perform_spin_once."""
        self._spin_in_progress.set()  # prevents new spins; current one gets killed by ESTOP branch

    # Ensure child is stopped on node shutdown
    def destroy_node(self):
        try:
            self._abort_spin_if_running()
            self._kill_child()
            self._publish_stop()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Log the robot's pose in the map frame and work out where pose jumps come from.

RViz draws the robot at map -> odom -> base_footprint, so a jump on screen
comes from one of these layers:

    map -> odom             AMCL correction, or a new initial pose from RViz
    odom -> base_footprint  the EKF / odometry
    /odom ground truth      the simulated robot really moved (slip, fall, hit)
    TF itself               two publishers for one frame, a frame changing
                            parent, sim time going backwards, or TF stalling
                            and then catching up

Every tick each layer is written to samples.csv. A jump in any layer goes to
events.csv with its cause, and Ctrl+C prints a summary (also saved as
summary.txt). Plot a run with plot_pose_log.

AMCL stamps map -> odom transform_tolerance (1 s by default) in the future, so
RViz draws each AMCL correction about that long after it is logged here.

Run it next to the simulation, from the workspace root:

    python3 src/unitree_go2_slam/unitree_go2_slam/pose_logger.py
    ros2 run unitree_go2_slam pose_logger --ros-args -p jump_dist:=0.05
"""

from collections import Counter, deque
import csv
from datetime import datetime
import math
import os
import statistics
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMsg
from tf2_msgs.msg import TFMessage
import tf2_ros

SAMPLE_COLUMNS = [
    'wall_time', 'sim_time', 'rtf',
    'map_x', 'map_y', 'map_yaw_deg',      # map -> base: where RViz draws the robot
    'odom_x', 'odom_y', 'odom_yaw_deg',   # odom -> base: EKF
    'corr_x', 'corr_y', 'corr_yaw_deg',   # map -> odom: latest AMCL correction
    'gt_x', 'gt_y', 'gt_yaw_deg',         # /odom: Gazebo ground truth
    'map_step', 'odom_step', 'gt_step',   # movement since the previous sample (m)
    'corr_shift',                         # how far the latest AMCL correction moved the robot (m)
    'loc_err',                            # |map - gt|, valid if the map is aligned with the world
    'amcl_x', 'amcl_y', 'amcl_yaw_deg', 'amcl_sigma_xy', 'amcl_sigma_yaw_deg',
    'cmd_vx', 'cmd_vy', 'cmd_wz',
    'tf_age',                             # sim_time minus the odom -> base stamp (s)
]
# x, y is where the robot is drawn in the map (in the layer's own frame before AMCL is up).
EVENT_COLUMNS = [
    'wall_time', 'sim_time', 'event', 'cause', 'x', 'y', 'dist_m', 'dyaw_deg', 'details']
JUMP_EVENTS = ('AMCL_JUMP', 'ODOM_JUMP', 'GT_JUMP', 'TF_STALL')

VERDICTS = {
    'amcl': 'AMCL corrections (map -> odom) while odometry stayed smooth, so look at '
            'AMCL tuning and how well the scan matches the map',
    'initial_pose': 'initial poses set from RViz (expected)',
    'odometry': 'the odometry/EKF layer (odom -> base) while the simulated robot moved smoothly',
    'physics': 'the simulated robot really moving abruptly in Gazebo (slip, fall, collision)',
    'tf_stall': 'TF stalls: RViz freezes and then catches up because the sim or the EKF '
                'is not keeping up (see rtf and tf_age)',
}


def _secs(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _compose(a, b):
    """Chain 2D poses (x, y, yaw, stamp): pose b given in the frame of pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]), b[3])


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _score(step):
    """Return how big a step is relative to the jump threshold (above 1.0 is a jump)."""
    return 0.0 if step is None else step[2]


def _fmt(value, digits=3):
    return '' if value is None else f'{value:.{digits}f}'


def _pose_cols(pose):
    if pose is None:
        return ['', '', '']
    return [f'{pose[0]:.4f}', f'{pose[1]:.4f}', f'{math.degrees(pose[2]):.2f}']


def _node_name(endpoint):
    return endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name


def _node_list(names):
    """Format a Counter of node names; two nodes with one name usually means a leftover launch."""
    return ', '.join(n if k == 1 else f'{n} (x{k})' for n, k in sorted(names.items())) or 'none'


class PoseLogger(Node):

    def __init__(self):
        super().__init__('pose_logger')
        param = self.declare_parameter
        self.map_frame = param('map_frame', 'map').value
        self.odom_frame = param('odom_frame', 'odom').value
        self.base_frame = param('base_frame', 'base_footprint').value
        rate_hz = param('rate_hz', 20.0).value
        # A step is a jump when it is bigger than these plus what the robot can really move
        # since the previous sample. CHAMP's gait limits are 0.3 m/s and 0.5 rad/s.
        self.jump_dist = max(param('jump_dist', 0.10).value, 1e-3)
        self.jump_yaw = math.radians(max(param('jump_yaw_deg', 6.0).value, 0.1))
        self.max_speed = param('max_speed', 0.6).value
        self.max_yaw_rate = math.radians(param('max_yaw_rate_deg', 60.0).value)
        self.stall_timeout = param('stall_timeout', 0.5).value
        output_dir = param('output_dir', 'pose_logs').value

        base = os.path.join(os.path.abspath(os.path.expanduser(output_dir)),
                            datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.run_dir, n = base, 1
        while os.path.exists(self.run_dir):  # never overwrite an earlier run
            self.run_dir, n = f'{base}_{n}', n + 1
        os.makedirs(self.run_dir)
        self._files = []
        self.samples = self._open_csv('samples.csv', SAMPLE_COLUMNS)
        self.events = self._open_csv('events.csv', EVENT_COLUMNS)

        self.tf_buffer = tf2_ros.Buffer()
        self.sim_time = None                  # latest /clock; stays None without a simulator
        self.gt = None                        # latest ground truth pose
        self.amcl = None                      # (x, y, yaw, sigma_xy, sigma_yaw)
        self.cmd = None                       # (vx, vy, wz)
        self.last_initial_pose = -math.inf    # monotonic times
        self.last_gt_jump = -math.inf
        self.prev = (None, None, None, None)  # last known (map, corr, odom, gt) poses
        self.last_odom_update = None          # (monotonic, sim) when odom -> base last changed
        self.stall_start = None
        self.rtf_window = deque()
        self.tf_edges = {'/tf': {}, '/tf_static': {}}  # child -> (parent, stamp, value, seq)
        self.topic_publishers = {}       # current publishers per watched topic
        self.topic_publishers_seen = {}  # every publisher seen during the run, per topic
        self.tf_last_report = {}
        self.last_shown = {}
        self.hidden = Counter()

        self.start = time.monotonic()
        self.n_samples = 0
        self.max_map_step = 0.0
        self.counts = Counter()       # events by type
        self.causes = Counter()       # (event, cause) for jumps and stalls
        self.tf_problems = Counter()  # (problem, frame)
        self.corrections = []         # (shift, dyaw) of every AMCL correction over 1 cm / 0.5 deg
        self.stalls = []              # stall durations (s)
        self.rtfs = []

        tf_qos = QoSProfile(depth=100)
        tf_static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf', self._on_tf, tf_qos)
        self.create_subscription(TFMessage, '/tf_static', self._on_tf_static, tf_static_qos)
        self.create_subscription(ClockMsg, '/clock', self._on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_ground_truth, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initial_pose, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        # Sample on wall time so that a stalled or paused simulation still shows up.
        wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / rate_hz, self._tick, clock=wall_clock)
        self.create_timer(5.0, self._check_publishers, clock=wall_clock)

        self.get_logger().info(
            f'Logging {self.map_frame} -> {self.odom_frame} -> {self.base_frame} at '
            f'{rate_hz:g} Hz to {self.run_dir}. A jump is more than {self.jump_dist:g} m or '
            f'{math.degrees(self.jump_yaw):g} deg beyond real motion. Ctrl+C for a summary.')

    def _open_csv(self, name, columns):
        f = open(os.path.join(self.run_dir, name), 'w', newline='', buffering=1)
        self._files.append(f)
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(columns)
        return writer

    # --- Subscriptions ---------------------------------------------------------------------

    def _on_tf(self, msg, info):
        seq = info.get('publication_sequence_number')  # None on Cyclone DDS
        for t in msg.transforms:
            self._check_tf(t, '/tf', seq)
            self.tf_buffer.set_transform(t, 'pose_logger')

    def _on_tf_static(self, msg, info):
        for t in msg.transforms:
            self._check_tf(t, '/tf_static', None)
            self.tf_buffer.set_transform_static(t, 'pose_logger')

    def _on_clock(self, msg):
        now = _secs(msg.clock)
        if self.sim_time is not None and now < self.sim_time:
            self._event('TIME_JUMP_BACK', 'clock', details=(
                f'sim time went from {self.sim_time:.3f} to {now:.3f} s '
                '(simulation reset?); TF history cleared'))
            self.tf_buffer.clear()
            self.tf_edges['/tf'].clear()
            self.prev = (None, None, None, None)
            self.gt = None
            self.last_odom_update = None
            self.stall_start = None
            self.rtf_window.clear()
        self.sim_time = now

    def _on_ground_truth(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, _yaw(p.orientation), _secs(msg.header.stamp))

    def _on_amcl(self, msg):
        p, cov = msg.pose.pose, msg.pose.covariance
        self.amcl = (p.position.x, p.position.y, _yaw(p.orientation),
                     math.sqrt(max(cov[0] + cov[7], 0.0)), math.sqrt(max(cov[35], 0.0)))

    def _on_initial_pose(self, msg):
        p = msg.pose.pose
        self.last_initial_pose = time.monotonic()
        self._event('INITIAL_POSE', 'initial_pose', (p.position.x, p.position.y), details=(
            f'initial pose set with yaw {math.degrees(_yaw(p.orientation)):.1f} deg, '
            'AMCL will jump there'))

    def _on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

    # --- TF health -------------------------------------------------------------------------

    def _check_tf(self, t, topic, seq):
        """Track each TF edge and flag the patterns that make tf2 (and RViz) misbehave."""
        child, parent = t.child_frame_id, t.header.frame_id
        stamp = _secs(t.header.stamp)
        tr, q = t.transform.translation, t.transform.rotation
        value = (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
        other = '/tf_static' if topic == '/tf' else '/tf'
        if child in self.tf_edges[other]:
            self._tf_problem('TF_STATIC_AND_DYNAMIC', child,
                             f'{child} is published on both /tf and /tf_static')
        edges = self.tf_edges[topic]
        prev = edges.get(child)
        edges[child] = (parent, stamp, value, seq)
        if prev is None:
            return
        p_parent, p_stamp, p_value, p_seq = prev
        changed = max(abs(a - b) for a, b in zip(value, p_value)) > 1e-3
        if parent != p_parent:
            self._tf_problem('TF_PARENT_CHANGED', child,
                             f'parent of {child} switched {p_parent} -> {parent} on {topic}')
        elif topic == '/tf_static':
            if changed:
                self._tf_problem('TF_CONFLICT', child,
                                 f'two different static {parent} -> {child} transforms')
        elif seq is not None and p_seq is not None and seq < p_seq:
            # Every publisher numbers its own messages, so a counter going back means a second
            # publisher (or the first one restarted).
            self._tf_problem('TF_MULTIPLE_PUBLISHERS', child,
                             f'{parent} -> {child} comes from more than one publisher')
        elif stamp < p_stamp:
            self._tf_problem('TF_OUT_OF_ORDER', child,
                             f'{parent} -> {child} stamp went back {p_stamp - stamp:.3f} s')
        elif stamp == p_stamp and changed:
            self._tf_problem('TF_CONFLICT', child,
                             f'two different {parent} -> {child} transforms with the same stamp')

    def _tf_problem(self, kind, frame, details):
        key = (kind, frame)
        self.tf_problems[key] += 1
        now = time.monotonic()
        if now - self.tf_last_report.get(key, -math.inf) >= 5.0:  # rate limit per frame
            self.tf_last_report[key] = now
            self._event(kind, 'tf', details=f'{details} ({self.tf_problems[key]}x so far)')

    def _check_publishers(self):
        # /odom is watched too: it must be Gazebo alone, because it is both the EKF input and
        # the ground truth here (the plain-sim launch remaps a CHAMP EKF's output onto it).
        for topic in ('/tf', '/tf_static', '/odom'):
            names = Counter(_node_name(e) for e in self.get_publishers_info_by_topic(topic))
            seen = self.topic_publishers_seen.setdefault(topic, Counter())
            for name, k in names.items():
                seen[name] = max(seen[name], k)
            nodes = _node_list(names)
            if nodes == self.topic_publishers.get(topic):
                continue
            self.topic_publishers[topic] = nodes
            self._event('PUBLISHERS', topic, details=f'{topic} publishers: {nodes}')
            if topic == '/odom' and sum(names.values()) > 1:
                self._event('GT_MULTIPLE_PUBLISHERS', 'odom', details=(
                    f'/odom has {sum(names.values())} publishers; the EKF fuses all of them '
                    'and the ground truth checks are unreliable'))

    # --- Sampling --------------------------------------------------------------------------

    def _tick(self):
        now = time.monotonic()
        sim = self._sim_now()
        current = (self._lookup(self.map_frame, self.base_frame),
                   self._lookup(self.map_frame, self.odom_frame),
                   self._lookup(self.odom_frame, self.base_frame),
                   self.gt)
        rtf = self._rtf(now, sim)
        self._check_stall(current[2], now, sim)
        steps = self._check_jumps(*current, now, sim)
        self._write_sample(sim, rtf, current, steps)
        # Keep the last good pose of each layer so a failed lookup cannot hide a jump.
        self.prev = tuple(c if c is not None else p for c, p in zip(current, self.prev))

    def _sim_now(self):
        if self.sim_time is not None:
            return self.sim_time
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup(self, target, source):
        """Return the latest transform as an (x, y, yaw, stamp) pose, or None."""
        try:
            t = self.tf_buffer.lookup_transform(target, source, Time())
        except tf2_ros.TransformException:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, _yaw(t.transform.rotation), _secs(t.header.stamp))

    def _rtf(self, now, sim):
        if self.sim_time is None:
            return None
        self.rtf_window.append((now, sim))
        while now - self.rtf_window[0][0] > 2.0:
            self.rtf_window.popleft()
        wall_span = now - self.rtf_window[0][0]
        if wall_span < 0.5:
            return None
        rtf = (sim - self.rtf_window[0][1]) / wall_span
        self.rtfs.append(rtf)
        return rtf

    def _check_stall(self, odom, now, sim):
        """Flag odom -> base TF that stops updating: RViz freezes, then jumps when it resumes."""
        if odom is None:
            return
        prev = self.prev[2]
        if prev is None or odom[3] != prev[3]:
            if self.stall_start is not None:
                self._end_stall(now, sim)
            self.last_odom_update = (now, sim)
        elif (self.stall_start is None and self.last_odom_update is not None
              and now - self.last_odom_update[0] > self.stall_timeout):
            self.stall_start = self.last_odom_update
            self.get_logger().warn(
                f'{self.odom_frame} -> {self.base_frame} TF has not updated for '
                f'{now - self.stall_start[0]:.1f} s')

    def _end_stall(self, now, sim, note=''):
        wall0, sim0 = self.stall_start
        self.stall_start = None
        self.stalls.append(now - wall0)
        self._event('TF_STALL', 'tf_stall', details=(
            f'no new {self.odom_frame} -> {self.base_frame} TF for {now - wall0:.2f} s '
            f'while sim time advanced {sim - sim0:.2f} s{note}'))

    def _step(self, prev, cur, allow_motion=True):
        """Return (dist, dyaw, score) between two poses; a score above 1 is a jump."""
        if prev is None or cur is None:
            return None
        dist, dyaw = _dist(prev, cur), abs(_wrap(cur[2] - prev[2]))
        dt = max(cur[3] - prev[3], 0.0) if allow_motion else 0.0
        return dist, dyaw, max(dist / (self.jump_dist + self.max_speed * dt),
                               dyaw / (self.jump_yaw + self.max_yaw_rate * dt))

    def _check_jumps(self, map_pose, corr, odom, gt, now, sim):
        """Check each layer against its own previous sample and log the ones that jumped."""
        p_map, p_corr, p_odom, p_gt = self.prev
        m = self._step(p_map, map_pose)
        o = self._step(p_odom, odom)
        g = self._step(p_gt, gt)
        c = None
        if p_corr is not None and corr is not None and odom is not None:
            # Where the robot sits in the map before vs after AMCL replaced map -> odom.
            before, after = _compose(p_corr, odom), _compose(corr, odom)
            c = self._step(before, after, allow_motion=False)
            if c[0] > 0.01 or c[1] > math.radians(0.5):
                self.corrections.append(c[:2])
        if m is not None:
            self.max_map_step = max(self.max_map_step, m[0])

        if _score(g) > 1.0:
            self.last_gt_jump = now
            self._event('GT_JUMP', 'physics', map_pose or gt, g, self._details(
                'the simulated robot itself moved; odometry follows it'))
        # The EKF fuses ground truth, so an odom jump right after a ground truth jump is physics.
        if _score(o) > 1.0 and now - self.last_gt_jump > 0.5:
            self._event('ODOM_JUMP', 'odometry', map_pose or odom, o, self._details(
                'odometry jumped while ground truth did not'))
        if _score(c) > 1.0:
            cause = 'initial_pose' if now - self.last_initial_pose < 3.0 else 'amcl'
            notes = []
            if gt is not None:
                notes.append(f'map vs ground truth {_dist(before, gt):.2f} -> '
                             f'{_dist(after, gt):.2f} m')
            if corr[3] - sim > 0.05:
                notes.append(f'RViz draws it in about {corr[3] - sim:.1f} s')
            self._event('AMCL_JUMP', cause, after, c, self._details(*notes))
        return m, o, g, c

    def _details(self, *notes):
        parts = list(notes)
        if self.amcl is not None:
            parts.append(f'amcl sigma {self.amcl[3]:.2f} m / '
                         f'{math.degrees(self.amcl[4]):.1f} deg')
        if self.cmd is not None:
            parts.append('cmd_vel vx {:.2f} vy {:.2f} wz {:.2f}'.format(*self.cmd))
        return '; '.join(parts)

    def _write_sample(self, sim, rtf, current, steps):
        map_pose, corr, odom, gt = current
        loc_err = None if map_pose is None or gt is None else _dist(map_pose, gt)
        amcl = ['', '', '', '', ''] if self.amcl is None else _pose_cols(self.amcl) + [
            f'{self.amcl[3]:.4f}', f'{math.degrees(self.amcl[4]):.2f}']
        cmd = ['', '', ''] if self.cmd is None else [f'{v:.3f}' for v in self.cmd]
        self.samples.writerow(
            [f'{time.time():.3f}', f'{sim:.3f}', _fmt(rtf, 2)]
            + _pose_cols(map_pose) + _pose_cols(odom) + _pose_cols(corr) + _pose_cols(gt)
            + [_fmt(None if s is None else s[0], 4) for s in steps]
            + [_fmt(loc_err, 4)] + amcl + cmd
            + [_fmt(None if odom is None else sim - odom[3])])
        self.n_samples += 1

    # --- Reporting -------------------------------------------------------------------------

    def _event(self, kind, cause, pose=None, step=None, details=''):
        self.counts[kind] += 1
        if kind in JUMP_EVENTS:
            self.causes[(kind, cause)] += 1
        x, y = (None, None) if pose is None else pose[:2]
        dist, deg = (None, None) if step is None else (step[0], math.degrees(step[1]))
        self.events.writerow([f'{time.time():.3f}', _fmt(self._sim_now()), kind, cause,
                              _fmt(x), _fmt(y), _fmt(dist), _fmt(deg, 1), details])
        # Every event goes to the CSV, but a flickering frame would flood the console, so show
        # each kind of jump at most once a second.
        key = (kind, cause)
        if kind in JUMP_EVENTS:
            now = time.monotonic()
            if now - self.last_shown.get(key, -math.inf) < 1.0:
                self.hidden[key] += 1
                return
            self.last_shown[key] = now
        text = f'{kind} [{cause}]'
        if step is not None:
            text += f' {dist:.2f} m / {deg:.1f} deg'
        if x is not None:
            text += f' at ({x:.2f}, {y:.2f})'
        text += f': {details}'
        hidden = self.hidden.pop(key, 0)
        if hidden:
            text += f' (+{hidden} more since the last one shown)'
        # rclpy fixes the severity of each logging call site, so use two of them.
        if not self.context.ok():  # after Ctrl+C the ROS logger can no longer publish
            print(text, flush=True)
        elif kind in ('INITIAL_POSE', 'PUBLISHERS'):
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    def _verdict(self):
        if self.counts['GT_MULTIPLE_PUBLISHERS']:
            return ('more than one node publishes /odom, so the EKF mixes sources: '
                    'see the /odom publishers above')
        chain = (self.odom_frame, self.base_frame)
        tf_bad = sorted({f'{kind} on {frame}' for kind, frame in self.tf_problems
                         if frame in chain})
        if tf_bad:
            return 'TF problems on the robot pose chain: ' + ', '.join(tf_bad)
        tally = Counter()
        for (_, cause), n in self.causes.items():
            tally[cause] += n
        if not tally:
            return (f'no jumps over {self.jump_dist:g} m / {math.degrees(self.jump_yaw):g} deg; '
                    'lower jump_dist or jump_yaw_deg to catch smaller ones')
        cause, n = tally.most_common(1)[0]
        return f'{VERDICTS[cause]} ({n} of {sum(tally.values())} jumps and stalls)'

    def _summary(self):
        counts = self.counts
        lines = ['', '=== pose_logger summary ===', f'Logs: {self.run_dir}']
        line = f'Ran {time.monotonic() - self.start:.0f} s, {self.n_samples} samples'
        if self.rtfs:
            line += (f', real-time factor mean {statistics.mean(self.rtfs):.2f} '
                     f'(min {min(self.rtfs):.2f})')
        lines.append(line)
        lines.append('Largest step of the drawn robot (map -> base) between samples: '
                     f'{self.max_map_step:.3f} m')
        line = f'AMCL jumps: {counts["AMCL_JUMP"]}'
        initial = self.causes[('AMCL_JUMP', 'initial_pose')]
        if initial:
            line += f' ({initial} after an RViz initial pose)'
        if self.corrections:
            shifts = [s for s, _ in self.corrections]
            max_yaw = math.degrees(max(a for _, a in self.corrections))
            line += (f'; all corrections over 1 cm / 0.5 deg: {len(shifts)}, median '
                     f'{statistics.median(shifts):.3f} m, max {max(shifts):.3f} m / '
                     f'{max_yaw:.1f} deg')
        lines.append(line)
        lines.append(f'Odometry jumps: {counts["ODOM_JUMP"]}; ground truth (physics) jumps: '
                     f'{counts["GT_JUMP"]}; sim time resets: {counts["TIME_JUMP_BACK"]}')
        if self.stalls:
            lines.append(f'TF stalls: {len(self.stalls)}, longest {max(self.stalls):.2f} s')
        problems = ', '.join(f'{kind} {frame} {n}x'
                             for (kind, frame), n in self.tf_problems.most_common())
        lines.append('TF problems: ' + (problems or 'none'))
        for topic, names in self.topic_publishers_seen.items():
            lines.append(f'{topic} publishers seen: {_node_list(names)}')
        lines.append(f'Most likely source: {self._verdict()}')
        return lines

    def close(self):
        if self.stall_start is not None:
            self._end_stall(time.monotonic(), self._sim_now(), ' (still stalled at exit)')
        text = '\n'.join(self._summary())
        print(text, flush=True)
        with open(os.path.join(self.run_dir, 'summary.txt'), 'w') as f:
            f.write(text + '\n')
        for f in self._files:
            f.close()


def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""
Plot a pose_logger run so each jump can be traced to the layer that caused it.

    python3 src/unitree_go2_slam/unitree_go2_slam/plot_pose_log.py [RUN_DIR]
    ros2 run unitree_go2_slam plot_pose_log [RUN_DIR]

RUN_DIR defaults to the newest run under ./pose_logs. The figure is saved as
RUN_DIR/pose_jumps.png.
"""

import argparse
import csv
import glob
import os
import re
import sys
import textwrap

from matplotlib.figure import Figure
import numpy as np

SURFACE, INK, INK2, MUTED, GRID, AXIS = (
    '#fcfcfb', '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7')
# One hue per layer that can cause a jump (categorical slots 1-3, validated all-pairs);
# the drawn pose and system health stay gray.
AMCL, ODOM, PHYS = '#2a78d6', '#eb6834', '#1baf7a'

# Event rows, top to bottom: label, color, marker. Shapes back up the colors.
ROWS = [
    ('AMCL jump', AMCL, 'o'),
    ('Initial pose', MUTED, 'o'),
    ('Odometry jump', ODOM, 's'),
    ('Physics jump', PHYS, '^'),
    ('TF stall', INK2, 'D'),
    ('TF problem', INK2, 'X'),
    ('Sim time reset', INK2, 'v'),
]
STYLE = {label: (color, marker) for label, color, marker in ROWS}
ROW_OF_EVENT = {'AMCL_JUMP': 'AMCL jump', 'ODOM_JUMP': 'Odometry jump',
                'GT_JUMP': 'Physics jump', 'TF_STALL': 'TF stall',
                'TIME_JUMP_BACK': 'Sim time reset'}


def _row(event):
    if event['event'] == 'INITIAL_POSE' or event['cause'] == 'initial_pose':
        return 'Initial pose'
    if event['event'] in ROW_OF_EVENT:
        return ROW_OF_EVENT[event['event']]
    if event['event'].startswith('TF_'):
        return 'TF problem'
    return None


def _read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def _column(rows, name):
    return np.array([float(r[name]) if r.get(name) else np.nan for r in rows])


def _style(ax, title):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(0.6)
    ax.tick_params(colors=AXIS, labelcolor=MUTED, labelsize=7.5, length=2)
    ax.grid(True, color=GRID, linewidth=0.5)
    ax.set_axisbelow(True)
    ax.set_title(title, loc='left', fontsize=8.5, color=INK, pad=3)


def _marker(ax, x, y, label, size=6.5):
    color, marker = STYLE[label]
    ax.plot(x, y, linestyle='none', marker=marker, ms=size, color=color, mec=SURFACE, mew=1.0,
            zorder=3)


def _legend(ax, **kwargs):
    ax.legend(frameon=False, fontsize=7.5, labelcolor=INK2, handlelength=1.6, **kwargs)


def plot(run_dir):
    samples = _read_csv(os.path.join(run_dir, 'samples.csv'))
    if not samples:
        sys.exit(f'{run_dir}/samples.csv has no samples yet')
    events = _read_csv(os.path.join(run_dir, 'events.csv'))
    data = {name: _column(samples, name) for name in samples[0]}
    t0 = data['wall_time'][0]
    t = data['wall_time'] - t0
    rows = {}
    for e in events:
        label = _row(e)
        if label is not None:
            rows.setdefault(label, []).append(e)

    fig = Figure(figsize=(16, 10), dpi=150, facecolor=SURFACE)
    outer = fig.add_gridspec(1, 2, width_ratios=[1, 1.55], wspace=0.13,
                             left=0.05, right=0.985, top=0.9, bottom=0.06)
    left = outer[0].subgridspec(3, 1, height_ratios=[3, 0.3, 1.05], hspace=0.16)
    right = outer[1].subgridspec(8, 1, height_ratios=[1.6, 0.8, 0.8, 0.8, 0.8, 1.0, 0.7, 0.7],
                                 hspace=0.62)
    fig.text(0.05, 0.965, f'Pose jumps: {os.path.basename(os.path.normpath(run_dir))}',
             fontsize=13, color=INK, weight='semibold')
    fig.text(0.05, 0.94, 'One color per layer that can move the robot in RViz: blue = AMCL '
             '(map → odom), orange = odometry/EKF (odom → base), teal = Gazebo physics. '
             'Grays are the drawn pose and system health.', fontsize=8.5, color=INK2)

    # Where: the drawn path, ground truth, and where each jump happened.
    ax = fig.add_subplot(left[0])
    _style(ax, 'Path in the map frame')
    ax.plot(data['gt_x'], data['gt_y'], color=MUTED, lw=2.4, solid_capstyle='round',
            label='Gazebo ground truth (world frame)')
    ax.plot(data['map_x'], data['map_y'], color=INK2, lw=1.0, solid_capstyle='round',
            label='Drawn in RViz (map → base)')
    for label, color, marker in ROWS:
        located = [e for e in rows.get(label, []) if e['x']]
        if located:
            ax.plot([float(e['x']) for e in located], [float(e['y']) for e in located],
                    linestyle='none', marker=marker, ms=7, color=color, mec=SURFACE, mew=1.0,
                    zorder=3, label=f'{label} ({len(located)})')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x (m)', fontsize=8, color=INK2)
    ax.set_ylabel('y (m)', fontsize=8, color=INK2)
    key = fig.add_subplot(left[1])  # the legend gets its own strip so it never covers the path
    key.axis('off')
    _legend(key, handles=ax.get_legend_handles_labels()[0], loc='upper left', ncol=3,
            borderaxespad=0)

    # When and why: one row per cause.
    strip = fig.add_subplot(right[0])
    _style(strip, 'Events by cause')
    for i, (label, _, _) in enumerate(ROWS):
        times = [float(e['wall_time']) - t0 for e in rows.get(label, [])]
        _marker(strip, times, [i] * len(times), label)
    strip.set_yticks(range(len(ROWS)), [label for label, _, _ in ROWS])
    strip.set_ylim(len(ROWS) - 0.5, -0.5)
    strip.grid(False, axis='x')
    end = max([t[-1]] + [float(e['wall_time']) - t0 for e in events])
    strip.set_xlim(0, end + 0.5)

    # How big: per-sample movement of each layer on one shared scale.
    layers = [
        ('map_step', INK2, 'Drawn robot (map → base), what RViz shows', None),
        ('corr_shift', AMCL, 'AMCL correction (map → odom); RViz draws it transform_tolerance '
         'later', 'AMCL_JUMP'),
        ('odom_step', ODOM, 'Odometry (odom → base, EKF)', 'ODOM_JUMP'),
        ('gt_step', PHYS, 'Gazebo ground truth', 'GT_JUMP'),
    ]
    axes, first = [strip], None
    for k, (name, color, title, kind) in enumerate(layers):
        ax = fig.add_subplot(right[1 + k], sharex=strip, sharey=first)
        first = first or ax
        axes.append(ax)
        _style(ax, f'{title}: movement per sample (m)')
        if np.all(np.isnan(data[name])):
            ax.text(0.5, 0.5, 'no data', transform=ax.transAxes, ha='center', va='center',
                    color=MUTED, fontsize=8)
        else:
            ax.plot(t, data[name], color=color, lw=1.0)
        for e in events:
            if e['event'] == kind and e['dist_m']:
                _marker(ax, float(e['wall_time']) - t0, float(e['dist_m']), _row(e))

    ax = fig.add_subplot(right[5], sharex=strip)
    axes.append(ax)
    _style(ax, 'AMCL uncertainty and drawn-pose error (m)')
    ax.plot(t, data['amcl_sigma_xy'], color=AMCL, lw=1.0, label='AMCL sigma xy')
    ax.plot(t, data['loc_err'], color=INK2, lw=1.0,
            label='Drawn vs ground truth (valid if the map matches the world)')
    _legend(ax, loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=2, borderaxespad=0.2)

    ax = fig.add_subplot(right[6], sharex=strip)
    axes.append(ax)
    _style(ax, 'Real-time factor (1 = sim runs at wall-clock speed)')
    ax.axhline(1.0, color=AXIS, lw=0.8)
    ax.plot(t, data['rtf'], color=INK2, lw=1.0)
    ax.set_ylim(bottom=0)

    ax = fig.add_subplot(right[7], sharex=strip)
    axes.append(ax)
    _style(ax, 'TF age: sim time minus odom → base stamp (s); a ramp is a stall')
    ax.plot(t, data['tf_age'], color=INK2, lw=1.0)
    ax.set_xlabel('Time since the logger started (s, wall clock)', fontsize=8, color=INK2)
    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)

    # The logger's own summary, verdict first.
    ax = fig.add_subplot(left[2])
    ax.axis('off')
    path = os.path.join(run_dir, 'summary.txt')
    lines = []
    if os.path.isfile(path):
        with open(path) as f:
            # A no-break space keeps each number on the same line as its unit.
            lines = [re.sub(r'(\d) (m|deg|s)\b', '\\1\u00a0\\2', line)
                     for line in f.read().splitlines()
                     if line and not line.startswith(('===', 'Logs:'))]
    verdict = [line for line in lines if line.startswith('Most likely source')]
    rest = [line for line in lines if line not in verdict] or [
        'No summary.txt yet: stop the logger with Ctrl+C to write one.']
    head = '\n'.join(textwrap.wrap(verdict[0], 78)) if verdict else ''
    body = '\n'.join(wrapped for line in rest
                     for wrapped in textwrap.wrap(line, 100, subsequent_indent='    '))
    ax.text(0, 1, head, va='top', fontsize=8.5, color=INK, weight='semibold',
            transform=ax.transAxes)
    ax.annotate(body, (0, 1), xycoords='axes fraction', va='top', fontsize=7.5, color=INK2,
                linespacing=1.45, xytext=(0, -16 * head.count('\n') - (22 if head else 0)),
                textcoords='offset points')

    out = os.path.join(run_dir, 'pose_jumps.png')
    fig.savefig(out, facecolor=SURFACE)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description='Plot a pose_logger run.')
    parser.add_argument('run_dir', nargs='?',
                        help='run directory (default: the newest one under ./pose_logs)')
    run_dir = parser.parse_args(argv).run_dir
    if run_dir is None:
        runs = sorted(d for d in glob.glob('pose_logs/*/')
                      if os.path.isfile(os.path.join(d, 'samples.csv')))
        if not runs:
            sys.exit('No runs under ./pose_logs; pass the run directory.')
        run_dir = runs[-1]
    print(plot(run_dir))


if __name__ == '__main__':
    main()

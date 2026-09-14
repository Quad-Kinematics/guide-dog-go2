import csv
import glob
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

WORKSPACE_DIR = os.path.expanduser('~/dog_v1_ws')
CSV_DIR = os.path.join(WORKSPACE_DIR, 'sim_results', 'csv')
PLOT_DIR = os.path.join(WORKSPACE_DIR, 'sim_results', 'plots')

STATE_COLORS = {
    'IDLE': '#9e9e9e',     # Grey
    'PATROL': '#2196f3',   # Blue
    'SCAN': '#ff9800',     # Orange
    'ALIGN': '#9c27b0',    # Purple
    'GREET': '#e91e63',    # Pink
    'GUIDE': '#4caf50',    # Green
    'ARRIVE': '#ffeb3b',   # Yellow
    'UNKNOWN': '#000000'   # Black
}

PREDEFINED_WAYPOINTS = [
    (-8.0, -5.0),
    (5.0, 2.0),
    (-2.0, 4.0)
]


def plot_trajectory(csv_file):
    person_x, person_y, person_names = [], [], []
    speech_events = []
    start_time = None

    paths_by_state = []
    current_state = 'IDLE'
    current_x, current_y = [], []
    raw_state_sequence = []
    fsm_transitions = {}

    # 1. Parse CSV Data
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)

        for row in reader:
            if not row:
                continue

            t_str, x_str, y_str, event_type, event_label = row
            t, x, y = float(t_str), float(x_str), float(y_str)

            if start_time is None:
                start_time = t
            rel_time = t - start_time

            if event_type == 'topology':
                parts = event_label.split('|')
                if len(parts) == 3:
                    source, target, outcome = parts
                    if (source, target) in fsm_transitions:
                        fsm_transitions[(source, target)] += f" / {outcome}"
                    else:
                        fsm_transitions[(source, target)] = outcome

            elif event_type == 'path':
                current_x.append(x)
                current_y.append(y)

            elif event_type == 'state':
                if current_x:
                    paths_by_state.append(
                        {'state': current_state, 'x': current_x, 'y': current_y})
                    current_x = [current_x[-1]]
                    current_y = [current_y[-1]]
                current_state = event_label
                if not raw_state_sequence or raw_state_sequence[-1] != event_label:
                    raw_state_sequence.append(event_label)

            elif event_type == 'person':
                person_x.append(x)
                person_y.append(y)
                person_names.append(event_label)

            elif event_type == 'speech':
                speech_events.append((rel_time, x, y, event_label))

    if current_x:
        paths_by_state.append(
            {'state': current_state, 'x': current_x, 'y': current_y})
    if not raw_state_sequence and paths_by_state:
        raw_state_sequence.append(paths_by_state[0]['state'])

    # Setup specific run directory
    base_name = os.path.splitext(os.path.basename(csv_file))[0]
    run_dir = os.path.join(PLOT_DIR, base_name)
    os.makedirs(run_dir, exist_ok=True)

    # ==========================================
    # 2. GENERATE & SAVE MAP TRAJECTORY PNG
    # ==========================================
    fig_map, ax_map = plt.subplots(figsize=(12, 8))

    seen_states = set()

    for segment in paths_by_state:
        state_name = segment['state']
        seen_states.add(state_name)
        color = STATE_COLORS.get(state_name, STATE_COLORS['UNKNOWN'])
        ax_map.plot(segment['x'], segment['y'],
                    color=color, linewidth=2.5, alpha=0.85)

    if paths_by_state and paths_by_state[0]['x']:
        ax_map.scatter(paths_by_state[0]['x'][0], paths_by_state[0]
                       ['y'][0], color='green', marker='o', s=100, zorder=4)
    if paths_by_state and paths_by_state[-1]['x']:
        ax_map.scatter(paths_by_state[-1]['x'][-1], paths_by_state[-1]
                       ['y'][-1], color='red', marker='X', s=100, zorder=4)

    wp_x = [wp[0] for wp in PREDEFINED_WAYPOINTS]
    wp_y = [wp[1] for wp in PREDEFINED_WAYPOINTS]
    ax_map.scatter(wp_x, wp_y, color='cyan', marker='o',
                   s=120, edgecolors='black', zorder=3)

    for i, (x, y) in enumerate(PREDEFINED_WAYPOINTS, start=1):
        ax_map.annotate(f"WP {i}", (x, y), xytext=(8, 8), textcoords='offset points',
                        fontsize=8, fontweight='bold', color='black',
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8, ec="gray"))

    for px, py, name in zip(person_x, person_y, person_names):
        ax_map.scatter(px, py, color='orange', marker='*', s=160, zorder=5)
        ax_map.annotate(f"Person: {name}", (px, py), xytext=(10, 10), textcoords='offset points',
                        bbox=dict(boxstyle="round,pad=0.3", fc="#ffeb3b", alpha=0.9), arrowprops=dict(arrowstyle="->"))

    for idx, (rel_t, sx, sy, speech_text) in enumerate(speech_events, start=1):
        ax_map.scatter(sx, sy, color='purple', marker='D', s=70, zorder=5)
        ax_map.annotate(f"TTS #{idx}", (sx, sy), xytext=(-20, 10), textcoords='offset points',
                        bbox=dict(boxstyle="round,pad=0.25", fc="#e1bee7", ec="#8e24aa", alpha=0.9), arrowprops=dict(arrowstyle="->", color="#8e24aa"))

    legend_elements = [
        mlines.Line2D([], [], color='green', marker='o',
                      linestyle='None', markersize=8, label='Start'),
        mlines.Line2D([], [], color='red', marker='X',
                      linestyle='None', markersize=8, label='End'),
        mlines.Line2D([], [], color='cyan', marker='o', markeredgecolor='black',
                                    linestyle='None', markersize=8, label='Predefined Waypoint')
    ]
    for state in sorted(seen_states):
        color = STATE_COLORS.get(state, STATE_COLORS['UNKNOWN'])
        legend_elements.append(mlines.Line2D(
            [], [], color=color, lw=2.5, label=f'State: {state}'))
    if person_names:
        legend_elements.append(mlines.Line2D(
            [], [], color='orange', marker='*', linestyle='None', markersize=12, label='Person Detected'))
    if speech_events:
        legend_elements.append(mlines.Line2D(
            [], [], color='purple', marker='D', linestyle='None', markersize=8, label='Speech Triggered'))

    ax_map.set_title(
        f"Simulation Trajectory ({base_name})", fontsize=14, fontweight='bold', pad=15)
    ax_map.set_xlabel("X Position (meters)")
    ax_map.set_ylabel("Y Position (meters)")

    ax_map.grid(True, linestyle='--', alpha=0.6, color='gray')
    ax_map.set_facecolor('#fcfcfc')

    ax_map.legend(handles=legend_elements, loc='center left',
                  bbox_to_anchor=(1.02, 0.5), borderaxespad=0.)

    ax_map.set_aspect('equal', adjustable='datalim')

    plt.tight_layout()
    map_out = os.path.join(run_dir, "trajectory_map.png")
    fig_map.savefig(map_out, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(fig_map)

    # ==========================================
    # 3. GENERATE & SAVE FSM MERMAID MD
    # ==========================================
    md_out = os.path.join(run_dir, "state_transitions.md")
    with open(md_out, 'w') as f:
        f.write(f"# FSM Analysis for {base_name}\n\n")
        f.write("## Timeline of State Transitions\n\n")
        f.write("```mermaid\n")
        f.write("flowchart LR\n")

        if raw_state_sequence:
            for state, hex_color in STATE_COLORS.items():
                f.write(
                    f"    classDef {state} fill:{hex_color},stroke:#333,stroke-width:2px,color:#fff\n")

            f.write("\n    %% Nodes\n")
            for i, state in enumerate(raw_state_sequence):
                class_name = state if state in STATE_COLORS else 'UNKNOWN'
                f.write(f"    N{i}[{state}]:::{class_name}\n")

            f.write("\n    %% Sequence\n")
            for i in range(len(raw_state_sequence) - 1):
                f.write(f"    N{i} --> N{i+1}\n")
        else:
            f.write("    N0[No state transitions logged]\n")

        f.write("```\n")

    # ==========================================
    # 4. GENERATE & SAVE AUDIO TEXT LOG
    # ==========================================
    txt_out = os.path.join(run_dir, "audio_log.txt")
    with open(txt_out, 'w') as f:
        f.write("AUDIO DIALOGUE & SPEECH LOG:\n")
        f.write("=" * 60 + "\n")
        if speech_events:
            for idx, (rel_t, sx, sy, text) in enumerate(speech_events, start=1):
                f.write(f"[+{rel_t:05.1f}s] TTS #{idx}: \"{text}\"\n")
                f.write(
                    f"           Triggered at Map Location -> X: {sx:.2f}, Y: {sy:.2f}\n")
                f.write("-" * 60 + "\n")
        else:
            f.write("No audio speech events recorded during this run.\n")

    print(f"   ✅ Saved map: {os.path.basename(map_out)}")
    print(f"   ✅ Saved FSM: {os.path.basename(md_out)}")
    print(f"   ✅ Saved log: {os.path.basename(txt_out)}")


if __name__ == '__main__':
    os.makedirs(CSV_DIR, exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)

    # If a specific file is passed via command line, plot just that one
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
        if os.path.exists(target_file):
            print(f"🔍 Processing single file: {os.path.basename(target_file)}")
            plot_trajectory(target_file)
        else:
            print(f"❌ Error: File not found -> {target_file}")
            sys.exit(1)

    # Otherwise, loop through ALL CSVs in the directory
    else:
        csv_files = glob.glob(os.path.join(CSV_DIR, '*.csv'))
        if not csv_files:
            print(f"No log files found in {CSV_DIR}")
            sys.exit(1)

        # Sort files by name or modification time so they process in order
        csv_files.sort()

        print(f"🔍 Found {len(csv_files)} CSV files. Processing all...\n")

        for idx, csv_file in enumerate(csv_files, start=1):
            print(f"[{idx}/{len(csv_files)}] Plotting: {os.path.basename(csv_file)}")
            try:
                plot_trajectory(csv_file)
            except Exception as e:
                print(
                    f"   ❌ Error processing {os.path.basename(csv_file)}: {e}")

        print("\n🎉 All CSV files processed successfully!")

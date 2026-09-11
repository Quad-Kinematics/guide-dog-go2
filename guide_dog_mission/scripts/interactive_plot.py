import csv
import os
import sys
import yaml
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

WORKSPACE_DIR = os.path.expanduser('~/dog_v1_ws')
CSV_DIR = os.path.join(WORKSPACE_DIR, 'sim_results', 'csv')

# --- MAP CONFIGURATION ---
MAP_YAML_FILE = os.path.join(
    WORKSPACE_DIR, 'src/unitree_go2_slam/maps/arena_map.yaml')
MAP_PGM_FILE = os.path.join(
    WORKSPACE_DIR, 'src/unitree_go2_slam/maps/arena_map.pgm')

STATE_COLORS = {
    'IDLE': '#9e9e9e',
    'PATROL': '#2196f3',
    'SCAN': '#ff9800',
    'ALIGN': '#9c27b0',
    'GREET': '#e91e63',
    'GUIDE': '#4caf50',
    'ARRIVE': '#ffeb3b',
    'UNKNOWN': '#000000'
}

PREDEFINED_WAYPOINTS = [
    (-8.0, -5.0),
    (5.0, 2.0),
    (-2.0, 4.0)
]


def plot_interactive_dashboard(csv_file):
    person_x, person_y, person_names = [], [], []
    speech_events = []
    start_time = None

    paths_by_state = []
    current_state = 'IDLE'
    current_x, current_y = [], []
    raw_state_sequence = []

    # ==========================================
    # 1. PARSE CSV DATA
    # ==========================================
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

            if event_type == 'path':
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

    base_name = os.path.basename(csv_file)

    # ==========================================
    # 2. SETUP MATPLOTLIB DASHBOARD (1 Window, 3 Areas)
    # ==========================================
    fig = plt.figure(figsize=(16, 9))
    fig.canvas.manager.set_window_title(f"Simulation Dashboard - {base_name}")

    # Grid Layout: Map on left (spans both rows), FSM top-right, Audio bottom-right
    gs = fig.add_gridspec(2, 2, width_ratios=[2.5, 1], height_ratios=[1, 1.5])

    ax_map = fig.add_subplot(gs[:, 0])
    ax_fsm = fig.add_subplot(gs[0, 1])
    ax_audio = fig.add_subplot(gs[1, 1])

    # ==========================================
    # 3. PLOT TRAJECTORY MAP (Interactive)
    # ==========================================

    # --- LOAD AND PLOT THE BACKGROUND MAP ---
    if os.path.exists(MAP_YAML_FILE) and os.path.exists(MAP_PGM_FILE):
        with open(MAP_YAML_FILE, 'r') as f:
            map_data = yaml.safe_load(f)

        resolution = map_data['resolution']
        origin = map_data['origin']  # [x, y, yaw]

        map_img = plt.imread(MAP_PGM_FILE)
        height, width = map_img.shape

        # Calculate physical boundaries in meters
        extent = [
            origin[0],
            origin[0] + width * resolution,
            origin[1],
            origin[1] + height * resolution
        ]

        ax_map.imshow(map_img, cmap='gray', origin='lower', extent=extent)
        print("✅ Background map loaded successfully.")
    else:
        print("⚠️ Map files not found. Plotting trajectory on an empty background.")

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

    # --- ADD THIS BLOCK: Plot Predefined Waypoints ---
    wp_x = [wp[0] for wp in PREDEFINED_WAYPOINTS]
    wp_y = [wp[1] for wp in PREDEFINED_WAYPOINTS]
    ax_map.scatter(wp_x, wp_y, color='cyan', marker='o',
                   s=120, edgecolors='black', zorder=3)

    for i, (x, y) in enumerate(PREDEFINED_WAYPOINTS, start=1):
        ax_map.annotate(f"WP {i}", (x, y), xytext=(8, 8), textcoords='offset points',
                        fontsize=8, fontweight='bold', color='black',
                        bbox=dict(boxstyle="round,pad=0.1", fc="white", alpha=0.7, ec="none"))

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

    ax_map.set_title(f"Trajectory Map", fontsize=14, fontweight='bold')
    ax_map.set_xlabel("X Position (meters)")
    ax_map.set_ylabel("Y Position (meters)")
    ax_map.grid(False)  # Turned off so gridlines don't clutter the image
    ax_map.legend(handles=legend_elements, loc='upper center',
                  bbox_to_anchor=(0.5, -0.1), ncol=4)

    if 'extent' in locals():
        ax_map.set_xlim(extent[0], extent[1])
        ax_map.set_ylim(extent[2], extent[3])
    ax_map.set_aspect('equal', adjustable='box')

    # ==========================================
    # 4. PLOT FSM FLOWCHART
    # ==========================================
    ax_fsm.axis('off')
    ax_fsm.set_title("State Transitions", fontsize=12,
                     fontweight='bold', loc='center')

    if raw_state_sequence:
        nodes_per_row = 4
        num_rows = max(1, (len(raw_state_sequence) - 1) // nodes_per_row + 1)
        ax_fsm.set_xlim(0, nodes_per_row * 2.5)
        ax_fsm.set_ylim(-1.5 * num_rows, 0.5)

        for i, state in enumerate(raw_state_sequence):
            row = i // nodes_per_row
            col = i % nodes_per_row
            x, y = col * 2.5 + 1.25, -row * 1.5
            color = STATE_COLORS.get(state, STATE_COLORS['UNKNOWN'])

            ax_fsm.text(x, y, state, ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.5',
                                  facecolor=color, edgecolor='black', alpha=0.9),
                        fontsize=9, fontweight='bold', zorder=3)

            if i > 0:
                prev_row, prev_col = (
                    i - 1) // nodes_per_row, (i - 1) % nodes_per_row
                prev_x, prev_y = prev_col * 2.5 + 1.25, -prev_row * 1.5
                if row == prev_row:
                    ax_fsm.annotate('', xy=(x - 0.6, y), xytext=(prev_x + 0.6, y),
                                    arrowprops=dict(arrowstyle="->", lw=2, color='gray'), zorder=2)
                else:
                    ax_fsm.annotate('', xy=(x - 0.5, y + 0.3), xytext=(prev_x + 0.5, prev_y - 0.3),
                                    arrowprops=dict(arrowstyle="->", lw=1.5, color='gray', connectionstyle="arc3,rad=-0.1"), zorder=2)
    else:
        ax_fsm.text(0.5, 0.5, "No states logged.", ha='center', va='center')

    # ==========================================
    # 5. RENDER AUDIO LOG TEXT
    # ==========================================
    ax_audio.axis('off')
    ax_audio.set_title("Audio Dialogue Log", fontsize=12,
                       fontweight='bold', loc='center')

    log_text = ""
    if speech_events:
        for idx, (rel_t, sx, sy, text) in enumerate(speech_events, start=1):
            log_text += f"[{rel_t:05.1f}s] TTS #{idx}:\n\"{text}\"\n(Loc: X:{sx:.1f}, Y:{sy:.1f})\n\n"
    else:
        log_text = "No audio speech events recorded."

    ax_audio.text(0.05, 0.95, log_text, transform=ax_audio.transAxes,
                  fontsize=9, family='monospace', va='top', ha='left', wrap=True)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("❌ Error: No CSV filename provided.")
        print("   Usage: python3 view_dashboard.py <csv_filename>")
        print("   Example: python3 view_dashboard.py run_01.csv")
        sys.exit(1)

    csv_filename = sys.argv[1]

    if not csv_filename.endswith('.csv'):
        csv_filename += '.csv'

    target_file = os.path.join(CSV_DIR, csv_filename)

    if not os.path.exists(target_file):
        print(f"❌ Error: File not found -> {target_file}")
        print(f"   Make sure the file exists in {CSV_DIR}")
        sys.exit(1)

    print(f"🔍 Opening interactive dashboard for: {csv_filename}...")
    plot_interactive_dashboard(target_file)

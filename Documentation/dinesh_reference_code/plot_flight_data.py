#!/usr/bin/env python3
"""
plot_flight_data.py - Generate demo plots from a flight CSV.

Reads ~/precision_landing/csv/flight_*.csv (produced by drone_test_landing.py)
and saves PNG plots into a plots/ subdirectory.

USAGE:
  python3 plot_flight_data.py                  # plot the most recent CSV
  python3 plot_flight_data.py <csv_path>       # plot a specific CSV
  python3 plot_flight_data.py --list           # list available CSV files

Generated plots:
  1. trajectory.png      - Top-down view of marker path in X-Y plane
  2. position_time.png   - body_x and body_y over time
  3. velocity_time.png   - velocity magnitude + components over time
  4. pred_vs_meas.png    - measured vs predicted position (lag compensation)
  5. altitude_time.png   - altitude over time (constant-altitude proof)
  6. overview.png        - all 5 plots combined into a single dashboard

DEPENDENCIES:
  pip install matplotlib pandas

  If matplotlib won't install via pip on Pi, try:
    sudo apt install python3-matplotlib python3-pandas
"""

import sys
import os
import glob
from datetime import datetime

try:
    import matplotlib
    matplotlib.use('Agg')   # non-interactive - works without display
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: matplotlib not installed. Run: pip install matplotlib")
    sys.exit(1)

try:
    import pandas as pd
except ImportError:
    print("ERROR: pandas not installed. Run: pip install pandas")
    sys.exit(1)


CSV_DIR = os.path.expanduser('~/precision_landing/csv')


def list_csv_files():
    """Print all CSVs sorted newest first."""
    files = sorted(glob.glob(os.path.join(CSV_DIR, 'flight_*.csv')),
                   key=os.path.getmtime, reverse=True)
    if not files:
        print(f"No CSV files in {CSV_DIR}")
        return
    print(f"Found {len(files)} CSV files in {CSV_DIR}:")
    for f in files:
        size_kb = os.path.getsize(f) / 1024.0
        mtime = datetime.fromtimestamp(os.path.getmtime(f))
        print(f"  {os.path.basename(f)}  ({size_kb:.1f} KB, {mtime:%Y-%m-%d %H:%M:%S})")


def find_latest_csv():
    files = sorted(glob.glob(os.path.join(CSV_DIR, 'flight_*.csv')),
                   key=os.path.getmtime, reverse=True)
    if not files:
        return None
    return files[0]


def load_and_clean(path):
    """Load CSV and return only the detected rows (most plots want these)."""
    df = pd.read_csv(path)
    print(f"Loaded {len(df)} rows ({df['detected'].sum()} detected, "
          f"{(~df['detected'].astype(bool)).sum()} not detected)")
    return df


def plot_trajectory(df, out_path):
    """Top-down marker path in X-Y body frame."""
    det = df[df['detected'] == 1]
    if len(det) < 2:
        print("  skip trajectory: not enough detected frames")
        return
    
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Path line with color = time progression
    sc = ax.scatter(det['body_y'], det['body_x'], c=det['t'],
                    cmap='viridis', s=8, alpha=0.7)
    cbar = plt.colorbar(sc, ax=ax, label='Time (s)')
    
    # Start and end markers
    ax.scatter(det['body_y'].iloc[0], det['body_x'].iloc[0],
               s=200, c='green', marker='o', edgecolors='black', linewidths=2,
               label='START', zorder=5)
    ax.scatter(det['body_y'].iloc[-1], det['body_x'].iloc[-1],
               s=200, c='red', marker='X', edgecolors='black', linewidths=2,
               label='END', zorder=5)
    
    # Drone position at origin
    ax.scatter(0, 0, s=300, c='black', marker='*',
               label='DRONE', zorder=5)
    
    ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax.axvline(0, color='gray', linewidth=0.5, linestyle='--')
    
    ax.set_xlabel('body_y (m)  -- LEFT  /  RIGHT --->', fontsize=12)
    ax.set_ylabel('body_x (m)  -- BACK  /  FWD --->', fontsize=12)
    ax.set_title('Marker Trajectory (top-down, body-frame)', fontsize=14)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  saved {out_path}")


def plot_position_time(df, out_path):
    """body_x and body_y over time."""
    det = df[df['detected'] == 1]
    if len(det) < 2:
        print("  skip position_time: not enough detected frames")
        return
    
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(det['t'], det['body_x'], label='body_x (FWD/BACK)',
            color='tab:blue', linewidth=1.5)
    ax.plot(det['t'], det['body_y'], label='body_y (RIGHT/LEFT)',
            color='tab:orange', linewidth=1.5)
    ax.axhline(0, color='gray', linewidth=0.5)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Position relative to drone (m)', fontsize=12)
    ax.set_title('Marker Position vs Time', fontsize=14)
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  saved {out_path}")


def plot_velocity_time(df, out_path):
    """Velocity magnitude and components over time."""
    det = df[df['detected'] == 1]
    if len(det) < 2:
        print("  skip velocity_time: not enough detected frames")
        return
    
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(det['t'], det['speed'], label='|velocity|',
            color='tab:red', linewidth=2)
    ax.plot(det['t'], det['vel_x'], label='vel_x',
            color='tab:blue', linewidth=1, alpha=0.7)
    ax.plot(det['t'], det['vel_y'], label='vel_y',
            color='tab:orange', linewidth=1, alpha=0.7)
    ax.axhline(0, color='gray', linewidth=0.5)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Velocity (m/s)', fontsize=12)
    ax.set_title('Marker Velocity vs Time (estimated by weighted LSQ)', fontsize=14)
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  saved {out_path}")


def plot_pred_vs_meas(df, out_path):
    """Measured position vs predicted position (with lag compensation)."""
    det = df[df['detected'] == 1]
    if len(det) < 2:
        print("  skip pred_vs_meas: not enough detected frames")
        return
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # X axis
    ax1.plot(det['t'], det['body_x'], label='measured body_x',
             color='tab:blue', linewidth=1.5)
    ax1.plot(det['t'], det['pred_x'], label='predicted body_x (lag-compensated)',
             color='tab:cyan', linewidth=1.5, linestyle='--')
    ax1.set_ylabel('body_x (m)', fontsize=11)
    ax1.set_title('Measured vs Predicted Position - X axis', fontsize=13)
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(0, color='gray', linewidth=0.5)
    
    # Y axis
    ax2.plot(det['t'], det['body_y'], label='measured body_y',
             color='tab:orange', linewidth=1.5)
    ax2.plot(det['t'], det['pred_y'], label='predicted body_y (lag-compensated)',
             color='gold', linewidth=1.5, linestyle='--')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('body_y (m)', fontsize=11)
    ax2.set_title('Measured vs Predicted Position - Y axis', fontsize=13)
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(0, color='gray', linewidth=0.5)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  saved {out_path}")


def plot_altitude_time(df, out_path):
    """Altitude over time."""
    det = df[df['detected'] == 1]
    if len(det) < 2:
        print("  skip altitude_time: not enough detected frames")
        return
    
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(det['t'], det['alt'], color='tab:green', linewidth=1.5)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Altitude above marker (m)', fontsize=12)
    ax.set_title('Drone Altitude vs Time (from AprilTag pose)', fontsize=14)
    ax.grid(True, alpha=0.3)
    
    # mean +/- std band
    mean_alt = det['alt'].mean()
    std_alt = det['alt'].std()
    ax.axhline(mean_alt, color='tab:red', linestyle='--',
               linewidth=1, alpha=0.5,
               label=f'mean = {mean_alt:.2f} m')
    ax.fill_between(det['t'], mean_alt - std_alt, mean_alt + std_alt,
                    alpha=0.15, color='tab:red',
                    label=f'+/- 1 std = {std_alt*100:.1f} cm')
    ax.legend(loc='best')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  saved {out_path}")


def plot_overview(df, out_path):
    """All 5 plots in one dashboard for slides."""
    det = df[df['detected'] == 1]
    if len(det) < 2:
        print("  skip overview: not enough detected frames")
        return
    
    fig = plt.figure(figsize=(16, 10))
    
    # Trajectory (top-left, square)
    ax_traj = plt.subplot(2, 3, 1)
    sc = ax_traj.scatter(det['body_y'], det['body_x'], c=det['t'],
                         cmap='viridis', s=5, alpha=0.7)
    ax_traj.scatter(0, 0, s=200, c='black', marker='*', label='DRONE', zorder=5)
    ax_traj.scatter(det['body_y'].iloc[0], det['body_x'].iloc[0],
                    s=100, c='green', marker='o', edgecolors='black', label='START')
    ax_traj.scatter(det['body_y'].iloc[-1], det['body_x'].iloc[-1],
                    s=100, c='red', marker='X', edgecolors='black', label='END')
    ax_traj.set_xlabel('body_y (m)')
    ax_traj.set_ylabel('body_x (m)')
    ax_traj.set_title('Marker Trajectory')
    ax_traj.set_aspect('equal')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.legend(fontsize=8)
    
    # Position vs time (top-mid + top-right)
    ax_pos = plt.subplot(2, 3, (2, 3))
    ax_pos.plot(det['t'], det['body_x'], label='body_x (FWD/BACK)',
                color='tab:blue')
    ax_pos.plot(det['t'], det['body_y'], label='body_y (RIGHT/LEFT)',
                color='tab:orange')
    ax_pos.axhline(0, color='gray', linewidth=0.5)
    ax_pos.set_xlabel('Time (s)')
    ax_pos.set_ylabel('Position (m)')
    ax_pos.set_title('Position vs Time')
    ax_pos.legend()
    ax_pos.grid(True, alpha=0.3)
    
    # Velocity vs time (bottom-left)
    ax_vel = plt.subplot(2, 3, 4)
    ax_vel.plot(det['t'], det['speed'], color='tab:red', linewidth=2,
                label='|v|')
    ax_vel.plot(det['t'], det['vel_x'], color='tab:blue', alpha=0.6, label='vx')
    ax_vel.plot(det['t'], det['vel_y'], color='tab:orange', alpha=0.6, label='vy')
    ax_vel.set_xlabel('Time (s)')
    ax_vel.set_ylabel('Velocity (m/s)')
    ax_vel.set_title('Marker Velocity')
    ax_vel.legend()
    ax_vel.grid(True, alpha=0.3)
    
    # Pred vs meas (bottom-mid)
    ax_pred = plt.subplot(2, 3, 5)
    ax_pred.plot(det['t'], det['body_x'], 'tab:blue', label='meas x')
    ax_pred.plot(det['t'], det['pred_x'], 'tab:cyan', linestyle='--', label='pred x')
    ax_pred.plot(det['t'], det['body_y'], 'tab:orange', label='meas y')
    ax_pred.plot(det['t'], det['pred_y'], 'gold', linestyle='--', label='pred y')
    ax_pred.set_xlabel('Time (s)')
    ax_pred.set_ylabel('Position (m)')
    ax_pred.set_title('Measured vs Predicted')
    ax_pred.legend(fontsize=8)
    ax_pred.grid(True, alpha=0.3)
    
    # Altitude (bottom-right)
    ax_alt = plt.subplot(2, 3, 6)
    ax_alt.plot(det['t'], det['alt'], color='tab:green', linewidth=1.5)
    mean_alt = det['alt'].mean()
    std_alt = det['alt'].std()
    ax_alt.axhline(mean_alt, color='tab:red', linestyle='--', alpha=0.5,
                   label=f'mean={mean_alt:.2f}m')
    ax_alt.fill_between(det['t'], mean_alt - std_alt, mean_alt + std_alt,
                        alpha=0.15, color='tab:red',
                        label=f'+/-sigma={std_alt*100:.1f}cm')
    ax_alt.set_xlabel('Time (s)')
    ax_alt.set_ylabel('Altitude (m)')
    ax_alt.set_title('Altitude vs Time')
    ax_alt.legend(fontsize=8)
    ax_alt.grid(True, alpha=0.3)
    
    plt.suptitle(f'Flight Overview - {os.path.basename(os.path.dirname(out_path))}',
                 fontsize=14, y=1.00)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  saved {out_path}")


def print_summary(df):
    det = df[df['detected'] == 1]
    if len(det) < 1:
        print("\nNo detected frames in CSV.")
        return
    
    print("\n=== FLIGHT SUMMARY ===")
    duration = df['t'].iloc[-1] - df['t'].iloc[0]
    print(f"Duration:           {duration:.1f} s")
    print(f"Frames total:       {len(df)}")
    print(f"Frames detected:    {len(det)} ({100*len(det)/len(df):.1f}%)")
    print(f"Frames jump-reject: {df['jump_rejected'].sum()}")
    print()
    print(f"Altitude:           mean={det['alt'].mean():.3f}m  "
          f"std={det['alt'].std()*100:.1f}cm  "
          f"range={det['alt'].min():.2f}-{det['alt'].max():.2f}m")
    print(f"X position:         mean={det['body_x'].mean():.3f}m  "
          f"range={det['body_x'].min():+.3f} to {det['body_x'].max():+.3f}m")
    print(f"Y position:         mean={det['body_y'].mean():.3f}m  "
          f"range={det['body_y'].min():+.3f} to {det['body_y'].max():+.3f}m")
    print(f"Peak speed:         {det['speed'].max():.3f} m/s")
    print(f"Mean speed:         {det['speed'].mean():.3f} m/s")
    print(f"Tags visible:       mean={det['n_tags'].mean():.1f}  "
          f"max={det['n_tags'].max()}")
    print()


def main():
    args = sys.argv[1:]
    
    if args and args[0] in ('-h', '--help'):
        print(__doc__)
        return
    
    if args and args[0] == '--list':
        list_csv_files()
        return
    
    if args:
        csv_path = args[0]
        if not os.path.isfile(csv_path):
            print(f"File not found: {csv_path}")
            sys.exit(1)
    else:
        csv_path = find_latest_csv()
        if csv_path is None:
            print(f"No CSV files found in {CSV_DIR}.")
            print("Run drone_test_landing.py first to generate flight data.")
            sys.exit(1)
        print(f"Using latest CSV: {csv_path}")
    
    # Output dir alongside the CSV
    csv_base = os.path.splitext(os.path.basename(csv_path))[0]
    plot_dir = os.path.join(os.path.dirname(csv_path), 'plots', csv_base)
    os.makedirs(plot_dir, exist_ok=True)
    print(f"Plots will be saved to: {plot_dir}\n")
    
    df = load_and_clean(csv_path)
    print_summary(df)
    
    print("=== Generating plots ===")
    plot_trajectory(df,   os.path.join(plot_dir, 'trajectory.png'))
    plot_position_time(df, os.path.join(plot_dir, 'position_time.png'))
    plot_velocity_time(df, os.path.join(plot_dir, 'velocity_time.png'))
    plot_pred_vs_meas(df,  os.path.join(plot_dir, 'pred_vs_meas.png'))
    plot_altitude_time(df, os.path.join(plot_dir, 'altitude_time.png'))
    plot_overview(df,      os.path.join(plot_dir, 'overview.png'))
    
    print(f"\nDone. View plots in: {plot_dir}")


if __name__ == '__main__':
    main()

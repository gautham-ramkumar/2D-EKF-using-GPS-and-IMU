"""Plot the C++ EKF output against the GPS points.

Usage (from cpp_ekf/):
    ./build/ekf                 # with wheel speed   -> output/trajectory.csv
    ./build/ekf --no-wheel      # optional, for comparison -> output/trajectory_nowheel.csv
    python3 tools/plot_trajectory.py
Saves output/trajectory.png and also opens a window.
"""
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

JUMP = 5.0   # m between consecutive logged rows (50 ms): a jump back onto GPS, don't connect


def read_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}


def plot_path(ax, traj, **style):
    """Plot the path, breaking the line where the filter jumps after a GPS gap."""
    step = np.hypot(np.diff(traj['px']), np.diff(traj['py']))
    cuts = np.flatnonzero(step > JUMP) + 1
    for i, idx in enumerate(np.split(np.arange(len(traj['px'])), cuts)):
        ax.plot(traj['px'][idx], traj['py'][idx], label=style.pop('label', None) if i == 0 else None,
                **style)


traj = read_csv('output/trajectory.csv')
gps = read_csv('output/gps_points.csv')
nowheel_path = 'output/trajectory_nowheel.csv'
nowheel = read_csv(nowheel_path) if os.path.exists(nowheel_path) else None
t0 = gps['t'][0]

fig, ax = plt.subplots(1, 2, figsize=(14, 6))

ax[0].plot(gps['east'], gps['north'], '.', color='0.6', ms=1, label='GPS')
if nowheel is not None:
    plot_path(ax[0], nowheel, color='tab:orange', lw=0.8, label='EKF, IMU + GPS')
plot_path(ax[0], traj, color='tab:blue', lw=0.8, label='EKF, IMU + wheel + GPS')
ax[0].set_xlabel('East (m)')
ax[0].set_ylabel('North (m)')
ax[0].axis('equal')
ax[0].grid(True, alpha=0.3)
ax[0].legend(loc='lower left')

if nowheel is not None:
    ax[1].plot(nowheel['t'] - t0, np.hypot(nowheel['std_px'], nowheel['std_py']),
               color='tab:orange', lw=0.8, label='IMU + GPS')
ax[1].plot(traj['t'] - t0, np.hypot(traj['std_px'], traj['std_py']),
           color='tab:blue', lw=0.8, label='IMU + wheel + GPS')
ax[1].set_yscale('log')
ax[1].set_xlabel('Time since first fix (s)')
ax[1].set_ylabel('Position std (m)')
ax[1].grid(True, alpha=0.3)
ax[1].legend(loc='upper right')

plt.tight_layout()
plt.savefig('output/trajectory.png', dpi=120)
print('Saved output/trajectory.png')
plt.show()

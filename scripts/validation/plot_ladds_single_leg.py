"""Plot the observed Gazebo trajectory and speed without altering test data."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    data = json.loads(Path(args.input).read_text(encoding='utf-8'))
    accepted = next(s['elapsed'] for s in data['status_changes'] if s['state'] == 'ACTIVE')
    trace = [p for p in data['trace'] if p['elapsed'] >= accepted]
    target = (83.38367674866956, 91.682939134985)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.7), layout='constrained')
    path, speed = axes
    path.plot([p['x'] for p in trace], [p['y'] for p in trace], color='#12658c', lw=2,
              label='Observed vessel trajectory')
    path.scatter([trace[0]['x']], [trace[0]['y']], color='#12658c', s=45, label='Start', zorder=4)
    path.scatter(*target, color='#c65232', marker='*', s=140, label='Requested target', zorder=5)
    path.scatter(0, 0, marker='+', color='#5e6870', s=80, label='Geographic origin')
    path.set(xlabel='East x (m)', ylabel='North y (m)', title='Single-leg voyage in the local ENU map')
    path.set_aspect('equal', adjustable='datalim')
    path.legend(loc='upper left', fontsize=8)
    speed.plot([p['elapsed'] - accepted for p in trace], [p['speed'] for p in trace],
               color='#12658c', lw=1.8, label='Observed planar speed')
    speed.axhline(4 * 1852 / 3600, color='#c65232', ls='--', label='Requested: 4 kn = 2.058 m/s')
    speed.axhline(.25, color='#89939b', ls=':', label='Arrival speed threshold: 0.25 m/s')
    speed.set(xlabel='Wall time since task accepted (s)', ylabel='Speed (m/s)', ylim=(0, 2.6),
              title='Requested speed and measured response')
    speed.legend(loc='lower center', fontsize=8)
    for ax in axes:
        ax.grid(alpha=.18)
    fig.suptitle('0211 single-leg DDS / ROS 2 / Gazebo validation', fontsize=14)
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    main()

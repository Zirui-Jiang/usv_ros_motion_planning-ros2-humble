"""Plot recorded Gazebo positions and speeds; never substitute predicted motion."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


def main():
    root = Path(__file__).resolve().parents[2]
    folder = root/'docs/validation/ladds-multi-waypoint-20260915'
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
    for row, (name, title) in enumerate((('gazebo-three-point', 'Two turns'),
                                       ('gazebo-straight', 'Short straight route'))):
        data = json.loads((folder/(name+'.json')).read_text(encoding='utf-8'))
        trace = data['trace']
        xy = np.array([[point['x'], point['y']] for point in trace])
        elapsed = np.array([point['elapsed'] for point in trace])
        elapsed -= elapsed[0]
        speed = np.array([point['speed'] for point in trace])
        targets = np.array(data['targets'])
        ax, velocity_ax = axes[row]
        ax.plot(xy[:, 0], xy[:, 1], color='#126c9c', lw=2, label='Measured position')
        ax.scatter([data['initial']['x']], [data['initial']['y']], marker='s',
                   color='#313b44', zorder=5, label='Start')
        current, switches = 0, []
        for t, point in zip(elapsed, xy):
            while current < len(targets)-1 and np.linalg.norm(point-targets[current]) <= 1.5:
                switches.append(float(t))
                current += 1
        for index, target in enumerate(targets):
            ax.add_patch(Circle(target, 1.5, facecolor='#cfdece', edgecolor='#477942',
                               alpha=.6, label='1.5 m arrival disk' if index == 0 else None))
            ax.scatter(*target, color='#477942', marker='+', zorder=5)
            ax.annotate(f'P{index+1}', target, xytext=(6, 8), textcoords='offset points')
        ax.set(aspect='equal', xlabel='ENU east (m)', ylabel='ENU north (m)',
               title=f'{title}: recorded Gazebo track')
        ax.grid(alpha=.2)
        ax.legend(loc='best', fontsize=8)
        requested = np.array([data['task']['points'][min(np.searchsorted(switches,t,side='right'), 2)]['speed']
                              *1852/3600 for t in elapsed])
        velocity_ax.step(elapsed, requested, where='post', color='#929aa1', ls='--',
                         label='Requested segment speed')
        velocity_ax.plot(elapsed, speed, color='#126c9c', label='Measured speed')
        for index, passed in enumerate(switches):
            velocity_ax.axvline(passed, color='#477942', lw=1, alpha=.65)
            velocity_ax.text(passed+.2, .1, f'P{index+1}', color='#477942')
        velocity_ax.set(xlabel='Time since first recorded sample (s)', ylabel='Speed (m/s)',
                        ylim=(0, 2.3), title='Actual speed and inbound segment request')
        velocity_ax.grid(alpha=.2)
        velocity_ax.legend(fontsize=8)
    fig.suptitle('0211 ordered mandatory waypoints | ADRC | one execution per route', fontsize=14)
    fig.savefig(folder/'gazebo-multi-waypoint.png', dpi=180)
    fig.savefig(folder/'gazebo-multi-waypoint.pdf')
    plt.close(fig)


if __name__ == '__main__':
    main()

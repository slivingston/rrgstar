#!/usr/bin/env python
"""Plot output from the RRG planner, as provided by rrg::dump_json()

  Usage: rrgplot.py FILE1 [-|FILE2]

where FILE1 is the name of a problem instance description file, and
(optional) FILE2 is the name of the output file. If FILE2 is omitted
or if the name "-" is given, then stdin (standard input stream) is
read instead.


SCL; 8 Feb 2015
"""
from __future__ import division
from __future__ import print_function

import sys
import json
import io

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib as mpl


def plot_obs(ax, obs_list):
    for obs in obs_list:
        corners = np.array([
            # lower-left
            [obs['center'][0]-obs['size'][0]/2,
             obs['center'][1]-obs['size'][1]/2],

            # lower-right
            [obs['center'][0]+obs['size'][0]/2,
             obs['center'][1]-obs['size'][1]/2],

            # upper-right
            [obs['center'][0]+obs['size'][0]/2,
             obs['center'][1]+obs['size'][1]/2],

            # upper-left
            [obs['center'][0]-obs['size'][0]/2,
             obs['center'][1]+obs['size'][1]/2]])
        ax.add_patch(mpl.patches.Polygon(corners,
                                         color=(0.8, 1., 0.8)))

if __name__ == '__main__':

    ############################################################
    # Parameters
    draw_rrgraph = True  # if True, also draw the RRG from which it came.
    edgeskip = 100  # 0 to draw all edges    
    show_between_states = True  # if True, then draw discretized state trajectory between adjacent vertices

    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    elif len(sys.argv) >= 3 and sys.argv[2] != '-':
        fp = io.open(sys.argv[2], 'r')
    else:
        fp = sys.stdin
    rrg_data = json.load(fp)
    if fp != sys.stdin:
        fp.close()

    G = nx.DiGraph()
    G.add_nodes_from([(s, {'state': rrg_data[s]['state']})
                      for s in rrg_data.keys() if s not in ('has_feasible', 'solution')])
    if draw_rrgraph:
        for n in G.nodes_iter():
            G.add_edges_from([(n, s) for s in rrg_data[n]['successors'].keys()])

    with io.open(sys.argv[1], 'r') as wspace_file:
        wspace = json.load(wspace_file)

    ax = plt.subplot()
    plot_obs(ax, wspace['obstacles'])
    plt.axis((wspace['bounds'][0], wspace['bounds'][1],
              wspace['bounds'][2], wspace['bounds'][3]))

    if draw_rrgraph:
        i = 0
        for (u, v) in G.edges_iter():
            if edgeskip == 0 or i == edgeskip:
                i = 0
                if not show_between_states:
                    local_traj = np.array([G.node[u]['state'],
                                           G.node[v]['state']])
                else:
                    rrg_data[u]['successors'][v].insert(0, G.node[u]['state'])
                    rrg_data[u]['successors'][v].append(G.node[v]['state'])
                    local_traj = np.array(rrg_data[u]['successors'][v])
                plt.plot(local_traj.T[0], local_traj.T[1], 'b-')
                plt.plot([G.node[u]['state'][0], G.node[v]['state'][0]],
                         [G.node[u]['state'][1], G.node[v]['state'][1]],
                         'b.')
            if edgeskip > 0:
                i += 1

    if rrg_data['has_feasible'] and len(np.array(rrg_data['solution'])) > 0:
        traj = np.array(rrg_data['solution'])
        ax.plot(traj.T[0], traj.T[1], 'r--', linewidth=2)
        ax.plot(traj.T[0][0], traj.T[1][0], 'r*', markersize=10)

    plt.axis('equal')
    plt.grid(True)
    plt.show()

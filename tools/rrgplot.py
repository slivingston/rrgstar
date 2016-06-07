#!/usr/bin/env python
"""Plot output from the RRG planner

Solutions are contained in JSON as provided by rrg::dump_json() or
rrgstar::dump_json().

  Usage: rrgplot.py FILE1 [-|FILE2]

where FILE1 is the name of a problem instance description file, and
(optional) FILE2 is the name of the output file. If FILE2 is omitted
or if the name "-" is given, then stdin (standard input stream) is
read instead.


SCL; Feb, Apr 2015
"""
from __future__ import division
from __future__ import print_function

import sys
import json
import io

import numpy as np
import networkx as nx


def plot_obs_3d(obs_list):
    for obs in obs_list:
        offset = [obs['center'][0]-obs['size'][0]/2,
                  obs['center'][1]-obs['size'][1]/2,
                  obs['center'][2]-obs['size'][2]/2]
        O = np.array([[offset[0], offset[1], offset[2]],
                      [offset[0]+obs['size'][0], offset[1], offset[2]],
                      [offset[0]+obs['size'][0], offset[1]+obs['size'][1], offset[2]],
                      [offset[0], offset[1]+obs['size'][1], offset[2]],
                      [offset[0], offset[1], offset[2]],
                      [offset[0], offset[1], offset[2]+obs['size'][2]],
                      [offset[0]+obs['size'][0], offset[1], offset[2]+obs['size'][2]],
                      [offset[0]+obs['size'][0], offset[1]+obs['size'][1], offset[2]+obs['size'][2]],
                      [offset[0], offset[1]+obs['size'][1], offset[2]+obs['size'][2]],
                      [offset[0], offset[1], offset[2]+obs['size'][2]]])
        mlab.plot3d(O.T[0], O.T[1], O.T[2], color=(0.0, 1.0, 0.0))
        O = np.array([[offset[0]+obs['size'][0], offset[1], offset[2]],
                      [offset[0]+obs['size'][0], offset[1], offset[2]+obs['size'][2]]])
        mlab.plot3d(O.T[0], O.T[1], O.T[2], color=(0.0, 1.0, 0.0))
        O = np.array([[offset[0], offset[1]+obs['size'][1], offset[2]],
                      [offset[0], offset[1]+obs['size'][1], offset[2]+obs['size'][2]]])
        mlab.plot3d(O.T[0], O.T[1], O.T[2], color=(0.0, 1.0, 0.0))
        O = np.array([[offset[0]+obs['size'][0], offset[1]+obs['size'][1], offset[2]],
                      [offset[0]+obs['size'][0], offset[1]+obs['size'][1], offset[2]+obs['size'][2]]])
        mlab.plot3d(O.T[0], O.T[1], O.T[2], color=(0.0, 1.0, 0.0))

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
    draw_rrgraph = False  # if True, also draw the RRG from which it came.
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

    if draw_rrgraph:
        if not rrg_data.has_key('V'):
            print('WARNING: Unable to draw sampled graph because it was not given.')
        else:
            G = nx.DiGraph()
            G.add_nodes_from([(s, {'state': rrg_data['V'][s]['state']})
                              for s in rrg_data['V'].keys()])
            for n in G.nodes_iter():
                G.add_edges_from([(n, s) for s in rrg_data['V'][n]['successors'].keys()])

    with io.open(sys.argv[1], 'r') as wspace_file:
        wspace = json.load(wspace_file)

    assert len(wspace['bounds']) == 4 or len(wspace['bounds']) == 6
    if len(wspace['bounds']) == 6:
        try:
            from mayavi import mlab
        except ImportError:
            # Fail cleanly if 3D plotting tool is unavailable
            mayavi = None
        is3D = True
    elif len(wspace['bounds']) == 4:
        import matplotlib.pyplot as plt
        import matplotlib as mpl
        is3D = False
    else:
        raise ValueError('"bounds" key in given file has invalid length: '
                         +str(len(wspace['bounds'])))

    if is3D:
        plot_obs_3d(wspace['regions'])
    else:
        fig = plt.figure()
        ax = fig.gca()
        plot_obs(ax, wspace['regions'])
        plt.axis((wspace['bounds'][0], wspace['bounds'][1],
                  wspace['bounds'][2], wspace['bounds'][3]))

    if draw_rrgraph and rrg_data.has_key('V'):
        i = 0
        for (u, v) in G.edges_iter():
            if edgeskip == 0 or i == edgeskip:
                i = 0
                if not show_between_states:
                    local_traj = np.array([G.node[u]['state'],
                                           G.node[v]['state']])
                else:
                    rrg_data['V'][u]['successors'][v].insert(0, G.node[u]['state'])
                    rrg_data['V'][u]['successors'][v].append(G.node[v]['state'])
                    local_traj = np.array(rrg_data['V'][u]['successors'][v])
                if is3D:
                    mlab.plot3d(local_traj.T[0], local_traj.T[1], local_traj.T[2],
                                color=(0.0, 0.0, 1.0), line_width=0.5)
                    mlab.plot3d([G.node[u]['state'][0], G.node[v]['state'][0]],
                                [G.node[u]['state'][1], G.node[v]['state'][1]],
                                [G.node[u]['state'][2], G.node[v]['state'][2]],
                                color=(0.0, 0.0, 1.0), line_width=0.5)
                else:
                    plt.plot(local_traj.T[0], local_traj.T[1], 'b-')
                    plt.plot([G.node[u]['state'][0], G.node[v]['state'][0]],
                             [G.node[u]['state'][1], G.node[v]['state'][1]],
                             'b.')
            if edgeskip > 0:
                i += 1

    if rrg_data['has_feasible'] and len(np.array(rrg_data['solution'])) > 0:
        traj = np.array(rrg_data['solution'])
        if is3D:
            mlab.plot3d(traj.T[0], traj.T[1], traj.T[2], color=(1.0, 0.0, 0.0))
            mlab.points3d(traj.T[0][0], traj.T[1][0], traj.T[2][0],
                          color=(1.0, 0.0, 0.0), mode='sphere', opacity=0.5, scale_factor=0.2)
        else:
            ax.plot(traj.T[0], traj.T[1], 'r--', linewidth=2)
            ax.plot(traj.T[0][0], traj.T[1][0], 'r*', markersize=10)
    elif not rrg_data['has_feasible']:
        print('No solution trajectory given.')

    if is3D:
        mlab.show()
    else:
        plt.axis('equal')
        plt.grid(True)
        plt.show()

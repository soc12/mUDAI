import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import numpy as np


def plot_polygon(p, color='red', angle=False, label=None):
    plt.plot(*p.exterior.xy, color=color, label=label)
    # plt.text(*p.exterior.xy[0][0],'rotation')
    if angle:
        plt.text(p.exterior.xy[0][0], p.exterior.xy[1][0], str(int(angle)), fontsize=10)


def plot_2d(NED_Coords, camera_dict):
    for count, parcel in enumerate(NED_Coords):
        plot_polygon(Polygon(parcel), 'b')
        plot_polygon(camera_dict[count], 'r')


def plot_2d_3d(solver, NED_Coords, camera_dict):
    fig = plt.figure(figsize=(15, 5))
    ax_2d = fig.add_subplot(121)
    ax_2d.set_aspect('equal')
    for count, parcel in enumerate(NED_Coords):
        plt.sca(ax_2d)
        plot_polygon(Polygon(parcel), 'b', angle=False, label='Parcel')
        plot_polygon(camera_dict[count], 'r', angle=False, label='FoV')
    for line in solver.plot_lines:
        plt.plot(line[0], line[1], color='peru', label='Path')
        plt.scatter(line[0][0], line[1][0], color='purple', label="Initial pos")
    hand, labl = ax_2d.get_legend_handles_labels()
    unique_labels, unique_handles = np.unique(labl, return_index=True)
    ax_2d.legend([hand[i] for i in unique_handles], unique_labels)
    ax_3d = fig.add_subplot(122, projection='3d')  # Initialize 3D axes
    for line in ax_2d.lines:
        if line._label == 'Path':  # and (solver.plot_lines[0][0] == line.get_xdata()).all():
            continue
        ax_3d.plot(line.get_xdata(), line.get_ydata(), zs=0, color=line.get_color())
    plt.sca(ax_3d)
    for UAV_path in solver.final_path_ned:
        X = []
        Y = []
        Z = []
        for wp in UAV_path:
            X.append(wp[0])
            Y.append(wp[1])
            Z.append(wp[2])
        ax_3d.plot3D(X, Y, Z, color='peru')
        ax_3d.scatter(X[0], Y[0], Z[0], color='purple')
    ax_3d.legend([hand[i] for i in unique_handles], unique_labels)
    return ax_2d, ax_3d


def multi_plot_2d_3d(solver, NED_Coords, camera_dict):
    # Figure 1: 2D Plots
    fig_2d, axes_2d = plt.subplots(1, len(NED_Coords), figsize=(15, 5), sharex=True, sharey=True)
    fig_2d.suptitle('2D Plots')

    for ax, parcel, count in zip(axes_2d, NED_Coords, range(len(NED_Coords))):
        ax.set_aspect('equal')
        plt.sca(ax)
        plot_polygon(Polygon(parcel), 'b', angle=False, label=f'Parcel {count}')
        plot_polygon(camera_dict[count], 'r', angle=False, label=f'FoV {count}')
        for line in solver.plot_lines:
            plt.plot(line[0], line[1], color='peru', label='Path')
        plt.scatter(solver.plot_lines[0][0][0], solver.plot_lines[0][1][0], color='purple', label="Initial pos")
        hand, labl = ax.get_legend_handles_labels()
        unique_labels, unique_handles = np.unique(labl, return_index=True)
        ax.legend([hand[i] for i in unique_handles], unique_labels)

    # Figure 2: 3D Plots
    fig_3d, axes_3d = plt.subplots(1, len(solver.final_path_ned), figsize=(15, 5), sharex=True, sharey=True)
    fig_3d.suptitle('3D Plots')

    for idx, (UAV_path, ax) in enumerate(zip(solver.final_path_ned, axes_3d)):
        X = []
        Y = []
        Z = []
        for wp in UAV_path:
            X.append(wp[0])
            Y.append(wp[1])
            Z.append(wp[2])
        ax = fig_3d.add_subplot(1, len(solver.final_path_ned), idx + 1, projection='3d')
        plt.sca(ax)
        ax.plot3D(X, Y, Z, color='peru', label=f'UAV Path {idx}')
        ax.scatter(X[0], Y[0], Z[0], color='purple')
        hand_3d, labl_3d = ax.get_legend_handles_labels()
        unique_labels_3d, unique_handles_3d = np.unique(labl_3d, return_index=True)
        ax.legend([hand_3d[i] for i in unique_handles_3d], unique_labels_3d)

    return fig_2d, fig_3d

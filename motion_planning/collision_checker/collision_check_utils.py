import matplotlib.pyplot as plt


def plot_points(pts):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(pts[:, 0], pts[:, 1], pts[:, 2], 'green')
    ax.set_title('pointcloud')
    plt.show()

import numpy as np
import matplotlib.pyplot as plt

class TraceVisualiser3D:
    def __init__(self, title: str ="Deposition trace"):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection="3d")
        self.ax.set_title(title)
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")
        self.ax.view_init(elev=25, azim=-60)
        self.layers = []
        self.lines = []

    def add_layer(self, z: float, points: list):
        xs, ys = zip(*points)
        line, = self.ax.plot(xs, ys, z, linewidth=1.5)
        self.layers.append((z, points))
        self.lines.append(line)

    def finish(self):
        if not self.layers:
            return

        colours = plt.cm.viridis(np.linspace(0, 1, len(self.lines)))
        for line, colour in zip(self.lines, colours):
            line.set_color(colour)

        all_points = np.array([(x, y, z) for z, pts in self.layers for x, y in pts])
        ranges = all_points.max(axis=0) - all_points.min(axis=0)
        self.ax.set_box_aspect(np.maximum(ranges, 0.01 * ranges.max()))

        self.fig.canvas.draw_idle()

    def save(self, filename):
        self.fig.savefig(filename, dpi=200, bbox_inches="tight")

if __name__ == "__main__":
    from toolpath import ToolPath, LayeredToolPath

    stack = LayeredToolPath(ToolPath.circle((0, 3), 1.5, 4), 0.2, 8)
    vis3d = TraceVisualiser3D("Hollow cylinder")
    for z, points in stack.layers():
        vis3d.add_layer(z, points)
    vis3d.finish()
    vis3d.save("cylinder.png")
    plt.show()
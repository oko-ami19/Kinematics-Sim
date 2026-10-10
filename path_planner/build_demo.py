import matplotlib.pyplot as plt

from arm import TwoLinkArm
from toolpath import ToolPath, LayeredToolPath
from visualiser import ArmVisualiser
from visualiser3d import TraceVisualiser3D

if __name__ == "__main__":
    arm = TwoLinkArm(5, 4)
    stack = LayeredToolPath(ToolPath.circle((0, 3), 1.5, 40), 0.2, 5)

    vis2d = ArmVisualiser(arm)
    vis3d = TraceVisualiser3D("Hollow cylinder")
    plt.show(block=False)

    layers = stack.layers()
    for k, (z, points) in enumerate(layers):
        vis2d.ax.set_title(f"Layer {k + 1}/{len(layers)}  (z = {z:.2f})")
        vis2d.draw_toolpath(ToolPath(points, stack.base_path.speed))

        vis3d.add_layer(z, points)
        vis3d.fig.canvas.draw_idle()
        plt.pause(0.5)

    vis3d.finish()
    vis3d.save("build.png")
    plt.show()
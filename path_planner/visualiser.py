import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
from arm import TwoLinkArm
from toolpath import ToolPath

class ArmVisualiser:
    def __init__(self, robot: TwoLinkArm):
        self.robot = robot
        self.fig, self.ax = plt.subplots()

        reach = self.robot.L1 + self.robot.L2
        self.ax.set_xlim(-reach - 0.5, reach + 0.5)
        self.ax.set_ylim(0, reach + 0.5)
        self.ax.set_aspect("equal")

        x1 = np.array([0, self.robot.elbow[0]])
        y1 = np.array([0, self.robot.elbow[1]])
        x2 = np.array([self.robot.elbow[0], self.robot.end_effector[0]])
        y2 = np.array([self.robot.elbow[1], self.robot.end_effector[1]])

        self.line1, = self.ax.plot(x1, y1, "-o")
        self.line2, = self.ax.plot(x2, y2, "-o")

        self.path_line, = self.ax.plot([], [], "r--", linewidth=1.5)
        self.path_x = []
        self.path_y = []

        self.slider1 = None
        self.slider2 = None

        self.theta1 = self.robot.theta1
        self.theta2 = self.robot.theta2

    def _redraw(self, theta1, theta2):
        elbow, end_effector = self.robot.forward_kinematics(theta1, theta2, True)

        self.line1.set_data([0, elbow[0]],[0, elbow[1]])
        self.line2.set_data([elbow[0], end_effector[0]],[elbow[1], end_effector[1]])

        self.theta1 = theta1
        self.theta2 = theta2

        self.fig.canvas.draw_idle()

    def add_sliders(self):
        plt.subplots_adjust(bottom=0.25)

        ax_theta1 = plt.axes([0.2, 0.05, 0.6, 0.03])
        ax_theta2 = plt.axes([0.2, 0.1, 0.6, 0.03])

        self.slider1 = Slider(ax_theta1, "theta1", 0, 180, valinit=self.theta1)
        self.slider2 = Slider(ax_theta2, "theta2", -180, 180, valinit=self.theta2)

        self.last_theta1 = self.theta1
        self.last_theta2 = self.theta2

        def update(_):
            theta1 = self.slider1.val
            theta2 = self.slider2.val

            if not self.robot.is_config_valid(theta1, theta2):
                for slider, last in ((self.slider1, self.last_theta1), (self.slider2, self.last_theta2)):
                    slider.eventson = False
                    slider.set_val(last)
                    slider.eventson = True
                return

            self._redraw(theta1, theta2)

            self.last_theta1 = theta1
            self.last_theta2 = theta2

        self.slider1.on_changed(update)
        self.slider2.on_changed(update)

    def _set_sliders(self, theta1, theta2):
        if self.slider1 is None:
            return
        for slider, val in ((self.slider1, theta1), (self.slider2, theta2)):
            slider.eventson = False
            slider.set_val(val)
            slider.eventson = True

    def _mark_path(self):
        self.path_line.set_data(self.path_x, self.path_y)
        self.fig.canvas.draw_idle()

    def animate_path(self, points: list, time_between_move: float = 0.5, hold_position: float = 0.5, steps: int = 20):
        start = (self.theta1, self.theta2)
        moves = []

        try:
            self.robot.forward_kinematics(*start, True)
            current = start

            for x, y in points:
                target = self.robot.inverse_kinematics(x, y, True)
                frames = np.linspace(current, target, steps + 1)

                for t1, t2 in frames:
                    if not self.robot.is_config_valid(t1, t2):
                        raise ValueError(f"Path to ({x}, {y}) goes below the ground between waypoints")
                moves.append(frames)
                current = target

        except ValueError:
            self.robot.forward_kinematics(*start, True)
            raise

        for frames in moves:
            for t1, t2 in frames:
                self._redraw(t1, t2)
                self._set_sliders(t1, t2)
                plt.pause(time_between_move / steps)

            plt.pause(hold_position)

    def draw_toolpath(self, toolpath: ToolPath):
        speed = toolpath.speed
        start = (self.theta1, self.theta2)
        moves = []

        self.path_x = []
        self.path_y = []

        try:
            self.robot.forward_kinematics(*start, True)
            current_pos = self.robot.end_effector.copy()

            for x, y in toolpath.points:
                target_pos = np.array([x, y], dtype=float)
                dt = 0.01

                distance = np.linalg.norm(target_pos - current_pos)
                steps = max(1, int(np.ceil(distance / (speed * dt))))

                frames_coord = np.linspace(current_pos, target_pos, steps + 1)
                frames_angles = []

                for px, py in frames_coord:
                    theta1, theta2 = self.robot.inverse_kinematics(px, py, True)

                    frames_angles.append((theta1, theta2))

                moves.append(frames_angles)
                current_pos = target_pos

        except ValueError:
            self.robot.forward_kinematics(*start, True)
            raise

        self.robot.forward_kinematics(*start, True)

        for i, frames in enumerate(moves):
            for j, (theta1, theta2) in enumerate(frames):
                self._redraw(theta1, theta2)

                if i > 0 or j == len(frames) - 1:
                    self.path_x.append(self.robot.end_effector[0])
                    self.path_y.append(self.robot.end_effector[1])
                    self._mark_path()

                self._set_sliders(theta1, theta2)
                plt.pause(dt)

if __name__ == "__main__":
    square = [(2, 1), (4, 1), (4, 3), (2, 3), (2, 1)]

    arm = TwoLinkArm(5,4)
    vis = ArmVisualiser(arm)

    path = ToolPath.line((-1,3), (3,3),4)

    vis.animate_toolpath(path)
    plt.show()
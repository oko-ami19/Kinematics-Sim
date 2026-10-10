import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import time

#This is first draft and the animation is inside the twolinkarm class, done purely through matplotlib

g = 9.81

class TwoLinkArm:
    def __init__(self, L1: float, L2: float):
        self.L1 = L1
        self.L2 = L2
        self.theta1 = 0
        self.theta2 = 0

    def forward_kinematics(self, theta1: float, theta2: float):
        self.theta1 = theta1
        self.theta2 = theta2

        theta1 = np.radians(theta1)
        theta2 = np.radians(theta2)

        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        elbow = (x1.item(), y1.item())
        x2 = x1 + self.L2 * np.cos(theta1 + theta2)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)
        end_effector = (x2.item(), y2.item())
        self.elbow = elbow
        self.end_effector = end_effector

    def __str__(self):
        return f"elbow: {self.elbow}\nend_effector: {self.end_effector}"

    def inverse_kinematics(self, x_target: float, y_target: float):
        if y_target < 0:
            raise ValueError("Target is below the ground")

        r = np.sqrt(x_target **2 + y_target ** 2)
        if r > self.L1 + self.L2 or r < abs(self.L1 - self.L2):
            raise ValueError("Target is outside the arm's reachable workspace")
        thetaT = np.arctan2(y_target, x_target)

        elbow_vertex_angle = np.arccos((self.L1**2 + self.L2**2 - r**2)/(2 * self.L1 * self.L2))
        origin_vertex_angle = np.arccos((self.L1**2 - self.L2**2 + r**2)/(2 * self.L1 * r))
        gamma = elbow_vertex_angle
        alpha = origin_vertex_angle

        self.theta1 = np.degrees(thetaT - alpha)
        self.theta2 = np.degrees(np.pi - gamma)

    def plot(self):
        self.fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.25)
        ax_theta1 = plt.axes([0.2, 0.05, 0.6, 0.03])
        ax_theta2 = plt.axes([0.2, 0.1, 0.6, 0.03])
        self.slider1 = Slider(ax_theta1, 'theta1', 0, 180, valinit=self.theta1)
        self.slider2 = Slider(ax_theta2, 'theta2', -180, 180, valinit=self.theta2)

        def update(val: float):
            self.forward_kinematics(self.slider1.val, self.slider2.val)

            x1 = np.array([0, self.elbow[0]])
            y1 = np.array([0, self.elbow[1]])
            x2 = np.array([self.elbow[0], self.end_effector[0]])
            y2 = np.array([self.elbow[1], self.end_effector[1]])

            self.line1.set_xdata(x1)
            self.line1.set_ydata(y1)
            self.line2.set_xdata(x2)
            self.line2.set_ydata(y2)

            self.fig.canvas.draw_idle()

        x1 = np.array([0,self.elbow[0]])
        y1 = np.array([0,self.elbow[1]])

        x2 = np.array([self.elbow[0], self.end_effector[0]])
        y2 = np.array([self.elbow[1], self.end_effector[1]])

        self.line1, = ax.plot(x1, y1, "-o")
        self.line2, = ax.plot(x2, y2, "-o")

        reach = self.L1 + self.L2
        ax.set_xlim(-reach - 0.5, reach + 0.5)
        ax.set_ylim(-reach - 0.5, reach + 0.5)
        ax.set_aspect("equal")

        plt.axis("equal")

        self.slider1.on_changed(update)
        self.slider2.on_changed(update)

        plt.show(block=False)

    def animate_path(self, points: list, pause: float =0.5, steps: int = 20):
        for x,y in points:
            self.inverse_kinematics(x, y)
            t1_target, t2_target = self.theta1, self.theta2

            t1_start, t2_start = self.slider1.val, self.slider2.val

            for i in range(steps + 1):
                frac = i / steps
                t1 = t1_start + frac * (t1_target - t1_start)
                t2 = t2_start + frac * (t2_target - t2_start)
                self.slider1.set_val(t1)
                self.slider2.set_val(t2)
                plt.pause(pause / steps)

robot1 = TwoLinkArm(2, 3)

print(f"IK gave: theta1={robot1.theta1}, theta2={robot1.theta2}")
robot1.forward_kinematics(30,30)
robot1.plot()

path = [(1,2), (4,1), (-2, 1.5)]
robot1.animate_path(path)
plt.show()

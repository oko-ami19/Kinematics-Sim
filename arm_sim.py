import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import time

#This is 2nd draft and the animation is determine by 2nd armvisualiser class, done purely through matplotlib, and with added constraints for the motion
# uses linear interpolation for
g = 9.81

class TwoLinkArm:
    def __init__(self, L1: float, L2: float):
        self.L1 = L1
        self.L2 = L2
        self.theta1 = 0
        self.theta2 = 0

    def is_config_valid(self, theta1: float, theta2: float):
        theta1 = np.radians(theta1)
        theta2 = np.radians(theta2)

        y1 = self.L1 * np.sin(theta1)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)

        return y1 >= 0 and y2 >= 0  

    def forward_kinematics(self, theta1: float, theta2: float):
        if not self.is_config_valid(theta1, theta2):
            raise ValueError("Configuration is not valid")

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

    def is_reachable(self, x_target: float, y_target: float):
        r = abs(self.L1 - self.L2)
        R = self.L1 + self.L2
        return r <= np.sqrt(x_target**2 + y_target**2 ) <= R

    def inverse_kinematics(self, x_target: float, y_target: float):
        if y_target < 0:
            raise ValueError("Target is below the ground")

        if not self.is_reachable(x_target,y_target):
            raise ValueError("Target is not reachable")

        r = np.sqrt(x_target **2 + y_target ** 2)
        thetaT = np.arctan2(y_target, x_target)

        elbow_vertex_angle = np.arccos((self.L1**2 + self.L2**2 - r**2)/(2 * self.L1 * self.L2))
        origin_vertex_angle = np.arccos((self.L1**2 - self.L2**2 + r**2)/(2 * self.L1 * r))
        gamma = elbow_vertex_angle
        alpha = origin_vertex_angle

        self.theta1 = np.degrees(thetaT - alpha)
        self.theta2 = np.degrees(np.pi - gamma)

class ArmVisualiser:
    def __init__(self, robot: TwoLinkArm):
        self.robot = robot

    def display(self):
        self.fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.25)
        ax_theta1 = plt.axes([0.2, 0.05, 0.6, 0.03])
        ax_theta2 = plt.axes([0.2, 0.1, 0.6, 0.03])
        self.slider1 = Slider(ax_theta1, 'theta1', 0, 180, valinit=self.robot.theta1)
        self.slider2 = Slider(ax_theta2, 'theta2', -180, 180, valinit=self.robot.theta2)
        self.robot.forward_kinematics(self.slider1.val, self.slider2.val)

        self.last_theta1 = self.robot.theta1
        self.last_theta2 = self.robot.theta2

        def update(_):
            theta1 = self.slider1.val
            theta2 = self.slider2.val

            if not self.robot.is_config_valid(theta1, theta2):
                self.slider1.set_val(self.last_theta1)
                self.slider2.set_val(self.last_theta2)
                return

            self.robot.forward_kinematics(theta1, theta2)

            x1 = np.array([0, self.robot.elbow[0]])
            y1 = np.array([0, self.robot.elbow[1]])

            x2 = np.array([self.robot.elbow[0], self.robot.end_effector[0]])
            y2 = np.array([self.robot.elbow[1], self.robot.end_effector[1]])

            self.line1.set_xdata(x1)
            self.line1.set_ydata(y1)
            self.line2.set_xdata(x2)
            self.line2.set_ydata(y2)

            self.last_theta1 = theta1
            self.last_theta2 = theta2

            self.fig.canvas.draw_idle()

        x1 = np.array([0, self.robot.elbow[0]])
        y1 = np.array([0, self.robot.elbow[1]])
        
        x2 = np.array([self.robot.elbow[0], self.robot.end_effector[0]])
        y2 = np.array([self.robot.elbow[1], self.robot.end_effector[1]])

        self.line1, = ax.plot(x1, y1, "-o")
        self.line2, = ax.plot(x2, y2, "-o")

        reach = self.robot.L1 + self.robot.L2
        ax.set_xlim(-reach - 0.5, reach + 0.5)
        ax.set_ylim(0, reach + 0.5)
        ax.set_aspect("equal")

        self.slider1.on_changed(update)
        self.slider2.on_changed(update)
        
        plt.show(block=False)

    def animate_path(self, points: list, pause: float = 0.5, steps: int = 20):
        for x, y in points:
            self.robot.inverse_kinematics(x, y)
            t1_target, t2_target = self.robot.theta1, self.robot.theta2
            t1_start, t2_start = self.slider1.val, self.slider2.val

            for i in range(steps + 1):
                frac = i / steps
                t1 = t1_start + frac * (t1_target - t1_start)
                t2 = t2_start + frac * (t2_target - t2_start)
                self.slider1.set_val(t1)
                self.slider2.set_val(t2)
                plt.pause(pause / steps)

robot1 = TwoLinkArm(2, 3)
robot2 = TwoLinkArm(5, 1)

robot1.forward_kinematics(30,30)
robot2.forward_kinematics(30,30)
print(robot1)
print(robot2)
visual1 = ArmVisualiser(robot1)
visual2 = ArmVisualiser(robot2)
visual1.display()
visual2.display()
plt.show()

path = [(1,2), (4,1), (-2, 1.5)]
# robot1.animate_path(path)
# plt.show()

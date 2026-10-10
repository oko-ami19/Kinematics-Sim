import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import time

#This is 3rd draft and the animation is determine by 2nd armvisualiser class, done purely through matplotlib
#the jacobian is derived and used for the animation for smoothness, (damped least squares)

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

    def jacobian(self, theta1: float, theta2: float):
        theta1 = np.radians(theta1)
        theta2 = np.radians(theta2)

        j1_1 = -self.L2 * np.sin(theta1 + theta2) - self.L1 * np.sin(theta1)
        j1_2 = -self.L2 * np.sin(theta1 + theta2)
        j2_1 = self.L2 * np.cos(theta1 + theta2) + self.L1 * np.cos(theta1)
        j2_2 = self.L2 * np.cos(theta1 + theta2)

        jaco = np.array([[j1_1,j1_2],[j2_1,j2_2]])
        return jaco

    def joint_velocities(self, theta1: float, theta2: float, end_effector_v_x: float, end_effector_v_y: float, damping: float = 0.1):
        end_effector_velocity = np.array([[end_effector_v_x],[end_effector_v_y]])
        j = self.jacobian(theta1, theta2)
        I = np.eye(2)

        A = j @ j.T + damping**2 * I

        velocities = j.T @ np.linalg.solve(A, end_effector_velocity)
        velocities = velocities.flatten()
        return float(velocities[0]), float(velocities[1])

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
                self.slider1.eventson = False
                self.slider2.eventson = False
                self.slider1.set_val(self.last_theta1)
                self.slider2.set_val(self.last_theta2)
                self.slider1.eventson = True
                self.slider2.eventson = True
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
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')

        self.slider1.on_changed(update)
        self.slider2.on_changed(update)
        
        plt.show(block=False)

    def animate_path(self, points: list, gain: float = 16, dt: float = 0.01):
        for x, y in points:
            target_not_reached = True
            while target_not_reached:
                current_theta1 = self.slider1.val #degrees
                current_theta2 = self.slider2.val
                current_x, current_y = self.robot.end_effector

                dx = x - current_x
                dy = y - current_y
                distance = np.sqrt(dx**2 + dy**2)
                vx = dx * gain
                vy = dy * gain

                if distance < 0.1:
                    target_not_reached = False

                #rads
                theta1_dot, theta2_dot = self.robot.joint_velocities(current_theta1, current_theta2, vx, vy)
                new_theta1 = current_theta1 + np.degrees(theta1_dot * dt)
                new_theta2 = current_theta2 + np.degrees(theta2_dot * dt)

                self.slider1.set_val(new_theta1)
                self.slider2.set_val(new_theta2)
                plt.pause(dt)
                

    

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
path = [(3, 4)]
visual1.animate_path(path)
plt.show()




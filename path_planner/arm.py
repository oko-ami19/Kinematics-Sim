import numpy as np

class TwoLinkArm:
    EPS = 1e-9

    def __init__(self, L1: float, L2: float):
        self.L1 = L1
        self.L2 = L2
        self.theta1 = 90
        self.theta2 = 0
        self.elbow, self.end_effector = self._positions(self.theta1, self.theta2)

    def _positions(self, theta1: float, theta2: float):
        theta1 = np.radians(theta1)
        theta2 = np.radians(theta2)

        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        elbow = np.array([x1, y1])

        x2 = x1 + self.L2 * np.cos(theta1 + theta2)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)
        end_effector = np.array([x2, y2])

        return elbow, end_effector
        
    def is_config_valid(self, theta1: float, theta2: float):
        elbow, end_effector = self._positions(theta1, theta2)

        return elbow[1] >= -self.EPS and end_effector[1] >= -self.EPS

    def forward_kinematics(self, theta1: float, theta2: float, if_setting_pose: bool = False):
        if not self.is_config_valid(theta1, theta2):
            raise ValueError("Configuration is invalid")

        elbow, end_effector = self._positions(theta1, theta2)

        if if_setting_pose:
            self.theta1 = theta1
            self.theta2 = theta2
            self.elbow = elbow
            self.end_effector = end_effector

        return elbow, end_effector

    def is_reachable(self, x_target: float, y_target: float):
        r = abs(self.L1 - self.L2)
        R = self.L1 + self.L2
        displacement = np.hypot(x_target, y_target)
        return r - self.EPS <= displacement <= R + self.EPS

    def inverse_kinematics(self, x_target: float, y_target: float, if_setting_pose: bool = False):
        if y_target < -self.EPS:
            raise ValueError("Target is below the ground")

        if not self.is_reachable(x_target, y_target):
            raise ValueError("Target is not reachable")

        R = np.hypot(x_target, y_target)
        gamma = np.arctan2(y_target, x_target)

        cos_alpha = np.clip((R ** 2 + self.L1 ** 2 - self.L2 ** 2) / (2 * R * self.L1), -1.0, 1.0)
        cos_beta = np.clip((- R ** 2 + self.L1 ** 2 + self.L2 ** 2) / (2 * self.L2 * self.L1), -1.0, 1.0)
        alpha = np.arccos(cos_alpha)
        beta = np.arccos(cos_beta)

        thetas_elbow_up = np.array([np.degrees(alpha + gamma), np.degrees(beta - np.pi)])
        thetas_elbow_down = np.array([np.degrees(gamma - alpha), np.degrees(np.pi - beta)])
        current_thetas = np.array([self.theta1, self.theta2])

        valid_up = self.is_config_valid(*thetas_elbow_up)
        valid_down = self.is_config_valid(*thetas_elbow_down)

        if not valid_up and not valid_down:
            raise ValueError("No valid configuration exists above ground for this target")

        if valid_up and valid_down:
            delta_up = np.linalg.norm(thetas_elbow_up - current_thetas)
            delta_down = np.linalg.norm(thetas_elbow_down - current_thetas)
            theta1, theta2 = thetas_elbow_up if delta_up <= delta_down else thetas_elbow_down
        elif valid_up:
            theta1, theta2 = thetas_elbow_up
        else:
            theta1, theta2 = thetas_elbow_down

        if if_setting_pose:
            self.theta1 = theta1
            self.theta2 = theta2
            self.elbow, self.end_effector = self._positions(theta1, theta2)

        return theta1, theta2
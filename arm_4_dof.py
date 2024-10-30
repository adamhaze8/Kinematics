import numpy as np
import math


class ArmKinematics:
    def __init__(
        self, L1=0.325, L2=0.325, base_height=0.1, EE_length=0.272, EE_Y_offset=0.102
    ):
        self.L1 = L1
        self.L2 = L2
        self.base_height = base_height
        self.EE_length = EE_length
        self.EE_Y_offset = EE_Y_offset

    def calculate_joint_angles(self, xyz_target):
        x_target = xyz_target[0]
        y_target = xyz_target[1]
        z_target = xyz_target[2]

        z_target -= self.base_height

        # Calculate the distance from the base to the end-effector
        distance = math.sqrt(
            (math.sqrt(x_target**2 + y_target**2) - self.EE_length) ** 2 + z_target**2
        )

        # Check if the target point is reachable
        if distance > self.L1 + self.L2:
            print("Cannot reach")
            return None
        else:
            x_y_distance = np.sqrt(x_target**2 + y_target**2)

            if self.EE_Y_offset > x_y_distance:
                theta = np.pi / 2
            else:
                theta = np.arcsin(self.EE_Y_offset / x_y_distance)

            extension_distance = self.EE_Y_offset / np.tan(theta) - self.EE_length

            base_joint = np.arctan2(y_target, x_target) + theta

            elbow_joint = -np.arccos(
                (extension_distance**2 + z_target**2 - self.L1**2 - self.L2**2)
                / (2 * self.L1 * self.L2)
            )

            shoulder_joint = np.arctan2(z_target, extension_distance) + np.arccos(
                (extension_distance**2 + z_target**2 + self.L1**2 - self.L2**2)
                / (2 * self.L1 * np.sqrt(extension_distance**2 + z_target**2))
            )

            wrist_joint = -(shoulder_joint + elbow_joint) + np.pi / 2

            elbow_joint = -elbow_joint

            return [base_joint, shoulder_joint, elbow_joint, wrist_joint, 0]


# Example usage:
if __name__ == "__main__":
    arm = ArmKinematics()
    joint_angles = arm.calculate_joint_angles([0.5, 0.5, 0.6])
    # print(np.rad2deg(joint_angles))

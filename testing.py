from arm_3_dof_planar import Arm_3_DOF_Planar
from arm_plotter import *

link_lengths = [0.245, 0.203, 0.25]

arm = Arm_3_DOF_Planar(link_lengths[0], link_lengths[1], link_lengths[2])

A1, A2, A3 = arm.get_RK(0.5, 0.1)

print(f"Calculated Joint Angles: {A1}, {A2}, {A3}")

# Example usage for 2D:
angles_2d = [A1, A2, A3]  # Example joint angles in degrees (2D)
# Example link lengths (2D)
plot_arm(angles_2d, link_lengths, plot_3d=False)  # Plot as 2D

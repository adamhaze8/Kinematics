import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting


def plot_arm(angles, link_lengths, plot_3d=False):
    """
    Plots a 2D or 3D arm based on the number of degrees of freedom (DOF).

    Parameters:
    angles (list or np.array): Joint angles in degrees. For 2D, provide in (theta1, theta2, ...).
                               For 3D, provide in (theta1, theta2, theta3, ...).
    link_lengths (list or np.array): Link lengths corresponding to each joint.
    plot_3d (bool): If True, plots a 3D arm. If False, plots a 2D arm.

    This function supports arbitrary DOF in both 2D and 3D.
    """
    angles = np.radians(angles)  # Convert angles to radians for calculation
    dof = len(angles)  # Degrees of freedom

    # Initialize coordinates (starting at the origin)
    x_coords = [0]
    y_coords = [0]
    z_coords = [0] if plot_3d else None  # For 3D case, track z-coordinates

    if plot_3d:
        # Calculate joint positions for a 3D arm
        for i in range(dof):
            # For 3D, assume angles are azimuth (yaw) and elevation (pitch)
            azimuth = np.sum(
                angles[: i + 1]
            )  # Sum all angles up to current joint for yaw
            elevation = angles[i] if i < dof - 1 else 0  # Optional pitch if needed

            x_new = x_coords[-1] + link_lengths[i] * np.cos(azimuth) * np.cos(elevation)
            y_new = y_coords[-1] + link_lengths[i] * np.sin(azimuth) * np.cos(elevation)
            z_new = z_coords[-1] + link_lengths[i] * np.sin(elevation)

            x_coords.append(x_new)
            y_coords.append(y_new)
            z_coords.append(z_new)

        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # Plot arm in 3D
        ax.plot(
            x_coords, y_coords, z_coords, marker="o", color="blue"
        )  # Arm links with joints marked by 'o'
        ax.set_title(f"{dof}-DOF 3D Arm")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio
        plt.grid(True)

    else:
        # Calculate joint positions for a 2D arm
        for i in range(dof):
            x_new = x_coords[-1] + link_lengths[i] * np.cos(np.sum(angles[: i + 1]))
            y_new = y_coords[-1] + link_lengths[i] * np.sin(np.sum(angles[: i + 1]))
            x_coords.append(x_new)
            y_coords.append(y_new)

        # Create the plot (2D)
        fig, ax = plt.subplots()

        # Plot arm in 2D
        ax.plot(
            x_coords, y_coords, marker="o", color="blue"
        )  # Arm links with joints marked by 'o'
        ax.set_title(f"{dof}-DOF Planar Arm")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_aspect("equal")
        plt.grid(True)

    # Show the plot
    plt.show()

    # Close the plot to prevent multiple plots on the same figure
    plt.close(fig)

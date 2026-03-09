from controller_tuning.trajectory.utils import TrajectoryProfile

import matplotlib.pyplot as plt
from math import pi, cos, sin, atan2, sqrt

TITLE_FONTSIZE = 20
SUBPLOT_TITLE_FONTSIZE = 14

def subplot_constraint_lines(
    ax, 
    constraint: float
) -> None:
    # Upper constraint line
    ax.axhline(y=constraint, color="black", linestyle='--', alpha=0.7, linewidth=1.5, label=f'Constraint = ±{constraint:.2f}')
    # Lower constraint line  
    ax.axhline(y=-constraint, color="black", linestyle='--', alpha=0.7, linewidth=1.5)
    ax.legend(loc='upper right')


def plot_trajectory_profile(
    trajectory: TrajectoryProfile,
    max_long_vel: float = None,
    max_angular_vel: float = None,
    max_long_accel: float = None,
    title: str = None,
    save_path: str = None
) -> None:
    # Create figure with 2 rows and 2 columns of subplots
    if trajectory.long_accels:
        num_rows = 3
    else:
        num_rows = 2
    fig, axes = plt.subplots(num_rows, 2, figsize=(16, 10))

    if title:
        fig.suptitle(title, fontsize=TITLE_FONTSIZE)

    # Unpack pose data into separate lists
    t = []
    x = []
    y = []
    yaw = []
    for p in trajectory.poses:
        t.append(p.t)
        x.append(p.x)
        y.append(p.y)
        yaw.append(p.yaw)
    # Unpack velocity command data into separate lists
    lin_v = []
    ang_v = []
    for v in trajectory.vels:
        lin_v.append(v.linear_x)
        ang_v.append(v.angular_z)

    # Plot x and y
    axes[0, 0].plot(t, x, color="tab:blue", label=r"$x$")
    axes[0, 0].plot(t, y, color="tab:red", label=r"$y$")
    axes[0, 0].set_title("Displacement", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[0, 0].set_xlabel(r"Time ($s$)")
    axes[0, 0].set_ylabel(r"Displacement ($m$)")
    axes[0, 0].legend(loc="lower left")

    # Plot yaw
    axes[0, 1].plot(t, yaw, color="tab:orange")
    axes[0, 1].set_title("Yaw angle", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[0, 1].set_xlabel(r"Time ($s$)")
    axes[0, 1].set_ylabel(r"Yaw ($rad$)")
    
    # Plot linear velocity
    axes[1, 0].plot(t, lin_v, color="tab:green")
    axes[1, 0].set_title("Linear velocity (along chassis x-axis)", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[1, 0].set_xlabel(r"Time ($s$)")
    axes[1, 0].set_ylabel(r"Velocity ($m/s$)")
    if max_long_vel is not None:
        subplot_constraint_lines(ax=axes[1, 0], constraint=max_long_vel)

    # Plot angular velocity
    axes[1, 1].plot(t, ang_v, color="tab:purple")
    axes[1, 1].set_title("Yaw angular velocity", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[1, 1].set_xlabel(r"Time ($s$)")
    axes[1, 1].set_ylabel(r"Angular velocity ($rad/s$)")
    if max_angular_vel is not None:
        subplot_constraint_lines(ax=axes[1, 1], constraint=max_angular_vel)

    if trajectory.long_accels:
        # Plot linear acceleration
        axes[2, 0].plot(t, trajectory.long_accels, color="tab:brown")
        axes[2, 0].set_title("Linear acceleration (along chassis x-axis)", fontsize=SUBPLOT_TITLE_FONTSIZE)
        axes[2, 0].set_xlabel(r"Time ($s$)")
        axes[2, 0].set_ylabel(r"Acceleration ($m/s^{2}$)")
        if max_long_accel is not None:
            subplot_constraint_lines(ax=axes[2, 0], constraint=max_long_accel)

        # Ignore this subplot
        axes[2,1].set_visible(False)

    # Add some padding 
    plt.tight_layout(
        pad=2.0,  # From figure edges
        w_pad=1.0,  # Horizontal between subplots
        h_pad=1.0  # Vertical between subplots
    )

    if save_path:
        plt.savefig(save_path)

    plt.show()


def plot_figure_eight(
    a: float,
    b: float, 
    w: float,
    x_lim: float = None,
    y_lim: float = None,
    save_path: str = None
) -> None:
    time_period = 2 * pi / w
    num_samples = 101
    time_delta = time_period / (num_samples - 1)
    x_arr = []
    y_arr = []
    t = 0
    for _ in range(num_samples):    
        x = a * cos(w * t)
        y = b * sin(2 * w * t)

        x_arr.append(x)
        y_arr.append(y)
        
        t += time_delta

    x_lim_abs = x_lim or max(abs(x) for x in x_arr) * 1.5
    y_lim_abs = y_lim or max(abs(y) for y in y_arr) * 1.5

    # Format LaTeX on separate lines
    params_text = "{}\n{}".format(
        r"$A = {:.2f}m$".format(a),
        r"$B = {:.2f}m$".format(b)
    )

    # Plot y vs x
    plt.plot(x_arr, y_arr, color="tab:blue")
    plt.title("Figure of Eight", fontsize=TITLE_FONTSIZE)
    plt.xlim(-x_lim_abs, x_lim_abs)
    plt.xlabel(r"x ($m$)")
    plt.ylim(-y_lim_abs, y_lim_abs)
    plt.ylabel(r"y ($m$)")
    plt.gca().text(
        x=0.95,
        y=0.95,
        s=params_text,
        transform=plt.gca().transAxes,
        fontsize=10,
        ha="right",  # Align right of text to x arg
        va="top",  # Align top of text to y arg
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5)
    )

    # Add some padding from figure edges
    plt.tight_layout(pad=2.0)

    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()


def plot_figure_eight_kinematics(
    a: float,
    b: float, 
    w: float,
    save_path: str = None
) -> None:
    time_period = 2 * pi / w
    num_samples = 101
    time_delta = time_period / (num_samples - 1)
    t_arr = []
    x_arr = []
    y_arr = []
    v_x_arr = []
    v_y_arr = []
    v_mag_arr = []
    yaw_arr = []
    v_yaw_arr = []
    a_x_arr = []
    a_y_arr = []
    a_long_arr = []
    t = 0
    for _ in range(num_samples):    
        x = a * cos(w * t)
        y = b * sin(2 * w * t)

        v_x = -w * a * sin(w * t)
        v_y = 2 * w * b * cos(2 * w * t)
        v_mag = sqrt((v_x ** 2) + (v_y ** 2))

        a_x = -(w ** 2) * a * cos(w * t)
        a_y = -4 * (w ** 2) * b * sin(2 * w * t)

        yaw = atan2(v_y, v_x)
        v_yaw = ((v_x * a_y) - (a_x * v_y)) / (v_mag ** 2)

        # Get component of total acceleration along heading direction
        a_long = a_x * cos(yaw) + a_y * sin(yaw)

        t_arr.append(t)
        x_arr.append(x)
        y_arr.append(y)
        v_x_arr.append(v_x)
        v_y_arr.append(v_y)
        v_mag_arr.append(v_mag)
        yaw_arr.append(yaw)
        v_yaw_arr.append(v_yaw)
        a_x_arr.append(a_x)
        a_y_arr.append(a_y)
        a_long_arr.append(a_long)

        t += time_delta

    fig, axes = plt.subplots(3, 2, figsize=(16, 10))

    fig.suptitle("Figure of Eight Kinematics", fontsize=TITLE_FONTSIZE)

    # Plot y vs x
    axes[0, 0].plot(t_arr, x_arr, color="tab:red", label=r"$x$")
    axes[0, 0].plot(t_arr, y_arr, color="tab:green", label=r"$y$")
    axes[0, 0].set_title("Displacement", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[0, 0].set_xlabel(r"Time ($s$)")
    axes[0, 0].set_ylabel(r"Displacement ($m$)")
    axes[0, 0].legend(loc="lower left")

    # Plot yaw
    axes[0, 1].plot(t_arr, yaw_arr, color="tab:purple")
    axes[0, 1].set_title("Yaw angle", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[0, 1].set_xlabel(r"Time ($s$)")
    axes[0, 1].set_ylabel(r"Yaw ($rad$)")
    
    # Plot velocities
    axes[1, 0].plot(t_arr, v_x_arr, color="tab:red", label=r"$v_x$")
    axes[1, 0].plot(t_arr, v_y_arr, color="tab:green", label=r"$v_y$")
    axes[1, 0].plot(t_arr, v_mag_arr, color="tab:blue", label=r"$|v|$")
    axes[1, 0].set_title("Velocities", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[1, 0].set_xlabel(r"Time ($s$)")
    axes[1, 0].set_ylabel(r"Velocity ($m/s$)")
    axes[1, 0].legend(loc="lower right")

    # Plot angular velocity
    axes[1, 1].plot(t_arr, v_yaw_arr, color="tab:purple")
    axes[1, 1].set_title("Yaw angular velocity", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[1, 1].set_xlabel(r"Time ($s$)")
    axes[1, 1].set_ylabel(r"Angular velocity ($rad/s$)")

    # Plot accelerations
    axes[2, 0].plot(t_arr, a_x_arr, color="tab:red", label=r"$a_x$")
    axes[2, 0].plot(t_arr, a_y_arr, color="tab:green", label=r"$a_y$")
    axes[2, 0].plot(t_arr, a_long_arr, color="tab:blue", label=r"$a_long$")
    axes[2, 0].set_title("Accelerations", fontsize=SUBPLOT_TITLE_FONTSIZE)
    axes[2, 0].set_xlabel(r"Time ($s$)")
    axes[2, 0].set_ylabel(r"Acceleration ($m/s^{2}$)")
    axes[2, 0].legend(loc="lower right")

    # Ignore this subplot
    axes[2,1].set_visible(False)

    # Add some padding 
    plt.tight_layout(
        pad=2.0,  # From figure edges
        w_pad=1.0,  # Horizontal between subplots
        h_pad=1.0  # Vertical between subplots
    )

    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()

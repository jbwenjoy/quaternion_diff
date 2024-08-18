import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import glob
# import ahrs
import matplotlib.pyplot as plt


def angular_velocities(q1, q2, dt):
    """
    Provided by https://mariogc.com/post/angular-velocity-quaternions/
    seems incorrect, but keep for reference
    """
    return (2 / dt) * np.array(
        [
            q1[0] * q2[1] - q1[1] * q2[0] - q1[2] * q2[3] + q1[3] * q2[2],
            q1[0] * q2[2] + q1[1] * q2[3] - q1[2] * q2[0] - q1[3] * q2[1],
            q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1] - q1[3] * q2[0],
        ]
    )


# Function to calculate velocities
def calculate_velocity(df, col_name, time_step):
    velocities = np.zeros(len(df))
    # Central diff
    velocities[1:-1] = (
        df[col_name].iloc[2:].values - df[col_name].iloc[:-2].values
    ) / (2 * time_step)
    # Forward diff
    velocities[0] = (df[col_name].iloc[1] - df[col_name].iloc[0]) / time_step
    # Backward diff
    velocities[-1] = (df[col_name].iloc[-1] - df[col_name].iloc[-2]) / time_step
    return velocities


def my_ang_vel(q1, q2, dt, order="wxyz", frame="bdy"):
    """
    Calc angular velocity from q1 -> q2 with interval dt using slerp
    """

    if order == "wxyz":
        # scipy uses [x, y, z, w]
        q1 = q1[[1, 2, 3, 0]]
        q2 = q2[[1, 2, 3, 0]]
    elif order == "xyzw":
        pass
    else:
        raise ValueError("Invalid quaternion order")
    
    # [x, y, z, w] (scipy)
    q1 = R.from_quat(q1)
    q2 = R.from_quat(q2)
    if np.dot(q1.as_quat(), q2.as_quat()) < 0:
        q2 = R.from_quat(-q2.as_quat())

    if frame == "wld":
        rel_rot = q2 * q1.inv()
    elif frame == "bdy":
        rel_rot = q1.inv() * q2

    angle = rel_rot.magnitude()
    axis = rel_rot.as_rotvec() / angle if angle != 0 else np.zeros(3)

    return axis * angle / dt


def main():

    csv_files = glob.glob("./data/*.csv")

    # load csv
    for input_file in csv_files:

        print(f"\nProcessing {input_file}...")

        df = pd.read_csv(input_file, header=0)
        df.dropna(inplace=True)  # rm rows with NaN

        # Time (s), Vicon Orientation W, Vicon Orientation X, Vicon Orientation Y, Vicon Orientation Z, IMU Acceleration X, IMU Acceleration Y, IMU Acceleration Z, IMU Gyroscope X, IMU Gyroscope Y, IMU Gyroscope Z, IMU Magnetometer X, IMU Magnetometer Y, IMU Magnetometer Z
        # 0,0,0,0,0,9.8721,0.53516,0.23926,0.0048828,0.0083008,0.00048828,-0.91211,-0.51855,0.44824
        # 0.012,0,0,0,0,9.9297,0.53027,0.25098,0.0029297,0.0048828,0.0014648,-0.9209,-0.5459,0.44824

        # print(df.head())   # Debug print to check first few rows of the DataFrame

        time_array = df['Time (s)'].values
        # diff time to get time step
        time_step = np.diff(time_array)  # 1 element less than time_array

        print(df.columns)

        vicon_orientation_columns = df[['Vicon Orientation W', 'Vicon Orientation X', 'Vicon Orientation Y', 'Vicon Orientation Z']]
        vicon_orientation_array = vicon_orientation_columns.to_numpy()

        imu_gyroscope_columns = df[['IMU Gyroscope X', 'IMU Gyroscope Y', 'IMU Gyroscope Z']]
        imu_gyroscope_array = imu_gyroscope_columns.to_numpy()
        # remove the last element to match the length of vicon_orientation_array
        imu_gyroscope_array = imu_gyroscope_array[:-1]

        quaternions = vicon_orientation_array
        quaternions /= np.linalg.norm(quaternions, axis=1)[:, None]
        ang_vel = np.zeros((len(df), 3))
        ang_vel_compare = np.zeros((len(df), 3))
        # extend quaternions to avoid edge case
        # quaternions_ext = np.concatenate((quaternions[0:1], quaternions, quaternions[-1:]), axis=0)
        # quaternions_ahrs = ahrs.QuaternionArray(quaternions_ext)

        for i in range(len(df) - 1):
            q_curr = quaternions[i]
            q_next = quaternions[i + 1]
            ang_vel[i] = my_ang_vel(q_curr, q_next, time_step[i])
            ang_vel_compare[i] = angular_velocities(q_curr, q_next, time_step[i])
            pass

        # plot angular velocities (ang_vel, ang_vel_compare, and imu_gyroscope_array)
        # same dim together
        fig, axs = plt.subplots(3, 1, figsize=(6, 8))
        fig.suptitle(input_file, fontsize=12)

        axs[0].plot(imu_gyroscope_array[:, 0], label='imu_gyroscope_array', linestyle='-', marker='s', color='grey', linewidth=0.2, markersize=0.1)
        axs[0].plot(ang_vel[:, 0], label='my_ang_vel', linestyle='-', marker='o', color='blue', linewidth=0.3, markersize=0.2)
        axs[0].plot(ang_vel_compare[:, 0], label='ang_vel_compare', linestyle='-', marker='x', color='red', linewidth=0.5, markersize=0.2)
        axs[0].legend()
        axs[0].set_title('X')
        
        axs[1].plot(imu_gyroscope_array[:, 1], label='imu_gyroscope_array', linestyle='-', marker='s', color='grey', linewidth=0.2, markersize=0.1)
        axs[1].plot(ang_vel[:, 1], label='my_ang_vel', linestyle='-', marker='o', color='blue', linewidth=0.3, markersize=0.2)
        axs[1].plot(ang_vel_compare[:, 1], label='ang_vel_compare', linestyle='-', marker='x', color='red', linewidth=0.5, markersize=0.2)
        axs[1].legend()
        axs[1].set_title('Y')
        
        axs[2].plot(imu_gyroscope_array[:, 2], label='imu_gyroscope_array', linestyle='-', marker='s', color='grey', linewidth=0.2, markersize=0.1)
        axs[2].plot(ang_vel[:, 2], label='my_ang_vel', linestyle='-', marker='o', color='blue', linewidth=0.3, markersize=0.2)
        axs[2].plot(ang_vel_compare[:, 2], label='ang_vel_compare', linestyle='-', marker='x', color='red', linewidth=0.5, markersize=0.2)
        axs[2].legend()
        axs[2].set_title('Z')
        # plt.show()

        # save the figure
        fig.savefig(f"{input_file}_ang_vel.png", dpi=800)

        pass


if __name__ == "__main__":
    main()
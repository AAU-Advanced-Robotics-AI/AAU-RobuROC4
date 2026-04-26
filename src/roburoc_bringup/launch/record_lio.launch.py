"""
Launch file to record a rosbag for offline LIO + RTK-GPS development.
Records: LiDAR pointclouds, IMU (Livox built-in), RTK-GPS, TF

Bags are saved to:  ~/rosbags/<label>_<YYYYMMDD_HHMMSS>/
Default label:      roburoc_lio

Usage:
    ros2 launch roburoc_bringup record_lio.launch.py
    ros2 launch roburoc_bringup record_lio.launch.py label:=outdoor_football_field
    ros2 launch roburoc_bringup record_lio.launch.py label:=test bag_dir:=/mnt/ssd/rosbags
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_record(context, *args, **kwargs):
    label   = LaunchConfiguration('label').perform(context)
    bag_dir = LaunchConfiguration('bag_dir').perform(context)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path  = os.path.join(
        os.path.expanduser(bag_dir),
        f'{label}_{timestamp}'
    )

    os.makedirs(os.path.expanduser(bag_dir), exist_ok=True)

    print(f'\n[record_lio] Saving bag to: {bag_path}\n')

    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                # Livox MID360 — point cloud at ~10 Hz
                '/livox/lidar',
                # Livox MID360 built-in IMU — ~200 Hz
                '/livox/imu',
                # RTK-GPS
                '/ublox_gps_node/fix',           # NavSatFix (lat/lon/alt + covariance)
                '/ublox_gps_node/fix_velocity',  # Doppler velocity
                '/ublox_gps_node/navpvt',        # Raw NavPVT (RTK fix type, PDOP, etc.)
                '/nmea',                         # NMEA sentences (raw GPS strings)
                '/rtcm',                         # RTCM corrections from NTRIP (incoming)
                '/rxmrtcm',                      # RTCM echoed back from receiver (confirms injection)
                
                # TF (static sensor extrinsics + dynamic transforms)
                '/tf',
                '/tf_static',
                # URDF — latched, contains full sensor geometry; needed for offline replay
                '/robot_description',
                # Wheel encoders — useful additional constraint for LIO-SAM
                '/joint_states',
                # Gamepad + commanded velocity — useful to know when/how robot was moved
                '/joy',
                '/roburoc/cmd_vel',
                # Localization outputs (robot_localization dual EKF)
                '/odometry/local',           # ekf_local: odom → base_link
                '/odometry/global',          # ekf_global: map → odom (GPS-fused)
                '/odometry/gps',             # navsat_transform: GPS in local frame
                '/gps/filtered',             # filtered GPS fix (lat/lon back-projected)
                # Per-message compression: each message compressed on write, safe Ctrl+C shutdown.
                # (file-mode defers compression to exit, gets SIGKILL'd on large bags)
                '--compression-mode', 'message',
                '--compression-format', 'zstd',
                '-o', bag_path,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'label',
            default_value='roburoc_lio',
            description='Descriptive label prepended to the timestamped bag folder name'
        ),
        DeclareLaunchArgument(
            'bag_dir',
            default_value='~/rosbags',
            description='Directory in which to create the bag folder'
        ),
        OpaqueFunction(function=_launch_record),
    ])

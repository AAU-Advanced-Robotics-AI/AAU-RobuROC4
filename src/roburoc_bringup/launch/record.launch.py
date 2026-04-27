"""
Launch file to record a rosbag for offline development.
Records: ALL topics (--all).

Recording everything during development costs almost nothing with per-message
zstd compression, and prevents the situation where you need a topic for
debugging but didn't record it.  Switch to a selective topic list only if
onboard storage becomes a hard constraint.

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
                '--all',
                # Per-message compression: each message compressed on write,
                # so Ctrl+C gives a valid bag even on large recordings.
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

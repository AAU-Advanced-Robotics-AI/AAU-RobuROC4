"""
offline_pgo.launch.py — run the GTSAM PGO backend offline against a raw bag

The PGO node needs three synchronized live inputs:
    /Odometry          FAST-LIO2 pose      (camera_init -> body)
    /cloud_registered  registered scan     (world frame)
    /ublox_gps_node/navpvt  RTK NavPVT     (replayed from the raw bag)

No recorded bag contains /cloud_registered, and pgo_node hard-syncs /Odometry
with /cloud_registered before creating a keyframe, so FAST-LIO must be re-run
live.  This launch therefore mirrors offline_fastlio.launch.py:

  raw bag (--clock)  ->  fast_lio (via roburoc_localization/fast_lio.launch.py)
                     ->  /Odometry + /cloud_registered + static TFs
       + /ublox_gps_node/navpvt (from the bag)
                     ->  pgo_node  ->  optimized trajectory + map

Everything runs on one simulation clock (use_sim_time:=true).  When bag replay
finishes the launch calls /pgo/save (writes optimized_poses_tum.txt, graph.g2o,
map.pcd, georeference.txt to save_directory) and shuts down.

Usage
-----
  # Source ros + lio_ws (fast_lio) + rtk_ws (ublox_msgs) + AAU-RobuROC4 first.
  ros2 launch lidar_gps_pgo offline_pgo.launch.py \\
      raw_bag:=$HOME/data/rosbags/test_day_05_28/roburoc_lio_20260528_161115 \\
      rate:=0.5

Notes
-----
  - FAST-LIO scan matching is the bottleneck; keep rate <= 1.0 and watch for
    "drop message" warnings.  Lower to 0.5 if scans are dropped.
  - Optimizer artifacts go to <raw_bag>_pgo_artifacts (overrides save_directory
    in pgo_offline.yaml).  scripts/pgo_to_bag.py then turns those into the
    PlotJuggler-replayable rosbag <raw_bag>_pgo.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    raw_bag = os.path.expanduser(context.launch_configurations['raw_bag'].rstrip('/'))
    rate    = context.launch_configurations['rate']

    # Optimizer artifacts (TUM/g2o/pcd/georeference).  The PlotJuggler-replayable
    # rosbag <raw_bag>_pgo is produced from these by scripts/pgo_to_bag.py.
    save_dir = raw_bag + '_pgo_artifacts'

    params_file = os.path.join(
        get_package_share_directory('lidar_gps_pgo'), 'config', 'pgo_offline.yaml')

    fast_lio_launch = os.path.join(
        get_package_share_directory('roburoc_localization'), 'launch', 'fast_lio.launch.py')

    nodes = [

        # ── FAST-LIO + static TFs (odom->camera_init, livox_frame->body) ──────
        # Publishes /Odometry and /cloud_registered which pgo_node consumes.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # ── PGO backend ───────────────────────────────────────────────────────
        Node(
            package='lidar_gps_pgo',
            executable='pgo_node',
            name='pgo',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': True,
                 'save_directory': save_dir},
            ],
        ),

        # ── Bag replay (delayed 3 s to let FAST-LIO + PGO initialise) ─────────
        LogInfo(msg='Waiting 3 s for FAST-LIO / PGO to initialise before bag replay...'),
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg=f'Starting bag replay at rate={rate}: {raw_bag}'),
                (bag_play := ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play', raw_bag,
                        '--clock',
                        '--rate', rate,
                        '--topics',
                        '/livox/lidar',
                        '/livox/imu',
                        '/ublox_gps_node/navpvt',   # RTK fixed/float/none flag
                        '/ublox_gps_node/fix',      # NavSatFix position + cov (fused mode)
                        '/tf_static',
                        '/robot_description',
                    ],
                    output='screen',
                )),
            ],
        ),

        # ── On replay finish: save PGO outputs, then shut down ────────────────
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_play,
                on_exit=[
                    LogInfo(msg=f'Bag replay finished — saving PGO outputs to {save_dir}'),
                    (save := ExecuteProcess(
                        cmd=['ros2', 'service', 'call',
                             '/pgo/save', 'std_srvs/srv/Trigger'],
                        output='screen',
                    )),
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=save,
                on_exit=[LogInfo(msg='PGO save complete — shutting down.'), Shutdown()],
            )
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'raw_bag',
            description='Absolute path to the raw input rosbag directory '
                        '(with /livox/lidar, /livox/imu, /ublox_gps_node/navpvt).',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description='Bag playback rate. FAST-LIO is the bottleneck; '
                        'lower to 0.5 if scans are dropped.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])

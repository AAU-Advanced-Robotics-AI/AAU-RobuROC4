"""
offline_pgo_online.launch.py — replay a raw bag through the ONLINE PGO stack and
record the LIVE outputs, simulating on-robot behaviour (analogous to
offline_ekf.launch.py for the dual-EKF).

Unlike offline_pgo.launch.py (which saves the final, post-hoc *optimized*
trajectory as TUM), this records what the robot would actually publish in real
time: the live /pgo/odometry stream and the map->odom TF as they evolve — with the
real-time loop-closure / GPS jumps — so it is directly comparable to the EKF's
live /odometry/filtered/global recorded by offline_ekf.

  raw bag ─(--clock)─► localization_pgo (FAST-LIO + lio_relay + ekf_local + pgo_node)
                       ──► record  /pgo/odometry /odom /tf ... ──► <bag>_pgo_online

At the end it also calls /pgo/save so the georeference (ENU<->map transform) is
written to <bag>_pgo_online_artifacts, which the comparison step uses to express
the live /pgo/odometry in the pipeline ENU frame.

Usage:
  ros2 launch roburoc_localization offline_pgo_online.launch.py \\
      raw_bag:=$HOME/data/rosbags/test_day_05_28/roburoc_lio_20260528_161115 rate:=1.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, Shutdown, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    raw_bag = os.path.expanduser(context.launch_configurations['raw_bag'].rstrip('/'))
    rate = context.launch_configurations['rate']
    record = context.launch_configurations['record'].lower() in ('true', '1', 'yes')

    out_bag = raw_bag + '_pgo_online'
    artifacts = raw_bag + '_pgo_online_artifacts'

    loc_pgo = os.path.join(get_package_share_directory('roburoc_localization'),
                           'launch', 'localization_pgo.launch.py')

    nodes = [
        # ── base_link -> livox_frame static TF (URDF joint) ──────────────────
        # On the robot robot_state_publisher provides this; when replaying a raw
        # bag it is absent, so lio_relay can't resolve body->base_link and drops
        # every message (no /odom). Values from sensor_mount.yaml [livox_mid360].
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_base_link_to_livox_frame', output='screen',
            arguments=['--x', '0.60', '--y', '0.0', '--z', '0.7625',
                       '--roll', '0.0', '--pitch', '0.44457789', '--yaw', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
            parameters=[{'use_sim_time': True}],
        ),

        # ── online localization stack on the sim clock ───────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(loc_pgo),
            launch_arguments={'use_sim_time': 'true',
                              'pgo_save_dir': artifacts}.items(),
        ),

        LogInfo(msg='Waiting 4 s for FAST-LIO / EKF / PGO to initialise...'),
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg=f'Replaying raw bag at rate={rate}: {raw_bag}'),
                (bag_play := ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', raw_bag, '--clock', '--rate', rate,
                         '--topics',
                         '/livox/lidar', '/livox/imu',
                         '/ublox_gps_node/fix', '/ublox_gps_node/navpvt',
                         '/tf_static', '/robot_description'],
                    output='screen')),
            ],
        ),

        # ── on replay end: save georeference, then shut down ─────────────────
        RegisterEventHandler(OnProcessExit(
            target_action=bag_play,
            on_exit=[
                LogInfo(msg=f'Replay done — saving PGO georeference to {artifacts}'),
                (save := ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/pgo/save',
                         'std_srvs/srv/Trigger'], output='screen')),
            ])),
        RegisterEventHandler(OnProcessExit(
            target_action=save,
            on_exit=[LogInfo(msg='Done — shutting down.'), Shutdown()])),
    ]

    if record:
        nodes.insert(0, ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', out_bag,
                 '/pgo/odometry',           # LIVE global pose (map frame) — key output
                 '/odom',                   # LIVE local pose (odom->base_link)
                 '/pgo/pose', '/pgo/path',  # keyframe-rate reference
                 '/tf', '/tf_static',       # map->odom + odom->base_link history
                 '/ublox_gps_node/fix', '/ublox_gps_node/navpvt',
                 '/robot_description',
                 '--use-sim-time'],
            output='screen'))
        nodes.insert(0, LogInfo(msg=f'Recording live PGO outputs to: {out_bag}'))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('raw_bag',
                              description='raw input rosbag directory'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='replay rate (1.0 = real time)'),
        DeclareLaunchArgument('record', default_value='true'),
        OpaqueFunction(function=_setup),
    ])

#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _find_launch_file(pkg_name: str, launch_name: str, keywords=None):
    keywords = keywords or []

    # 绝对路径优先
    if launch_name and os.path.isabs(launch_name):
        if os.path.exists(launch_name):
            return launch_name
        raise RuntimeError(f"launch file does not exist: {launch_name}")

    ws_root = os.path.expanduser("~/maize_weeding_system")

    launch_dirs = []

    # 1. 优先找 install/share 下的 launch
    try:
        pkg_share = get_package_share_directory(pkg_name)
        launch_dirs.append(os.path.join(pkg_share, "launch"))
    except PackageNotFoundError:
        pass

    # 2. 再找源码目录
    launch_dirs.append(os.path.join(ws_root, "src", pkg_name, "launch"))

    # 3. 特殊包名兜底
    special_dirs = {
        "livox_ros_driver2": [
            os.path.join(ws_root, "src", "livox_ros_driver2", "launch"),
            os.path.join(ws_root, "src", "livox_ros_driver2", "launch_ROS2"),
        ],
        "fast_livo": [
            os.path.join(ws_root, "src", "FAST-LIVO2", "launch"),
        ],
        "miivii_gmsl_camera": [
            os.path.join(ws_root, "src", "miivii_gmsl_camera", "launch"),
        ],
    }

    for d in special_dirs.get(pkg_name, []):
        launch_dirs.append(d)

    searched = []

    for launch_dir in launch_dirs:
        searched.append(launch_dir)

        if not os.path.isdir(launch_dir):
            continue

        # 指定 launch 文件名
        if launch_name and launch_name != "auto":
            path = os.path.join(launch_dir, launch_name)
            if os.path.exists(path):
                return path

        # 自动选择
        files = []
        for name in sorted(os.listdir(launch_dir)):
            if name.endswith(".launch.py") or name.endswith("_launch.py") or name.endswith(".py"):
                files.append(name)

        if not files:
            continue

        for kw in keywords:
            for name in files:
                if kw.lower() in name.lower():
                    return os.path.join(launch_dir, name)

        return os.path.join(launch_dir, files[0])

    raise RuntimeError(
        f"launch file '{launch_name}' for package '{pkg_name}' not found. searched: {searched}"
    )


def _include_one(context, label, use_key, pkg_key, launch_key, delay_key, keywords=None, extra_args=None):
    use_it = _as_bool(LaunchConfiguration(use_key).perform(context))
    if not use_it:
        return [LogInfo(msg=f"[bringup] skip {label}")]

    pkg_name = LaunchConfiguration(pkg_key).perform(context)
    launch_name = LaunchConfiguration(launch_key).perform(context)
    delay = float(LaunchConfiguration(delay_key).perform(context))

    try:
        launch_path = _find_launch_file(pkg_name, launch_name, keywords=keywords)
    except (PackageNotFoundError, RuntimeError) as exc:
        return [LogInfo(msg=f"[bringup] {label} not started: {exc}")]

    launch_arguments = {}
    if extra_args:
        for k, v in extra_args.items():
            vv = LaunchConfiguration(v).perform(context) if isinstance(v, str) else str(v)
            if vv:
                launch_arguments[k] = vv

    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=launch_arguments.items(),
    )

    actions = [
        LogInfo(msg=f"[bringup] start {label}: package={pkg_name}, launch={os.path.basename(launch_path)}")
    ]

    if delay > 0.0:
        actions.append(TimerAction(period=delay, actions=[action]))
    else:
        actions.append(action)

    return actions


def _launch_setup(context, *args, **kwargs):
    actions = []

    actions += _include_one(
        context,
        label="Livox MID360",
        use_key="use_livox",
        pkg_key="livox_pkg",
        launch_key="livox_launch",
        delay_key="livox_delay",
        keywords=["MID360", "mid360", "livox"],
    )

    actions += _include_one(
        context,
        label="GMSL camera",
        use_key="use_camera",
        pkg_key="camera_pkg",
        launch_key="camera_launch",
        delay_key="camera_delay",
        keywords=["camera", "gmsl", "miivii"],
    )

    actions += _include_one(
        context,
        label="FAST-LIVO2",
        use_key="use_fast_livo",
        pkg_key="fast_livo_pkg",
        launch_key="fast_livo_launch",
        delay_key="fast_livo_delay",
        keywords=["fast", "livo", "mapping", "avia"],
    )

    actions += _include_one(
        context,
        label="seedling perception",
        use_key="use_perception",
        pkg_key="perception_pkg",
        launch_key="perception_launch",
        delay_key="perception_delay",
        keywords=["seedling", "orange", "pipeline"],
        extra_args={"config_file": "perception_config"},
    )

    actions += _include_one(
        context,
        label="ground mapper",
        use_key="use_ground_mapper",
        pkg_key="ground_mapper_pkg",
        launch_key="ground_mapper_launch",
        delay_key="ground_mapper_delay",
        keywords=["ground", "mapper"],
    )

    use_rviz = _as_bool(LaunchConfiguration("use_rviz").perform(context))
    if use_rviz:
        rviz_config = LaunchConfiguration("rviz_config").perform(context)
        rviz_args = []
        if rviz_config and os.path.exists(rviz_config):
            rviz_args = ["-d", rviz_config]

        delay = float(LaunchConfiguration("rviz_delay").perform(context))
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
        )

        if delay > 0.0:
            actions.append(TimerAction(period=delay, actions=[rviz_node]))
        else:
            actions.append(rviz_node)
    else:
        actions.append(LogInfo(msg="[bringup] skip RViz"))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Whether to start each subsystem.
        DeclareLaunchArgument("use_livox", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_fast_livo", default_value="true"),
        DeclareLaunchArgument("use_perception", default_value="true"),
        DeclareLaunchArgument("use_ground_mapper", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),

        # Launch package names.
        DeclareLaunchArgument("livox_pkg", default_value="livox_ros_driver2"),
        DeclareLaunchArgument("camera_pkg", default_value="miivii_gmsl_camera"),
        DeclareLaunchArgument("fast_livo_pkg", default_value="fast_livo"),
        DeclareLaunchArgument("perception_pkg", default_value="seedling_semantic_mapping"),
        DeclareLaunchArgument("ground_mapper_pkg", default_value="ground_mapper"),

        # Launch file names. Use "auto" if you are unsure.
        DeclareLaunchArgument("livox_launch", default_value="auto"),
        DeclareLaunchArgument("camera_launch", default_value="auto"),
        DeclareLaunchArgument("fast_livo_launch", default_value="auto"),
        DeclareLaunchArgument("perception_launch", default_value="seedling_pipeline.launch.py"),
        DeclareLaunchArgument("ground_mapper_launch", default_value="ground_mapper.launch.py"),

        # Optional perception config.
        DeclareLaunchArgument("perception_config", default_value=""),

        # Startup delays in seconds.
        DeclareLaunchArgument("livox_delay", default_value="0.0"),
        DeclareLaunchArgument("camera_delay", default_value="1.0"),
        DeclareLaunchArgument("fast_livo_delay", default_value="4.0"),
        DeclareLaunchArgument("perception_delay", default_value="8.0"),
        DeclareLaunchArgument("ground_mapper_delay", default_value="10.0"),
        DeclareLaunchArgument("rviz_delay", default_value="12.0"),

        # RViz.
        DeclareLaunchArgument("rviz_config", default_value=""),

        OpaqueFunction(function=_launch_setup),
    ])

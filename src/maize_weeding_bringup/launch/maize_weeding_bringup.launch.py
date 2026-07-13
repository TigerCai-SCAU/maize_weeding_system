#!/usr/bin/env python3
import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    """Convert a launch argument string to bool."""
    return str(value).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def _find_launch_file(
    pkg_name: str,
    launch_name: str,
    keywords=None,
):
    """Find a launch file in install or source directories.

    If launch_name is a concrete filename, only that filename is accepted.
    Automatic selection is used only when launch_name is "auto".
    """
    keywords = keywords or []

    # Absolute path has the highest priority.
    if launch_name and os.path.isabs(launch_name):
        if os.path.exists(launch_name):
            return launch_name

        raise RuntimeError(
            f"launch file does not exist: {launch_name}"
        )

    ws_root = os.path.expanduser(
        "~/maize_weeding_system"
    )

    launch_dirs = []

    # 1. Installed package launch directory.
    try:
        pkg_share = get_package_share_directory(
            pkg_name
        )
        launch_dirs.append(
            os.path.join(pkg_share, "launch")
        )
    except PackageNotFoundError:
        pass

    # 2. Generic source package path.
    launch_dirs.append(
        os.path.join(
            ws_root,
            "src",
            pkg_name,
            "launch",
        )
    )

    # 3. Repository-specific fallback paths.
    special_dirs = {
        "livox_ros_driver2": [
            os.path.join(
                ws_root,
                "src",
                "livox_ros_driver2",
                "launch",
            ),
            os.path.join(
                ws_root,
                "src",
                "livox_ros_driver2",
                "launch_ROS2",
            ),
        ],
        "fast_livo": [
            os.path.join(
                ws_root,
                "src",
                "FAST-LIVO2",
                "launch",
            ),
        ],
        "miivii_gmsl_camera": [
            os.path.join(
                ws_root,
                "src",
                "miivii_gmsl_camera",
                "miivii_gmsl_ros",
                "launch",
            ),
        ],
    }

    for launch_dir in special_dirs.get(
        pkg_name,
        [],
    ):
        launch_dirs.append(launch_dir)

    # Remove duplicate paths while preserving order.
    unique_dirs = []
    for launch_dir in launch_dirs:
        if launch_dir not in unique_dirs:
            unique_dirs.append(launch_dir)

    searched = []

    for launch_dir in unique_dirs:
        searched.append(launch_dir)

        if not os.path.isdir(launch_dir):
            continue

        # Concrete filename: only search for that file.
        if launch_name and launch_name != "auto":
            path = os.path.join(
                launch_dir,
                launch_name,
            )

            if os.path.exists(path):
                return path

            # Continue to the next candidate directory.
            continue

        # Automatic selection is only used when launch_name=auto.
        files = []

        for name in sorted(
            os.listdir(launch_dir)
        ):
            if (
                name.endswith(".launch.py")
                or name.endswith("_launch.py")
            ):
                files.append(name)

        if not files:
            continue

        # Prefer filenames matching the supplied keywords.
        for keyword in keywords:
            for name in files:
                if keyword.lower() in name.lower():
                    return os.path.join(
                        launch_dir,
                        name,
                    )

        # No keyword matched: use the first launch file.
        return os.path.join(
            launch_dir,
            files[0],
        )

    raise RuntimeError(
        f"launch file '{launch_name}' "
        f"for package '{pkg_name}' not found. "
        f"searched: {searched}"
    )


def _include_one(
    context,
    label,
    use_key,
    pkg_key,
    launch_key,
    delay_key,
    keywords=None,
    extra_args=None,
):
    """Create one delayed IncludeLaunchDescription action."""
    use_it = _as_bool(
        LaunchConfiguration(
            use_key
        ).perform(context)
    )

    if not use_it:
        return [
            LogInfo(
                msg=f"[bringup] skip {label}"
            )
        ]

    pkg_name = LaunchConfiguration(
        pkg_key
    ).perform(context)

    launch_name = LaunchConfiguration(
        launch_key
    ).perform(context)

    delay = float(
        LaunchConfiguration(
            delay_key
        ).perform(context)
    )

    try:
        launch_path = _find_launch_file(
            pkg_name,
            launch_name,
            keywords=keywords,
        )
    except (
        PackageNotFoundError,
        RuntimeError,
    ) as exc:
        return [
            LogInfo(
                msg=(
                    f"[bringup] {label} "
                    f"not started: {exc}"
                )
            )
        ]

    launch_arguments = {}

    if extra_args:
        for launch_arg_name, config_key in extra_args.items():
            value = LaunchConfiguration(
                config_key
            ).perform(context)

            # Do not pass an empty argument.
            if value:
                launch_arguments[
                    launch_arg_name
                ] = value

    include_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_path
        ),
        launch_arguments=(
            launch_arguments.items()
        ),
    )

    actions = [
        LogInfo(
            msg=(
                f"[bringup] schedule {label}: "
                f"package={pkg_name}, "
                f"launch={os.path.basename(launch_path)}, "
                f"delay={delay:.1f}s"
            )
        )
    ]

    if delay > 0.0:
        actions.append(
            TimerAction(
                period=delay,
                actions=[include_action],
            )
        )
    else:
        actions.append(include_action)

    return actions


def _launch_setup(
    context,
    *args,
    **kwargs,
):
    """Resolve launch paths and assemble all subsystem actions."""
    actions = []

    # 1. Livox MID360.
    actions += _include_one(
        context,
        label="Livox MID360",
        use_key="use_livox",
        pkg_key="livox_pkg",
        launch_key="livox_launch",
        delay_key="livox_delay",
        keywords=[
            "MID360",
            "mid360",
            "livox",
        ],
    )

    # 2. GMSL camera.
    actions += _include_one(
        context,
        label="GMSL camera",
        use_key="use_camera",
        pkg_key="camera_pkg",
        launch_key="camera_launch",
        delay_key="camera_delay",
        keywords=[
            "camera",
            "gmsl",
            "miivii",
            "single",
        ],
    )

    # 3. FAST-LIVO2.
    actions += _include_one(
        context,
        label="FAST-LIVO2",
        use_key="use_fast_livo",
        pkg_key="fast_livo_pkg",
        launch_key="fast_livo_launch",
        delay_key="fast_livo_delay",
        keywords=[
            "mapping_mid360",
            "mid360",
            "mapping",
            "fast",
            "livo",
        ],
    )

    # 4. Seedling perception.
    actions += _include_one(
        context,
        label="seedling perception",
        use_key="use_perception",
        pkg_key="perception_pkg",
        launch_key="perception_launch",
        delay_key="perception_delay",
        keywords=[
            "seedling",
            "pipeline",
        ],
        extra_args={
            "config_file": (
                "perception_config"
            ),
            "localizer_executable": (
                "perception_localizer"
            ),
        },
    )

    # 5. Ground mapper.
    actions += _include_one(
        context,
        label="ground mapper",
        use_key="use_ground_mapper",
        pkg_key="ground_mapper_pkg",
        launch_key="ground_mapper_launch",
        delay_key="ground_mapper_delay",
        keywords=[
            "ground",
            "mapper",
        ],
    )

    # 6. RViz.
    use_rviz = _as_bool(
        LaunchConfiguration(
            "use_rviz"
        ).perform(context)
    )

    if use_rviz:
        rviz_config = LaunchConfiguration(
            "rviz_config"
        ).perform(context)

        rviz_args = []

        if (
            rviz_config
            and os.path.exists(rviz_config)
        ):
            rviz_args = [
                "-d",
                rviz_config,
            ]

        rviz_delay = float(
            LaunchConfiguration(
                "rviz_delay"
            ).perform(context)
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
        )

        if rviz_delay > 0.0:
            actions.append(
                TimerAction(
                    period=rviz_delay,
                    actions=[rviz_node],
                )
            )
        else:
            actions.append(rviz_node)
    else:
        actions.append(
            LogInfo(
                msg="[bringup] skip RViz"
            )
        )

    return actions


def generate_launch_description():
    """Generate the complete maize weeding system launch description."""
    return LaunchDescription(
        [
            # Enable/disable subsystems.
            DeclareLaunchArgument(
                "use_livox",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_camera",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_fast_livo",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_perception",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_ground_mapper",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
            ),

            # Package names.
            DeclareLaunchArgument(
                "livox_pkg",
                default_value="livox_ros_driver2",
            ),
            DeclareLaunchArgument(
                "camera_pkg",
                default_value="miivii_gmsl_camera",
            ),
            DeclareLaunchArgument(
                "fast_livo_pkg",
                default_value="fast_livo",
            ),
            DeclareLaunchArgument(
                "perception_pkg",
                default_value="seedling_semantic_mapping",
            ),
            DeclareLaunchArgument(
                "ground_mapper_pkg",
                default_value="ground_mapper",
            ),

            # Launch filenames.
            DeclareLaunchArgument(
                "livox_launch",
                default_value="msg_MID360_launch.py",
            ),
            DeclareLaunchArgument(
                "camera_launch",
                default_value="single.launch.py",
            ),
            DeclareLaunchArgument(
                "fast_livo_launch",
                default_value="mapping_mid360.launch.py",
            ),
            DeclareLaunchArgument(
                "perception_launch",
                default_value="seedling_pipeline.launch.py",
            ),
            DeclareLaunchArgument(
                "ground_mapper_launch",
                default_value="ground_mapper.launch.py",
            ),

            # Perception configuration.
            DeclareLaunchArgument(
                "perception_config",
                default_value="",
            ),
            DeclareLaunchArgument(
                "perception_localizer",
                default_value="color_sep_localizer",
            ),

            # Startup delays, relative to bringup start.
            DeclareLaunchArgument(
                "livox_delay",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "camera_delay",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "fast_livo_delay",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "perception_delay",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "ground_mapper_delay",
                default_value="12.0",
            ),
            DeclareLaunchArgument(
                "rviz_delay",
                default_value="15.0",
            ),

            # Optional RViz config path.
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
            ),

            OpaqueFunction(
                function=_launch_setup
            ),
        ]
    )

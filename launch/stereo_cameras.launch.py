from ament_index_python.resources import has_resource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    """
    Stereo launch with two separate containers and a delay before starting the right camera
    to avoid color issues on my goofy ah module
    """
    left_camera_arg = DeclareLaunchArgument(
        "left_camera", default_value="0", description="ID of left camera"
    )
    left_format_arg = DeclareLaunchArgument(
        "left_format", default_value="RGB888", description="Pixel format of left camera"
    )
    right_camera_arg = DeclareLaunchArgument(
        "right_camera", default_value="1", description="ID of right camera"
    )
    right_format_arg = DeclareLaunchArgument(
        "right_format", default_value="RGB888", description="Pixel format of right camera"
    )

    left_camera = LaunchConfiguration("left_camera")
    left_format = LaunchConfiguration("left_format")
    right_camera = LaunchConfiguration("right_camera")
    right_format = LaunchConfiguration("right_format")

    left_nodes = [
        ComposableNode(
            package="camera_ros",
            plugin="camera::CameraNode",
            name="left_camera_node",
            namespace="left",
            parameters=[{
                "camera": left_camera,
                "width": 640,
                "height": 480,
                "format": left_format,
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]

    if has_resource("packages", "image_view"):
        left_nodes.append(
            ComposableNode(
                package="image_view",
                plugin="image_view::ImageViewNode",
                name="left_view",
                namespace="left",
                remappings=[("/image", "/left/image_raw")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    left_container = ComposableNodeContainer(
        name="left_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=left_nodes,
        output="screen",
    )

    right_nodes = [
        ComposableNode(
            package="camera_ros",
            plugin="camera::CameraNode",
            name="right_camera_node",
            namespace="right",
            parameters=[{
                "camera": right_camera,
                "width": 640,
                "height": 480,
                "format": right_format,
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]

    if has_resource("packages", "image_view"):
        right_nodes.append(
            ComposableNode(
                package="image_view",
                plugin="image_view::ImageViewNode",
                name="right_view",
                namespace="right",
                remappings=[("/image", "/right/image_raw")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    right_container = ComposableNodeContainer(
        name="right_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=right_nodes,
        output="screen",
    )

    delayed_right_container = TimerAction(
        period=2.0,
        actions=[right_container]
    )


    return LaunchDescription([
        left_camera_arg,
        left_format_arg,
        right_camera_arg,
        right_format_arg,
        left_container,
        delayed_right_container,
    ])

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect all 41h12 tags
cfg_41h12 = {
    "image_transport": "raw",
    "family": "Standard41h12",
    "size": 0.096,
    "max_hamming": 0,
    "z_up": True,
    "decimate": 1.0,
    "blur": 0.0,
    "refine-edges": 1,
    "threads": 1,
    "refine-decode": 0,
    "refine-pose": 1,
    "debug": 0

}

def generate_launch_description():
    rgbd_node = Node(
        package='apriltag_ros',
        node_executable='apriltag_node',
        node_namespace='',
        output='screen',
        remappings=[("/apriltag/image", "/camera/color/image_raw"), ("/apriltag/camera_info", "/camera/color/camera_info")],
        parameters=[cfg_41h12]
        )
    return launch.LaunchDescription([rgbd_node])
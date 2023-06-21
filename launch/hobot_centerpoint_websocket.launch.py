import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # 拷贝config中文件
    hobot_centerpoint_path = os.path.join(
        get_package_prefix('hobot_centerpoint'),
        "lib/hobot_centerpoint")
    print("hobot_centerpoint_path is ", hobot_centerpoint_path)
    cp_cmd = "cp -r " + hobot_centerpoint_path + "/config ."
    print("cp_cmd is ", cp_cmd)
    os.system(cp_cmd)

    hobot_centerpoint_node = Node(
        package='hobot_centerpoint',
        executable='hobot_centerpoint',
        output='screen',
        parameters=[
            {"is_show": True},
            {"is_loop": True}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # hobot_centerpoint图片编码&发布pkg
    # jpeg_codec_node = Node(
    #     package='hobot_codec',
    #     executable='hobot_codec_republish',
    #     output='screen',
    #     parameters=[
    #         {"channel": 1},
    #         {"in_mode": "ros"},
    #         {"in_format": "bgr8"},
    #         {"out_mode": "ros"},
    #         {"out_format": "jpeg"},
    #         {"sub_topic": "/hobot_centerpoint"},
    #         {"pub_topic": "/image"}
    #     ],
    #     arguments=['--ros-args', '--log-level', 'error']
    # )

    # web展示pkg
    web_node = Node(
        package='websocket',
        executable='websocket',
        output='screen',
        parameters=[
            {"image_topic": "/hobot_centerpoint"},
            {"image_type": "mjpeg"},
            {"only_show_image": True}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    return LaunchDescription([
        # jpeg_codec_node,
        hobot_centerpoint_node,
        web_node
    ])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description with multiple components."""
    
    ld = LaunchDescription()

    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("canopen_core"),  # 确保canopen_core包安装正常
                "launch",
                "canopen.launch.py",  # 确保该文件存在于指定目录
            )
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("erob_canopen"),  # 替换为实际包名
                "config",
                "robot_control",  # 替换为实际配置名称
                "master.dcf",
            ),
            "master_bin": os.path.join(
                get_package_share_directory("erob_canopen"),
                "config",
                "robot_control",
                "master.bin",
            ),
            "bus_config": os.path.join(
                get_package_share_directory("erob_canopen"),
                "config",
                "robot_control",
                "bus.yml",
            ),
            "can_interface_name": "can0",  # 固定值为 can0
        }.items(),
    )

    ld.add_action(device_container)

    return ld


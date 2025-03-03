from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明参数
    can_interface = LaunchConfiguration('can_interface')
    node_id = LaunchConfiguration('node_id')
    auto_start = LaunchConfiguration('auto_start')
    
    # 检查CAN接口状态
    check_can = ExecuteProcess(
        cmd=['bash', '-c', 'ip -details link show can0 || echo "CAN接口不存在"'],
        output='screen'
    )
    
    # 启动CANopenROS2节点
    canopen_ros2_node = Node(
        package='erob_master',
        executable='erob_canoepn_ros2',
        name='canopen_ros2',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'can_interface': can_interface,
                'node_id': node_id,
                'auto_start': auto_start
            }
        ]
    )
    
    # 列出节点、话题和服务
    list_info = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 5 && echo "列出所有节点:" && ros2 node list && echo "列出所有话题:" && ros2 topic list && echo "列出所有服务:" && ros2 service list'],
        output='screen'
    )
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN接口名称'
        ),
        DeclareLaunchArgument(
            'node_id',
            default_value='2',
            description='CANopen节点ID'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='是否自动启动电机'
        ),
        
        # 执行命令
        LogInfo(msg="启动CANopenROS2节点..."),
        check_can,
        canopen_ros2_node,
        list_info
    ]) 
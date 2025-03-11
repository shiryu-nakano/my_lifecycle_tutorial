import launch
import launch.actions
import launch_ros.actions
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return launch.LaunchDescription([
        # Lifecycle Node: Hiragana
        LifecycleNode(
            package='my_lifecycle_tutorial',
            executable='lowercase_node',
            name='lowercase_node',
            namespace='',
            output='screen'
        ),
        # Lifecycle Node: Katakana
        LifecycleNode(
            package='my_lifecycle_tutorial',
            executable='uppercase_node',
            name='uppercase_node',
            namespace='',
            output='screen'
        ),
        # 通常ノード: Sakasama
        launch_ros.actions.Node(
            package='my_lifecycle_tutorial',
            executable='reverse_node',
            name='reverse_node',
            output='screen'
        ),
        # Supervisor (通常ノード; ライフサイクル制御)
        launch_ros.actions.Node(
            package='my_lifecycle_tutorial',
            executable='supervisor_node',
            name='supervisor',
            output='screen'
        ),
    ])

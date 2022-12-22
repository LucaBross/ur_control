from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    current_launch_arg = DeclareLaunchArgument(
        "thresh", default_value=TextSubstitution(text="0.6")
    )

    succesfull_grasp_detect_node=Node(
        package='subscriber_test',
        executable='succesful_grasp_detect',
        name='succesfull_grasp_detect',
        output='screen',
        emulate_tty=True,
        parameters=[
            { "thresh": LaunchConfiguration('thresh')}
        ]
    )

    return LaunchDescription([
        current_launch_arg,
        succesfull_grasp_detect_node,
    ])
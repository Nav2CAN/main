from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), '/rs_ours_camera1_launch.py']),
            ),
        Node(
            package='multi_person_tracker',
            executable='multi_person_tracker',
            name='multi_person_tracker',
        ),
        Node(
            package='interaction_detection',
            executable='interaction_detection',
            name='interaction_detection',
        )
    ])
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from geometry_msgs.msg import PoseWithCovarianceStamped

def generate_launch_description():

    initial_pose_x = 0.0  # X position in meters
    initial_pose_y = 0.0  # Y position in meters
    initial_pose_theta = 0.0  # Orientation in radians
    return LaunchDescription([
        # Step 1: Launch Turtlebot3 in Gazebo
        
      
        
        SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle_pi"),

        ExecuteProcess(
            cmd=["ros2", "launch", "turtlebot3_gazebo", "park.launch.py"],
            output="screen"
        ),

        # Step 2: Launch Turtlebot3 Navigation2 (with params and map file)
        TimerAction(
            period=5.0,  # Wait for 5 seconds (adjust as needed)
            actions=[
            
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle_pi"),
                ExecuteProcess(
                    cmd=[
                        "ros2", "launch", "turtlebot3_navigation2", "navigation2.launch.py",
                        "params_file:=/home/student/ros2_ws/src/turtlebot3/turtlebot3_navigation2/param/sprint335.yaml",
                        "map:=/home/student/ros2_ws/map3.yaml"
                        
                    ],
                    output="screen"
                )
            ]
        ),

        # Step 3: Run sprint4_35 node
        TimerAction(
            period=10.0,  # Wait for 10 seconds (adjust based on navigation start time)
            actions=[
                ExecuteProcess(
                    cmd=["gnome-terminal", "--", "ros2", "run", "navigation_handler", "navigation_handler"],
                    output="screen"
                )
            ]
        ),
        TimerAction(
            period=10.0,  # Wait for 10 seconds (adjust based on navigation start time)
            actions=[
                ExecuteProcess(
                    cmd=["gnome-terminal", "--", "ros2", "run", "GoalCycleNode", "GoalCycleNode"],
                    output="screen"
                )
            ]
        ),
        TimerAction(
            period=10.0,  # Wait for 10 seconds (adjust based on navigation start time)
            actions=[
                ExecuteProcess(
                    cmd=["gnome-terminal", "--", "python3", "pythonControlGui.py"],
                    output="screen"
                )
            ]
        ),
    ])

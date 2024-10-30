from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable

def generate_launch_description():
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
                    cmd=["gnome-terminal", "--", "ros2", "run", "sprint4_35", "sprint4_35"],
                    output="screen"
                )
            ]
        ),
    ])

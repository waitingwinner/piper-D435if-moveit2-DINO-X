from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 新增：立即执行 can_activate.sh（在最前面添加）
        TimerAction(
            period=0.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 
                         'cd /home/hu/piper_sdk-master && bash can_activate.sh can0 1000000; exec bash'],
                    output='screen'
                )
            ]
        ),
        
        # 原步骤1：延迟8秒执行
        TimerAction(
            period=8.0, 
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 'python3 /home/hu/piper_sdk_demo-master/piper_reset.py; exec bash'],
                    output='screen'
                )
            ]
        ),
        
        # 原步骤2：延迟16秒执行
        TimerAction(
            period=12.0, 
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 'python3 /home/hu/piper_sdk_demo-master/piper_enable.py; exec bash'],
                    output='screen'
                )
            ]
        ),
        
        # 原步骤3：延迟24秒执行
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 launch piper start_single_piper.launch.py; exec bash'],
                    output='screen'
                )
            ]
        ),
        
        # 原步骤4：延迟32秒执行
        TimerAction(
            period=27.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 launch hu_moveitv5 demo.launch.py; exec bash'],
                    output='screen'
                )
            ]
        ),
        # 原步骤5：延迟40秒执行
        TimerAction(
            period=34.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run learnning_piper move_target; exec bash'],
                    output='screen'
                )
            ]
        ),
        # 原步骤6：延迟48秒执行
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 'python3 /home/hu/DINO-X-API-main/test.py; exec bash'],
                    output='screen'
                )
            ]
        )
    ])

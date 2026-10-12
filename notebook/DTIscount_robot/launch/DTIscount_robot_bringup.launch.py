from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

# นำเข้าแพ็คเกจสร้างเงื่อนไขสำหรับ เปิด-ปิด หน้าจอพรีวิวบน Notebook
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # หา path ของ config YAML สำหรับ teleop_twist_joy
    pkg_share = get_package_share_directory('DTIscount_robot')
    teleop_config = os.path.join(pkg_share, 'config', 'teleop_joy.yaml')

    # =========================================================================
    # QoS Logger - Dynamic SINGLE / MULTICAST
    # =========================================================================
    qos_logger_script = os.path.expanduser(
        '~/ros2_ws/src/DTIscount_robot/scripts/qos_logger_dynamic.py'
    )

    qos_log_file = os.path.expanduser(
        '~/qos_logs/video_qos.csv'
    )

    qos_logger_process = ExecuteProcess(
        cmd=[
            'python3',
            qos_logger_script,

            '--mode',
            'single',

            '--experiment-id',
            'EXP001',

            '--output',
            qos_log_file,

            '--interval',
            '1.0',

            '--ros-mode-topic',
            '/qos/transport_mode'
        ],
        output='screen'
    )

    # =========================================================================
    # 1. ประกาศตัวแปรเปิด-ปิด "หน้าจอ Viewer" บน Notebook
    # =========================================================================
    # view_usb_cam_arg = DeclareLaunchArgument(
    #     'view_usb_cam',
    #     default_value='true',
    #     description='เปิดหน้าจอแสดงภาพจากกล้อง USB (true/false)'
    # )
    
    # view_pi_cam_arg = DeclareLaunchArgument(
    #     'view_pi_cam',
    #     default_value='true',
    #     description='เปิดหน้าจอแสดงภาพจากกล้องสายแพ Pi Camera (true/false)'
    # )

    # show_usb = LaunchConfiguration('view_usb_cam')
    # show_pi = LaunchConfiguration('view_pi_cam')

    return LaunchDescription([
        # view_usb_cam_arg,
        # view_pi_cam_arg,

        # ===============================
        # Joystick driver
        # ===============================
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{"dev": "/dev/input/js0"}]
        ),

        # ===============================
        # Teleop twist from joystick
        # ===============================
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[teleop_config],
            remappings=[
                ('cmd_vel', '/joy_cmd_vel')
            ]
        ),
        
        # ===============================
        # Human Follow Function
        # ===============================
        # Node(
        #     package="DTIscount_robot",
        #     executable="humanFollow_node",
        #     name="human_follow_controller",
        #     output="screen"
        # ),
        
        # ===============================
        # Human Follow IBVS Function
        # ===============================
        Node(
            package="DTIscount_robot",
            executable="humanFollowIBVS_node",
            name="human_follow_IBVS",
            output="screen"
        ),
        
        # ===============================
        # Joy Relay Node
        # ===============================
        Node(
            package="DTIscount_robot",
            executable="joyRelay_node",
            name="joy_relay_node",
            output="screen"
        ),
        
        # =========================================================================
        # 📺 จอแสดงผลที่ 1: ดึงภาพจากกล้อง USB (แก้กลับมาใช้ชื่อทอปปิกหลัก)
        # =========================================================================
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='usb_camera_viewer',
        #     arguments=[
        #         '/usb_cam/image_raw',  # 💡 แก้ไข: ใช้ชื่อทอปปิกหลักตัวเดิม (ห้ามเติม /compressed ท้ายชื่อ)
        #         '--ros-args', 
        #         '-p', 'image_transport:=compressed',  # ระบบจะดึงภาพบีบอัดให้ผ่านตรงนี้แทน
        #         '-p', 'policy:=best_effort'
        #     ],
        #     output='screen',
        #     condition=IfCondition(show_usb)
        # ),

        # =========================================================================
        # 📺 จอแสดงผลที่ 2: ดึงภาพจากกล้องสายแพ (แก้กลับมาใช้ชื่อทอปปิกหลัก)
        # =========================================================================
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='pi_camera_viewer',
        #     arguments=[
        #         '/pi_cam/image_raw',   # 💡 แก้ไข: ใช้ชื่อทอปปิกหลักตัวเดิม (ห้ามเติม /compressed ท้ายชื่อ)
        #         '--ros-args', 
        #         '-p', 'image_transport:=compressed',  # ระบบจะดึงภาพบีบอัดให้ผ่านตรงนี้แทน
        #         '-p', 'policy:=best_effort'
        #     ],
        #     output='screen',
        #     condition=IfCondition(show_pi)
        # ),
        
        # =========================================================================
        # QoS Logger
        # =========================================================================
        qos_logger_process,        
    ])